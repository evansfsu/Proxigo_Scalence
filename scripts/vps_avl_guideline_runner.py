#!/usr/bin/env python3
"""
Dataset-guideline runner for UAV-AVL Benchmark layout.

Expected structure (as in UAV-AVL Benchmark README):
  <dataset_root>/
    Data/
      metadata/<REGION>.json
      Reference_map/<REGION>/
      UAV_image/<REGION>/<PLACE>/*.JPG
    Regions_params/<REGION>.yaml

This script resolves paths from Regions_params YAML, prepares a Proxigo-style
reference region directory, and executes scripts/vps_avl_benchmark.py with the
resolved dataset inputs.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

from PIL import Image


def _require_yaml():
    try:
        import yaml  # type: ignore
    except Exception:
        return None
    return yaml


def _require_pyproj():
    try:
        from pyproj import Transformer  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("pyproj is required. Install with: pip install pyproj") from exc
    return Transformer


def _resolve_maybe_relative(path_str: str, dataset_root: Path) -> Path:
    p = Path(path_str.replace("\\", os.sep).replace("/", os.sep))
    if p.is_absolute():
        return p
    return dataset_root / p


def _parse_utm_epsg(utm_system: str) -> str:
    # Example: "50N" -> EPSG:32650, "33S" -> EPSG:32733
    m = re.match(r"^\s*(\d{1,2})\s*([NnSs])\s*$", utm_system)
    if not m:
        raise ValueError(f"Unsupported UTM system format: {utm_system}")
    zone = int(m.group(1))
    hemi = m.group(2).upper()
    return f"EPSG:{32600 + zone if hemi == 'N' else 32700 + zone}"


def _prepare_region_dir(
    *,
    dataset_root: Path,
    region: str,
    ref_type: str,
    region_cfg: dict,
    out_root: Path,
) -> tuple[Path, Path, dict]:
    key_prefix = f"{region}_{ref_type}"
    ref_tif = _resolve_maybe_relative(str(region_cfg[f"{key_prefix}_REF_PATH"]), dataset_root)
    dsm_tif = _resolve_maybe_relative(str(region_cfg[f"{key_prefix}_DSM_PATH"]), dataset_root)
    initial_x = float(region_cfg[f"{key_prefix}_REF_initialX"])
    initial_y = float(region_cfg[f"{key_prefix}_REF_initialY"])
    ref_res = float(region_cfg[f"{key_prefix}_REF_resolution"])
    utm_system = str(region_cfg[f"{region}_UTM_SYSTEM"])

    if not ref_tif.exists():
        raise FileNotFoundError(f"Reference map not found: {ref_tif}")
    if not dsm_tif.exists():
        raise FileNotFoundError(f"DSM map not found: {dsm_tif}")

    region_dir = out_root / f"{region}_{ref_type.lower()}"
    region_dir.mkdir(parents=True, exist_ok=True)
    sat_png = region_dir / "satellite.png"
    region_json = region_dir / "region.json"
    metadata_json = region_dir / "metadata.json"

    Image.MAX_IMAGE_PIXELS = None
    img = Image.open(ref_tif)
    w_px, h_px = img.size
    if not sat_png.exists():
        img.save(sat_png)

    west_x = initial_x
    north_y = initial_y
    east_x = initial_x + w_px * ref_res
    south_y = initial_y - h_px * ref_res

    Transformer = _require_pyproj()
    src_epsg = _parse_utm_epsg(utm_system)
    transformer = Transformer.from_crs(src_epsg, "EPSG:4326", always_xy=True)
    west_lon, north_lat = transformer.transform(west_x, north_y)
    east_lon, south_lat = transformer.transform(east_x, south_y)

    payload = {
        "center_lat": (north_lat + south_lat) * 0.5,
        "center_lon": (west_lon + east_lon) * 0.5,
        "north_lat": north_lat,
        "south_lat": south_lat,
        "west_lon": west_lon,
        "east_lon": east_lon,
        "width_px": w_px,
        "height_px": h_px,
        "meters_per_pixel": ref_res,
    }
    with open(region_json, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    proxigo_meta = {
        "region_id": f"{region}_{ref_type.lower()}",
        "center": {
            "latitude": payload["center_lat"],
            "longitude": payload["center_lon"],
        },
        "imagery": {
            "file": "satellite.png",
            "resolution_m": ref_res,
        },
    }
    with open(metadata_json, "w", encoding="utf-8") as f:
        json.dump(proxigo_meta, f, indent=2)

    align = {
        "ref_coordinate": region_cfg.get(f"{key_prefix}_REF_COORDINATE"),
        "dsm_coordinate": region_cfg.get(f"{key_prefix}_DSM_COORDINATE"),
        "dsm_resolution": region_cfg.get(f"{key_prefix}_DSM_resolution"),
    }
    return region_dir, dsm_tif, align


def _default_place(region_cfg: dict, region: str) -> str:
    places = region_cfg.get(f"{region}_UAV_PLACES") or region_cfg.get(f"{region}_PLACES")
    if not places or not isinstance(places, list):
        raise ValueError(f"Could not infer place list from region YAML for {region}")
    return str(places[0])


def _load_region_yaml(yaml_path: Path) -> dict:
    yaml_mod = _require_yaml()
    if yaml_mod is not None:
        with open(yaml_path, "r", encoding="utf-8") as f:
            return yaml_mod.safe_load(f)

    # Fallback parser for simple key/value + list YAML used by UAV-AVL region files.
    out: dict = {}
    current_list_key = None
    with open(yaml_path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("- ") and current_list_key:
                out.setdefault(current_list_key, []).append(line[2:].strip())
                continue
            current_list_key = None
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            key = k.strip()
            val = v.strip()
            if val == "":
                out[key] = []
                current_list_key = key
                continue
            # Coerce numbers when safe.
            if re.fullmatch(r"[-+]?\d+", val):
                out[key] = int(val)
            elif re.fullmatch(r"[-+]?\d*\.\d+", val):
                out[key] = float(val)
            else:
                out[key] = val
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Run Proxigo AVL pipeline on UAV-AVL guideline dataset layout.")
    parser.add_argument("--dataset-root", type=str, required=True, help="Root containing Data/ and Regions_params/")
    parser.add_argument("--region", type=str, required=True, help="Region name (e.g., QZ_Town)")
    parser.add_argument("--ref-type", type=str, default="HIGH", choices=["HIGH", "LOW"], help="Reference type from YAML")
    parser.add_argument("--place", type=str, default=None, help="Subfolder under Data/UAV_image/<region>/")
    parser.add_argument("--metadata-json", type=str, default=None, help="Optional metadata JSON override")
    parser.add_argument("--regions-yaml", type=str, default=None,
                        help="Optional override to region YAML path (used in compatibility mode).")
    parser.add_argument("--source-dir-override", type=str, default=None,
                        help="Compatibility mode: explicit UAV image directory.")
    parser.add_argument("--metadata-json-override", type=str, default=None,
                        help="Compatibility mode: explicit metadata JSON path.")
    parser.add_argument("--reference-map-override", type=str, default=None,
                        help="Compatibility mode: explicit reference map TIF path.")
    parser.add_argument("--dsm-path-override", type=str, default=None,
                        help="Compatibility mode: explicit DSM TIF path.")
    parser.add_argument("--out-dir", type=str, default="test_data/avl_guideline", help="Prepared region output root")
    parser.add_argument("--output-csv", type=str, default=None, help="Output CSV path for benchmark results")
    parser.add_argument("--topk", type=int, default=5)
    parser.add_argument("--chip-size", type=int, default=768)
    parser.add_argument("--stride", type=int, default=512)
    parser.add_argument("--retrieval-device", type=str, default="cpu")
    parser.add_argument("--map-match-every", type=int, default=5)
    parser.add_argument("--map-noise", type=float, default=30.0)
    parser.add_argument("--min-pnp-inliers", type=int, default=12)
    parser.add_argument("--ratio", type=float, default=0.85)
    parser.add_argument("--nfeatures", type=int, default=2000)
    parser.add_argument("--max-dist", type=float, default=99999.0)
    parser.add_argument("--calibration-frames", type=int, default=5)
    parser.add_argument("--max-frames", type=int, default=None, help="Optional quick-test frame cap.")
    parser.add_argument("--prior-gate-m", type=float, default=0.0,
                        help="Optional benchmark prior gate in metres.")
    parser.add_argument("--yaw-prior-rotate-ref", action="store_true",
                        help="Enable yaw-prior compensation during retrieval/matching.")
    parser.add_argument("--disable-dsm-anchor-alignment", action="store_true",
                        help="Use scale-only ref->DSM mapping, ignoring REF/DSM anchor coordinates.")
    parser.add_argument("--diagnostics-json", type=str, default=None,
                        help="Optional diagnostics JSON output path.")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    dataset_root = Path(args.dataset_root).resolve()
    yaml_path = Path(args.regions_yaml).resolve() if args.regions_yaml else (dataset_root / "Regions_params" / f"{args.region}.yaml")
    if not yaml_path.exists():
        raise FileNotFoundError(f"Region YAML not found: {yaml_path}")
    region_cfg = _load_region_yaml(yaml_path)

    place = args.place or _default_place(region_cfg, args.region)
    strict_source_dir = dataset_root / "Data" / "UAV_image" / args.region / place
    strict_metadata_json = dataset_root / "Data" / "metadata" / f"{args.region}.json"
    strict_mode = strict_source_dir.exists() and strict_metadata_json.exists()

    if strict_mode:
        source_dir = strict_source_dir
        metadata_json = Path(args.metadata_json).resolve() if args.metadata_json else strict_metadata_json
        mode = "strict-guideline-layout"
    else:
        source_dir = Path(args.source_dir_override).resolve() if args.source_dir_override else (dataset_root / "anyvis_qz" / "images_nadir")
        metadata_json = Path(args.metadata_json_override).resolve() if args.metadata_json_override else (
            Path(args.metadata_json).resolve() if args.metadata_json else (dataset_root / "anyvis_temp" / f"{args.region}.json")
        )
        mode = "compatibility-overrides"
    if not source_dir.exists():
        raise FileNotFoundError(
            f"UAV image directory not found: {source_dir}\n"
            f"Provide --source-dir-override for compatibility mode."
        )
    if not metadata_json.exists():
        raise FileNotFoundError(
            f"Metadata JSON not found: {metadata_json}\n"
            f"Provide --metadata-json-override for compatibility mode."
        )

    if args.reference_map_override:
        key_prefix = f"{args.region}_{args.ref_type}"
        region_cfg = dict(region_cfg)
        region_cfg[f"{key_prefix}_REF_PATH"] = str(Path(args.reference_map_override).resolve())
        if args.dsm_path_override:
            region_cfg[f"{key_prefix}_DSM_PATH"] = str(Path(args.dsm_path_override).resolve())
    elif args.dsm_path_override:
        key_prefix = f"{args.region}_{args.ref_type}"
        region_cfg = dict(region_cfg)
        region_cfg[f"{key_prefix}_DSM_PATH"] = str(Path(args.dsm_path_override).resolve())

    out_root = Path(args.out_dir).resolve()
    reference_dir, dsm_path, align = _prepare_region_dir(
        dataset_root=dataset_root,
        region=args.region,
        ref_type=args.ref_type,
        region_cfg=region_cfg,
        out_root=out_root,
    )

    output_csv = Path(args.output_csv).resolve() if args.output_csv else (out_root / f"results_{args.region}_{place}_{args.ref_type.lower()}.csv")
    output_csv.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        sys.executable,
        str(Path(__file__).resolve().parent / "vps_avl_benchmark.py"),
        "--source-dir", str(source_dir),
        "--reference", str(reference_dir),
        "--metadata-json", str(metadata_json),
        "--flight-filter", str(place),
        "--dsm-path", str(dsm_path),
        "--output-csv", str(output_csv),
        "--topk", str(args.topk),
        "--chip-size", str(args.chip_size),
        "--stride", str(args.stride),
        "--retrieval-device", str(args.retrieval_device),
        "--map-match-every", str(args.map_match_every),
        "--map-noise", str(args.map_noise),
        "--min-pnp-inliers", str(args.min_pnp_inliers),
        "--ratio", str(args.ratio),
        "--nfeatures", str(args.nfeatures),
        "--max-dist", str(args.max_dist),
        "--calibration-frames", str(args.calibration_frames),
        "--use-metadata-camera",
        "--prior-gate-m", str(args.prior_gate_m),
    ]
    if args.max_frames is not None:
        cmd += ["--max-frames", str(args.max_frames)]
    if args.yaw_prior_rotate_ref:
        cmd += ["--yaw-prior-rotate-ref"]
    if args.disable_dsm_anchor_alignment:
        cmd += ["--disable-dsm-anchor-alignment"]
    if args.diagnostics_json:
        cmd += ["--diagnostics-json", str(Path(args.diagnostics_json).resolve())]

    ref_coordinate = align.get("ref_coordinate")
    dsm_coordinate = align.get("dsm_coordinate")
    dsm_resolution = align.get("dsm_resolution")
    if (
        not args.disable_dsm_anchor_alignment
        and isinstance(ref_coordinate, list)
        and isinstance(dsm_coordinate, list)
        and len(ref_coordinate) >= 2
        and len(dsm_coordinate) >= 2
        and dsm_resolution is not None
    ):
        cmd += [
            "--ref-coordinate-x", str(float(ref_coordinate[0])),
            "--ref-coordinate-y", str(float(ref_coordinate[1])),
            "--dsm-coordinate-x", str(float(dsm_coordinate[0])),
            "--dsm-coordinate-y", str(float(dsm_coordinate[1])),
            "--dsm-resolution", str(float(dsm_resolution)),
        ]

    print("Mode:", mode)
    print("Region YAML:", yaml_path)
    print("Prepared reference:", reference_dir)
    print("UAV images:", source_dir)
    print("Metadata:", metadata_json)
    print("DSM:", dsm_path)
    if (
        isinstance(ref_coordinate, list)
        and isinstance(dsm_coordinate, list)
        and len(ref_coordinate) >= 2
        and len(dsm_coordinate) >= 2
    ):
        print(f"DSM alignment anchors: REF={ref_coordinate[:2]} DSM={dsm_coordinate[:2]} res={dsm_resolution}")
    print("Output CSV:", output_csv)
    print("\nCommand:")
    print(" ".join(cmd))

    if args.dry_run:
        return 0

    cp = subprocess.run(cmd, cwd=Path(__file__).resolve().parents[1])
    return int(cp.returncode)


if __name__ == "__main__":
    raise SystemExit(main())

