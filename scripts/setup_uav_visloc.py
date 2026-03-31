#!/usr/bin/env python3
"""
Extract a section from the UAV-VisLoc dataset zip and convert it
to Proxigo region format for use with vps_live.py.

Usage:
    python setup_uav_visloc.py                          # extracts section 07 (default, smallest)
    python setup_uav_visloc.py --section 05             # extracts section 05
    python setup_uav_visloc.py --zip path/to/dataset.zip --section 07
"""

import argparse
import csv
import io
import json
import sys
import zipfile
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _SCRIPT_DIR.parent
_DEFAULT_ZIP = _PROJECT_ROOT / "UAV_VisLoc_dataset.zip"
_DEFAULT_OUT = _PROJECT_ROOT / "test_data" / "uav_visloc"


def main():
    parser = argparse.ArgumentParser(
        description="Extract a UAV-VisLoc section and convert to Proxigo region format.",
    )
    parser.add_argument("--zip", type=str, default=str(_DEFAULT_ZIP),
                        help="Path to UAV_VisLoc_dataset.zip")
    parser.add_argument("--section", type=str, default="07",
                        help="Section number to extract (default: 07, the smallest)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output directory (default: test_data/uav_visloc/<section>)")
    args = parser.parse_args()

    zip_path = Path(args.zip)
    if not zip_path.exists():
        print(f"ERROR: Zip file not found: {zip_path}", file=sys.stderr)
        sys.exit(1)

    sec = args.section.zfill(2)
    out_dir = Path(args.output) if args.output else _DEFAULT_OUT / sec
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Opening {zip_path.name} ...")
    z = zipfile.ZipFile(str(zip_path), "r")

    # --- Read satellite coordinate bounds ---
    sat_csv_name = "satellite_ coordinates_range.csv"
    sat_csv_data = z.read(sat_csv_name).decode("utf-8")
    sat_bounds = {}
    for row in csv.DictReader(io.StringIO(sat_csv_data)):
        name = row["mapname"].replace(".tif", "").replace("satellite", "")
        sat_bounds[name] = {
            "north": float(row["LT_lat_map"]),
            "west": float(row["LT_lon_map"]),
            "south": float(row["RB_lat_map"]),
            "east": float(row["RB_lon_map"]),
            "region": row["region"],
        }

    # Match this section's satellite map
    sat_key = sec
    if sat_key not in sat_bounds:
        for k in sat_bounds:
            if k.startswith(sec):
                sat_key = k
                break
    if sat_key not in sat_bounds:
        print(f"ERROR: No satellite bounds found for section {sec}", file=sys.stderr)
        sys.exit(1)

    bounds = sat_bounds[sat_key]
    print(f"Section {sec}: {bounds['region']}")
    print(f"  Satellite bounds: N={bounds['north']:.6f} S={bounds['south']:.6f} "
          f"E={bounds['east']:.6f} W={bounds['west']:.6f}")

    # --- Extract and convert satellite GeoTIFF to PNG ---
    tif_candidates = [n for n in z.namelist()
                      if n.startswith(f"{sec}/satellite") and n.endswith(".tif")]
    if not tif_candidates:
        print(f"ERROR: No satellite .tif found for section {sec}", file=sys.stderr)
        sys.exit(1)

    tif_name = tif_candidates[0]
    print(f"  Extracting {tif_name} ...")
    tif_data = z.read(tif_name)

    import cv2
    import numpy as np

    tif_array = np.frombuffer(tif_data, dtype=np.uint8)
    sat_img = cv2.imdecode(tif_array, cv2.IMREAD_COLOR)
    if sat_img is None:
        # Try PIL as fallback for GeoTIFF
        try:
            from PIL import Image
            pil_img = Image.open(io.BytesIO(tif_data)).convert("RGB")
            sat_img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
        except Exception as e:
            print(f"ERROR: Cannot decode satellite image: {e}", file=sys.stderr)
            sys.exit(1)

    sat_path = out_dir / "satellite.png"
    cv2.imwrite(str(sat_path), sat_img)
    h, w = sat_img.shape[:2]
    print(f"  Saved {sat_path} ({w}x{h} px)")

    # --- Compute resolution_m from bounds and pixel dimensions ---
    import math
    center_lat = (bounds["north"] + bounds["south"]) / 2.0
    center_lon = (bounds["east"] + bounds["west"]) / 2.0
    m_per_deg_lat = 6378137.0 * math.pi / 180.0
    m_per_deg_lon = m_per_deg_lat * math.cos(math.radians(center_lat))
    height_m = abs(bounds["north"] - bounds["south"]) * m_per_deg_lat
    width_m = abs(bounds["east"] - bounds["west"]) * m_per_deg_lon
    res_y = height_m / h
    res_x = width_m / w
    resolution_m = (res_x + res_y) / 2.0

    metadata = {
        "region_id": f"uav_visloc_{sec}",
        "center": {"latitude": center_lat, "longitude": center_lon},
        "bbox": bounds,
        "imagery": {
            "file": "satellite.png",
            "width_px": w,
            "height_px": h,
            "resolution_m": resolution_m,
            "source": "UAV-VisLoc dataset",
        },
    }
    meta_path = out_dir / "metadata.json"
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"  Saved {meta_path}")
    print(f"  Resolution: {resolution_m:.3f} m/px, center: ({center_lat:.6f}, {center_lon:.6f})")

    # --- Extract drone images ---
    drone_dir = out_dir / "drone"
    drone_dir.mkdir(exist_ok=True)
    drone_entries = sorted([n for n in z.namelist()
                            if n.startswith(f"{sec}/drone/") and n.endswith(".JPG")])
    print(f"  Extracting {len(drone_entries)} drone images ...")
    for entry in drone_entries:
        fname = Path(entry).name
        img_data = z.read(entry)
        (drone_dir / fname).write_bytes(img_data)

    # --- Extract CSV ---
    csv_name = f"{sec}/{sec}.csv"
    if csv_name in z.namelist():
        csv_data = z.read(csv_name)
        csv_path = out_dir / f"{sec}.csv"
        csv_path.write_bytes(csv_data)
        print(f"  Saved {csv_path}")

    z.close()
    print()
    print(f"Done. Region ready at: {out_dir}")
    print()
    print("Run VPS on it:")
    print(f"  python scripts/vps_live.py \\")
    print(f"      --source-dir {out_dir / 'drone'} \\")
    print(f"      --source-csv {out_dir / f'{sec}.csv'} \\")
    print(f"      --reference {out_dir} \\")
    print(f"      --altitude 689 --output-csv results.csv")


if __name__ == "__main__":
    main()
