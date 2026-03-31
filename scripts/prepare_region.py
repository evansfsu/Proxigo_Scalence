#!/usr/bin/env python3
"""
Download satellite imagery for a geographic area and save it as a Proxigo
region directory (satellite.png + metadata.json).

Usage:
    python prepare_region.py --lat 36.23 --lon -116.81 --radius-m 500 \
                             --output-dir satellite_data/regions/death_valley

    python prepare_region.py --north 36.235 --south 36.225 --east -116.805 --west -116.815 \
                             --output-dir satellite_data/regions/death_valley --zoom 18
"""

import argparse
import json
import math
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Reuse the tile-fetching logic from vps_sim_web/fetch_region.py
# ---------------------------------------------------------------------------
_SCRIPT_DIR = Path(__file__).resolve().parent
_FETCH_MODULE = _SCRIPT_DIR / "vps_sim_web" / "fetch_region.py"


def _import_fetch_region():
    """Import fetch_region module from sibling directory."""
    import importlib.util
    if not _FETCH_MODULE.exists():
        print(f"ERROR: {_FETCH_MODULE} not found.", file=sys.stderr)
        sys.exit(1)
    spec = importlib.util.spec_from_file_location("fetch_region", str(_FETCH_MODULE))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _radius_to_bbox(lat: float, lon: float, radius_m: float):
    """Convert center + radius (metres) to (north, south, east, west)."""
    m_per_deg_lat = 6378137.0 * math.pi / 180.0
    m_per_deg_lon = m_per_deg_lat * math.cos(math.radians(lat))
    d_lat = radius_m / m_per_deg_lat
    d_lon = radius_m / m_per_deg_lon
    return lat + d_lat, lat - d_lat, lon + d_lon, lon - d_lon


def main():
    parser = argparse.ArgumentParser(
        description="Download satellite imagery and save as a Proxigo region.",
    )

    group = parser.add_argument_group("Center + radius (simpler)")
    group.add_argument("--lat", type=float, help="Center latitude")
    group.add_argument("--lon", type=float, help="Center longitude")
    group.add_argument("--radius-m", type=float, default=500.0,
                       help="Radius in metres (default 500)")

    bbox_group = parser.add_argument_group("Bounding box (alternative)")
    bbox_group.add_argument("--north", type=float, help="North latitude")
    bbox_group.add_argument("--south", type=float, help="South latitude")
    bbox_group.add_argument("--east", type=float, help="East longitude")
    bbox_group.add_argument("--west", type=float, help="West longitude")

    parser.add_argument("--output-dir", required=True,
                        help="Output directory for the region")
    parser.add_argument("--zoom", type=int, default=18,
                        help="Tile zoom level (default 18)")
    parser.add_argument("--region-id", type=str, default=None,
                        help="Region ID (defaults to directory name)")

    args = parser.parse_args()

    # Determine bbox
    if args.north is not None and args.south is not None:
        north, south, east, west = args.north, args.south, args.east, args.west
    elif args.lat is not None and args.lon is not None:
        north, south, east, west = _radius_to_bbox(args.lat, args.lon, args.radius_m)
    else:
        parser.error("Provide either --lat/--lon/--radius-m or --north/--south/--east/--west")
        return

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    region_id = args.region_id or out_dir.name

    print(f"Region: {region_id}")
    print(f"Bbox: N={north:.6f}  S={south:.6f}  E={east:.6f}  W={west:.6f}")
    print(f"Zoom: {args.zoom}")
    print(f"Output: {out_dir}")
    print()

    fetch_region = _import_fetch_region()

    print("Fetching satellite tiles ...")
    img_rgb, center_lat, center_lon, resolution_m = fetch_region.fetch_imagery_bbox(
        north, south, east, west, zoom=args.zoom,
    )

    if img_rgb is None:
        print("ERROR: Failed to fetch imagery (PIL/numpy not available).", file=sys.stderr)
        sys.exit(1)

    import cv2
    import numpy as np

    img_path = out_dir / "satellite.png"
    bgr = cv2.cvtColor(np.array(img_rgb), cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(img_path), bgr)
    h, w = bgr.shape[:2]

    metadata = {
        "region_id": region_id,
        "center": {
            "latitude": center_lat,
            "longitude": center_lon,
        },
        "bbox": {
            "north": north,
            "south": south,
            "east": east,
            "west": west,
        },
        "imagery": {
            "file": "satellite.png",
            "width_px": w,
            "height_px": h,
            "resolution_m": resolution_m,
            "zoom": args.zoom,
            "source": "Esri World Imagery",
        },
    }

    meta_path = out_dir / "metadata.json"
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=2)

    print()
    print(f"Saved {img_path}  ({w}x{h} px, {resolution_m:.3f} m/px)")
    print(f"Saved {meta_path}")
    print("Done.")


if __name__ == "__main__":
    main()
