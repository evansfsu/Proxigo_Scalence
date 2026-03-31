"""
Dataset adapters for benchmark-style evaluation scripts.

Goal: normalize various dataset metadata formats to a common per-frame schema.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional, Sequence


@dataclass(frozen=True)
class FrameRecord:
    path: str
    filename: str
    lat: float
    lon: float
    altitude_m: float
    yaw_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    roll_deg: Optional[float] = None
    width_px: Optional[int] = None
    height_px: Optional[int] = None
    fov_deg: Optional[float] = None  # diagonal FOV if provided by dataset
    fx: Optional[float] = None
    fy: Optional[float] = None
    cx: Optional[float] = None
    cy: Optional[float] = None


def load_uav_avl_metadata_json(path: Path) -> List[FrameRecord]:
    """
    Load UAV-AVL / AnyVisLoc-style metadata JSON.

    Expected JSON: list of dicts with keys like:
      - name (image path), lat, lon, rel_alt
      - yaw, pitch, roll
      - width, height, fov
      - (optional) focal_len / cam_size; not converted here
    """
    with open(path, "r") as f:
        data = json.load(f)

    if not isinstance(data, list):
        raise ValueError(f"Expected list in {path}, got {type(data)}")

    records: List[FrameRecord] = []
    for row in data:
        name = str(row.get("name", ""))
        filename = Path(name).name if name else ""
        if not filename:
            continue
        records.append(
            FrameRecord(
                path=name,
                filename=filename,
                lat=float(row["lat"]),
                lon=float(row["lon"]),
                altitude_m=float(row.get("rel_alt", row.get("height", 0.0))),
                yaw_deg=float(row["yaw"]) if "yaw" in row and row["yaw"] is not None else None,
                pitch_deg=float(row["pitch"]) if "pitch" in row and row["pitch"] is not None else None,
                roll_deg=float(row["roll"]) if "roll" in row and row["roll"] is not None else None,
                width_px=int(float(row["width"])) if "width" in row and row["width"] is not None else None,
                height_px=int(float(row["height"])) if "height" in row and row["height"] is not None else None,
                fov_deg=float(row["fov"]) if "fov" in row and row["fov"] is not None else None,
            )
        )

    # Sort by filename for sequential runs (DJI_0001.JPG ...).
    records.sort(key=lambda r: r.filename)
    return records


def index_by_filename(records: Sequence[FrameRecord]) -> Dict[str, FrameRecord]:
    return {r.filename: r for r in records}


def index_by_path(records: Sequence[FrameRecord]) -> Dict[str, FrameRecord]:
    """
    Index by the full metadata 'name' path string.
    Use this when filenames collide across flights (common in UAV-AVL/AnyVisLoc).
    """
    return {r.path: r for r in records}

