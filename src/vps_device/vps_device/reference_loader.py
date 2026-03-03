"""
Load reference imagery and metadata (Berkeley-style or Proxigo-style).
Returns a common structure for the estimator.
"""

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np


def _load_image_grayscale(path: Path) -> np.ndarray:
    img = cv2.imread(str(path))
    if img is None:
        raise FileNotFoundError(f"Could not load image: {path}")
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


class ReferenceImage:
    """Single reference image with geo metadata and optional precomputed features."""

    def __init__(
        self,
        image: np.ndarray,
        center_lat: float,
        center_lon: float,
        y_m_per_px: float,
        x_m_per_px: float,
        image_id: Optional[str] = None,
    ):
        self.image = image  # grayscale
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.y_m_per_px = y_m_per_px
        self.x_m_per_px = x_m_per_px
        self.image_id = image_id or "ref"
        self.height_px, self.width_px = image.shape[:2]

    @property
    def shape(self) -> Tuple[int, int]:
        return self.image.shape[:2]


def load_proxigo_region(
    region_path: Path,
    satellite_data_root: Optional[Path] = None,
) -> ReferenceImage:
    """
    Load a Proxigo-style region: region_path is .../regions/<region_id>/ or
    satellite_data_root/regions/<region_id>/.
    """
    if not region_path.is_dir():
        raise NotADirectoryError(f"Region path is not a directory: {region_path}")

    metadata_path = region_path / "metadata.json"
    if not metadata_path.exists():
        raise FileNotFoundError(f"metadata.json not found in {region_path}")

    with open(metadata_path, "r") as f:
        meta = json.load(f)

    center = meta.get("center", {})
    center_lat = center.get("latitude", 0.0)
    center_lon = center.get("longitude", 0.0)

    imagery = meta.get("imagery", {})
    resolution_m = float(imagery.get("resolution_m", 0.5))
    img_file = imagery.get("file", "satellite.png")
    img_path = region_path / img_file

    if not img_path.exists():
        raise FileNotFoundError(f"Reference image not found: {img_path}")

    image = _load_image_grayscale(img_path)
    h, w = image.shape[:2]
    # Proxigo: uniform resolution_m so same m/px in x and y
    return ReferenceImage(
        image=image,
        center_lat=center_lat,
        center_lon=center_lon,
        y_m_per_px=resolution_m,
        x_m_per_px=resolution_m,
        image_id=meta.get("region_id", "proxigo"),
    )


def load_berkeley_reference(
    path: str,
    center_lat: float,
    center_lon: float,
    height_m: float,
    width_m: float,
    image_id: Optional[str] = None,
) -> ReferenceImage:
    """
    Load a Berkeley-style reference image. Physical size of the image in metres
    (height_m x width_m) gives metres per pixel.
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Reference image not found: {p}")

    image = _load_image_grayscale(p)
    h, w = image.shape[:2]
    y_m_per_px = height_m / h
    x_m_per_px = width_m / w

    return ReferenceImage(
        image=image,
        center_lat=center_lat,
        center_lon=center_lon,
        y_m_per_px=y_m_per_px,
        x_m_per_px=x_m_per_px,
        image_id=image_id or p.stem,
    )


def load_berkeley_dict(
    ref_dict: Dict[int, Tuple[str, float, float, float, float]],
    base_path: Optional[Path] = None,
) -> List[ReferenceImage]:
    """
    Load multiple Berkeley-style references. ref_dict maps index -> (path, lat, lon, height_m, width_m).
    If path is relative, base_path is prepended.
    """
    refs = []
    for idx, tup in ref_dict.items():
        path, lat, lon, height_m, width_m = tup
        p = Path(path)
        if base_path and not p.is_absolute():
            p = base_path / p
        refs.append(
            load_berkeley_reference(
                str(p), lat, lon, height_m, width_m, image_id=str(idx)
            )
        )
    return refs
