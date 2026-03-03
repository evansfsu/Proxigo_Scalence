"""
Simulation-only helpers: template matching to localize a query crop in the reference.
Used when feature-based estimate() fails but we know the query is a crop of the ref (e.g. yaw=0).
"""

from typing import Optional, Tuple

import cv2
import numpy as np

from .config import VPSDeviceConfig
from .geo_transform import ref_pixel_to_geo
from .reference_loader import ReferenceImage


def localize_crop_in_ref(
    query_bgr: np.ndarray,
    ref_grayscale: np.ndarray,
    ref: ReferenceImage,
    config: Optional[VPSDeviceConfig] = None,
) -> Optional[Tuple[float, float]]:
    """
    Find where the query crop sits in the reference using normalized cross-correlation.
    Assumes query is an exact (or near-exact) crop of the ref; valid for yaw=0 simulation.

    Returns (lat, lon) of the center of the crop in ref (camera position), or None if
    query is not smaller than ref or matching fails.
    """
    if query_bgr.ndim == 3:
        query_gray = cv2.cvtColor(query_bgr, cv2.COLOR_BGR2GRAY)
    else:
        query_gray = np.asarray(query_bgr, dtype=np.uint8)

    h_ref, w_ref = ref_grayscale.shape[:2]
    h_q, w_q = query_gray.shape[:2]
    if h_q >= h_ref or w_q >= w_ref:
        return None

    # matchTemplate(image, templ, method): search for templ in image
    result = cv2.matchTemplate(ref_grayscale, query_gray, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    # CCOEFF_NORMED: best match is max
    bx, by = max_loc
    # Reject very weak matches (e.g. noise)
    if max_val < 0.3:
        return None

    # Subpixel refinement: parabolic fit in 3x3 neighborhood to reduce pixel quantization error
    h_r, w_r = result.shape[:2]
    bx_sub, by_sub = float(bx), float(by)
    if 1 <= bx < w_r - 1 and 1 <= by < h_r - 1:
        rx = result[by, bx - 1]
        cx = result[by, bx]
        rxp = result[by, bx + 1]
        dx = 0.5 * (rxp - rx) / (2 * cx - rx - rxp) if (2 * cx - rx - rxp) != 0 else 0
        dx = max(-1, min(1, dx))
        bx_sub = bx + dx
        ry = result[by - 1, bx]
        cy = result[by, bx]
        ryp = result[by + 1, bx]
        dy = 0.5 * (ryp - ry) / (2 * cy - ry - ryp) if (2 * cy - ry - ryp) != 0 else 0
        dy = max(-1, min(1, dy))
        by_sub = by + dy

    # Center of the crop in ref pixels (subpixel)
    center_x = bx_sub + w_q / 2.0
    center_y = by_sub + h_q / 2.0

    lat, lon = ref_pixel_to_geo(
        center_x,
        center_y,
        ref.center_lat,
        ref.center_lon,
        ref.height_px,
        ref.width_px,
        ref.y_m_per_px,
        ref.x_m_per_px,
    )
    return (lat, lon)
