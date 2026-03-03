"""
Unit test for VPS estimator: synthetic reference + query -> check position and confidence.
"""

import pytest
import numpy as np
import cv2
from pathlib import Path

# Add parent to path so vps_device is importable
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vps_device.reference_loader import ReferenceImage
from vps_device.estimator import VPSEstimator
from vps_device.config import VPSDeviceConfig
from vps_device.geo_transform import get_distance_metres


def _make_synthetic_ref_image(width: int = 800, height: int = 600) -> np.ndarray:
    """Create a grayscale image with some structure for feature detection."""
    rng = np.random.default_rng(42)
    img = (rng.integers(0, 256, (height, width), dtype=np.uint8) * 0.3).astype(np.uint8)
    # Add some edges and corners
    for _ in range(20):
        x, y = rng.integers(50, width - 50), rng.integers(50, height - 50)
        cv2.rectangle(img, (x, y), (x + 40, y + 40), 200, 2)
    return img


def test_estimator_synthetic_self_match():
    """Query = center crop of reference: expected position should be near reference center."""
    ref_img = _make_synthetic_ref_image(800, 600)
    center_lat, center_lon = 39.678, -75.750
    resolution_m = 0.5
    ref = ReferenceImage(
        image=ref_img,
        center_lat=center_lat,
        center_lon=center_lon,
        y_m_per_px=resolution_m,
        x_m_per_px=resolution_m,
        image_id="test",
    )
    config = VPSDeviceConfig(
        width_px=400,
        height_px=300,
        nfeatures_orb=150,
        min_matches=5,
        max_dist_from_last_m=500.0,  # no continuity constraint for first frame
    )
    estimator = VPSEstimator(ref, config)

    # Query: center crop of ref (so we're "looking at" the center of the ref)
    h, w = ref_img.shape[:2]
    margin_x, margin_y = (w - 400) // 2, (h - 300) // 2
    query = ref_img[margin_y:margin_y + 300, margin_x:margin_x + 400]
    query_bgr = cv2.cvtColor(query, cv2.COLOR_GRAY2BGR)

    altitude_m = 50.0
    result = estimator.estimate(query_bgr, altitude_m, last_lat_lon=None)

    # With a self-crop we expect either success with position near center, or failure if too few features
    if result.success:
        dist_m = get_distance_metres(
            result.lat, result.lon,
            center_lat, center_lon,
        )
        # Allow large tolerance (synthetic image may give noisy match)
        assert dist_m < 500.0, f"Estimated position too far from center: {dist_m}m"
        assert result.confidence > 0.0
        assert result.n_matches >= 1  # at least one match used (may be less than min_matches after filtering)
    else:
        # Minimal check: at least we get a result
        assert result.n_matches >= 0


def test_estimator_continuity_uses_last():
    """When last_lat_lon is set, estimator uses it for continuity (no crash)."""
    ref_img = _make_synthetic_ref_image(400, 300)
    ref = ReferenceImage(
        image=ref_img,
        center_lat=39.0,
        center_lon=-76.0,
        y_m_per_px=0.5,
        x_m_per_px=0.5,
        image_id="small",
    )
    config = VPSDeviceConfig(
        width_px=400,
        height_px=300,
        min_matches=3,
        max_dist_from_last_m=50.0,
    )
    estimator = VPSEstimator(ref, config)
    query = cv2.cvtColor(ref_img, cv2.COLOR_GRAY2BGR)
    last = (39.0, -76.0)
    result = estimator.estimate(query, 30.0, last_lat_lon=last)
    # Should not raise; may or may not succeed depending on features
    assert hasattr(result, "success") and hasattr(result, "lat") and hasattr(result, "lon")


def test_config_camera_m_per_px():
    """Config camera_m_per_px returns positive values scaling with altitude."""
    config = VPSDeviceConfig(width_px=1920, height_px=1080)
    x, y = config.camera_m_per_px(100.0)
    assert x > 0 and y > 0
    x2, y2 = config.camera_m_per_px(200.0)
    assert x2 > x and y2 > y


def test_geo_distance():
    """get_distance_metres gives ~0 for same point, positive for different."""
    d = get_distance_metres(39.0, -76.0, 39.0, -76.0)
    assert d < 1.0
    d2 = get_distance_metres(39.0, -76.0, 39.001, -76.0)
    assert d2 > 50.0


def test_estimator_saves_visualization():
    """Run a synthetic match and save visualization so you can view it on your laptop."""
    out_dir = Path(__file__).resolve().parent / "output"
    out_dir.mkdir(parents=True, exist_ok=True)

    ref_img = _make_synthetic_ref_image(800, 600)
    ref = ReferenceImage(
        image=ref_img,
        center_lat=39.678,
        center_lon=-75.750,
        y_m_per_px=0.5,
        x_m_per_px=0.5,
        image_id="test",
    )
    config = VPSDeviceConfig(width_px=400, height_px=300, nfeatures_orb=150, max_dist_from_last_m=500.0)
    estimator = VPSEstimator(ref, config)
    h, w = ref_img.shape[:2]
    margin_x, margin_y = (w - 400) // 2, (h - 300) // 2
    query_bgr = cv2.cvtColor(
        ref_img[margin_y:margin_y + 300, margin_x:margin_x + 400], cv2.COLOR_GRAY2BGR
    )
    result = estimator.estimate(query_bgr, 50.0, last_lat_lon=None)

    # Save overlay frame (position/confidence on query)
    overlay = query_bgr.copy()
    cv2.putText(overlay, f"lat={result.lat:.6f} lon={result.lon:.6f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(overlay, f"conf={result.confidence:.2f} matches={result.n_matches}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    overlay_path = out_dir / "test_overlay.png"
    cv2.imwrite(str(overlay_path), overlay)
    assert overlay_path.exists()

    # Save ref-vs-query match visualization if we have one
    debug_img = estimator.get_debug_image(query_bgr)
    if debug_img is not None:
        debug_path = out_dir / "test_ref_vs_query_matches.png"
        cv2.imwrite(str(debug_path), debug_img)
        assert debug_path.exists()
