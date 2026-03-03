#!/usr/bin/env python3
"""
Run one synthetic VPS test and show the visualization on your laptop.
Saves images to src/vps_device/test/output/ and optionally opens them in a window.
Usage: python src/vps_device/scripts/view_test_viz.py [--show]
  --show  open OpenCV windows (press any key to close)
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np

from vps_device.reference_loader import ReferenceImage
from vps_device.estimator import VPSEstimator
from vps_device.config import VPSDeviceConfig


def _make_synthetic_ref_image(width: int = 800, height: int = 600) -> np.ndarray:
    rng = np.random.default_rng(42)
    img = (rng.integers(0, 256, (height, width), dtype=np.uint8) * 0.3).astype(np.uint8)
    for _ in range(20):
        x, y = rng.integers(50, width - 50), rng.integers(50, height - 50)
        cv2.rectangle(img, (x, y), (x + 40, y + 40), 200, 2)
    return img


def main():
    parser = argparse.ArgumentParser(description="Run one VPS synthetic test and view visualization.")
    parser.add_argument("--show", action="store_true", help="Open visualization windows (press key to close)")
    args = parser.parse_args()

    out_dir = Path(__file__).resolve().parents[1] / "test" / "output"
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"Saving images to: {out_dir}")

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
        ref_img[margin_y : margin_y + 300, margin_x : margin_x + 400], cv2.COLOR_GRAY2BGR
    )
    result = estimator.estimate(query_bgr, 50.0, last_lat_lon=None)

    overlay = query_bgr.copy()
    cv2.putText(overlay, f"lat={result.lat:.6f} lon={result.lon:.6f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(overlay, f"conf={result.confidence:.2f} matches={result.n_matches}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    overlay_path = out_dir / "view_overlay.png"
    cv2.imwrite(str(overlay_path), overlay)
    print(f"  {overlay_path}")

    debug_img = estimator.get_debug_image(query_bgr)
    if debug_img is not None:
        debug_path = out_dir / "view_ref_vs_query_matches.png"
        cv2.imwrite(str(debug_path), debug_img)
        print(f"  {debug_path}")

    if args.show:
        cv2.imshow("VPS test: overlay", overlay)
        if debug_img is not None:
            cv2.imshow("VPS test: ref vs query (matches)", debug_img)
        print("Press any key in a window to close.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
