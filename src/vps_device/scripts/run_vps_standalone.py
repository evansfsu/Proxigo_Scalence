#!/usr/bin/env python3
"""
VPS Device standalone runner: video / RTSP / image directory + altitude -> position estimates.
No ROS2 required. For testing the device pipeline without SITL/PX4.
"""

import argparse
import csv
import sys
from pathlib import Path

# Add parent so vps_device can be imported when run as script
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2

from vps_device.config import VPSDeviceConfig
from vps_device.estimator import VPSEstimator, create_estimator_from_proxigo_region, EstimateResult
from vps_device.reference_loader import load_berkeley_reference


def _altitude_for_frame(altitude_m: float, altitude_file: Path, frame_index: int) -> float:
    """Return altitude: constant or from CSV (timestamp, alt)."""
    if altitude_file is None:
        return altitude_m
    with open(altitude_file) as f:
        reader = csv.reader(f)
        rows = list(reader)
    if not rows:
        return altitude_m
    # Simple: row index = frame index, second column = altitude
    if frame_index < len(rows):
        try:
            return float(rows[frame_index][1] if len(rows[frame_index]) > 1 else rows[frame_index][0])
        except (IndexError, ValueError):
            pass
    return float(rows[-1][1]) if len(rows[-1]) > 1 else float(rows[-1][0])


def run_video(
    cap: cv2.VideoCapture,
    estimator: VPSEstimator,
    altitude_m: float,
    altitude_file: Path,
    frame_skip: int,
    output_path: Path,
    verbose: bool,
    visualize: bool = False,
) -> None:
    frame_index = 0
    last_lat_lon = None
    out_file = None
    if output_path:
        out_file = open(output_path, "w", newline="")
        writer = csv.writer(out_file)
        writer.writerow(["frame", "lat", "lon", "confidence", "n_matches", "success"])

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            if frame_skip and frame_index % (frame_skip + 1) != 0:
                frame_index += 1
                continue
            alt = _altitude_for_frame(altitude_m, altitude_file, frame_index)
            result = estimator.estimate(frame, alt, last_lat_lon)
            if result.success:
                last_lat_lon = (result.lat, result.lon)
            if verbose:
                print(f"frame={frame_index} lat={result.lat:.6f} lon={result.lon:.6f} "
                      f"conf={result.confidence:.3f} n={result.n_matches} ok={result.success}")
            if out_file:
                writer.writerow([frame_index, result.lat, result.lon, result.confidence, result.n_matches, result.success])

            if visualize:
                # Overlay position and status on current frame
                vis_frame = frame.copy()
                cv2.putText(vis_frame, f"lat={result.lat:.6f} lon={result.lon:.6f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(vis_frame, f"conf={result.confidence:.2f} matches={result.n_matches}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(vis_frame, "OK" if result.success else "NO MATCH",
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if result.success else (0, 0, 255), 2)
                debug_img = estimator.get_debug_image(frame)
                if debug_img is not None:
                    # Show match visualization in second window
                    cv2.imshow("VPS: Ref vs Query (matches)", debug_img)
                cv2.imshow("VPS: Live", vis_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            frame_index += 1
    finally:
        if out_file:
            out_file.close()
        if visualize:
            cv2.destroyAllWindows()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run VPS device on video/RTSP/images + altitude (no ROS2)."
    )
    parser.add_argument("--video", type=str, help="Video file path")
    parser.add_argument("--rtsp", type=str, help="RTSP URL (e.g. rtsp://host:8554/stream)")
    parser.add_argument("--images", type=str, help="Directory of images (sorted by name)")
    parser.add_argument("--reference", type=str, default="",
                        help="Proxigo region dir (with metadata.json); required unless --berkeley is used")
    parser.add_argument("--berkeley", type=str, nargs=5, metavar=("LAT", "LON", "HEIGHT_M", "WIDTH_M", "PATH"),
                        help="Berkeley-style: center_lat center_lon height_m width_m image_path")
    parser.add_argument("--altitude", type=float, default=30.0, help="Constant altitude (m AGL)")
    parser.add_argument("--altitude-file", type=str, help="CSV: frame index or timestamp, altitude_m")
    parser.add_argument("--output", "-o", type=str, help="Output CSV path")
    parser.add_argument("--frame-skip", type=int, default=0, help="Process every (1+frame_skip)-th frame")
    parser.add_argument("--fov-h", type=float, default=71.5, help="Horizontal FOV (deg)")
    parser.add_argument("--fov-d", type=float, default=79.5, help="Diagonal FOV (deg)")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("--visualize", action="store_true",
                        help="Show live visualization: match overlay and position on frame (press 'q' to quit)")
    args = parser.parse_args()

    if not args.video and not args.rtsp and not args.images:
        parser.error("One of --video, --rtsp, or --images is required")

    config = VPSDeviceConfig(
        h_fov_deg=args.fov_h,
        d_fov_deg=args.fov_d,
        width_px=args.width,
        height_px=args.height,
    )

    # Build estimator
    if args.berkeley:
        lat, lon, h_m, w_m, path = args.berkeley
        ref = load_berkeley_reference(
            path, float(lat), float(lon), float(h_m), float(w_m)
        )
        estimator = VPSEstimator(ref, config)
    else:
        if not args.reference:
            parser.error("--reference or --berkeley is required")
        ref_path = Path(args.reference)
        if not ref_path.exists():
            print(f"Reference path does not exist: {ref_path}", file=sys.stderr)
            return 1
        estimator = create_estimator_from_proxigo_region(ref_path, config)

    altitude_file = Path(args.altitude_file) if args.altitude_file else None
    output_path = Path(args.output) if args.output else None

    if args.video:
        cap = cv2.VideoCapture(args.video)
    elif args.rtsp:
        cap = cv2.VideoCapture(args.rtsp)
    elif args.images:
        im_dir = Path(args.images)
        images = sorted(im_dir.glob("*"))[:]
        images = [p for p in images if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp")]
        if not images:
            print(f"No images in {im_dir}", file=sys.stderr)
            return 1
        cap = None
        last_lat_lon = None
        out_file = None
        if output_path:
            out_file = open(output_path, "w", newline="")
            writer = csv.writer(out_file)
            writer.writerow(["frame", "lat", "lon", "confidence", "n_matches", "success"])
        for i, img_path in enumerate(images):
            frame = cv2.imread(str(img_path))
            if frame is None:
                continue
            alt = _altitude_for_frame(args.altitude, altitude_file, i)
            result = estimator.estimate(frame, alt, last_lat_lon)
            if result.success:
                last_lat_lon = (result.lat, result.lon)
            if args.verbose:
                print(f"frame={i} lat={result.lat:.6f} lon={result.lon:.6f} "
                      f"conf={result.confidence:.3f} n={result.n_matches} ok={result.success}")
            if out_file:
                writer.writerow([i, result.lat, result.lon, result.confidence, result.n_matches, result.success])
            if args.visualize:
                vis_frame = frame.copy()
                cv2.putText(vis_frame, f"lat={result.lat:.6f} lon={result.lon:.6f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(vis_frame, f"conf={result.confidence:.2f} matches={result.n_matches}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(vis_frame, "OK" if result.success else "NO MATCH",
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if result.success else (0, 0, 255), 2)
                debug_img = estimator.get_debug_image(frame)
                if debug_img is not None:
                    cv2.imshow("VPS: Ref vs Query (matches)", debug_img)
                cv2.imshow("VPS: Live", vis_frame)
                key = cv2.waitKey(0) & 0xFF
                if key == ord("q"):
                    break
        if args.visualize:
            cv2.destroyAllWindows()
        if out_file:
            out_file.close()
        return 0

    if not cap.isOpened():
        print("Failed to open video source", file=sys.stderr)
        return 1
    run_video(
        cap, estimator, args.altitude, altitude_file,
        args.frame_skip, output_path, args.verbose,
        visualize=args.visualize,
    )
    cap.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
