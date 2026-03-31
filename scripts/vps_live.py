#!/usr/bin/env python3
"""
Standalone VPS live runner -- no ROS2 required.

Reads frames from a camera, video file, or image directory, runs
VPSEstimator against a pre-loaded satellite reference, and displays
real-time feature matching, position estimates, and a position trail.

Usage:
    # Live camera on Orin Nano
    python vps_live.py --source /dev/video0 \
                       --reference satellite_data/regions/my_area \
                       --altitude 50

    # Pre-recorded video
    python vps_live.py --source flight.mp4 \
                       --reference satellite_data/regions/my_area \
                       --altitude 50 --output-csv results.csv

    # UAV-VisLoc image sequence with ground truth
    python vps_live.py --source-dir test_data/uav_visloc/07/drone \
                       --source-csv test_data/uav_visloc/07/07.csv \
                       --reference test_data/uav_visloc/07 \
                       --altitude 689 --output-csv results.csv

    # Headless (no display, just CSV)
    python vps_live.py --source flight.mp4 \
                       --reference satellite_data/regions/my_area \
                       --altitude 50 --headless --output-csv results.csv
"""

import argparse
import csv as csv_mod
import hashlib
import io
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Import vps_device library from src/
# ---------------------------------------------------------------------------
_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _SCRIPT_DIR.parent
_VPS_DEVICE_PKG = _PROJECT_ROOT / "src" / "vps_device"

if str(_VPS_DEVICE_PKG) not in sys.path:
    sys.path.insert(0, str(_VPS_DEVICE_PKG))

from vps_device.config import VPSDeviceConfig
from vps_device.ekf import VPSFusionEKF, EKFConfig
from vps_device.estimator import EstimateResult, VPSEstimator, create_estimator_from_proxigo_region
from vps_device.features import (
    _get_descriptor_extractor,
    build_flann_matcher,
    extract_features,
    match_with_ratio_test,
)
from vps_device.geo_transform import get_distance_metres
from vps_device.pnp import camera_matrix_from_fov, planar_pose_fix_from_matches, pnp_fix_with_dsm
from vps_device.visual_odometry import VisualOdometry, HeadingCalibrator


# ---------------------------------------------------------------------------
# Ground-truth CSV loader
# ---------------------------------------------------------------------------

def _load_ground_truth_csv(csv_path: Path) -> dict:
    """Load ground-truth CSV. Returns {filename: {lat, lon, height, yaw?}}.

    Columns: filename, lat, lon, height[, yaw]
    The yaw column (degrees, clockwise from north) is optional.
    """
    gt = {}
    with open(csv_path, "r") as f:
        reader = csv_mod.DictReader(f)
        has_yaw = "yaw" in (reader.fieldnames or [])
        for row in reader:
            fname = row["filename"].strip()
            entry = {
                "lat": float(row["lat"]),
                "lon": float(row["lon"]),
                "height": float(row["height"]),
            }
            if has_yaw and row.get("yaw"):
                entry["yaw"] = float(row["yaw"])
            gt[fname] = entry
    return gt


# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------

def _draw_keypoints_on_frame(frame: np.ndarray, config: VPSDeviceConfig) -> np.ndarray:
    """Draw detected keypoints on a copy of the frame."""
    vis = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    detector, desc_ext = _get_descriptor_extractor(
        config.use_beblid, getattr(config, 'use_sift', False), config.nfeatures_orb,
    )
    if hasattr(detector, 'setMaxFeatures'):
        detector.setMaxFeatures(config.nfeatures_orb)
    kpts = detector.detect(gray, None)
    cv2.drawKeypoints(vis, kpts, vis, color=(0, 255, 0),
                      flags=cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)
    return vis


def _build_minimap(ref_gray: np.ndarray, trail: list,
                   truth_trail: list = None, minimap_h: int = 200) -> np.ndarray:
    """Build a small reference image with position trail dots."""
    h, w = ref_gray.shape[:2]
    scale = minimap_h / h
    minimap_w = int(w * scale)
    mini = cv2.resize(ref_gray, (minimap_w, minimap_h))
    mini_bgr = cv2.cvtColor(mini, cv2.COLOR_GRAY2BGR)

    # Draw ground-truth trail in blue if available
    if truth_trail and len(truth_trail) >= 2:
        for i in range(1, len(truth_trail)):
            pt1 = (int(truth_trail[i - 1][0] * scale), int(truth_trail[i - 1][1] * scale))
            pt2 = (int(truth_trail[i][0] * scale), int(truth_trail[i][1] * scale))
            cv2.line(mini_bgr, pt1, pt2, (255, 180, 0), 1, cv2.LINE_AA)

    # Draw estimate trail in orange/red
    if len(trail) >= 2:
        for i in range(1, len(trail)):
            pt1 = (int(trail[i - 1][0] * scale), int(trail[i - 1][1] * scale))
            pt2 = (int(trail[i][0] * scale), int(trail[i][1] * scale))
            cv2.line(mini_bgr, pt1, pt2, (0, 200, 255), 1, cv2.LINE_AA)

    for px, py in trail:
        cx = int(px * scale)
        cy = int(py * scale)
        cv2.circle(mini_bgr, (cx, cy), 3, (0, 0, 255), -1)

    if trail:
        last = trail[-1]
        cx = int(last[0] * scale)
        cy = int(last[1] * scale)
        cv2.circle(mini_bgr, (cx, cy), 5, (0, 255, 0), 2)

    return mini_bgr


def _build_stats_bar(width: int, fused_pos, speed, vo_quality,
                     map_result, fps: float, frame_idx: int,
                     error_m: float = None) -> np.ndarray:
    """Build a stats bar image showing EKF fused state."""
    bar_h = 40
    bar = np.zeros((bar_h, width, 3), dtype=np.uint8)
    bar[:] = (40, 40, 40)

    if fused_pos is not None:
        txt = (f"Fused: ({fused_pos[0]:.6f}, {fused_pos[1]:.6f})   "
               f"Speed: {speed:.1f}m/s   VO: {vo_quality:.0%}")
        if map_result and map_result.success:
            txt += f"   MAP:{map_result.n_matches}m"
        if error_m is not None:
            txt += f"   Err: {error_m:.1f}m"
        txt += f"   FPS: {fps:.1f}   #{frame_idx}"
        color = (0, 255, 0)
    else:
        txt = f"Waiting for map match...   FPS: {fps:.1f}   #{frame_idx}"
        color = (200, 200, 200)

    cv2.putText(bar, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return bar


def _resize_to_height(img: np.ndarray, target_h: int) -> np.ndarray:
    h, w = img.shape[:2]
    if h == target_h:
        return img
    scale = target_h / h
    return cv2.resize(img, (int(w * scale), target_h))


# ---------------------------------------------------------------------------
# Geo -> ref pixel (for minimap trail)
# ---------------------------------------------------------------------------

def _geo_to_ref_pixel(lat, lon, ref):
    """Convert (lat, lon) to reference image pixel coordinates."""
    m_per_deg_lat = 6378137.0 * math.pi / 180.0
    m_per_deg_lon = m_per_deg_lat * math.cos(math.radians(ref.center_lat))
    north_m = (lat - ref.center_lat) * m_per_deg_lat
    east_m = (lon - ref.center_lon) * m_per_deg_lon
    dx_px = east_m / ref.x_m_per_px
    dy_px = -north_m / ref.y_m_per_px
    px = ref.width_px / 2 + dx_px
    py = ref.height_px / 2 + dy_px
    return px, py


# ---------------------------------------------------------------------------
# Frame sources
# ---------------------------------------------------------------------------

def _iter_video(source_str: str):
    """Yield (frame_bgr, filename_or_None) from a video file or camera."""
    source = source_str
    if source.isdigit():
        source = int(source)
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"ERROR: Cannot open video source: {source_str}", file=sys.stderr)
        sys.exit(1)
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    print(f"Video source opened: {source_str}  ({src_fps:.1f} fps)")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                if isinstance(source, int):
                    continue
                break
            yield frame, None
    finally:
        cap.release()


def _iter_image_dir(dir_path: Path):
    """Yield (frame_bgr, filename) from sorted JPG/PNG files in a directory."""
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
    files = sorted([f for f in dir_path.iterdir()
                    if f.suffix.lower() in exts])
    if not files:
        print(f"ERROR: No image files found in {dir_path}", file=sys.stderr)
        sys.exit(1)
    print(f"Image directory: {dir_path}  ({len(files)} images)")
    for fpath in files:
        frame = cv2.imread(str(fpath))
        if frame is not None:
            yield frame, fpath.name


def _build_retrieval_cache_path(cache_dir: Path, reference_dir: Path, chip_size: int, stride: int) -> Path:
    """Stable cache path for retrieval index without hard-coding image filename."""
    cache_dir.mkdir(parents=True, exist_ok=True)
    key_src = f"{reference_dir.resolve()}::{chip_size}::{stride}"
    key = hashlib.sha1(key_src.encode("utf-8")).hexdigest()[:16]
    return cache_dir / f"retrieval_index_{key}.npz"


def _load_camera_calibration(calib_path: Path):
    """Load OpenCV YAML/JSON calibration with camera_matrix + distortion_coefficients."""
    if not calib_path.exists():
        raise FileNotFoundError(f"Calibration file not found: {calib_path}")
    fs = cv2.FileStorage(str(calib_path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Could not open calibration file: {calib_path}")
    try:
        K = fs.getNode("camera_matrix").mat()
        D = fs.getNode("distortion_coefficients").mat()
    finally:
        fs.release()
    if K is None or D is None:
        raise RuntimeError("Calibration must contain camera_matrix and distortion_coefficients")
    return K.astype(np.float32), D.astype(np.float32)


def _preprocess_frame_for_matching(
    frame_bgr: np.ndarray,
    apply_clahe: bool,
    gamma: float,
    sharpen: float,
    center_mask_ratio: float,
) -> np.ndarray:
    """Lightweight normalization to stabilize map matching under lighting changes."""
    out = frame_bgr.copy()
    if apply_clahe:
        lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        out = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)
    if abs(gamma - 1.0) > 1e-3:
        inv_gamma = 1.0 / max(gamma, 1e-3)
        lut = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)], dtype=np.uint8)
        out = cv2.LUT(out, lut)
    if sharpen > 1e-3:
        blur = cv2.GaussianBlur(out, (0, 0), 1.2)
        out = cv2.addWeighted(out, 1.0 + sharpen, blur, -sharpen, 0)
    if 0.0 < center_mask_ratio < 1.0:
        h, w = out.shape[:2]
        cx, cy = w // 2, h // 2
        hw = int((w * center_mask_ratio) * 0.5)
        hh = int((h * center_mask_ratio) * 0.5)
        mask = np.zeros((h, w), dtype=np.uint8)
        x0, x1 = max(0, cx - hw), min(w, cx + hw)
        y0, y1 = max(0, cy - hh), min(h, cy + hh)
        mask[y0:y1, x0:x1] = 255
        out = cv2.bitwise_and(out, out, mask=mask)
    return out


def _match_spread_score(ref_pts: np.ndarray, chip_w: int, chip_h: int) -> float:
    """How well matches cover the chip area (0..1)."""
    if ref_pts is None or len(ref_pts) < 8:
        return 0.0
    min_xy = np.min(ref_pts, axis=0)
    max_xy = np.max(ref_pts, axis=0)
    span_x = float(max_xy[0] - min_xy[0]) / max(float(chip_w), 1.0)
    span_y = float(max_xy[1] - min_xy[1]) / max(float(chip_h), 1.0)
    return float(np.clip(math.sqrt(max(span_x, 0.0) * max(span_y, 0.0)), 0.0, 1.0))


def _estimate_topk_chip_pose(
    frame_bgr: np.ndarray,
    ref_gray: np.ndarray,
    config: VPSDeviceConfig,
    index,
    K: np.ndarray,
    ref_center_lat: float,
    ref_center_lon: float,
    ref_width_px: int,
    ref_height_px: int,
    ref_m_per_px: float,
    dsm_m: np.ndarray | None,
    prior_latlon: tuple[float, float] | None,
    prior_gate_m: float,
    min_inliers: int,
    topk: int,
) -> EstimateResult:
    """
    Retrieve top-K chips, run local matching per chip, and keep best geometric fix.
    """
    q_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) if frame_bgr.ndim == 3 else frame_bgr
    detector, desc_ext = _get_descriptor_extractor(
        config.use_beblid, getattr(config, "use_sift", False), config.nfeatures_orb
    )
    flann = build_flann_matcher(getattr(config, "use_sift", False))

    q_kpts, q_des = extract_features(q_gray, detector, desc_ext, nfeatures=config.nfeatures_orb)
    if q_des is None or len(q_kpts) < max(config.min_matches, 8):
        return EstimateResult(False, 0.0, 0.0, 0.0, len(q_kpts) if q_kpts else 0)

    candidates = index.query(frame_bgr, topk=max(1, int(topk)))
    if not candidates:
        return EstimateResult(False, 0.0, 0.0, 0.0, 0)

    best = None
    best_score = None
    best_rank = None
    for rank, cand in enumerate(candidates, start=1):
        chip = cand.chip
        chip_gray = ref_gray[chip.y0:chip.y0 + chip.h, chip.x0:chip.x0 + chip.w]
        ref_kpts, ref_des = extract_features(chip_gray, detector, desc_ext, nfeatures=config.nfeatures_orb)
        if ref_des is None or len(ref_kpts) < max(config.min_matches, 8):
            continue
        matches = match_with_ratio_test(ref_des, q_des, flann, ratio=config.ratio_threshold)
        if len(matches) < max(config.min_matches, 8):
            continue

        ref_pts = np.float32([ref_kpts[m.queryIdx].pt for m in matches])
        qry_pts = np.float32([q_kpts[m.trainIdx].pt for m in matches])
        ref_pts_full = [(p[0] + chip.x0, p[1] + chip.y0) for p in ref_pts]

        if dsm_m is not None:
            fix = pnp_fix_with_dsm(
                ref_pts_full,
                qry_pts.tolist(),
                K=K,
                dsm_m=dsm_m,
                ref_center_lat=ref_center_lat,
                ref_center_lon=ref_center_lon,
                ref_width_px=ref_width_px,
                ref_height_px=ref_height_px,
                ref_m_per_px=ref_m_per_px,
            )
        else:
            fix = planar_pose_fix_from_matches(
                ref_pts_full,
                qry_pts.tolist(),
                K=K,
                ref_center_lat=ref_center_lat,
                ref_center_lon=ref_center_lon,
                ref_width_px=ref_width_px,
                ref_height_px=ref_height_px,
                ref_m_per_px=ref_m_per_px,
            )
        if not fix.success or fix.n_inliers < min_inliers:
            continue
        if prior_latlon is not None and prior_gate_m > 0:
            d = get_distance_metres(prior_latlon[0], prior_latlon[1], fix.lat, fix.lon)
            if d > prior_gate_m:
                continue

        spread = _match_spread_score(ref_pts, chip.w, chip.h)
        rmse_score = float(np.clip(1.0 / (1.0 + (fix.reproj_rmse_px / 4.0)), 0.0, 1.0))
        inlier_score = float(np.clip(fix.n_inliers / 30.0, 0.0, 1.0))
        retrieval_score = float(np.clip((cand.score + 1.0) * 0.5, 0.0, 1.0))
        quality = 0.35 * inlier_score + 0.25 * rmse_score + 0.20 * spread + 0.20 * retrieval_score
        score = (
            int(fix.n_inliers),
            float(quality),
            -float(fix.reproj_rmse_px),
        )
        if best_score is None or score > best_score:
            best_score = score
            best = (fix, len(matches), quality)
            best_rank = rank

    if best is None:
        return EstimateResult(False, 0.0, 0.0, 0.0, 0)

    best_fix, best_match_count, quality = best
    rank_weight = 1.0 - 0.1 * max(0, (best_rank or 1) - 1)
    rank_weight = float(np.clip(rank_weight, 0.5, 1.0))
    conf = float(np.clip(best_fix.confidence * rank_weight * (0.7 + 0.6 * quality), 0.0, 1.0))
    weighted_matches = max(min_inliers, int(round(best_fix.n_inliers * (0.7 + 0.6 * quality))))
    return EstimateResult(
        success=True,
        lat=best_fix.lat,
        lon=best_fix.lon,
        confidence=conf,
        n_matches=int(max(weighted_matches, best_match_count)),
    )


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run(args):
    print(f"Loading reference from: {args.reference}")
    config_kwargs = dict(
        h_fov_deg=args.fov_h,
        d_fov_deg=args.fov_d,
        width_px=args.width,
        height_px=args.height,
        nfeatures_orb=args.nfeatures,
        use_sift=args.sift,
        use_beblid=args.beblid,
        matching_flow=args.matching_flow,
    )
    if args.ratio is not None:
        config_kwargs["ratio_threshold"] = args.ratio
    elif args.sift:
        config_kwargs["ratio_threshold"] = 0.75
    if args.max_dist is not None:
        config_kwargs["max_dist_from_last_m"] = args.max_dist
    config = VPSDeviceConfig(**config_kwargs)
    estimator = create_estimator_from_proxigo_region(args.reference, config)
    ref = estimator._ref
    ref_m_per_px = float((ref.x_m_per_px + ref.y_m_per_px) * 0.5)
    print(f"Reference loaded: {ref.width_px}x{ref.height_px} px, "
          f"center=({ref.center_lat:.6f}, {ref.center_lon:.6f}), "
          f"res={ref.x_m_per_px:.3f} m/px")

    # Optional benchmark-style top-K chip retrieval + geometric pose scoring.
    use_topk_map = args.topk_map_update > 1
    retrieval_index = None
    K = None
    dsm_m = None
    undistort_K = None
    undistort_D = None
    if args.dsm_path:
        dsm_path = Path(args.dsm_path)
        dsm = cv2.imread(str(dsm_path), cv2.IMREAD_UNCHANGED)
        if dsm is None:
            print(f"WARNING: Could not load DSM: {dsm_path}. Continuing without DSM.")
        else:
            dsm_m = dsm.astype(np.float32)
            print(f"DSM loaded: {dsm_path.name}  shape={dsm_m.shape}")
    if args.camera_calibration:
        try:
            undistort_K, undistort_D = _load_camera_calibration(Path(args.camera_calibration))
            print(f"Camera calibration loaded: {args.camera_calibration}")
        except Exception as e:
            print(f"WARNING: Failed to load calibration ({e}); undistortion disabled.")

    if use_topk_map:
        try:
            from vps_device.retrieval import ReferenceChipIndex, TorchResNet18Backend
        except Exception as e:
            print(f"WARNING: top-K map mode unavailable ({e}); falling back to single-frame estimator.")
            use_topk_map = False
        else:
            backend = TorchResNet18Backend(device=args.retrieval_device)
            ref_bgr = cv2.cvtColor(ref.image, cv2.COLOR_GRAY2BGR)
            cache_path = _build_retrieval_cache_path(
                Path(args.cache_dir),
                Path(args.reference),
                int(args.chip_size),
                int(args.chip_stride),
            )
            if cache_path.exists() and not args.rebuild_index:
                retrieval_index = ReferenceChipIndex.load(cache_path, ref_bgr=ref_bgr, backend=backend)
                print(f"Top-K retrieval index loaded: {len(retrieval_index.chips)} chips")
            else:
                retrieval_index = ReferenceChipIndex(
                    ref_bgr=ref_bgr,
                    backend=backend,
                    chip_size=int(args.chip_size),
                    stride=int(args.chip_stride),
                )
                print(f"Building top-K retrieval index (chip={args.chip_size}, stride={args.chip_stride}) ...")
                retrieval_index.build()
                retrieval_index.save(cache_path)
                print(f"Top-K retrieval index saved: {cache_path}")
            K = camera_matrix_from_fov(args.width, args.height, args.fov_h)

    # --- VO + EKF fusion ---
    vo = VisualOdometry(config, nfeatures=max(config.nfeatures_orb, 1000))
    heading_cal = HeadingCalibrator(min_fixes=args.calibration_frames)
    ekf = VPSFusionEKF(EKFConfig(
        map_position_noise=args.map_noise,
    ))
    map_match_every = args.map_match_every
    min_map_inliers = args.min_map_inliers
    print(f"Fusion: VO every frame, map match every {map_match_every} frames, "
          f"map noise={args.map_noise:.0f}m, min inliers={min_map_inliers}, "
          f"cal frames={args.calibration_frames}")
    if use_topk_map:
        print(
            f"Map mode: top-K chip retrieval (K={args.topk_map_update}, "
            f"chip={args.chip_size}, stride={args.chip_stride})"
        )
    else:
        print("Map mode: single-estimator full-reference")

    # Load ground truth if provided
    ground_truth = None
    if args.source_csv:
        gt_path = Path(args.source_csv)
        if gt_path.exists():
            ground_truth = _load_ground_truth_csv(gt_path)
            print(f"Ground truth loaded: {len(ground_truth)} entries from {gt_path.name}")
        else:
            print(f"WARNING: CSV not found: {gt_path}", file=sys.stderr)

    # Select frame source
    use_image_dir = args.source_dir is not None
    if use_image_dir:
        frame_source = _iter_image_dir(Path(args.source_dir))
    else:
        frame_source = _iter_video(args.source)

    # CSV output
    has_gt = ground_truth is not None
    csv_file = None
    csv_writer = None
    if args.output_csv:
        csv_file = open(args.output_csv, "w", newline="")
        csv_writer = csv_mod.writer(csv_file)
        header = [
            "frame", "timestamp_s", "fused_lat", "fused_lon",
            "speed_m_s", "vo_quality",
            "map_lat", "map_lon", "map_conf", "map_matches",
        ]
        if has_gt:
            header += ["truth_lat", "truth_lon", "truth_alt", "error_m"]
        csv_writer.writerow(header)

    trail_pixels = []        # fused EKF trail (green)
    map_trail_pixels = []    # raw map-match trail (cyan)
    truth_trail_pixels = []  # ground truth trail (blue)
    frame_idx = 0
    raw_frame_idx = 0
    last_map_result = None
    last_error_m = None
    last_vo_quality = 0.0
    prev_yaw = None          # for detecting large heading changes
    t_start = time.monotonic()
    total_error = 0.0
    n_errors = 0
    n_map_matches = 0

    display_w = args.display_width

    try:
        for frame, fname in frame_source:
            if raw_frame_idx < args.start_frame:
                raw_frame_idx += 1
                continue
            if args.frame_stride > 1:
                rel_idx = raw_frame_idx - args.start_frame
                if rel_idx % args.frame_stride != 0:
                    raw_frame_idx += 1
                    continue
            if args.max_frames is not None and frame_idx >= args.max_frames:
                break

            t_now = time.monotonic()
            elapsed = t_now - t_start
            fps = (frame_idx + 1) / max(elapsed, 0.001)

            gt_entry = None
            if ground_truth and fname:
                gt_entry = ground_truth.get(fname)

            altitude = gt_entry["height"] if gt_entry else args.altitude
            frame_yaw = gt_entry.get("yaw") if gt_entry else None

            map_frame = frame
            if undistort_K is not None and undistort_D is not None:
                map_frame = cv2.undistort(map_frame, undistort_K, undistort_D)
            map_frame = _preprocess_frame_for_matching(
                map_frame,
                apply_clahe=not args.no_clahe,
                gamma=args.match_gamma,
                sharpen=args.match_sharpen,
                center_mask_ratio=args.center_mask_ratio,
            )

            # Reset VO on large heading changes (e.g. U-turns in survey flights)
            if frame_yaw is not None and prev_yaw is not None:
                yaw_delta = abs(frame_yaw - prev_yaw)
                if yaw_delta > 180:
                    yaw_delta = 360 - yaw_delta
                if yaw_delta > 30:
                    vo.reset()
            prev_yaw = frame_yaw

            # --- 1. Visual Odometry (every frame, ~3-5ms) ---
            vo_result = vo.process_frame(frame, altitude)
            if vo_result:
                last_vo_quality = vo_result.quality
                if not heading_cal.calibrated and vo_result.quality > 0:
                    heading_cal.accumulate_vo(vo_result.dx, vo_result.dy)

                if ekf.initialized and heading_cal.calibrated:
                    if vo_result.quality > 0:
                        if heading_cal._has_yaw and frame_yaw is not None:
                            vo_east, vo_north = heading_cal.transform_with_yaw(
                                vo_result.dx, vo_result.dy, frame_yaw)
                        else:
                            vo_east, vo_north = heading_cal.transform(
                                vo_result.dx, vo_result.dy)
                        ekf.predict_with_vo(vo_east, vo_north, vo_result.quality,
                                            altitude_m=altitude)
                        heading_cal.accumulate_recal_vo(vo_result.dx, vo_result.dy)
                    else:
                        ekf.predict_constant_velocity()

            # Initialize EKF from first ground truth if available
            if not ekf.initialized and gt_entry:
                ekf.initialize(gt_entry["lat"], gt_entry["lon"])
                heading_cal.add_fix(gt_entry["lat"], gt_entry["lon"],
                                    yaw_deg=frame_yaw)
                print(f"  EKF initialized from ground truth: "
                      f"({gt_entry['lat']:.6f}, {gt_entry['lon']:.6f})")

            # Add GT fix for heading calibration (first few frames)
            if gt_entry and not heading_cal.calibrated and frame_idx > 0:
                heading_cal.add_fix(gt_entry["lat"], gt_entry["lon"],
                                    yaw_deg=frame_yaw)
                if heading_cal.try_calibrate():
                    yaw_mode = "per-frame yaw" if heading_cal._has_yaw else "fixed"
                    print(f"  Heading calibrated ({yaw_mode}): "
                          f"{heading_cal.heading_deg:.0f} deg, "
                          f"scale={heading_cal._scale:.3f}")
                    ekf.initialize(gt_entry["lat"], gt_entry["lon"])

            # --- 2. Map Matching (every Nth frame) ---
            should_map_match = (frame_idx % map_match_every == 0)
            if should_map_match:
                prior_latlon = ekf.position if ekf.initialized else None
                prior_gate_m = max(float(args.prior_gate_m), altitude * args.prior_gate_alt_scale)
                if use_topk_map and retrieval_index is not None and K is not None:
                    last_map_result = _estimate_topk_chip_pose(
                        frame_bgr=map_frame,
                        ref_gray=ref.image,
                        config=config,
                        index=retrieval_index,
                        K=K,
                        ref_center_lat=ref.center_lat,
                        ref_center_lon=ref.center_lon,
                        ref_width_px=ref.width_px,
                        ref_height_px=ref.height_px,
                        ref_m_per_px=ref_m_per_px,
                        dsm_m=dsm_m,
                        prior_latlon=prior_latlon,
                        prior_gate_m=prior_gate_m,
                        min_inliers=min_map_inliers,
                        topk=args.topk_map_update,
                    )
                else:
                    last_map_result = estimator.estimate(map_frame, altitude)
                if last_map_result.success and last_map_result.n_matches >= min_map_inliers:
                    mm_conf = last_map_result.confidence
                    mm_inliers = last_map_result.n_matches
                    if prior_latlon is not None:
                        prior_dist_m = get_distance_metres(
                            prior_latlon[0], prior_latlon[1], last_map_result.lat, last_map_result.lon
                        )
                        # Continuously down-weight updates near the prior gate edge.
                        if prior_gate_m > 0:
                            prior_weight = float(np.clip(1.0 - (prior_dist_m / prior_gate_m), 0.15, 1.0))
                            mm_conf *= prior_weight
                            mm_inliers = max(min_map_inliers, int(round(mm_inliers * prior_weight)))
                    mm_conf = float(np.clip(mm_conf, 0.01, 1.0))
                    accepted = ekf.update_map_match(
                        last_map_result.lat, last_map_result.lon,
                        confidence=mm_conf,
                        n_inliers=mm_inliers,
                        altitude_m=altitude,
                    )
                    if accepted:
                        n_map_matches += 1
                        px, py = _geo_to_ref_pixel(last_map_result.lat, last_map_result.lon, ref)
                        map_trail_pixels.append((px, py))
                        # Online recalibration from trusted map matches
                        if heading_cal.calibrated:
                            refined = heading_cal.try_recalibrate(
                                last_map_result.lat, last_map_result.lon)
                            if not refined:
                                heading_cal.begin_recalibration_segment(
                                    last_map_result.lat, last_map_result.lon)
                    if not heading_cal.calibrated:
                        heading_cal.add_fix(last_map_result.lat,
                                            last_map_result.lon,
                                            yaw_deg=frame_yaw)
                        heading_cal.try_calibrate()

            # --- 3. Fused position from EKF ---
            fused_pos = ekf.position if ekf.initialized else None
            speed = ekf.speed if ekf.initialized else 0.0
            last_error_m = None

            if fused_pos:
                px, py = _geo_to_ref_pixel(fused_pos[0], fused_pos[1], ref)
                trail_pixels.append((px, py))
                if len(trail_pixels) > 2000:
                    trail_pixels = trail_pixels[-1000:]

                if gt_entry:
                    last_error_m = get_distance_metres(
                        gt_entry["lat"], gt_entry["lon"],
                        fused_pos[0], fused_pos[1])
                    total_error += last_error_m
                    n_errors += 1

            if gt_entry:
                gt_px, gt_py = _geo_to_ref_pixel(gt_entry["lat"], gt_entry["lon"], ref)
                truth_trail_pixels.append((gt_px, gt_py))

            # --- CSV ---
            if csv_writer:
                row = [
                    frame_idx,
                    f"{elapsed:.3f}",
                    f"{fused_pos[0]:.8f}" if fused_pos else "",
                    f"{fused_pos[1]:.8f}" if fused_pos else "",
                    f"{speed:.2f}",
                    f"{last_vo_quality:.2f}",
                    f"{last_map_result.lat:.8f}" if last_map_result and last_map_result.success and should_map_match else "",
                    f"{last_map_result.lon:.8f}" if last_map_result and last_map_result.success and should_map_match else "",
                    f"{last_map_result.confidence:.4f}" if last_map_result and should_map_match else "",
                    last_map_result.n_matches if last_map_result and should_map_match else "",
                ]
                if has_gt:
                    if gt_entry:
                        row += [
                            f"{gt_entry['lat']:.8f}",
                            f"{gt_entry['lon']:.8f}",
                            f"{gt_entry['height']:.1f}",
                            f"{last_error_m:.2f}" if last_error_m is not None else "",
                        ]
                    else:
                        row += ["", "", "", ""]
                csv_writer.writerow(row)

            # --- Console output ---
            if not args.headless:
                status = f"[{elapsed:7.2f}s] "
                if fused_pos:
                    status += f"FUSED({fused_pos[0]:.6f},{fused_pos[1]:.6f}) "
                    status += f"spd={speed:.1f}m/s vo={last_vo_quality:.0%}"
                    if should_map_match and last_map_result:
                        if last_map_result.success:
                            status += f" MAP:{last_map_result.n_matches}inl"
                        else:
                            status += " MAP:miss"
                    if last_error_m is not None:
                        status += f" err={last_error_m:.1f}m"
                else:
                    status += "waiting for init..."
                if fname:
                    status += f"  [{fname}]"
                print(status)

            frame_idx += 1
            raw_frame_idx += 1

            if args.headless:
                continue

            # --- Build display ---
            cam_vis = _draw_keypoints_on_frame(frame, config)
            panel_h = 400
            cam_panel = _resize_to_height(cam_vis, panel_h)

            debug_img = estimator.get_debug_image(frame)
            if debug_img is not None:
                match_panel = _resize_to_height(debug_img, panel_h)
            else:
                match_panel = np.zeros((panel_h, cam_panel.shape[1], 3), dtype=np.uint8)
                cv2.putText(match_panel, "No match data", (20, panel_h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2)

            top_row = np.hstack([cam_panel, match_panel])
            top_w = top_row.shape[1]

            stats = _build_stats_bar(
                top_w, fused_pos, speed, last_vo_quality,
                last_map_result if should_map_match else None,
                fps, frame_idx, last_error_m,
            )

            minimap = _build_minimap(ref.image, trail_pixels,
                                     truth_trail=truth_trail_pixels, minimap_h=180)
            minimap_row = np.zeros((minimap.shape[0], top_w, 3), dtype=np.uint8)
            minimap_row[:] = (30, 30, 30)
            mx = 10
            mw = min(minimap.shape[1], top_w - 2 * mx)
            minimap_cropped = minimap[:, :mw]
            minimap_row[:minimap_cropped.shape[0], mx:mx + mw] = minimap_cropped

            cv2.putText(minimap_row, "Position Trail", (mw + mx + 10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            if has_gt:
                cv2.putText(minimap_row, "Blue=truth  Green=fused", (mw + mx + 10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
            if fused_pos:
                cv2.putText(minimap_row,
                            f"({fused_pos[0]:.6f}, {fused_pos[1]:.6f})",
                            (mw + mx + 10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            display = np.vstack([top_row, stats, minimap_row])

            if display.shape[1] > display_w:
                scale = display_w / display.shape[1]
                display = cv2.resize(display, (display_w, int(display.shape[0] * scale)))

            cv2.imshow("VPS Live", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if not args.headless:
            cv2.destroyAllWindows()
        if csv_file:
            csv_file.close()
            print(f"CSV saved to: {args.output_csv}")

    # Summary
    print(f"\nProcessed {frame_idx} frames in {elapsed:.1f}s "
          f"({fps:.1f} fps)")
    print(f"  Map matches attempted: {frame_idx // map_match_every}, "
          f"successful: {n_map_matches}")
    if n_errors > 0:
        avg_err = total_error / n_errors
        print(f"  Mean error vs ground truth: {avg_err:.1f} m  ({n_errors} frames)")
    if ekf.initialized:
        pos = ekf.position
        print(f"  Final fused position: ({pos[0]:.6f}, {pos[1]:.6f})")


def main():
    parser = argparse.ArgumentParser(
        description="Standalone VPS live runner (camera, video file, or image directory).",
    )

    src_group = parser.add_argument_group("Input source (pick one)")
    src_group.add_argument("--source", type=str, default=None,
                           help="Camera device (/dev/video0), device index (0), "
                                "GStreamer pipeline, or video file path")
    src_group.add_argument("--source-dir", type=str, default=None,
                           help="Directory of sequential images (JPG/PNG)")
    src_group.add_argument("--source-csv", type=str, default=None,
                           help="CSV with ground truth (columns: filename, lat, lon, height). "
                                "Per-frame altitude overrides --altitude.")

    parser.add_argument("--reference", required=True,
                        help="Path to Proxigo region directory")
    parser.add_argument("--altitude", type=float, default=100.0,
                        help="Camera altitude above ground in metres (default 100). "
                             "Overridden per-frame when --source-csv is provided.")
    parser.add_argument("--rate", type=float, default=2.0,
                        help="VPS estimation rate in Hz for video/camera (default 2.0). "
                             "Image directories process every frame regardless.")
    parser.add_argument("--start-frame", type=int, default=0,
                        help="Skip frames before this index (default 0).")
    parser.add_argument("--frame-stride", type=int, default=1,
                        help="Process every Nth frame after start-frame (default 1).")
    parser.add_argument("--max-frames", type=int, default=None,
                        help="Maximum number of processed frames (default: all).")
    parser.add_argument("--headless", action="store_true",
                        help="Disable display window (CSV-only mode)")
    parser.add_argument("--output-csv", type=str, default=None,
                        help="Path to output CSV file for position log")
    parser.add_argument("--display-width", type=int, default=1400,
                        help="Maximum display window width in pixels")
    parser.add_argument("--fov-h", type=float, default=71.5,
                        help="Camera horizontal FOV in degrees")
    parser.add_argument("--fov-d", type=float, default=79.5,
                        help="Camera diagonal FOV in degrees")
    parser.add_argument("--width", type=int, default=1920,
                        help="Camera frame width in pixels")
    parser.add_argument("--height", type=int, default=1080,
                        help="Camera frame height in pixels")
    parser.add_argument("--nfeatures", type=int, default=2000,
                        help="ORB features to extract (default 2000)")
    parser.add_argument("--sift", action="store_true",
                        help="Use SIFT features (float, more distinctive) "
                             "instead of ORB (binary, faster)")
    parser.add_argument("--beblid", action="store_true",
                        help="Use BEBLID descriptors with ORB keypoints (requires opencv-contrib)")
    parser.add_argument("--matching-flow", type=str, default="homography",
                        choices=["homography", "cluster"],
                        help="Map-matching flow: homography or cluster-gated geo flow (default homography)")
    parser.add_argument("--topk-map-update", type=int, default=1,
                        help="Top-K chip retrieval map update (default 1 = disabled, estimator-only).")
    parser.add_argument("--chip-size", type=int, default=768,
                        help="Retrieval chip size (px) when top-K map mode is enabled.")
    parser.add_argument("--chip-stride", type=int, default=512,
                        help="Retrieval chip stride (px) when top-K map mode is enabled.")
    parser.add_argument("--cache-dir", type=str, default="test_data/cache",
                        help="Directory for retrieval index cache files.")
    parser.add_argument("--rebuild-index", action="store_true",
                        help="Force rebuild of retrieval index cache.")
    parser.add_argument("--retrieval-device", type=str, default="cpu",
                        help="Torch device for retrieval backend (cpu, cuda:0, ...).")
    parser.add_argument("--dsm-path", type=str, default=None,
                        help="Optional DSM raster aligned with reference (enables DSM-backed PnP in top-K mode).")
    parser.add_argument("--prior-gate-m", type=float, default=180.0,
                        help="Base prior gate radius in metres for map updates.")
    parser.add_argument("--prior-gate-alt-scale", type=float, default=1.5,
                        help="Additional prior-gate metres per altitude metre (default 1.5x alt).")
    parser.add_argument("--camera-calibration", type=str, default=None,
                        help="OpenCV calibration YAML/JSON (camera_matrix + distortion_coefficients).")
    parser.add_argument("--no-clahe", action="store_true",
                        help="Disable CLAHE normalization before map matching.")
    parser.add_argument("--match-gamma", type=float, default=1.0,
                        help="Gamma correction for map matching preprocessing (default 1.0).")
    parser.add_argument("--match-sharpen", type=float, default=0.2,
                        help="Unsharp amount for map matching preprocessing (default 0.2).")
    parser.add_argument("--center-mask-ratio", type=float, default=0.85,
                        help="Keep central ROI ratio [0..1] for map matching (default 0.85).")
    parser.add_argument("--ratio", type=float, default=None,
                        help="Lowe's ratio test threshold (default: 0.75 for SIFT, "
                             "0.95 for ORB). Lower = stricter matching.")
    parser.add_argument("--max-dist", type=float, default=None,
                        help="Continuity: max metres from last estimate "
                             "(default 50, auto-scaled by altitude). "
                             "Set very high (e.g. 99999) to disable.")

    fusion_group = parser.add_argument_group("EKF Fusion")
    fusion_group.add_argument("--map-match-every", type=int, default=5,
                              help="Run map matching every N frames (default 5). "
                                   "VO runs every frame regardless.")
    fusion_group.add_argument("--map-noise", type=float, default=30.0,
                              help="Map match position noise in metres (default 30). "
                                   "Higher = trust VO more, lower = trust map more.")
    fusion_group.add_argument("--min-map-inliers", type=int, default=15,
                              help="Minimum RANSAC inliers to accept a map match (default 15).")
    fusion_group.add_argument("--calibration-frames", type=int, default=5,
                              help="Number of position fixes before heading calibration (default 5).")

    args = parser.parse_args()
    args.reference = Path(args.reference)

    if not args.reference.is_dir():
        print(f"ERROR: Reference directory not found: {args.reference}", file=sys.stderr)
        sys.exit(1)

    if args.source is None and args.source_dir is None:
        parser.error("Provide either --source (camera/video) or --source-dir (image directory)")
    if args.start_frame < 0:
        parser.error("--start-frame must be >= 0")
    if args.frame_stride < 1:
        parser.error("--frame-stride must be >= 1")
    if args.max_frames is not None and args.max_frames < 1:
        parser.error("--max-frames must be >= 1 when provided")
    if args.topk_map_update < 1:
        parser.error("--topk-map-update must be >= 1")
    if args.chip_size < 64 or args.chip_stride < 16:
        parser.error("--chip-size must be >= 64 and --chip-stride must be >= 16")
    if args.prior_gate_m < 0 or args.prior_gate_alt_scale < 0:
        parser.error("--prior-gate-m and --prior-gate-alt-scale must be >= 0")
    if args.match_gamma <= 0:
        parser.error("--match-gamma must be > 0")
    if not (0.0 <= args.center_mask_ratio <= 1.0):
        parser.error("--center-mask-ratio must be between 0 and 1")

    run(args)


if __name__ == "__main__":
    main()
