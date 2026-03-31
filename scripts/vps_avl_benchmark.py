#!/usr/bin/env python3
"""
Benchmark-style runner inspired by UAV-AVL baseline structure:
  retrieval -> local matching -> planar pose fix (homography decomposition) -> EKF fusion.

This is intentionally separate from scripts/vps_live.py to keep the live pipeline stable.
"""

from __future__ import annotations

import argparse
import csv as csv_mod
import sys
import time
from pathlib import Path
from typing import Optional

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
from vps_device.dataset_adapters import load_uav_avl_metadata_json
from vps_device.ekf import VPSFusionEKF, EKFConfig
from vps_device.features import _get_descriptor_extractor, build_flann_matcher, extract_features, match_with_ratio_test
from vps_device.geo_transform import get_distance_metres
from vps_device.pnp import camera_matrix_from_fov, planar_pose_fix_from_matches, pnp_fix_with_dsm
from vps_device.reference_loader import load_proxigo_region
from vps_device.retrieval import ReferenceChipIndex, TorchResNet18Backend, default_cache_path
from vps_device.visual_odometry import VisualOdometry, HeadingCalibrator


def _iter_image_dir(dir_path: Path):
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
    files = sorted([f for f in dir_path.iterdir() if f.suffix.lower() in exts])
    if not files:
        raise FileNotFoundError(f"No images found in {dir_path}")
    for fpath in files:
        frame = cv2.imread(str(fpath))
        if frame is not None:
            yield frame, fpath.name


def _local_match_on_chip(
    chip_gray: np.ndarray,
    query_bgr: np.ndarray,
    config: VPSDeviceConfig,
):
    """Return (ref_pts_px, query_pts_px, n_inliers, n_matches) in chip-local ref coords."""
    detector, desc_ext = _get_descriptor_extractor(False, getattr(config, "use_sift", False), config.nfeatures_orb)
    flann = build_flann_matcher(getattr(config, "use_sift", False))

    q_gray = cv2.cvtColor(query_bgr, cv2.COLOR_BGR2GRAY) if query_bgr.ndim == 3 else query_bgr
    ref_kpts, ref_des = extract_features(chip_gray, detector, desc_ext, nfeatures=config.nfeatures_orb)
    q_kpts, q_des = extract_features(q_gray, detector, desc_ext, nfeatures=config.nfeatures_orb)
    if ref_des is None or q_des is None or len(ref_kpts) < 10 or len(q_kpts) < 10:
        return [], [], 0, 0

    matches = match_with_ratio_test(ref_des, q_des, flann, ratio=config.ratio_threshold)
    if len(matches) < max(config.min_matches, 8):
        return [], [], 0, len(matches)

    ref_pts = np.float32([ref_kpts[m.queryIdx].pt for m in matches])
    q_pts = np.float32([q_kpts[m.trainIdx].pt for m in matches])
    # Inlier count is computed inside pose module, but return points for it.
    return ref_pts.tolist(), q_pts.tolist(), 0, len(matches)


def run(args):
    ref_path = Path(args.reference)
    region = load_proxigo_region(ref_path)
    ref_gray = region.image
    ref_bgr = cv2.cvtColor(ref_gray, cv2.COLOR_GRAY2BGR)
    dsm_m = None
    if args.dsm_path:
        dsm_path = Path(args.dsm_path)
        dsm = cv2.imread(str(dsm_path), cv2.IMREAD_UNCHANGED)
        if dsm is None:
            raise FileNotFoundError(f"Could not load DSM: {dsm_path}")
        dsm_m = dsm.astype(np.float32)
        print(f"DSM loaded: {dsm_path.name}  shape={dsm_m.shape} dtype={dsm_m.dtype}")

    # Load UAV-AVL/AnyVisLoc metadata (optional, used for GT + yaw/fov)
    records = None
    per_frame = None
    if args.metadata_json:
        records = load_uav_avl_metadata_json(Path(args.metadata_json))
        print(f"Loaded metadata: {len(records)} frames")

        # Resolve filename collisions by selecting only records that belong to the chosen flight subdir.
        # Example: "QZ_SongCity" vs "Qingzhou_3_2" both contain DJI_0534.JPG etc.
        if args.flight_filter:
            filtered = [r for r in records if args.flight_filter in r.path.replace("\\", "/")]
            print(f"Metadata flight_filter='{args.flight_filter}': {len(filtered)} frames")
            records = filtered

        per_frame = {r.filename: r for r in records}

    # Auto camera params from metadata (if present) so we don't accidentally run with defaults.
    if args.use_metadata_camera and records:
        for rec in records:
            if rec.width_px and rec.height_px and rec.fov_deg:
                w = int(rec.width_px)
                h = int(rec.height_px)
                dfov = float(rec.fov_deg)
                diag = float(np.hypot(w, h))
                hfov = 2.0 * np.arctan((w / diag) * np.tan(np.deg2rad(dfov) / 2.0))
                args.width = w
                args.height = h
                args.fov_d = dfov
                args.fov_h = float(np.rad2deg(hfov))
                print(f"Camera from metadata: {w}x{h}  dFOV={dfov:.1f}  hFOV={args.fov_h:.1f}")
                break

    # Retrieval index
    backend = TorchResNet18Backend(device=args.retrieval_device)
    cache_path = default_cache_path(Path(args.cache_dir), ref_path / "satellite.png", backend,
                                    args.chip_size, args.stride)
    if cache_path.exists() and not args.rebuild_index:
        index = ReferenceChipIndex.load(cache_path, ref_bgr=ref_bgr, backend=backend)
        print(f"Retrieval index loaded: {len(index.chips)} chips ({backend.name})")
    else:
        index = ReferenceChipIndex(ref_bgr=ref_bgr, backend=backend,
                                  chip_size=args.chip_size, stride=args.stride)
        print(f"Building retrieval index: chip={args.chip_size} stride={args.stride} ...")
        index.build()
        index.save(cache_path)
        print(f"Retrieval index saved to: {cache_path.name}")

    # VO + EKF
    config = VPSDeviceConfig(
        h_fov_deg=args.fov_h,
        d_fov_deg=args.fov_d,
        width_px=args.width,
        height_px=args.height,
        nfeatures_orb=args.nfeatures,
        ratio_threshold=args.ratio,
        max_dist_from_last_m=args.max_dist,
    )
    vo = VisualOdometry(config, nfeatures=max(config.nfeatures_orb, 1000))
    heading_cal = HeadingCalibrator(min_fixes=args.calibration_frames)
    ekf = VPSFusionEKF(EKFConfig(map_position_noise=args.map_noise))

    # Camera intrinsics from FOV (benchmark assumes known intrinsics)
    K = camera_matrix_from_fov(args.width, args.height, args.fov_h)

    # CSV output
    csv_path = Path(args.output_csv) if args.output_csv else None
    csv_file = open(csv_path, "w", newline="") if csv_path else None
    csv_writer = csv_mod.writer(csv_file) if csv_file else None
    if csv_writer:
        csv_writer.writerow(
            [
                "frame",
                "filename",
                "fused_lat",
                "fused_lon",
                "speed_m_s",
                "vo_quality",
                "retrieval_score",
                "retrieval_rank",
                "chip_id",
                "chip_matches",
                "pnp_lat",
                "pnp_lon",
                "pnp_conf",
                "pnp_inliers",
                "pnp_rmse_px",
                "accepted",
                "truth_lat",
                "truth_lon",
                "truth_alt",
                "error_m",
            ]
        )

    frame_idx = 0
    n_updates = 0
    n_errors = 0
    total_error = 0.0
    t0 = time.monotonic()

    try:
        for frame, fname in _iter_image_dir(Path(args.source_dir)):
            meta = per_frame.get(fname) if per_frame else None
            altitude = float(meta.altitude_m) if meta else args.altitude
            yaw = float(meta.yaw_deg) if meta and meta.yaw_deg is not None else None

            # Initialize EKF from first GT if available
            if not ekf.initialized and meta:
                ekf.initialize(meta.lat, meta.lon)
                heading_cal.add_fix(meta.lat, meta.lon, yaw_deg=yaw)
                print(f"EKF init from GT: ({meta.lat:.6f},{meta.lon:.6f})")

            # VO predict
            vo_result = vo.process_frame(frame, altitude)
            vo_quality = 0.0
            if vo_result:
                vo_quality = vo_result.quality
                if not heading_cal.calibrated and vo_quality > 0 and meta:
                    heading_cal.accumulate_vo(vo_result.dx, vo_result.dy)
                    heading_cal.add_fix(meta.lat, meta.lon, yaw_deg=yaw)
                    if heading_cal.try_calibrate():
                        mode = "per-frame yaw" if heading_cal._has_yaw else "fixed"
                        print(f"Heading calibrated ({mode}): {heading_cal.heading_deg:.0f}deg scale={heading_cal._scale:.3f}")
                        ekf.initialize(meta.lat, meta.lon)

                if ekf.initialized and heading_cal.calibrated:
                    if vo_quality > 0:
                        if heading_cal._has_yaw and yaw is not None:
                            de, dn = heading_cal.transform_with_yaw(vo_result.dx, vo_result.dy, yaw)
                        else:
                            de, dn = heading_cal.transform(vo_result.dx, vo_result.dy)
                        ekf.predict_with_vo(de, dn, vo_quality, altitude_m=altitude)
                    else:
                        ekf.predict_constant_velocity()

            # Retrieval + local matching + pose fix (every N frames)
            accepted = False
            pnp_fix = None
            best_score = 0.0
            best_rank = None
            best_chip = None
            best_matches = 0
            if ekf.initialized and (frame_idx % args.map_match_every == 0):
                results = index.query(frame, topk=args.topk)
                if results:
                    best_candidate = None
                    best_candidate_score = None

                    # Evaluate all retrieved chips and keep the strongest PnP hypothesis.
                    for rank, cand in enumerate(results, start=1):
                        chip = cand.chip
                        chip_gray = ref_gray[chip.y0 : chip.y0 + chip.h, chip.x0 : chip.x0 + chip.w]
                        ref_pts, q_pts, _, n_matches = _local_match_on_chip(chip_gray, frame, config)
                        if not ref_pts or not q_pts:
                            continue

                        ref_pts_full = [(p[0] + chip.x0, p[1] + chip.y0) for p in ref_pts]
                        if dsm_m is not None:
                            fix = pnp_fix_with_dsm(
                                ref_pts_full,
                                q_pts,
                                K=K,
                                dsm_m=dsm_m,
                                ref_center_lat=region.center_lat,
                                ref_center_lon=region.center_lon,
                                ref_width_px=region.width_px,
                                ref_height_px=region.height_px,
                                ref_m_per_px=float((region.x_m_per_px + region.y_m_per_px) / 2.0),
                            )
                        else:
                            fix = planar_pose_fix_from_matches(
                                ref_pts_full,
                                q_pts,
                                K=K,
                                ref_center_lat=region.center_lat,
                                ref_center_lon=region.center_lon,
                                ref_width_px=region.width_px,
                                ref_height_px=region.height_px,
                                ref_m_per_px=float((region.x_m_per_px + region.y_m_per_px) / 2.0),
                            )

                        if not fix.success or fix.n_inliers < args.min_pnp_inliers:
                            continue

                        # Prioritize geometric strength (inliers, reprojection quality), then confidence and retrieval score.
                        candidate_score = (
                            int(fix.n_inliers),
                            -float(fix.reproj_rmse_px),
                            float(fix.confidence),
                            float(cand.score),
                        )
                        if best_candidate_score is None or candidate_score > best_candidate_score:
                            best_candidate_score = candidate_score
                            best_candidate = (rank, cand, fix, n_matches)

                    if best_candidate is not None:
                        best_rank, best_cand, pnp_fix, best_matches = best_candidate
                        best_score = best_cand.score
                        best_chip = best_cand.chip

                        # Slightly boost confidence for top-ranked retrievals, damp lower ranks.
                        rank_weight = 1.0 - 0.1 * max(0, best_rank - 1)
                        rank_weight = float(np.clip(rank_weight, 0.5, 1.0))
                        adj_conf = float(np.clip(pnp_fix.confidence * rank_weight, 0.0, 1.0))

                        accepted = ekf.update_map_match(
                            pnp_fix.lat,
                            pnp_fix.lon,
                            confidence=adj_conf,
                            n_inliers=pnp_fix.n_inliers,
                            altitude_m=altitude,
                        )
                        if accepted:
                            n_updates += 1

            fused = ekf.position if ekf.initialized else None
            speed = ekf.speed if ekf.initialized else 0.0

            # Error vs GT if available
            err_m = None
            if fused and meta:
                err_m = get_distance_metres(meta.lat, meta.lon, fused[0], fused[1])
                total_error += float(err_m)
                n_errors += 1

            if csv_writer:
                csv_writer.writerow(
                    [
                        frame_idx,
                        fname,
                        f"{fused[0]:.8f}" if fused else "",
                        f"{fused[1]:.8f}" if fused else "",
                        f"{speed:.2f}",
                        f"{vo_quality:.3f}",
                        f"{best_score:.5f}" if best_chip else "",
                        int(best_rank) if best_rank is not None else "",
                        int(best_chip.chip_id) if best_chip else "",
                        int(best_matches) if best_chip else "",
                        f"{pnp_fix.lat:.8f}" if pnp_fix and pnp_fix.success else "",
                        f"{pnp_fix.lon:.8f}" if pnp_fix and pnp_fix.success else "",
                        f"{pnp_fix.confidence:.4f}" if pnp_fix and pnp_fix.success else "",
                        int(pnp_fix.n_inliers) if pnp_fix and pnp_fix.success else "",
                        f"{pnp_fix.reproj_rmse_px:.2f}" if pnp_fix and pnp_fix.success else "",
                        int(1 if accepted else 0),
                        f"{meta.lat:.8f}" if meta else "",
                        f"{meta.lon:.8f}" if meta else "",
                        f"{altitude:.1f}" if meta else "",
                        f"{err_m:.2f}" if err_m is not None else "",
                    ]
                )

            frame_idx += 1

    finally:
        if csv_file:
            csv_file.close()

    dt = max(1e-6, time.monotonic() - t0)
    fps = frame_idx / dt
    print(f"Processed {frame_idx} frames in {dt:.1f}s ({fps:.2f} fps)")
    print(f"Map/PnP updates accepted: {n_updates}")
    if n_errors:
        print(f"Mean error vs GT: {total_error / n_errors:.1f} m ({n_errors} frames)")


def main():
    p = argparse.ArgumentParser(description="UAV-AVL-style benchmark runner (retrieval + PnP + EKF)")
    p.add_argument("--source-dir", required=True, help="Directory with query images (JPG/PNG)")
    p.add_argument("--reference", required=True, help="Proxigo region directory (contains satellite.png + metadata.json)")
    p.add_argument("--metadata-json", default=None, help="UAV-AVL/AnyVisLoc metadata JSON (per-frame GT + yaw/FOV)")
    p.add_argument("--flight-filter", default=None,
                   help="Substring filter applied to metadata record paths to avoid filename collisions "
                        "(e.g. 'QZ_SongCity').")
    p.add_argument("--dsm-path", default=None, help="DSM GeoTIFF aligned to reference (meters). If provided, uses true PnP.")
    p.add_argument("--output-csv", default="results_avl.csv", help="Output CSV path")

    p.add_argument("--altitude", type=float, default=100.0, help="Fallback altitude if metadata missing")
    p.add_argument("--fov-h", type=float, default=71.5, help="Horizontal FOV deg (used for intrinsics)")
    p.add_argument("--fov-d", type=float, default=79.5, help="Diagonal FOV deg (kept for consistency)")
    p.add_argument("--width", type=int, default=1920)
    p.add_argument("--height", type=int, default=1080)
    p.add_argument("--use-metadata-camera", action="store_true", help="Override width/height/FOV from metadata when available")

    p.add_argument("--chip-size", type=int, default=768)
    p.add_argument("--stride", type=int, default=512)
    p.add_argument("--topk", type=int, default=5)
    p.add_argument("--cache-dir", type=str, default="test_data/cache")
    p.add_argument("--rebuild-index", action="store_true")
    p.add_argument("--retrieval-device", type=str, default="cpu", help="torch device string (cpu, cuda:0, ...)")

    p.add_argument("--nfeatures", type=int, default=2000)
    p.add_argument("--ratio", type=float, default=0.85)
    p.add_argument("--max-dist", type=float, default=99999.0)

    p.add_argument("--map-match-every", type=int, default=5)
    p.add_argument("--map-noise", type=float, default=30.0)
    p.add_argument("--min-pnp-inliers", type=int, default=12)
    p.add_argument("--calibration-frames", type=int, default=5)

    args = p.parse_args()
    run(args)


if __name__ == "__main__":
    main()

