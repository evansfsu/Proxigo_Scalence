#!/usr/bin/env python3
"""
Separate architecture-style VPS runner:
  1) Persistent optical-flow tracks (short-term motion)
  2) Retrieval + local geometric map matching (absolute correction)
  3) EKF fusion (drift control and continuity)
  4) Optional keyframe + uncertainty-gated map update mode
"""

from __future__ import annotations

import argparse
import csv as csv_mod
import json
import math
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _SCRIPT_DIR.parent
_VPS_DEVICE_PKG = _PROJECT_ROOT / "src" / "vps_device"
if str(_VPS_DEVICE_PKG) not in sys.path:
    sys.path.insert(0, str(_VPS_DEVICE_PKG))

from vps_device.ekf import EKFConfig, VPSFusionEKF
from vps_device.features import _get_descriptor_extractor, build_flann_matcher, extract_features, match_with_ratio_test
from vps_device.geo_transform import geo_to_ref_pixel, get_distance_metres, ref_pixel_to_geo
from vps_device.pnp import camera_matrix_from_fov, planar_pose_fix_from_matches, pnp_fix_with_dsm
from vps_device.reference_loader import load_proxigo_region
from vps_device.retrieval import ReferenceChipIndex, TorchResNet18Backend, default_cache_path


@dataclass
class Track:
    track_id: int
    pt: Tuple[float, float]
    age: int
    history: deque


@dataclass
class MapFix:
    success: bool
    lat: float
    lon: float
    confidence: float
    n_inliers: int
    rmse_px: float
    homography_q2r: Optional[np.ndarray]
    retrieval_score: float
    stage: str = "none"
    quality: float = 0.0
    spread: float = 0.0
    noise_m: float = 0.0


@dataclass
class KeyframeManager:
    min_motion_m: float = 25.0
    min_interval: int = 5
    max_interval: int = 18
    low_track_age: float = 3.0
    min_points: int = 40

    keyframe_gray: Optional[np.ndarray] = None
    key_pts: Optional[np.ndarray] = None
    last_rel: Optional[np.ndarray] = None
    motion_since_keyframe_m: float = 0.0
    last_keyframe_idx: int = -1
    last_keyframe_reason: str = "init"

    def _detect(self, gray: np.ndarray):
        pts = cv2.goodFeaturesToTrack(gray, maxCorners=700, qualityLevel=0.01, minDistance=8, blockSize=7)
        if pts is None:
            self.key_pts = None
        else:
            self.key_pts = pts.astype(np.float32)

    def set_keyframe(self, frame_bgr: np.ndarray, frame_idx: int, reason: str):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        self.keyframe_gray = gray
        self._detect(gray)
        self.last_rel = np.zeros(2, dtype=np.float32)
        self.motion_since_keyframe_m = 0.0
        self.last_keyframe_idx = int(frame_idx)
        self.last_keyframe_reason = reason

    def add_motion(self, dist_m: float):
        self.motion_since_keyframe_m += max(float(dist_m), 0.0)

    def should_promote(self, frame_idx: int, track_ages: List[int]) -> bool:
        if self.last_keyframe_idx < 0:
            return True
        gap = frame_idx - self.last_keyframe_idx
        if gap >= self.max_interval:
            return True
        if gap < self.min_interval:
            return False
        med_age = float(np.median(track_ages)) if track_ages else 0.0
        if self.motion_since_keyframe_m >= self.min_motion_m:
            return True
        if med_age <= self.low_track_age:
            return True
        return False

    def delta_from_keyframe(self, frame_bgr: np.ndarray) -> Tuple[Optional[float], Optional[float], float, int]:
        if self.keyframe_gray is None or self.key_pts is None or len(self.key_pts) < self.min_points:
            return None, None, 0.0, 0

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.keyframe_gray, gray, self.key_pts, None, winSize=(21, 21), maxLevel=3)
        if p1 is None or st is None:
            return None, None, 0.0, 0
        p0r, st2, _ = cv2.calcOpticalFlowPyrLK(gray, self.keyframe_gray, p1, None, winSize=(21, 21), maxLevel=3)
        if p0r is None or st2 is None:
            return None, None, 0.0, 0

        valid = (st.reshape(-1) == 1) & (st2.reshape(-1) == 1)
        fb = np.linalg.norm(self.key_pts.reshape(-1, 2) - p0r.reshape(-1, 2), axis=1)
        valid = valid & (fb < 1.8)
        n_valid = int(valid.sum())
        if n_valid < self.min_points:
            return None, None, 0.0, n_valid

        p0 = self.key_pts.reshape(-1, 2)[valid]
        p1v = p1.reshape(-1, 2)[valid]
        rel = np.array([float(np.median(p1v[:, 0] - p0[:, 0])), float(np.median(p1v[:, 1] - p0[:, 1]))], dtype=np.float32)
        if self.last_rel is None:
            delta = np.zeros(2, dtype=np.float32)
        else:
            delta = rel - self.last_rel
        self.last_rel = rel
        q = float(min(1.0, n_valid / max(len(self.key_pts), 1)))
        return float(delta[0]), float(delta[1]), q, n_valid


class PersistentFlowTracker:
    def __init__(
        self,
        max_corners: int = 600,
        quality_level: float = 0.01,
        min_distance: int = 8,
        fb_thresh_px: float = 1.5,
        min_tracks_for_motion: int = 20,
        track_history: int = 25,
    ):
        self.max_corners = int(max_corners)
        self.quality_level = float(quality_level)
        self.min_distance = int(min_distance)
        self.fb_thresh_px = float(fb_thresh_px)
        self.min_tracks_for_motion = int(min_tracks_for_motion)
        self.track_history = int(track_history)

        self.prev_gray: Optional[np.ndarray] = None
        self.tracks: Dict[int, Track] = {}
        self.next_id = 0

    def reset(self):
        self.prev_gray = None
        self.tracks.clear()

    def _seed_points(self, gray: np.ndarray):
        corners = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_corners,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance,
            blockSize=7,
            useHarrisDetector=False,
        )
        if corners is None:
            return
        for p in corners.reshape(-1, 2):
            tid = self.next_id
            self.next_id += 1
            self.tracks[tid] = Track(
                track_id=tid,
                pt=(float(p[0]), float(p[1])),
                age=1,
                history=deque([(float(p[0]), float(p[1]))], maxlen=self.track_history),
            )

    def update(self, frame_bgr: np.ndarray) -> Tuple[float, float, float, int]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        if self.prev_gray is None:
            self.prev_gray = gray
            self._seed_points(gray)
            return 0.0, 0.0, 0.0, len(self.tracks)

        if not self.tracks:
            self._seed_points(self.prev_gray)
            if not self.tracks:
                self.prev_gray = gray
                return 0.0, 0.0, 0.0, 0

        ids = list(self.tracks.keys())
        p0 = np.array([self.tracks[i].pt for i in ids], dtype=np.float32).reshape(-1, 1, 2)

        p1, st1, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, p0, None, winSize=(21, 21), maxLevel=3)
        if p1 is None or st1 is None:
            self.prev_gray = gray
            self._seed_points(gray)
            return 0.0, 0.0, 0.0, len(self.tracks)

        p0r, st2, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev_gray, p1, None, winSize=(21, 21), maxLevel=3)
        if p0r is None or st2 is None:
            self.prev_gray = gray
            return 0.0, 0.0, 0.0, len(self.tracks)

        valid = (st1.reshape(-1) == 1) & (st2.reshape(-1) == 1)
        fb = np.linalg.norm(p0.reshape(-1, 2) - p0r.reshape(-1, 2), axis=1)
        valid = valid & (fb < self.fb_thresh_px)

        motions = []
        new_tracks: Dict[int, Track] = {}
        p0_flat = p0.reshape(-1, 2)
        p1_flat = p1.reshape(-1, 2)
        for idx, tid in enumerate(ids):
            if not bool(valid[idx]):
                continue
            x0, y0 = p0_flat[idx]
            x1, y1 = p1_flat[idx]
            tr = self.tracks[tid]
            tr.pt = (float(x1), float(y1))
            tr.age += 1
            tr.history.append((float(x1), float(y1)))
            new_tracks[tid] = tr
            motions.append((float(x1 - x0), float(y1 - y0)))
        self.tracks = new_tracks

        if len(self.tracks) < self.max_corners // 2:
            mask = np.ones_like(gray, dtype=np.uint8) * 255
            for tr in self.tracks.values():
                cv2.circle(mask, (int(tr.pt[0]), int(tr.pt[1])), 7, 0, -1)
            add = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=max(0, self.max_corners - len(self.tracks)),
                qualityLevel=self.quality_level,
                minDistance=self.min_distance,
                mask=mask,
                blockSize=7,
                useHarrisDetector=False,
            )
            if add is not None:
                for p in add.reshape(-1, 2):
                    tid = self.next_id
                    self.next_id += 1
                    self.tracks[tid] = Track(
                        track_id=tid,
                        pt=(float(p[0]), float(p[1])),
                        age=1,
                        history=deque([(float(p[0]), float(p[1]))], maxlen=self.track_history),
                    )

        self.prev_gray = gray

        if len(motions) < self.min_tracks_for_motion:
            return 0.0, 0.0, 0.0, len(self.tracks)
        arr = np.array(motions, dtype=np.float32)
        med_dx = float(np.median(arr[:, 0]))
        med_dy = float(np.median(arr[:, 1]))
        quality = float(min(1.0, len(motions) / float(self.max_corners)))
        return med_dx, med_dy, quality, len(self.tracks)


def _iter_video(source_str: str):
    source = int(source_str) if source_str.isdigit() else source_str
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open source: {source_str}")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                if isinstance(source, int):
                    continue
                break
            yield frame, None
    finally:
        cap.release()


def _iter_image_dir(dir_path: Path):
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
    files = sorted([f for f in dir_path.iterdir() if f.suffix.lower() in exts])
    if not files:
        raise FileNotFoundError(f"No images in {dir_path}")
    for f in files:
        frame = cv2.imread(str(f))
        if frame is not None:
            yield frame, f.name


def _load_gt_csv(path: Optional[Path]) -> Dict[str, dict]:
    if path is None:
        return {}
    out = {}
    with open(path, "r", newline="") as f:
        r = csv_mod.DictReader(f)
        for row in r:
            name = row.get("filename", "").strip()
            if not name:
                continue
            rec = {}
            for k in ("lat", "lon", "height", "yaw"):
                if row.get(k, "").strip():
                    rec[k] = float(row[k])
            out[name] = rec
    return out


def _derive_vfov_deg(hfov_deg: float, width: int, height: int) -> float:
    hfov = math.radians(hfov_deg)
    tan_v = math.tan(hfov / 2.0) * (height / max(width, 1))
    return math.degrees(2.0 * math.atan(tan_v))


def _flow_px_to_m(dx_px: float, dy_px: float, altitude_m: float, width: int, height: int, hfov_deg: float):
    vfov_deg = _derive_vfov_deg(hfov_deg, width, height)
    fov_x_m = 2.0 * altitude_m * math.tan(math.radians(hfov_deg) * 0.5)
    fov_y_m = 2.0 * altitude_m * math.tan(math.radians(vfov_deg) * 0.5)
    cam_dx_m = -dx_px * (fov_x_m / max(width, 1))
    cam_dy_m = -dy_px * (fov_y_m / max(height, 1))
    return cam_dx_m, cam_dy_m


def _rotate_en(east_m: float, north_m: float, yaw_deg: float) -> Tuple[float, float]:
    th = math.radians(yaw_deg)
    e2 = math.cos(th) * east_m + math.sin(th) * north_m
    n2 = -math.sin(th) * east_m + math.cos(th) * north_m
    return e2, n2


def _match_spread_score(ref_pts: np.ndarray, chip_w: int, chip_h: int) -> float:
    if ref_pts.shape[0] < 4:
        return 0.0
    min_xy = np.min(ref_pts, axis=0)
    max_xy = np.max(ref_pts, axis=0)
    area = max(0.0, float(max_xy[0] - min_xy[0])) * max(0.0, float(max_xy[1] - min_xy[1]))
    denom = max(1.0, float(chip_w * chip_h))
    return float(np.clip(area / denom, 0.0, 1.0))


def _quality_to_noise_m(map_fix: MapFix, base_noise_m: float) -> float:
    q = float(np.clip(map_fix.quality, 0.0, 1.0))
    scale = 2.2 - 1.8 * q
    return float(np.clip(base_noise_m * scale, 4.0, base_noise_m * 3.0))


def _local_match_and_fix(
    frame_bgr: np.ndarray,
    ref_gray: np.ndarray,
    region,
    index: ReferenceChipIndex,
    dsm_m: Optional[np.ndarray],
    K: np.ndarray,
    topk: int,
    ratio: float,
    min_matches: int,
    min_inliers: int,
    nfeatures: int,
    prior_latlon: Optional[Tuple[float, float]],
    prior_gate_m: float,
) -> Tuple[MapFix, Dict[str, int], List[Tuple[float, float, bool]]]:
    results = index.query(frame_bgr, topk=topk)
    diag = {
        "retrieval_candidates": len(results),
        "retrieval_gated": 0,
        "match_candidates": 0,
        "pnp_candidates": 0,
        "prior_rejects": 0,
    }
    candidate_pts: List[Tuple[float, float, bool]] = []
    if not results:
        return MapFix(False, 0.0, 0.0, 0.0, 0, 1e9, None, 0.0, stage="no_retrieval"), diag, candidate_pts

    detector, desc_ext = _get_descriptor_extractor(False, False, nfeatures)
    matcher = build_flann_matcher(False)
    q_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    q_kp, q_des = extract_features(q_gray, detector, desc_ext, nfeatures=nfeatures)
    if q_des is None or len(q_kp) < 10:
        return MapFix(False, 0.0, 0.0, 0.0, 0, 1e9, None, 0.0, stage="match_fail"), diag, candidate_pts

    best = None
    best_score = None
    ref_mpp = float((region.x_m_per_px + region.y_m_per_px) * 0.5)
    for cand in results:
        chip = cand.chip
        chip_lat, chip_lon = ref_pixel_to_geo(
            chip.center_px[0], chip.center_px[1],
            region.center_lat, region.center_lon,
            region.height_px, region.width_px,
            region.y_m_per_px, region.x_m_per_px,
        )
        gate_ok = True
        if prior_latlon is not None and prior_gate_m > 0:
            d = get_distance_metres(prior_latlon[0], prior_latlon[1], chip_lat, chip_lon)
            if d > prior_gate_m:
                gate_ok = False
                diag["retrieval_gated"] += 1
        candidate_pts.append((chip.center_px[0], chip.center_px[1], gate_ok))
        if not gate_ok:
            continue

        chip_gray = ref_gray[chip.y0 : chip.y0 + chip.h, chip.x0 : chip.x0 + chip.w]
        r_kp, r_des = extract_features(chip_gray, detector, desc_ext, nfeatures=nfeatures)
        if r_des is None or len(r_kp) < 10:
            continue
        matches = match_with_ratio_test(r_des, q_des, matcher, ratio=ratio)
        if len(matches) < min_matches:
            continue
        diag["match_candidates"] += 1

        ref_pts_local = np.float32([r_kp[m.queryIdx].pt for m in matches])
        qry_pts = np.float32([q_kp[m.trainIdx].pt for m in matches])
        ref_pts_full = np.column_stack([ref_pts_local[:, 0] + chip.x0, ref_pts_local[:, 1] + chip.y0]).astype(np.float32)

        H_q2r, mask = cv2.findHomography(qry_pts, ref_pts_full, cv2.RANSAC, ransacReprojThreshold=4.0)
        if H_q2r is None or mask is None:
            continue
        h_inliers = int(mask.ravel().sum())
        if h_inliers < min_inliers:
            continue

        if dsm_m is not None:
            fix = pnp_fix_with_dsm(
                ref_pts_full.tolist(),
                qry_pts.tolist(),
                K=K,
                dsm_m=dsm_m,
                ref_center_lat=region.center_lat,
                ref_center_lon=region.center_lon,
                ref_width_px=region.width_px,
                ref_height_px=region.height_px,
                ref_m_per_px=ref_mpp,
            )
        else:
            fix = planar_pose_fix_from_matches(
                ref_pts_full.tolist(),
                qry_pts.tolist(),
                K=K,
                ref_center_lat=region.center_lat,
                ref_center_lon=region.center_lon,
                ref_width_px=region.width_px,
                ref_height_px=region.height_px,
                ref_m_per_px=ref_mpp,
            )
        if not fix.success or fix.n_inliers < min_inliers:
            continue
        diag["pnp_candidates"] += 1

        if prior_latlon is not None and prior_gate_m > 0:
            d_fix = get_distance_metres(prior_latlon[0], prior_latlon[1], fix.lat, fix.lon)
            if d_fix > prior_gate_m:
                diag["prior_rejects"] += 1
                continue

        spread = _match_spread_score(ref_pts_full, chip.w, chip.h)
        rmse_score = float(np.clip(1.0 / (1.0 + (fix.reproj_rmse_px / 3.5)), 0.0, 1.0))
        inlier_score = float(np.clip(fix.n_inliers / 40.0, 0.0, 1.0))
        retrieval_score = float(np.clip((cand.score + 1.0) * 0.5, 0.0, 1.0))
        quality = 0.40 * inlier_score + 0.30 * rmse_score + 0.20 * spread + 0.10 * retrieval_score

        score = (int(fix.n_inliers), float(quality), -float(fix.reproj_rmse_px), float(cand.score))
        if best_score is None or score > best_score:
            best_score = score
            best = (fix, H_q2r, cand.score, quality, spread)

    if best is None:
        if diag["retrieval_candidates"] > 0 and diag["retrieval_gated"] == diag["retrieval_candidates"]:
            stage = "retrieval_gated"
        elif diag["match_candidates"] == 0:
            stage = "match_fail"
        elif diag["pnp_candidates"] == 0:
            stage = "pnp_fail"
        elif diag["prior_rejects"] > 0:
            stage = "prior_reject"
        else:
            stage = "pnp_fail"
        return MapFix(False, 0.0, 0.0, 0.0, 0, 1e9, None, 0.0, stage=stage), diag, candidate_pts

    fix, H, ret_score, quality, spread = best
    return MapFix(
        success=True,
        lat=fix.lat,
        lon=fix.lon,
        confidence=fix.confidence,
        n_inliers=fix.n_inliers,
        rmse_px=fix.reproj_rmse_px,
        homography_q2r=H,
        retrieval_score=float(ret_score),
        stage="map_success",
        quality=float(quality),
        spread=float(spread),
    ), diag, candidate_pts


def _draw_tracks(frame: np.ndarray, tracks: Dict[int, Track]) -> np.ndarray:
    vis = frame.copy()
    for tr in tracks.values():
        color = (0, min(255, 30 + tr.age * 4), 255 - min(180, tr.age * 2))
        pts = list(tr.history)
        for i in range(1, len(pts)):
            cv2.line(vis, (int(pts[i - 1][0]), int(pts[i - 1][1])), (int(pts[i][0]), int(pts[i][1])), color, 1, cv2.LINE_AA)
        cv2.circle(vis, (int(tr.pt[0]), int(tr.pt[1])), 2, color, -1)
    return vis


def _build_map_panel(
    ref_gray: np.ndarray,
    trail_px: List[Tuple[float, float]],
    map_tracks_px: List[Tuple[float, float]],
    prior_center_px: Optional[Tuple[float, float]] = None,
    prior_gate_px: Optional[float] = None,
    candidate_pts: Optional[List[Tuple[float, float, bool]]] = None,
    last_fix_px: Optional[Tuple[float, float]] = None,
) -> np.ndarray:
    panel = cv2.cvtColor(ref_gray, cv2.COLOR_GRAY2BGR)
    for i in range(1, len(trail_px)):
        cv2.line(panel, (int(trail_px[i - 1][0]), int(trail_px[i - 1][1])), (int(trail_px[i][0]), int(trail_px[i][1])), (0, 200, 255), 2)
    if trail_px:
        cv2.circle(panel, (int(trail_px[-1][0]), int(trail_px[-1][1])), 7, (0, 255, 0), 2)

    if prior_center_px is not None and prior_gate_px is not None and prior_gate_px > 1:
        cv2.circle(panel, (int(prior_center_px[0]), int(prior_center_px[1])), int(prior_gate_px), (255, 180, 0), 1)

    if candidate_pts:
        for px, py, ok in candidate_pts:
            color = (0, 220, 0) if ok else (40, 40, 220)
            cv2.circle(panel, (int(px), int(py)), 3, color, -1)

    if last_fix_px is not None:
        cv2.drawMarker(panel, (int(last_fix_px[0]), int(last_fix_px[1])), (255, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)

    for p in map_tracks_px:
        cv2.circle(panel, (int(p[0]), int(p[1])), 1, (255, 80, 80), -1)
    return panel


def run(args):
    region = load_proxigo_region(Path(args.reference))
    ref_gray = region.image
    ref_bgr = cv2.cvtColor(ref_gray, cv2.COLOR_GRAY2BGR)
    gt = _load_gt_csv(Path(args.source_csv)) if args.source_csv else {}

    dsm_m = None
    if args.dsm_path:
        dsm = cv2.imread(str(Path(args.dsm_path)), cv2.IMREAD_UNCHANGED)
        if dsm is not None:
            dsm_m = dsm.astype(np.float32)

    backend = TorchResNet18Backend(device=args.retrieval_device)
    cache_path = default_cache_path(Path(args.cache_dir), Path(args.reference) / "satellite.png", backend, args.chip_size, args.chip_stride)
    if cache_path.exists() and not args.rebuild_index:
        index = ReferenceChipIndex.load(cache_path, ref_bgr=ref_bgr, backend=backend)
    else:
        index = ReferenceChipIndex(ref_bgr=ref_bgr, backend=backend, chip_size=args.chip_size, stride=args.chip_stride)
        index.build()
        index.save(cache_path)

    flow = PersistentFlowTracker(max_corners=args.max_tracks)
    keyframe = KeyframeManager(
        min_motion_m=args.keyframe_min_motion_m,
        min_interval=args.keyframe_min_interval,
        max_interval=args.keyframe_max_interval,
        low_track_age=args.keyframe_low_track_age,
        min_points=args.keyframe_min_points,
    )
    ekf = VPSFusionEKF(EKFConfig(map_position_noise=args.map_noise))
    K = camera_matrix_from_fov(args.width, args.height, args.fov_h)
    save_debug_dir = Path(args.save_debug_dir).resolve() if args.save_debug_dir else None
    if save_debug_dir is not None:
        save_debug_dir.mkdir(parents=True, exist_ok=True)
    saved_debug = 0

    trail_px: List[Tuple[float, float]] = []
    last_H_q2r = None
    last_fix_px = None
    t0 = time.monotonic()
    frame_idx = 0
    accepted = 0
    nerr = 0
    sum_err = 0.0
    map_lock_age = 0

    counters = {
        "map_attempted": 0,
        "accepted": 0,
        "innovation_reject": 0,
        "no_retrieval": 0,
        "retrieval_gated": 0,
        "match_fail": 0,
        "pnp_fail": 0,
        "prior_reject": 0,
    }

    csv_file = open(args.output_csv, "w", newline="", encoding="utf-8") if args.output_csv else None
    writer = csv_mod.writer(csv_file) if csv_file else None
    if writer:
        writer.writerow([
            "frame", "filename", "fused_lat", "fused_lon", "flow_q", "keyframe_q", "track_count",
            "map_stage", "map_success", "map_inliers", "map_conf", "map_rmse", "map_quality", "map_noise_m",
            "prior_gate_m", "retrieval_candidates", "retrieval_gated", "match_candidates", "pnp_candidates",
            "prior_rejects", "accepted", "error_m"
        ])

    iterator: Iterable[Tuple[np.ndarray, Optional[str]]]
    if args.source_dir:
        iterator = _iter_image_dir(Path(args.source_dir))
    else:
        iterator = _iter_video(args.source)

    try:
        for frame, fname in iterator:
            if args.max_frames is not None and frame_idx >= args.max_frames:
                break
            if frame.shape[1] != args.width or frame.shape[0] != args.height:
                frame = cv2.resize(frame, (args.width, args.height), interpolation=cv2.INTER_LINEAR)

            gt_row = gt.get(fname, {}) if fname else {}
            altitude = float(gt_row.get("height", args.altitude))
            yaw = gt_row.get("yaw", None)

            if args.enable_keyframe_mode and keyframe.keyframe_gray is None:
                keyframe.set_keyframe(frame, frame_idx, reason="init")

            if not ekf.initialized and "lat" in gt_row and "lon" in gt_row:
                ekf.initialize(float(gt_row["lat"]), float(gt_row["lon"]))

            dx_px, dy_px, flow_q, track_count = flow.update(frame)
            kdx, kdy, kf_q, _ = (None, None, 0.0, 0)
            if args.enable_keyframe_mode:
                kdx, kdy, kf_q, _ = keyframe.delta_from_keyframe(frame)

            use_dx, use_dy, use_q = dx_px, dy_px, flow_q
            if args.enable_keyframe_mode and kdx is not None and kdy is not None and flow_q < args.keyframe_fallback_q and kf_q > 0:
                use_dx, use_dy, use_q = kdx, kdy, max(flow_q, kf_q)

            if ekf.initialized and use_q > 0:
                e_m, n_m = _flow_px_to_m(use_dx, use_dy, altitude, args.width, args.height, args.fov_h)
                if yaw is not None:
                    e_m, n_m = _rotate_en(e_m, n_m, float(yaw))
                ekf.predict_with_vo(e_m, n_m, quality=use_q, altitude_m=altitude)
                if args.enable_keyframe_mode:
                    keyframe.add_motion(float(math.hypot(e_m, n_m)))
            elif ekf.initialized:
                ekf.predict_constant_velocity()

            map_fix = MapFix(False, 0.0, 0.0, 0.0, 0, 1e9, None, 0.0, stage="skip")
            update_ok = False
            stage = "skip"
            diag = {"retrieval_candidates": 0, "retrieval_gated": 0, "match_candidates": 0, "pnp_candidates": 0, "prior_rejects": 0}
            candidate_pts = []
            prior_gate_m = 0.0
            prior_center_px = None
            prior_gate_px = None

            if frame_idx % args.map_match_every == 0:
                counters["map_attempted"] += 1
                prior_latlon = None
                if ekf.initialized and args.prior_gate_sigma > 0:
                    std_e, std_n = ekf.position_std
                    prior_gate_m = args.prior_gate_sigma * max(std_e, std_n)
                    prior_gate_m = float(np.clip(prior_gate_m, args.prior_gate_min_m, args.prior_gate_max_m))
                    prior_latlon = ekf.position
                    px_c, py_c = geo_to_ref_pixel(
                        prior_latlon[0], prior_latlon[1],
                        region.center_lat, region.center_lon,
                        region.height_px, region.width_px,
                        region.y_m_per_px, region.x_m_per_px,
                    )
                    prior_center_px = (px_c, py_c)
                    prior_gate_px = prior_gate_m / max(1e-6, (region.x_m_per_px + region.y_m_per_px) * 0.5)

                map_fix, diag, candidate_pts = _local_match_and_fix(
                    frame_bgr=frame,
                    ref_gray=ref_gray,
                    region=region,
                    index=index,
                    dsm_m=dsm_m,
                    K=K,
                    topk=args.topk,
                    ratio=args.ratio,
                    min_matches=args.min_matches,
                    min_inliers=args.min_inliers,
                    nfeatures=args.nfeatures,
                    prior_latlon=prior_latlon,
                    prior_gate_m=prior_gate_m,
                )
                stage = map_fix.stage
                if map_fix.success:
                    map_fix.noise_m = _quality_to_noise_m(map_fix, args.map_noise)
                    if not ekf.initialized:
                        ekf.initialize(map_fix.lat, map_fix.lon)
                        update_ok = True
                    else:
                        update_ok = ekf.update_map_match_with_noise(map_fix.lat, map_fix.lon, noise_m=map_fix.noise_m)
                    if update_ok:
                        accepted += 1
                        stage = "accepted"
                        last_H_q2r = map_fix.homography_q2r
                        last_fix_px = geo_to_ref_pixel(
                            map_fix.lat, map_fix.lon,
                            region.center_lat, region.center_lon,
                            region.height_px, region.width_px,
                            region.y_m_per_px, region.x_m_per_px,
                        )
                        map_lock_age = 0
                        if args.enable_keyframe_mode:
                            keyframe.set_keyframe(frame, frame_idx, reason="map_accept")
                    else:
                        stage = "innovation_reject"

                if stage in counters:
                    counters[stage] += 1

            fused = ekf.position if ekf.initialized else None
            err_m = None
            if fused and "lat" in gt_row and "lon" in gt_row:
                err_m = get_distance_metres(float(gt_row["lat"]), float(gt_row["lon"]), fused[0], fused[1])
                nerr += 1
                sum_err += float(err_m)

            if fused:
                px, py = geo_to_ref_pixel(
                    fused[0], fused[1],
                    region.center_lat, region.center_lon,
                    region.height_px, region.width_px,
                    region.y_m_per_px, region.x_m_per_px,
                )
                trail_px.append((px, py))
                if len(trail_px) > 1500:
                    trail_px.pop(0)

            if args.enable_keyframe_mode and keyframe.should_promote(frame_idx, [t.age for t in flow.tracks.values()]):
                keyframe.set_keyframe(frame, frame_idx, reason="motion_or_age")

            map_track_pts = []
            if last_H_q2r is not None and len(flow.tracks) > 0:
                qpts = np.array([tr.pt for tr in flow.tracks.values()], dtype=np.float32).reshape(-1, 1, 2)
                try:
                    rpt = cv2.perspectiveTransform(qpts, last_H_q2r).reshape(-1, 2)
                    for p in rpt:
                        if 0 <= p[0] < region.width_px and 0 <= p[1] < region.height_px:
                            map_track_pts.append((float(p[0]), float(p[1])))
                except cv2.error:
                    pass

            if writer:
                writer.writerow([
                    frame_idx, fname or "", f"{fused[0]:.8f}" if fused else "", f"{fused[1]:.8f}" if fused else "",
                    f"{use_q:.4f}", f"{kf_q:.4f}", int(track_count),
                    stage, int(map_fix.success), int(map_fix.n_inliers), f"{map_fix.confidence:.4f}", f"{map_fix.rmse_px:.2f}",
                    f"{map_fix.quality:.4f}", f"{map_fix.noise_m:.2f}", f"{prior_gate_m:.2f}",
                    int(diag["retrieval_candidates"]), int(diag["retrieval_gated"]), int(diag["match_candidates"]),
                    int(diag["pnp_candidates"]), int(diag["prior_rejects"]),
                    int(update_ok), f"{err_m:.2f}" if err_m is not None else "",
                ])

            need_merged = (not args.headless) or (save_debug_dir is not None)
            merged = None
            if need_merged:
                cam_vis = _draw_tracks(frame, flow.tracks)
                map_vis = _build_map_panel(
                    ref_gray=ref_gray,
                    trail_px=trail_px,
                    map_tracks_px=map_track_pts,
                    prior_center_px=prior_center_px,
                    prior_gate_px=prior_gate_px,
                    candidate_pts=candidate_pts,
                    last_fix_px=last_fix_px,
                )
                map_vis = cv2.resize(map_vis, (cam_vis.shape[1], cam_vis.shape[0]), interpolation=cv2.INTER_LINEAR)
                merged = np.hstack([cam_vis, map_vis])
                status = (
                    f"#{frame_idx} tracks={track_count} q={use_q:.2f} map={stage} "
                    f"upd={int(update_ok)} acc={accepted} kf={keyframe.last_keyframe_reason}"
                )
                if err_m is not None:
                    status += f" err={err_m:.1f}m"
                cv2.putText(merged, status, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 0), 2, cv2.LINE_AA)

            if save_debug_dir is not None and merged is not None:
                if (frame_idx % max(1, args.save_debug_every) == 0) and saved_debug < args.save_debug_max:
                    out_path = save_debug_dir / f"frame_{frame_idx:05d}.png"
                    ok = bool(cv2.imwrite(str(out_path), merged))
                    if ok:
                        saved_debug += 1
                        print(f"Saved debug frame: {out_path}")
                    else:
                        print(f"WARNING: Failed to save debug frame: {out_path}")

            if not args.headless and merged is not None:
                cv2.imshow("VPS VNav-like (camera | map)", merged)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break

            frame_idx += 1
            map_lock_age += 1
    finally:
        if csv_file:
            csv_file.close()
        if not args.headless:
            cv2.destroyAllWindows()

    dt = max(1e-6, time.monotonic() - t0)
    print(f"Processed {frame_idx} frames in {dt:.1f}s ({frame_idx / dt:.2f} fps)")
    print(f"Accepted map updates: {accepted}")
    if nerr > 0:
        print(f"Mean error: {sum_err / nerr:.2f} m  (n={nerr})")
    print("Stage diagnostics:")
    for k, v in counters.items():
        print(f"  {k}: {v}")
    if args.diagnostics_json:
        out = Path(args.diagnostics_json).resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(counters, f, indent=2)
        print(f"Diagnostics JSON: {out}")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Separate architecture-style VPS runner.")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--source", type=str, help="Video file path or camera index.")
    src.add_argument("--source-dir", type=str, help="Directory of image frames.")
    p.add_argument("--source-csv", type=str, default=None, help="Optional GT CSV with filename,lat,lon,height,yaw.")

    p.add_argument("--reference", required=True, help="Reference region directory (satellite.png + metadata.json).")
    p.add_argument("--dsm-path", default=None, help="Optional DSM tif for true PnP.")
    p.add_argument("--output-csv", default="results_vnav_like.csv")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--max-frames", type=int, default=None)
    p.add_argument("--diagnostics-json", type=str, default=None, help="Optional diagnostics JSON path.")

    p.add_argument("--altitude", type=float, default=120.0)
    p.add_argument("--width", type=int, default=1920)
    p.add_argument("--height", type=int, default=1080)
    p.add_argument("--fov-h", type=float, default=63.0)

    p.add_argument("--map-match-every", type=int, default=5)
    p.add_argument("--map-noise", type=float, default=30.0)
    p.add_argument("--min-matches", type=int, default=14)
    p.add_argument("--min-inliers", type=int, default=10)

    p.add_argument("--max-tracks", type=int, default=600)
    p.add_argument("--nfeatures", type=int, default=2000)
    p.add_argument("--ratio", type=float, default=0.85)

    p.add_argument("--topk", type=int, default=5)
    p.add_argument("--chip-size", type=int, default=768)
    p.add_argument("--chip-stride", type=int, default=512)
    p.add_argument("--cache-dir", type=str, default="test_data/cache")
    p.add_argument("--retrieval-device", type=str, default="cpu")
    p.add_argument("--rebuild-index", action="store_true")
    p.add_argument("--save-debug-dir", type=str, default=None, help="Optional directory to save merged camera+map debug frames.")
    p.add_argument("--save-debug-every", type=int, default=10, help="Save one debug frame every N frames.")
    p.add_argument("--save-debug-max", type=int, default=8, help="Maximum number of debug frames to save.")

    p.add_argument("--enable-keyframe-mode", action="store_true", help="Enable keyframe lifecycle and keyframe flow fallback.")
    p.add_argument("--keyframe-min-motion-m", type=float, default=25.0)
    p.add_argument("--keyframe-min-interval", type=int, default=5)
    p.add_argument("--keyframe-max-interval", type=int, default=18)
    p.add_argument("--keyframe-low-track-age", type=float, default=3.0)
    p.add_argument("--keyframe-min-points", type=int, default=40)
    p.add_argument("--keyframe-fallback-q", type=float, default=0.12, help="Use keyframe delta when frame-to-frame quality is below this.")

    p.add_argument("--prior-gate-sigma", type=float, default=0.0, help="EKF-std multiplier for retrieval prior gate (0 disables).")
    p.add_argument("--prior-gate-min-m", type=float, default=60.0)
    p.add_argument("--prior-gate-max-m", type=float, default=400.0)
    return p


def main():
    parser = build_parser()
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()

