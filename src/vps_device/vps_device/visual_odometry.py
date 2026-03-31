"""
Visual Odometry -- frame-to-frame feature matching for ground displacement.

Uses ORB feature matching + RANSAC between consecutive frames to estimate
camera displacement in the image plane.  At known altitude, pixel displacement
maps to ground-plane distance (flat-earth, nadir camera assumption).

Without compass/IMU data the image axes have unknown orientation relative to
geographic north.  The caller is responsible for rotating the returned
(dx, dy) from image-frame to geographic-frame once a heading estimate is
available (see `HeadingCalibrator` below).
"""

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from .config import VPSDeviceConfig


@dataclass
class VOResult:
    """Single-frame visual odometry output (image-frame metres)."""
    dx: float          # displacement in image-x (right-positive) in metres
    dy: float          # displacement in image-y (down-positive) in metres
    dt: float          # seconds since last frame
    n_inliers: int     # number of RANSAC inlier matches
    quality: float     # 0-1, inlier ratio
    dist: float        # total displacement magnitude in metres


class VisualOdometry:
    """Camera-only visual odometry via feature matching between consecutive frames."""

    _OUTLIER_RATIO = 0.15  # reject if displacement < 15% of running average

    def __init__(self, config: VPSDeviceConfig, nfeatures: int = 500):
        self.config = config
        self._orb = cv2.ORB_create(nfeatures=nfeatures)
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_kpts = None
        self._prev_des = None
        self._prev_time: Optional[float] = None
        self._recent_dists: list = []  # running window of displacement magnitudes

    def reset(self):
        """Clear state (e.g. after a scene cut or large gap)."""
        self._prev_gray = None
        self._prev_kpts = None
        self._prev_des = None
        self._prev_time = None

    _SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    _SUBPIX_WIN = (5, 5)

    def _refine_keypoints(self, kpts, gray: np.ndarray):
        """Refine keypoint locations to sub-pixel accuracy."""
        if not kpts:
            return kpts
        corners = np.float32([k.pt for k in kpts]).reshape(-1, 1, 2)
        refined = cv2.cornerSubPix(gray, corners, self._SUBPIX_WIN, (-1, -1),
                                   self._SUBPIX_CRITERIA)
        for i, kp in enumerate(kpts):
            kp.pt = (float(refined[i, 0, 0]), float(refined[i, 0, 1]))
        return kpts

    def process_frame(
        self,
        frame: np.ndarray,
        altitude_m: float,
        timestamp: Optional[float] = None,
    ) -> Optional[VOResult]:
        """Compute ground displacement from feature matching with the previous frame.

        Returns displacement in IMAGE-FRAME metres (x-right, y-down).
        Returns None on the very first frame.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        now = timestamp if timestamp is not None else time.monotonic()

        kpts = self._orb.detect(gray, None)
        kpts = self._refine_keypoints(kpts, gray)
        kpts, des = self._orb.compute(gray, kpts)

        if des is None or len(kpts) < 10:
            self._prev_gray = gray
            self._prev_kpts = kpts
            self._prev_des = des
            self._prev_time = now
            return None

        if self._prev_des is None or self._prev_kpts is None:
            self._prev_gray = gray
            self._prev_kpts = kpts
            self._prev_des = des
            self._prev_time = now
            return None

        dt = now - self._prev_time
        if dt <= 0:
            dt = 1e-6

        raw_matches = self._bf.knnMatch(self._prev_des, des, k=2)
        good = []
        for pair in raw_matches:
            if len(pair) == 2 and pair[0].distance < 0.8 * pair[1].distance:
                good.append(pair[0])

        n_good = len(good)
        if n_good < 8:
            self._prev_gray = gray
            self._prev_kpts = kpts
            self._prev_des = des
            self._prev_time = now
            return VOResult(dx=0, dy=0, dt=dt, n_inliers=n_good, quality=0, dist=0)

        src_pts = np.float32([self._prev_kpts[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kpts[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        _, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,
                                     ransacReprojThreshold=5.0)
        if mask is None:
            self._prev_gray = gray
            self._prev_kpts = kpts
            self._prev_des = des
            self._prev_time = now
            return VOResult(dx=0, dy=0, dt=dt, n_inliers=0, quality=0, dist=0)

        inlier_mask = mask.ravel().astype(bool)
        n_inliers = int(inlier_mask.sum())

        if n_inliers < 5:
            self._prev_gray = gray
            self._prev_kpts = kpts
            self._prev_des = des
            self._prev_time = now
            return VOResult(dx=0, dy=0, dt=dt, n_inliers=n_inliers, quality=0, dist=0)

        src_inl = src_pts[inlier_mask].reshape(-1, 2)
        dst_inl = dst_pts[inlier_mask].reshape(-1, 2)
        displacements = dst_inl - src_inl
        tx_px = float(np.median(displacements[:, 0]))
        ty_px = float(np.median(displacements[:, 1]))

        # Convert pixel displacement to ground metres using actual frame dimensions
        h_frame, w_frame = gray.shape[:2]
        fov_x_m = 2 * altitude_m * math.tan(math.radians(self.config.h_fov_deg) / 2)
        # Derive vertical FOV from horizontal FOV and aspect ratio
        v_fov_rad = 2 * math.atan(h_frame / w_frame *
                                   math.tan(math.radians(self.config.h_fov_deg) / 2))
        fov_y_m = 2 * altitude_m * math.tan(v_fov_rad / 2)
        x_m_per_px = fov_x_m / w_frame
        y_m_per_px = fov_y_m / h_frame

        # Feature displacement = opposite of camera displacement
        cam_dx = -tx_px * x_m_per_px  # image-right metres (camera frame)
        cam_dy = -ty_px * y_m_per_px  # image-up metres (camera frame, y-inverted)

        quality = n_inliers / max(n_good, 1)
        dist = math.sqrt(cam_dx ** 2 + cam_dy ** 2)

        # Outlier rejection: if this displacement is suspiciously small
        # compared to the running average, mark it as low quality.
        if self._recent_dists:
            avg_dist = sum(self._recent_dists) / len(self._recent_dists)
            if avg_dist > 1.0 and dist < avg_dist * self._OUTLIER_RATIO:
                quality = 0.0
                cam_dx = 0.0
                cam_dy = 0.0
                dist = 0.0

        if dist > 0.5:
            self._recent_dists.append(dist)
            if len(self._recent_dists) > 10:
                self._recent_dists.pop(0)

        self._prev_gray = gray
        self._prev_kpts = kpts
        self._prev_des = des
        self._prev_time = now

        return VOResult(dx=cam_dx, dy=cam_dy, dt=dt,
                        n_inliers=n_inliers, quality=quality, dist=dist)


class HeadingCalibrator:
    """Estimates the rotation from image-frame to geographic-frame.

    Supports two modes:

    1. **Fixed heading** (no per-frame yaw): accumulates position fixes and VO
       displacement, computes a single rotation + scale.  Works when the drone
       maintains roughly constant heading.

    2. **Per-frame yaw** (``transform_with_yaw``): the caller supplies the
       drone's yaw each frame.  Calibration finds only the *camera mount
       offset* (fixed angle between image-up and drone-forward) and scale.
       This handles survey flights with large heading changes.

    Once calibrated, ``transform(dx, dy)`` or ``transform_with_yaw(dx, dy,
    yaw_deg)`` returns ``(east_m, north_m)``.
    """

    def __init__(self, min_displacement_m: float = 10.0, min_fixes: int = 3):
        self._min_disp = min_displacement_m
        self._min_fixes = max(2, min_fixes)
        self._fixes: list = []       # [(lat, lon)]
        self._yaws: list = []        # yaw (degrees) at each fix (may be empty)
        self._vo_accum = [0.0, 0.0]  # accumulated (dx, dy) in image frame
        self._rotation = 0.0         # radians, image-frame → geographic
        self._scale = 1.0            # scale correction for FOV mismatch
        self._calibrated = False
        self._mount_offset = 0.0     # camera mount offset (radians)
        self._has_yaw = False        # True when calibrated with per-frame yaw

    @property
    def calibrated(self) -> bool:
        return self._calibrated

    @property
    def heading_deg(self) -> float:
        """Estimated image-up direction in degrees from geographic north, clockwise."""
        return math.degrees(self._rotation) % 360

    def add_fix(self, lat: float, lon: float, yaw_deg: Optional[float] = None):
        """Record a position fix (from map match or ground truth)."""
        self._fixes.append((lat, lon))
        if yaw_deg is not None:
            self._yaws.append(yaw_deg)

    def accumulate_vo(self, dx: float, dy: float):
        """Add VO displacement (image-frame metres) between fixes."""
        self._vo_accum[0] += dx
        self._vo_accum[1] += dy

    def try_calibrate(self) -> bool:
        """Attempt calibration from accumulated fixes and VO vectors.

        If yaw was supplied with every fix, calibrates the camera mount offset
        and scale (per-frame yaw mode).  Otherwise calibrates fixed heading.
        """
        if self._calibrated or len(self._fixes) < self._min_fixes:
            return False

        lat0, lon0 = self._fixes[0]
        lat1, lon1 = self._fixes[-1]

        m_per_deg_lat = 111320.0
        m_per_deg_lon = m_per_deg_lat * math.cos(math.radians((lat0 + lat1) / 2))
        geo_east = (lon1 - lon0) * m_per_deg_lon
        geo_north = (lat1 - lat0) * m_per_deg_lat
        geo_dist = math.sqrt(geo_east ** 2 + geo_north ** 2)

        if geo_dist < self._min_disp:
            return False

        vo_dx, vo_dy = self._vo_accum
        vo_dist = math.sqrt(vo_dx ** 2 + vo_dy ** 2)
        if vo_dist < self._min_disp:
            return False

        cross = vo_dx * geo_north - vo_dy * geo_east
        dot = vo_dx * geo_east + vo_dy * geo_north
        self._rotation = math.atan2(cross, dot)
        self._scale = geo_dist / vo_dist

        # Per-frame yaw mode: extract camera mount offset
        if len(self._yaws) == len(self._fixes):
            avg_yaw_rad = math.radians(sum(self._yaws) / len(self._yaws))
            self._mount_offset = self._rotation - avg_yaw_rad
            self._has_yaw = True

        self._calibrated = True
        return True

    def transform(self, dx: float, dy: float) -> Tuple[float, float]:
        """Rotate + scale image-frame displacement to geographic (east, north).

        Uses the fixed heading calibrated at startup.  For flights with
        heading changes, prefer ``transform_with_yaw``.
        """
        if not self._calibrated:
            return dx, dy
        cos_r = math.cos(self._rotation)
        sin_r = math.sin(self._rotation)
        s = self._scale
        east = s * (dx * cos_r - dy * sin_r)
        north = s * (dx * sin_r + dy * cos_r)
        return east, north

    def transform_with_yaw(self, dx: float, dy: float,
                           yaw_deg: float) -> Tuple[float, float]:
        """Rotate + scale using per-frame yaw + calibrated mount offset."""
        if not self._calibrated:
            return dx, dy
        r = math.radians(yaw_deg) + self._mount_offset
        cos_r = math.cos(r)
        sin_r = math.sin(r)
        s = self._scale
        east = s * (dx * cos_r - dy * sin_r)
        north = s * (dx * sin_r + dy * cos_r)
        return east, north

    # --- Online recalibration from map matches ---

    def begin_recalibration_segment(self, lat: float, lon: float):
        """Start a new recalibration segment at a trusted position fix."""
        self._recal_fix = (lat, lon)
        self._recal_vo = [0.0, 0.0]

    def accumulate_recal_vo(self, dx: float, dy: float):
        """Accumulate VO displacement during a recalibration segment."""
        if hasattr(self, '_recal_fix') and self._recal_fix is not None:
            self._recal_vo[0] += dx
            self._recal_vo[1] += dy

    def try_recalibrate(self, lat: float, lon: float,
                        alpha: float = 0.15, min_disp_m: float = 5.0) -> bool:
        """Refine heading/scale from a new trusted fix using exponential smoothing.

        Returns True if refinement was applied.
        """
        if not self._calibrated:
            return False
        if not hasattr(self, '_recal_fix') or self._recal_fix is None:
            return False

        lat0, lon0 = self._recal_fix
        m_per_deg_lat = 111320.0
        m_per_deg_lon = m_per_deg_lat * math.cos(math.radians((lat0 + lat) / 2))
        geo_east = (lon - lon0) * m_per_deg_lon
        geo_north = (lat - lat0) * m_per_deg_lat
        geo_dist = math.sqrt(geo_east ** 2 + geo_north ** 2)

        if geo_dist < min_disp_m:
            return False

        vo_dx, vo_dy = self._recal_vo
        vo_dist = math.sqrt(vo_dx ** 2 + vo_dy ** 2)
        if vo_dist < min_disp_m:
            return False

        cross = vo_dx * geo_north - vo_dy * geo_east
        dot = vo_dx * geo_east + vo_dy * geo_north
        new_rotation = math.atan2(cross, dot)
        new_scale = geo_dist / vo_dist

        # Exponential smoothing so single bad matches can't ruin calibration
        self._rotation = (1 - alpha) * self._rotation + alpha * new_rotation
        self._scale = (1 - alpha) * self._scale + alpha * new_scale

        self._recal_fix = (lat, lon)
        self._recal_vo = [0.0, 0.0]
        return True
