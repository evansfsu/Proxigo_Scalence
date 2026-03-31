"""
Pose / position fix from local matching.

This module provides:
  - K (camera intrinsics) construction helpers
  - Planar pose solve via homography decomposition (stable when world points lie on ground plane)
  - Optional solvePnPRansac path when 3D points are available (e.g., DSM heights)

Outputs a lat/lon fix + a rough covariance proxy for fusion/gating.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np


def camera_matrix_from_fov(width_px: int, height_px: int, hfov_deg: float) -> np.ndarray:
    """Compute pinhole intrinsics from horizontal FOV and image size."""
    w = float(width_px)
    h = float(height_px)
    fx = (w * 0.5) / math.tan(math.radians(hfov_deg) * 0.5)
    fy = fx  # if only hfov known, assume square pixels and derive fy similarly
    cx = w * 0.5
    cy = h * 0.5
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    return K


@dataclass(frozen=True)
class PnPFix:
    success: bool
    lat: float
    lon: float
    confidence: float
    n_inliers: int
    reproj_rmse_px: float


def _meters_per_deg(lat_deg: float) -> Tuple[float, float]:
    m_per_deg_lat = 6378137.0 * math.pi / 180.0
    m_per_deg_lon = m_per_deg_lat * math.cos(math.radians(lat_deg))
    return m_per_deg_lat, m_per_deg_lon


def _ref_px_to_local_m(
    ref_px: np.ndarray,
    ref_center_lat: float,
    ref_center_lon: float,
    ref_width_px: int,
    ref_height_px: int,
    ref_m_per_px: float,
) -> np.ndarray:
    """
    Convert reference pixel coordinates (x,y) to local EN (east,north) metres
    in a tangent plane centered at ref center.
    """
    # origin at image center
    cx = ref_width_px * 0.5
    cy = ref_height_px * 0.5
    dx_px = ref_px[:, 0] - cx
    dy_px = ref_px[:, 1] - cy
    east_m = dx_px * ref_m_per_px
    north_m = -dy_px * ref_m_per_px
    return np.stack([east_m, north_m], axis=1)


def _local_m_to_latlon(
    east_m: float,
    north_m: float,
    center_lat: float,
    center_lon: float,
) -> Tuple[float, float]:
    mlat, mlon = _meters_per_deg(center_lat)
    lat = center_lat + (north_m / mlat)
    lon = center_lon + (east_m / mlon)
    return lat, lon


def planar_pose_fix_from_matches(
    ref_pts_px: Sequence[Tuple[float, float]],
    query_pts_px: Sequence[Tuple[float, float]],
    K: np.ndarray,
    ref_center_lat: float,
    ref_center_lon: float,
    ref_width_px: int,
    ref_height_px: int,
    ref_m_per_px: float,
    ransac_reproj_px: float = 4.0,
) -> PnPFix:
    """
    Compute a position fix by:
      1) estimating homography H (ref->query) with RANSAC
      2) decomposing H into pose candidates with K
      3) choosing a plausible solution and converting camera position to lat/lon

    This is robust for planar scenes (orthophoto reference), and is usually
    more stable than solvePnP on strictly planar 3D points.
    """
    if len(ref_pts_px) < 8 or len(query_pts_px) < 8:
        return PnPFix(False, 0.0, 0.0, 0.0, 0, reproj_rmse_px=1e9)

    ref = np.asarray(ref_pts_px, dtype=np.float64).reshape(-1, 1, 2)
    qry = np.asarray(query_pts_px, dtype=np.float64).reshape(-1, 1, 2)

    H, mask = cv2.findHomography(ref, qry, cv2.RANSAC, ransacReprojThreshold=ransac_reproj_px)
    if H is None or mask is None:
        return PnPFix(False, 0.0, 0.0, 0.0, 0, reproj_rmse_px=1e9)

    inl = mask.ravel().astype(bool)
    n_inliers = int(inl.sum())
    if n_inliers < 8:
        return PnPFix(False, 0.0, 0.0, 0.0, n_inliers, reproj_rmse_px=1e9)

    # Reprojection RMSE in pixels on inliers
    ref_inl = ref[inl].reshape(-1, 2)
    qry_inl = qry[inl].reshape(-1, 2)
    ref_h = cv2.convertPointsToHomogeneous(ref_inl.reshape(-1, 1, 2)).reshape(-1, 3).T
    proj = (H @ ref_h)
    proj = (proj[:2, :] / proj[2:3, :]).T
    err = proj - qry_inl
    rmse = float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))

    try:
        nsol, Rs, ts, _ = cv2.decomposeHomographyMat(H, K)
    except cv2.error:
        return PnPFix(False, 0.0, 0.0, 0.0, n_inliers, reproj_rmse_px=rmse)

    if nsol <= 0:
        return PnPFix(False, 0.0, 0.0, 0.0, n_inliers, reproj_rmse_px=rmse)

    # Convert inlier reference pixels to local world points (east,north,0)
    ref_local_en = _ref_px_to_local_m(
        ref_inl,
        ref_center_lat,
        ref_center_lon,
        ref_width_px,
        ref_height_px,
        ref_m_per_px,
    )

    best = None
    for i in range(nsol):
        R = Rs[i]
        t = ts[i].reshape(3, 1)
        # Choose solution with camera in front of plane (heuristic: tz > 0)
        if float(t[2]) <= 0:
            continue
        # Camera position in world coordinates: C = -R^T t
        C = -R.T @ t
        east_m = float(C[0])
        north_m = float(C[1])
        lat, lon = _local_m_to_latlon(east_m, north_m, ref_center_lat, ref_center_lon)

        best = (lat, lon)
        break

    if best is None:
        return PnPFix(False, 0.0, 0.0, 0.0, n_inliers, reproj_rmse_px=rmse)

    # Confidence proxy: inlier ratio * pixel-fit quality
    inlier_ratio = n_inliers / max(len(ref_pts_px), 1)
    fit = 1.0 / (1.0 + rmse)
    confidence = float(np.clip(inlier_ratio * fit, 0.0, 1.0))

    return PnPFix(True, best[0], best[1], confidence, n_inliers, reproj_rmse_px=rmse)


def pnp_fix_with_dsm(
    ref_pts_px: Sequence[Tuple[float, float]],
    query_pts_px: Sequence[Tuple[float, float]],
    K: np.ndarray,
    dsm_m: np.ndarray,
    ref_center_lat: float,
    ref_center_lon: float,
    ref_width_px: int,
    ref_height_px: int,
    ref_m_per_px: float,
    ref_anchor_px: Optional[Tuple[float, float]] = None,
    dsm_anchor_px: Optional[Tuple[float, float]] = None,
    dsm_resolution_m: Optional[float] = None,
    ransac_reproj_px: float = 4.0,
) -> PnPFix:
    """
    True PnP fix using 3D points from DSM (height map aligned to reference).

    World frame: local ENU-like tangent plane centered at reference center:
      X=east_m, Y=north_m, Z=up_m (from DSM)
    """
    if len(ref_pts_px) < 8 or len(query_pts_px) < 8:
        return PnPFix(False, 0.0, 0.0, 0.0, 0, reproj_rmse_px=1e9)

    ref = np.asarray(ref_pts_px, dtype=np.float64)
    qry = np.asarray(query_pts_px, dtype=np.float64)

    # Let solvePnPRansac do the robust inlier selection directly.
    ref_inl = ref
    qry_inl = qry

    # Build 3D points from DSM sampled at ref pixel locations.
    # Preferred path (benchmark-style): use anchor points that align reference px
    # to DSM px and convert by ref/dsm resolution ratio.
    # Fallback path: global image-size scaling.
    h_dsm, w_dsm = dsm_m.shape[:2]
    if (
        ref_anchor_px is not None
        and dsm_anchor_px is not None
        and dsm_resolution_m is not None
        and dsm_resolution_m > 0
        and ref_m_per_px > 0
    ):
        scale = ref_m_per_px / float(dsm_resolution_m)
        xs_f = (ref_inl[:, 0] - float(ref_anchor_px[0])) * scale + float(dsm_anchor_px[0])
        ys_f = (ref_inl[:, 1] - float(ref_anchor_px[1])) * scale + float(dsm_anchor_px[1])
        xs = np.clip(np.round(xs_f).astype(int), 0, w_dsm - 1)
        ys = np.clip(np.round(ys_f).astype(int), 0, h_dsm - 1)
    else:
        sx = (w_dsm - 1) / max(ref_width_px - 1, 1)
        sy = (h_dsm - 1) / max(ref_height_px - 1, 1)
        xs = np.clip(np.round(ref_inl[:, 0] * sx).astype(int), 0, w_dsm - 1)
        ys = np.clip(np.round(ref_inl[:, 1] * sy).astype(int), 0, h_dsm - 1)
    zs = dsm_m[ys, xs].astype(np.float64)
    # Replace invalid / NaN heights
    zs = np.where(np.isfinite(zs), zs, np.nanmedian(zs[np.isfinite(zs)]) if np.any(np.isfinite(zs)) else 0.0)

    en = _ref_px_to_local_m(
        ref_inl,
        ref_center_lat,
        ref_center_lon,
        ref_width_px,
        ref_height_px,
        ref_m_per_px,
    )
    obj_pts = np.column_stack([en[:, 0], en[:, 1], zs]).reshape(-1, 1, 3)
    img_pts = qry_inl.reshape(-1, 1, 2)

    dist = np.zeros((4, 1), dtype=np.float64)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        objectPoints=obj_pts,
        imagePoints=img_pts,
        cameraMatrix=K,
        distCoeffs=dist,
        flags=cv2.SOLVEPNP_ITERATIVE,
        reprojectionError=ransac_reproj_px,
        iterationsCount=200,
        confidence=0.99,
    )
    if not ok or inliers is None or len(inliers) < 8:
        return PnPFix(False, 0.0, 0.0, 0.0, int(len(inliers) if inliers is not None else 0), reproj_rmse_px=1e9)

    # Compute reprojection RMSE on PnP inliers
    inliers = inliers.reshape(-1)
    obj_inl = obj_pts[inliers]
    img_inl = img_pts[inliers]
    proj, _ = cv2.projectPoints(obj_inl, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)
    img_inl2 = img_inl.reshape(-1, 2)
    err = proj - img_inl2
    rmse = float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))

    R, _ = cv2.Rodrigues(rvec)
    C = -R.T @ tvec  # camera position in world
    east_m = float(C[0])
    north_m = float(C[1])

    # Sanity gating: if solution explodes, treat as failure.
    if not np.isfinite(east_m) or not np.isfinite(north_m) or abs(east_m) > 50000 or abs(north_m) > 50000:
        return PnPFix(False, 0.0, 0.0, 0.0, int(len(inliers)), reproj_rmse_px=rmse)
    lat, lon = _local_m_to_latlon(east_m, north_m, ref_center_lat, ref_center_lon)

    inlier_ratio = len(inliers) / max(len(ref_pts_px), 1)
    fit = 1.0 / (1.0 + rmse)
    confidence = float(np.clip(inlier_ratio * fit, 0.0, 1.0))
    return PnPFix(True, lat, lon, confidence, int(len(inliers)), reproj_rmse_px=rmse)

