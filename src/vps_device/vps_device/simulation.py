"""
VPS software simulation: generate synthetic camera views from reference imagery
at different altitudes, positions, and yaw angles; run estimator and compare to ground truth.
"""

import base64
import io
import math
from dataclasses import dataclass, replace
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .config import VPSDeviceConfig
from .estimator import VPSEstimator
from .geo_transform import geo_to_ref_pixel, get_distance_metres
from .reference_loader import ReferenceImage
from .simulation_utils import localize_crop_in_ref


def _side_by_side_b64(ref_image: np.ndarray, query_bgr: np.ndarray, label_right: str = "Query") -> str:
    """Build ref (left) | query (right) same height, add labels; return PNG base64."""
    ref_bgr = cv2.cvtColor(ref_image, cv2.COLOR_GRAY2BGR)
    h_ref, w_ref = ref_bgr.shape[:2]
    h_q, w_q = query_bgr.shape[:2]
    scale = h_ref / h_q if h_q else 1.0
    if abs(scale - 1.0) > 0.01:
        query_bgr = cv2.resize(query_bgr, (int(w_q * scale), int(h_q * scale)))
    h_q, w_q = query_bgr.shape[:2]
    if h_ref < h_q:
        ref_bgr = cv2.copyMakeBorder(ref_bgr, 0, h_q - h_ref, 0, 0, cv2.BORDER_CONSTANT, value=(128, 128, 128))
    elif h_q < h_ref:
        query_bgr = cv2.copyMakeBorder(query_bgr, 0, h_ref - h_q, 0, 0, cv2.BORDER_CONSTANT, value=(128, 128, 128))
    vis = np.hstack([ref_bgr, query_bgr])
    cv2.putText(vis, "Reference", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(vis, label_right, (w_ref + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    _, png = cv2.imencode(".png", vis)
    return base64.b64encode(png.tobytes()).decode("ascii")


@dataclass
class SimulationResult:
    """Single simulation run: one synthetic view + estimate."""
    altitude_m: float
    yaw_deg: float
    truth_lat: float
    truth_lon: float
    est_lat: float
    est_lon: float
    success: bool
    error_m: float
    confidence: float
    n_matches: int
    debug_image_b64: Optional[str] = None  # ref vs query visualization (feature matches or side-by-side)


@dataclass
class TrajectorySample:
    """
    Single time step in a trajectory simulation.

    - truth_*: ground-truth camera position from the motion model.
    - vps_*: position estimated by the VPS estimator (may be NaN if failed).
    - vo_*: position from integrating nominal motion with noise (simulated VO drift).
    """

    time_s: float
    altitude_m: float
    yaw_deg: float
    truth_lat: float
    truth_lon: float
    vps_lat: float
    vps_lon: float
    vo_lat: float
    vo_lon: float
    error_vps_m: float
    error_vo_m: float
    confidence: float
    n_matches: int
    debug_image_b64: Optional[str] = None


def generate_synthetic_view(
    ref: ReferenceImage,
    camera_lat: float,
    camera_lon: float,
    altitude_m: float,
    yaw_deg: float,
    config: VPSDeviceConfig,
) -> np.ndarray:
    """
    Generate a synthetic camera image as if the camera were at (camera_lat, camera_lon)
    at altitude_m with yaw_deg (degrees, 0 = north-up). Returns BGR image at config resolution.
    """
    x_m_per_px, y_m_per_px = config.camera_m_per_px(altitude_m)
    view_width_m = config.width_px * x_m_per_px
    view_height_m = config.height_px * y_m_per_px

    # Camera center in reference image pixels
    cx, cy = geo_to_ref_pixel(
        camera_lat, camera_lon,
        ref.center_lat, ref.center_lon,
        ref.height_px, ref.width_px,
        ref.y_m_per_px, ref.x_m_per_px,
    )
    # View size in reference pixels (how many ref pixels span the camera view)
    view_w_px_ref = view_width_m / ref.x_m_per_px
    view_h_px_ref = view_height_m / ref.y_m_per_px

    # Crop in reference image (center at cx, cy, size view_w_px_ref x view_h_px_ref)
    x0 = int(cx - view_w_px_ref / 2)
    y0 = int(cy - view_h_px_ref / 2)
    x1 = int(cx + view_w_px_ref / 2)
    y1 = int(cy + view_h_px_ref / 2)

    h, w = ref.image.shape[:2]
    # Clamp to image bounds
    x0 = max(0, min(x0, w - 2))
    y0 = max(0, min(y0, h - 2))
    x1 = max(x0 + 1, min(x1, w))
    y1 = max(y0 + 1, min(y1, h))
    crop = ref.image[y0:y1, x0:x1]
    if crop.size == 0:
        crop = ref.image[:1, :1]  # fallback

    # Same-scale: return crop at ref resolution (no resize) for better feature matching when yaw=0
    same_scale = getattr(config, "simulation_same_scale", False) and abs(yaw_deg) < 0.5
    if same_scale:
        return cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)

    # Resize to camera resolution
    crop_bgr = cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)
    resized = cv2.resize(crop_bgr, (config.width_px, config.height_px), interpolation=cv2.INTER_LINEAR)

    if abs(yaw_deg) > 0.5:
        M = cv2.getRotationMatrix2D((config.width_px / 2, config.height_px / 2), -yaw_deg, 1.0)
        resized = cv2.warpAffine(resized, M, (config.width_px, config.height_px),
                                 borderMode=cv2.BORDER_REPLICATE)
    return resized


def run_simulation(
    ref: ReferenceImage,
    config: Optional[VPSDeviceConfig] = None,
    altitudes_m: Optional[List[float]] = None,
    yaw_angles_deg: Optional[List[float]] = None,
    positions: Optional[List[Tuple[float, float]]] = None,
    num_random_positions: int = 5,
    seed: int = 42,
    return_debug_images: bool = False,
) -> List[SimulationResult]:
    """
    Run VPS simulation: for each (position, altitude, yaw) generate synthetic view,
    run estimator, record error. Positions are (lat, lon). If positions is None,
    sample num_random_positions inside the reference image bounds.
    """
    config = config or VPSDeviceConfig()
    altitudes_m = altitudes_m or [30.0, 50.0, 100.0]
    yaw_angles_deg = yaw_angles_deg or [0.0]
    estimator = VPSEstimator(ref, config)

    if positions is None:
        rng = np.random.default_rng(seed)
        h, w = ref.height_px, ref.width_px
        # Sample positions in ref image center 60% to avoid edges
        margin = 0.2
        positions = []
        for _ in range(num_random_positions):
            px = rng.uniform(w * margin, w * (1 - margin))
            py = rng.uniform(h * margin, h * (1 - margin))
            lat, lon = ref.center_lat, ref.center_lon
            # Pixel offset to geo
            dx_px = px - w / 2
            dy_px = py - h / 2
            m_per_deg_lat = 6378137 * math.pi / 180
            m_per_deg_lon = 6378137 * math.pi / 180 * math.cos(math.radians(ref.center_lat))
            north_m = -dy_px * ref.y_m_per_px
            east_m = dx_px * ref.x_m_per_px
            lat = ref.center_lat + (north_m / m_per_deg_lat)
            lon = ref.center_lon + (east_m / m_per_deg_lon)
            positions.append((lat, lon))

    results = []
    same_scale = getattr(config, "simulation_same_scale", False)
    for (cam_lat, cam_lon) in positions:
        for alt in altitudes_m:
            for yaw in yaw_angles_deg:
                view = generate_synthetic_view(ref, cam_lat, cam_lon, alt, yaw, config)
                # When same_scale, view is crop-sized; override config for this call
                config_override = None
                if same_scale and abs(yaw) < 0.5 and view.shape[0] > 0 and view.shape[1] > 0:
                    config_override = replace(
                        config, width_px=view.shape[1], height_px=view.shape[0]
                    )
                est = estimator.estimate(
                    view, alt,
                    last_lat_lon=(cam_lat, cam_lon),
                    config_override=config_override,
                )
                if est.success:
                    error_m = get_distance_metres(cam_lat, cam_lon, est.lat, est.lon)
                    debug_b64 = None
                    if return_debug_images:
                        dbg = estimator.get_debug_image(view)
                        if dbg is not None:
                            _, png = cv2.imencode(".png", dbg)
                            debug_b64 = base64.b64encode(png.tobytes()).decode("ascii")
                    results.append(SimulationResult(
                        altitude_m=alt,
                        yaw_deg=yaw,
                        truth_lat=cam_lat,
                        truth_lon=cam_lon,
                        est_lat=est.lat,
                        est_lon=est.lon,
                        success=True,
                        error_m=error_m,
                        confidence=est.confidence,
                        n_matches=est.n_matches,
                        debug_image_b64=debug_b64,
                    ))
                elif abs(yaw) < 0.5:
                    # Fallback: template matching (query is crop of ref; yaw=0)
                    fallback = localize_crop_in_ref(view, ref.image, ref, config)
                    if fallback is not None:
                        lat_fb, lon_fb = fallback
                        error_m = get_distance_metres(cam_lat, cam_lon, lat_fb, lon_fb)
                        debug_b64 = _side_by_side_b64(ref.image, view, "Query (template match)") if return_debug_images else None
                        results.append(SimulationResult(
                            altitude_m=alt,
                            yaw_deg=yaw,
                            truth_lat=cam_lat,
                            truth_lon=cam_lon,
                            est_lat=lat_fb,
                            est_lon=lon_fb,
                            success=True,
                            error_m=error_m,
                            confidence=1.0,
                            n_matches=0,
                            debug_image_b64=debug_b64,
                        ))
                    else:
                        debug_b64 = _side_by_side_b64(ref.image, view, "Query (no match)") if return_debug_images else None
                        results.append(SimulationResult(
                            altitude_m=alt,
                            yaw_deg=yaw,
                            truth_lat=cam_lat,
                            truth_lon=cam_lon,
                            est_lat=est.lat,
                            est_lon=est.lon,
                            success=False,
                            error_m=float("nan"),
                            confidence=est.confidence,
                            n_matches=est.n_matches,
                            debug_image_b64=debug_b64,
                        ))
                else:
                    error_m = float("nan")
                    debug_b64 = _side_by_side_b64(ref.image, view, "Query") if return_debug_images else None
                    results.append(SimulationResult(
                        altitude_m=alt,
                        yaw_deg=yaw,
                        truth_lat=cam_lat,
                        truth_lon=cam_lon,
                        est_lat=est.lat,
                        est_lon=est.lon,
                        success=False,
                        error_m=error_m,
                        confidence=est.confidence,
                        n_matches=est.n_matches,
                        debug_image_b64=debug_b64,
                    ))
    return results


def generate_trajectory(
    ref: ReferenceImage,
    start_lat: float,
    start_lon: float,
    altitude_m: float,
    duration_s: float,
    step_hz: float,
    north_speed_m_s: float = 0.0,
    east_speed_m_s: float = 4.0,
    yaw_rate_deg_s: float = 0.0,
    bbox: Optional[Tuple[float, float, float, float]] = None,
) -> List[Tuple[float, float, float, float]]:
    """
    Generate a simple planar trajectory over the reference area.

    Returns a list of (t, lat, lon, yaw_deg).
    Motion model:
      - Constant north/east velocity (in metres per second).
      - Optional constant yaw rate (degrees per second).
      - Positions are clamped to the provided bbox when given.
    """
    duration_s = max(duration_s, 0.0)
    step_hz = max(step_hz, 0.1)
    dt = 1.0 / step_hz
    num_steps = int(duration_s * step_hz) + 1

    # Precompute metres-per-degree at reference center latitude
    m_per_deg_lat = 6378137 * math.pi / 180.0
    m_per_deg_lon = 6378137 * math.pi / 180.0 * math.cos(math.radians(ref.center_lat))

    north, south, east, west = (None, None, None, None)
    if bbox is not None:
        north, south, east, west = bbox

    samples: List[Tuple[float, float, float, float]] = []
    lat = start_lat
    lon = start_lon
    yaw = 0.0
    t = 0.0
    for _ in range(num_steps):
        # Clamp to bbox if provided
        if north is not None:
            lat = max(min(lat, north), south)
            lon = max(min(lon, east), west)
        samples.append((t, lat, lon, yaw))

        # Propagate state
        north_m = north_speed_m_s * dt
        east_m = east_speed_m_s * dt
        lat += north_m / m_per_deg_lat
        lon += east_m / m_per_deg_lon
        yaw += yaw_rate_deg_s * dt
        t += dt

    return samples


def run_trajectory_simulation(
    ref: ReferenceImage,
    config: Optional[VPSDeviceConfig] = None,
    trajectory: Optional[List[Tuple[float, float, float, float]]] = None,
    altitude_m: float = 50.0,
    seed: int = 42,
    return_debug_images: bool = True,
) -> List[TrajectorySample]:
    """
    Run a VO-style trajectory simulation.

    Args:
        ref: Loaded ReferenceImage.
        config: VPSDeviceConfig; if None, a default will be constructed.
        trajectory: Optional precomputed list of (t, lat, lon, yaw_deg). If None,
            a simple straight-line trajectory starting at ref.center will be used.
        altitude_m: Altitude used for all frames (metres).
        seed: RNG seed for VO noise (deterministic drift).
        return_debug_images: Whether to include base64-encoded debug images.

    Returns:
        List of TrajectorySample ordered by time.
    """
    config = config or VPSDeviceConfig()
    estimator = VPSEstimator(ref, config)

    # Simple default trajectory: straight east over centre of region.
    if not trajectory:
        traj = generate_trajectory(
            ref,
            start_lat=ref.center_lat,
            start_lon=ref.center_lon,
            altitude_m=altitude_m,
            duration_s=5.0,
            step_hz=5.0,
        )
    else:
        traj = trajectory

    rng = np.random.default_rng(seed)
    samples: List[TrajectorySample] = []

    same_scale = getattr(config, "simulation_same_scale", False)

    # Initialise VO path at first truth position
    if not traj:
        return samples
    _, first_lat, first_lon, _ = traj[0]
    vo_lat = first_lat
    vo_lon = first_lon
    prev_truth_lat = first_lat
    prev_truth_lon = first_lon

    # VO noise parameters (metres per step)
    vo_noise_std_m = 0.05

    last_vps_lat_lon: Optional[Tuple[float, float]] = None

    for (t, cam_lat, cam_lon, yaw_deg) in traj:
        # Generate synthetic view at truth pose
        view = generate_synthetic_view(ref, cam_lat, cam_lon, altitude_m, yaw_deg, config)

        # Adjust config for same-scale crops (as in run_simulation)
        config_override = None
        if same_scale and abs(yaw_deg) < 0.5 and view.shape[0] > 0 and view.shape[1] > 0:
            config_override = replace(
                config, width_px=view.shape[1], height_px=view.shape[0]
            )

        # VPS estimate
        est = estimator.estimate(
            view,
            altitude_m,
            last_lat_lon=last_vps_lat_lon,
            config_override=config_override,
        )

        if est.success:
            vps_lat = est.lat
            vps_lon = est.lon
            last_vps_lat_lon = (vps_lat, vps_lon)
            error_vps_m = get_distance_metres(cam_lat, cam_lon, vps_lat, vps_lon)
        else:
            vps_lat = float("nan")
            vps_lon = float("nan")
            error_vps_m = float("nan")

        # Simulated VO drift: integrate truth displacement + noise
        # Approximate metres-per-degree at current latitude
        m_per_deg_lat = 6378137 * math.pi / 180.0
        m_per_deg_lon = 6378137 * math.pi / 180.0 * math.cos(math.radians(cam_lat))
        dlat = cam_lat - prev_truth_lat
        dlon = cam_lon - prev_truth_lon
        north_m = dlat * m_per_deg_lat
        east_m = dlon * m_per_deg_lon
        # Add zero-mean Gaussian noise
        north_m += rng.normal(0.0, vo_noise_std_m)
        east_m += rng.normal(0.0, vo_noise_std_m)
        vo_lat += north_m / m_per_deg_lat
        vo_lon += east_m / m_per_deg_lon
        prev_truth_lat = cam_lat
        prev_truth_lon = cam_lon

        error_vo_m = get_distance_metres(cam_lat, cam_lon, vo_lat, vo_lon)

        debug_b64: Optional[str] = None
        if return_debug_images:
            dbg = estimator.get_debug_image(view)
            if dbg is not None:
                _, png = cv2.imencode(".png", dbg)
                debug_b64 = base64.b64encode(png.tobytes()).decode("ascii")

        samples.append(
            TrajectorySample(
                time_s=float(t),
                altitude_m=float(altitude_m),
                yaw_deg=float(yaw_deg),
                truth_lat=float(cam_lat),
                truth_lon=float(cam_lon),
                vps_lat=float(vps_lat),
                vps_lon=float(vps_lon),
                vo_lat=float(vo_lat),
                vo_lon=float(vo_lon),
                error_vps_m=float(error_vps_m),
                error_vo_m=float(error_vo_m),
                confidence=float(est.confidence),
                n_matches=int(est.n_matches),
                debug_image_b64=debug_b64,
            )
        )

    return samples
