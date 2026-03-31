"""
Standalone Extended Kalman Filter for VPS fusion.

State vector:  x = [lat, lon, vx, vy]
  - lat, lon  in degrees
  - vx        eastward ground velocity  (m/s)
  - vy        northward ground velocity (m/s)

Two measurement sources:
  1. Visual Odometry  -> observes (vx, vy)          -- every frame
  2. Map Matching     -> observes (lat, lon)         -- intermittent

Modeled after the existing state_fusion/fusion_node.py EKF but requires
no ROS2 dependency.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# Flat-earth conversions
_EARTH_RADIUS = 6378137.0
_M_PER_DEG_LAT = _EARTH_RADIUS * math.pi / 180.0


def _m_per_deg_lon(lat_deg: float) -> float:
    return _M_PER_DEG_LAT * math.cos(math.radians(lat_deg))


@dataclass
class EKFConfig:
    """Tuning knobs for the VPS fusion EKF."""
    vo_velocity_noise: float = 0.3       # m/s  std-dev of VO velocity measurement
    map_position_noise: float = 30.0     # m    std-dev of map-match position (base)
    process_noise_position: float = 0.05 # m^2  per second of position random walk
    process_noise_velocity: float = 1.0  # (m/s)^2  per second of velocity random walk
    max_vo_speed: float = 50.0           # m/s  reject VO outliers faster than this
    innovation_gate: float = 5.0         # Mahalanobis distance threshold for map matches


class VPSFusionEKF:
    """4-state EKF fusing Visual Odometry with Map Matching.

    Usage::

        ekf = VPSFusionEKF()
        for frame in frames:
            vo = visual_odometry.process_frame(frame, alt)
            if vo:
                ekf.predict(vo.dt)
                ekf.update_vo(vo.vx, vo.vy)
            if should_map_match:
                result = estimator.estimate(frame, alt)
                if result.success:
                    ekf.update_map_match(result.lat, result.lon, result.confidence)
            lat, lon = ekf.position
    """

    def __init__(self, cfg: Optional[EKFConfig] = None):
        self.cfg = cfg or EKFConfig()
        # State: [lat, lon, vx, vy]
        self.x = np.zeros(4)
        # Covariance
        self.P = np.diag([1e-6, 1e-6, 1.0, 1.0])  # small pos uncertainty, 1 m/s vel
        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    def initialize(self, lat: float, lon: float,
                   vx: float = 0.0, vy: float = 0.0):
        """Set initial state (called on first map-match fix or ground truth)."""
        self.x = np.array([lat, lon, vx, vy], dtype=np.float64)
        self.P = np.diag([
            (self.cfg.map_position_noise / _M_PER_DEG_LAT) ** 2,
            (self.cfg.map_position_noise / _m_per_deg_lon(lat)) ** 2,
            1.0,
            1.0,
        ])
        self._initialized = True

    # ------------------------------------------------------------------
    # Prediction
    # ------------------------------------------------------------------

    def predict(self, dt: float):
        """Propagate state forward by dt seconds using constant-velocity model."""
        if not self._initialized or dt <= 0:
            return

        lat, lon, vx, vy = self.x

        mpl = _M_PER_DEG_LAT
        mplo = _m_per_deg_lon(lat)

        # Position update: lat += vy*dt / m_per_deg_lat, lon += vx*dt / m_per_deg_lon
        dlat = vy * dt / mpl
        dlon = vx * dt / mplo

        self.x[0] += dlat
        self.x[1] += dlon

        # State transition Jacobian
        F = np.eye(4)
        F[0, 3] = dt / mpl          # d(lat)/d(vy)
        F[1, 2] = dt / mplo         # d(lon)/d(vx)

        # Process noise
        Q = np.zeros((4, 4))
        q_pos = self.cfg.process_noise_position * dt
        q_vel = self.cfg.process_noise_velocity * dt
        Q[0, 0] = q_pos / (mpl ** 2)
        Q[1, 1] = q_pos / (mplo ** 2)
        Q[2, 2] = q_vel
        Q[3, 3] = q_vel

        self.P = F @ self.P @ F.T + Q

    # ------------------------------------------------------------------
    # VO measurement update
    # ------------------------------------------------------------------

    def update_vo(self, vx: float, vy: float, quality: float = 1.0):
        """Update velocity from visual odometry (use for real-time / known dt)."""
        if not self._initialized:
            return

        if abs(vx) > self.cfg.max_vo_speed or abs(vy) > self.cfg.max_vo_speed:
            return

        z = np.array([vx, vy])

        H = np.zeros((2, 4))
        H[0, 2] = 1.0
        H[1, 3] = 1.0

        base_noise = self.cfg.vo_velocity_noise ** 2
        noise_scale = 1.0 / max(quality, 0.1)
        R = np.eye(2) * base_noise * noise_scale

        self._kalman_update(z, H, R)

    def predict_with_vo(self, d_east_m: float, d_north_m: float,
                        quality: float = 1.0, altitude_m: float = 100.0):
        """Propagate position using VO displacement (replaces predict + update_vo).

        The VO displacement is applied directly to the position state. This
        works correctly regardless of wall-clock vs real-capture dt because
        it uses displacement (metres) not velocity.

        Also updates the velocity state as a running estimate (used for
        dead-reckoning during VO outages via predict_constant_velocity).

        Covariance grows proportionally to displacement, inversely to quality,
        and scales with altitude (lower altitude = tighter features = less noise).
        """
        if not self._initialized:
            return

        mpl = _M_PER_DEG_LAT
        mplo = _m_per_deg_lon(self.x[0])

        self.x[0] += d_north_m / mpl
        self.x[1] += d_east_m / mplo

        alpha = 0.3
        self.x[2] = (1 - alpha) * self.x[2] + alpha * d_east_m
        self.x[3] = (1 - alpha) * self.x[3] + alpha * d_north_m

        dist = math.sqrt(d_east_m ** 2 + d_north_m ** 2)

        # Altitude-adaptive VO noise: at 50m altitude features are very
        # distinctive (~2% of displacement); at 500m they're coarser (~15%).
        altitude_factor = max(0.5, min(altitude_m / 100.0, 5.0))
        base_frac = 0.02 * altitude_factor
        noise_frac = base_frac / max(quality, 0.05)
        vo_noise_m = max(dist * noise_frac, 0.1)

        F = np.eye(4)
        Q = np.zeros((4, 4))
        Q[0, 0] = (vo_noise_m / mpl) ** 2
        Q[1, 1] = (vo_noise_m / mplo) ** 2
        Q[2, 2] = self.cfg.process_noise_velocity
        Q[3, 3] = self.cfg.process_noise_velocity

        self.P = F @ self.P @ F.T + Q

    def predict_constant_velocity(self):
        """Dead-reckon one step using the stored velocity estimate.

        Use this when VO fails (quality=0) to avoid freezing the position.
        Grows covariance more aggressively since there's no measurement.
        """
        if not self._initialized:
            return

        mpl = _M_PER_DEG_LAT
        mplo = _m_per_deg_lon(self.x[0])

        # Apply stored velocity as displacement
        self.x[0] += self.x[3] / mpl
        self.x[1] += self.x[2] / mplo

        dist = math.sqrt(self.x[2] ** 2 + self.x[3] ** 2)
        noise_m = max(dist * 0.5, 2.0)

        Q = np.zeros((4, 4))
        Q[0, 0] = (noise_m / mpl) ** 2
        Q[1, 1] = (noise_m / mplo) ** 2
        Q[2, 2] = self.cfg.process_noise_velocity * 2
        Q[3, 3] = self.cfg.process_noise_velocity * 2

        self.P = self.P + Q

    # ------------------------------------------------------------------
    # Map-match measurement update  (observes lat, lon)
    # ------------------------------------------------------------------

    def update_map_match(self, lat: float, lon: float,
                         confidence: float = 0.5,
                         n_inliers: int = 10,
                         altitude_m: float = 100.0) -> bool:
        """Update position from satellite map matching.

        Returns True if the measurement was accepted (passed innovation gate).
        """
        if not self._initialized:
            self.initialize(lat, lon)
            return True

        # Altitude-adaptive noise: higher altitude = less reliable map match
        altitude_factor = max(1.0, altitude_m / 100.0)
        base_m = self.cfg.map_position_noise * altitude_factor
        scale = 1.0 / max(confidence * 10, 0.1)
        inlier_scale = max(1.0, 20.0 / max(n_inliers, 1))
        noise_m = base_m * scale * inlier_scale
        return self.update_map_match_with_noise(lat, lon, noise_m=noise_m)

    def update_map_match_with_noise(self, lat: float, lon: float, noise_m: float) -> bool:
        """Update position using an explicit position-noise measurement model (metres)."""
        if not self._initialized:
            self.initialize(lat, lon)
            return True

        z = np.array([lat, lon])

        H = np.zeros((2, 4))
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        cur_lat = self.x[0]
        noise_m = max(float(noise_m), 0.5)
        R = np.diag([
            (noise_m / _M_PER_DEG_LAT) ** 2,
            (noise_m / _m_per_deg_lon(cur_lat)) ** 2,
        ])

        # Mahalanobis innovation gate: reject measurements that are
        # statistically too far from the predicted position
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        mahal_sq = float(y @ np.linalg.inv(S) @ y)
        if mahal_sq > self.cfg.innovation_gate ** 2:
            return False

        self._kalman_update(z, H, R)
        return True

    # ------------------------------------------------------------------
    # Core Kalman math
    # ------------------------------------------------------------------

    def _kalman_update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray):
        y = z - H @ self.x                       # innovation
        S = H @ self.P @ H.T + R                 # innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)      # Kalman gain
        self.x = self.x + K @ y                   # state update
        I_KH = np.eye(len(self.x)) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T   # Joseph form (numerically stable)

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    @property
    def position(self) -> Tuple[float, float]:
        """Current fused (lat, lon)."""
        return float(self.x[0]), float(self.x[1])

    @property
    def velocity(self) -> Tuple[float, float]:
        """Current fused (vx_east, vy_north) in m/s."""
        return float(self.x[2]), float(self.x[3])

    @property
    def position_std(self) -> Tuple[float, float]:
        """Position standard deviation in metres (east, north)."""
        lat = self.x[0]
        std_lat_m = math.sqrt(max(self.P[0, 0], 0)) * _M_PER_DEG_LAT
        std_lon_m = math.sqrt(max(self.P[1, 1], 0)) * _m_per_deg_lon(lat)
        return std_lon_m, std_lat_m

    @property
    def speed(self) -> float:
        """Ground speed in m/s."""
        return math.sqrt(self.x[2] ** 2 + self.x[3] ** 2)
