"""
VPS Device configuration: camera FOV, resolution, matcher parameters.
Defaults aligned with Berkeley fa24_106a (e.g. Siyi ZR10).
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class VPSDeviceConfig:
    """Configuration for the VPS device estimator and matcher."""

    # Camera (Berkeley defaults: Siyi ZR10 79.5 deg FOV, 1920x1080)
    h_fov_deg: float = 71.5
    d_fov_deg: float = 79.5
    width_px: int = 1920
    height_px: int = 1080

    # ORB / matcher
    nfeatures_orb: int = 250
    ratio_threshold: float = 0.95  # Lowe's ratio test
    n_clusters: int = 6
    max_dist_from_last_m: float = 50.0  # continuity: cluster median within this of last prediction
    max_dist_from_cluster_median_m: float = 5.0  # keep matches within this of cluster median
    min_matches: int = 10

    # BEBLID (optional, requires opencv-contrib)
    use_beblid: bool = False

    # Paths (for Proxigo-style region loading)
    default_satellite_data_path: Optional[str] = None

    # Simulation: when True, generate_synthetic_view returns crop at ref scale (no resize)
    simulation_same_scale: bool = False

    def camera_m_per_px(self, altitude_m: float) -> tuple:
        """Return (x_m_per_px, y_m_per_px) for given altitude (flat ground assumption)."""
        import math
        cam_x = 2 * altitude_m * math.tan(math.radians(self.h_fov_deg) / 2)
        cam_y = 2 * altitude_m * math.tan(math.radians(self.d_fov_deg) / 2)
        x_m_per_px = cam_x / self.width_px
        y_m_per_px = cam_y / self.height_px
        return x_m_per_px, y_m_per_px
