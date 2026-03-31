"""
VPS Device configuration: camera FOV, resolution, matcher parameters.
Defaults for IMX477 (12.3 MP) with 6 mm lens on NVIDIA Orin Nano.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class VPSDeviceConfig:
    """Configuration for the VPS device estimator and matcher."""

    # Camera: IMX477 sensor (4056x3040) with 6 mm lens
    h_fov_deg: float = 63.0
    d_fov_deg: float = 73.0
    width_px: int = 1920
    height_px: int = 1080

    # ORB / matcher
    nfeatures_orb: int = 2000
    ratio_threshold: float = 0.85  # Lowe's ratio test
    n_clusters: int = 6
    max_dist_from_last_m: float = 50.0  # continuity: cluster median within this of last prediction
    max_dist_from_cluster_median_m: float = 5.0  # keep matches within this of cluster median
    min_matches: int = 10
    # Matching flow:
    # - "homography": geometric inlier filtering using RANSAC homography
    # - "cluster": K-means clustering + continuity gating + geo transform
    matching_flow: str = "homography"

    # Feature type: SIFT (float, more distinctive) vs ORB (binary, faster)
    use_sift: bool = False
    # BEBLID (optional, requires opencv-contrib, ORB only)
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
