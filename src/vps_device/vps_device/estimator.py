"""
Main API: load reference, then estimate(image, altitude_m, last_lat_lon) -> (lat, lon), confidence.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple, Union

import cv2
import numpy as np

from .config import VPSDeviceConfig
from .continuity import select_cluster_by_continuity
from .features import (
    build_flann_matcher,
    extract_features,
    filter_matches_kmeans_continuity,
    match_with_ratio_test,
    _get_descriptor_extractor,
)
from .geo_transform import (
    camera_offset_to_geo,
    get_distance_metres,
    ref_pixel_to_geo,
)
from .reference_loader import (
    ReferenceImage,
    load_proxigo_region,
    load_berkeley_reference,
    load_berkeley_dict,
)


@dataclass
class EstimateResult:
    """Result of a single VPS estimate."""
    success: bool
    lat: float
    lon: float
    confidence: float
    n_matches: int
    details: Optional[dict] = None


class VPSEstimator:
    """
    Visual Positioning System estimator. Compares live image + altitude to
    reference imagery and returns estimated (lat, lon).
    """

    def __init__(
        self,
        reference: Union[ReferenceImage, List[ReferenceImage]],
        config: Optional[VPSDeviceConfig] = None,
    ):
        self.config = config or VPSDeviceConfig()
        if isinstance(reference, ReferenceImage):
            self.references = [reference]
        else:
            self.references = reference
        if not self.references:
            raise ValueError("At least one reference image required")
        self._ref = self.references[0]  # use first for now; can extend to multi-ref
        self._orb, self._desc = _get_descriptor_extractor(self.config.use_beblid)
        self._orb.setMaxFeatures(self.config.nfeatures_orb)
        self._flann = build_flann_matcher()
        self._ref_kpts, self._ref_des = self._compute_ref_features()
        self._last_lat_lon: Optional[Tuple[float, float]] = None
        self._last_debug: Optional[Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]] = None

    def _compute_ref_features(self) -> Tuple[List, np.ndarray]:
        kpts, des = extract_features(
            self._ref.image,
            self._orb,
            self._desc,
            self.config.nfeatures_orb,
        )
        if des is None:
            des = np.array([], dtype=np.uint8).reshape(0, 32)
        return kpts, des

    def estimate(
        self,
        image: np.ndarray,
        altitude_m: float,
        last_lat_lon: Optional[Tuple[float, float]] = None,
        config_override: Optional["VPSDeviceConfig"] = None,
    ) -> EstimateResult:
        """
        Estimate position from a single image and altitude.
        Optionally pass last_lat_lon for continuity filtering; otherwise uses internal state.
        config_override: when set (e.g. for same-scale simulation), use it for width_px/height_px and camera_m_per_px.
        """
        last = last_lat_lon if last_lat_lon is not None else self._last_lat_lon
        cfg = config_override if config_override is not None else self.config

        x_m_per_px, y_m_per_px = cfg.camera_m_per_px(altitude_m)
        cam_center_x = cfg.width_px / 2.0
        cam_center_y = cfg.height_px / 2.0

        query_kpts, query_des = extract_features(
            image,
            self._orb,
            self._desc,
            self.config.nfeatures_orb,
        )
        if query_des is None or len(query_kpts) < self.config.min_matches:
            return EstimateResult(
                success=False,
                lat=0.0,
                lon=0.0,
                confidence=0.0,
                n_matches=len(query_kpts) if query_kpts else 0,
            )

        matches = match_with_ratio_test(
            self._ref_des,
            query_des,
            self._flann,
            ratio=self.config.ratio_threshold,
        )
        if len(matches) < self.config.min_matches:
            return EstimateResult(
                success=False,
                lat=0.0,
                lon=0.0,
                confidence=0.0,
                n_matches=len(matches),
            )

        filtered = filter_matches_kmeans_continuity(
            self._ref_kpts,
            query_kpts,
            matches,
            self._ref,
            last,
            self.config.max_dist_from_last_m,
            self.config.max_dist_from_cluster_median_m,
            self.config.n_clusters,
            get_distance_metres,
            ref_pixel_to_geo,
        )
        if not filtered:
            self._last_debug = None
            return EstimateResult(
                success=False,
                lat=0.0,
                lon=0.0,
                confidence=0.0,
                n_matches=len(matches),
            )

        lats = []
        lons = []
        for (ref_pt, query_pt) in filtered:
            ref_lat, ref_lon = ref_pixel_to_geo(
                ref_pt[0], ref_pt[1],
                self._ref.center_lat,
                self._ref.center_lon,
                self._ref.height_px,
                self._ref.width_px,
                self._ref.y_m_per_px,
                self._ref.x_m_per_px,
            )
            cam_lat, cam_lon = camera_offset_to_geo(
                ref_lat,
                ref_lon,
                query_pt[0],
                query_pt[1],
                cam_center_x,
                cam_center_y,
                x_m_per_px,
                y_m_per_px,
            )
            lats.append(cam_lat)
            lons.append(cam_lon)

        # Median is more robust to outlier matches than mean
        lat = float(np.median(lats))
        lon = float(np.median(lons))
        confidence = len(filtered) / max(len(matches), 1)

        self._last_lat_lon = (lat, lon)
        self._last_debug = ([p for p, _ in filtered], [q for _, q in filtered])
        return EstimateResult(
            success=True,
            lat=lat,
            lon=lon,
            confidence=confidence,
            n_matches=len(filtered),
            details={"all_matches": len(matches)},
        )

    def set_last_position(self, lat: float, lon: float) -> None:
        """Set last predicted position for continuity (e.g. from external source)."""
        self._last_lat_lon = (lat, lon)

    def get_debug_image(self, query_bgr: np.ndarray) -> Optional[np.ndarray]:
        """
        Return a visualization image: reference (left) and query (right) with
        lines connecting matched keypoints. Returns None if no match data.
        """
        if self._last_debug is None:
            return None
        ref_pts, query_pts = self._last_debug
        if not ref_pts or not query_pts:
            return None
        ref_bgr = cv2.cvtColor(self._ref.image, cv2.COLOR_GRAY2BGR)
        # Resize so both have same height for side-by-side
        h_ref, w_ref = ref_bgr.shape[:2]
        h_q, w_q = query_bgr.shape[:2]
        scale = h_ref / h_q if h_q else 1.0
        if abs(scale - 1.0) > 0.01:
            query_bgr = cv2.resize(query_bgr, (int(w_q * scale), int(h_q * scale)))
            query_pts = [(p[0] * scale, p[1] * scale) for p in query_pts]
        h_q, w_q = query_bgr.shape[:2]
        # Pad ref or query to same height
        if h_ref < h_q:
            ref_bgr = cv2.copyMakeBorder(ref_bgr, 0, h_q - h_ref, 0, 0, cv2.BORDER_CONSTANT, value=(128, 128, 128))
        elif h_q < h_ref:
            query_bgr = cv2.copyMakeBorder(query_bgr, 0, h_ref - h_q, 0, 0, cv2.BORDER_CONSTANT, value=(128, 128, 128))
        vis = np.hstack([ref_bgr, query_bgr])
        w_off = w_ref
        for (rx, ry), (qx, qy) in zip(ref_pts, query_pts):
            pt1 = (int(rx), int(ry))
            pt2 = (int(qx) + w_off, int(qy))
            cv2.line(vis, pt1, pt2, (0, 255, 0), 1)
        cv2.putText(vis, "Reference", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis, "Query", (w_off + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return vis


def create_estimator_from_proxigo_region(
    region_path: Union[str, Path],
    config: Optional[VPSDeviceConfig] = None,
) -> VPSEstimator:
    """Create estimator from a Proxigo-style region directory."""
    ref = load_proxigo_region(Path(region_path))
    return VPSEstimator(ref, config)


def create_estimator_from_berkeley(
    path: str,
    center_lat: float,
    center_lon: float,
    height_m: float,
    width_m: float,
    config: Optional[VPSDeviceConfig] = None,
) -> VPSEstimator:
    """Create estimator from a single Berkeley-style reference image."""
    ref = load_berkeley_reference(path, center_lat, center_lon, height_m, width_m)
    return VPSEstimator(ref, config)
