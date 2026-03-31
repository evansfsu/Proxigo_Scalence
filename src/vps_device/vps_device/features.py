"""
Feature detection, FLANN matching, Lowe's ratio test, K-means filtering.
ORB + optional BEBLID (opencv-contrib). Output: list of (ref_pt, query_pt) in pixels.
"""

from typing import List, Optional, Tuple

import cv2
import numpy as np
from sklearn.cluster import KMeans

from .reference_loader import ReferenceImage


def _get_descriptor_extractor(use_beblid: bool = False, use_sift: bool = False,
                               nfeatures: int = 2000):
    """Return (detector, descriptor_extractor) pair.

    use_sift=True  → SIFT (float32 descriptors, L2 matching)
    use_sift=False → ORB  (binary descriptors, Hamming matching)
    """
    if use_sift:
        sift = cv2.SIFT_create(nfeatures=nfeatures)
        return sift, None
    orb = cv2.ORB_create(nfeatures=nfeatures)
    desc = None
    if use_beblid:
        try:
            desc = cv2.xfeatures2d.BEBLID_create(0.75)
        except AttributeError:
            pass
    return orb, desc


def extract_features(
    image: np.ndarray,
    detector: cv2.Feature2D,
    descriptor_extractor: Optional[cv2.Feature2D],
    nfeatures: int = 250,
) -> Tuple[List[cv2.KeyPoint], Optional[np.ndarray]]:
    """Extract keypoints and descriptors from grayscale image."""
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image

    if hasattr(detector, 'setMaxFeatures'):
        detector.setMaxFeatures(nfeatures)

    kpts = detector.detect(gray, None)
    if descriptor_extractor is not None:
        kpts, des = descriptor_extractor.compute(gray, kpts)
    else:
        kpts, des = detector.compute(gray, kpts)
    return kpts, des


def build_flann_matcher(use_sift: bool = False):
    """FLANN matcher. Uses KDTree for float descriptors, LSH for binary."""
    if use_sift:
        index_params = dict(algorithm=1, trees=5)  # FLANN_INDEX_KDTREE
    else:
        index_params = dict(
            algorithm=6,  # FLANN_INDEX_LSH
            table_number=6,
            key_size=12,
            multi_probe_level=1,
        )
    search_params = dict(checks=50)
    return cv2.FlannBasedMatcher(index_params, search_params)


def match_with_ratio_test(
    des_ref: np.ndarray,
    des_query: np.ndarray,
    flann: cv2.FlannBasedMatcher,
    ratio: float = 0.95,
    k: int = 2,
) -> List[cv2.DMatch]:
    """Lowe's ratio test: keep match if best distance < ratio * second_best."""
    if des_ref is None or des_query is None or len(des_ref) < 2 or len(des_query) < 2:
        return []
    if des_ref.dtype in (np.float32, np.float64):
        des_ref = np.float32(des_ref)
        des_query = np.float32(des_query)
    else:
        if des_ref.dtype != np.uint8:
            des_ref = np.uint8(des_ref)
        if des_query.dtype != np.uint8:
            des_query = np.uint8(des_query)
    matches = flann.knnMatch(des_ref, des_query, k=k)
    good = []
    for m_n in matches:
        if len(m_n) == 2 and m_n[0].distance < ratio * m_n[1].distance:
            good.append(m_n[0])
    return good


def filter_matches_kmeans_continuity(
    ref_kpts: List[cv2.KeyPoint],
    query_kpts: List[cv2.KeyPoint],
    matches: List[cv2.DMatch],
    ref_image: ReferenceImage,
    last_lat_lon: Optional[Tuple[float, float]],
    max_dist_from_last_m: float,
    max_dist_from_cluster_median_m: float,
    n_clusters: int,
    get_distance_metres,
    ref_pixel_to_geo,
) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """
    K-means on ref match coordinates; select cluster whose median (in geo) is within
    max_dist_from_last_m of last_lat_lon; keep matches within max_dist_from_cluster_median_m
    of that cluster median. Return list of (ref_pt, query_pt) in pixels.
    """
    if not matches:
        return []

    ref_pts = np.array([ref_kpts[m.queryIdx].pt for m in matches])
    query_pts = np.array([query_kpts[m.trainIdx].pt for m in matches])

    # K-means on ref image coordinates (still_x, still_y)
    X = ref_pts
    kmeans = KMeans(n_clusters=n_clusters, n_init=10, random_state=42)
    labels = kmeans.fit_predict(X)

    # For each cluster: median ref pixel -> ref geo; check distance to last_lat_lon
    from collections import Counter
    from operator import itemgetter

    count_per_cluster = Counter(labels)
    sorted_clusters = sorted(count_per_cluster.items(), key=itemgetter(1), reverse=True)

    r = ref_image
    selected_ref_pts = []
    selected_query_pts = []
    cluster_median_lat_lon = None

    for cluster_idx, _ in sorted_clusters[:3]:  # top 3 by count
        mask = labels == cluster_idx
        median_ref = np.median(X[mask], axis=0)
        ref_lat, ref_lon = ref_pixel_to_geo(
            median_ref[0], median_ref[1],
            r.center_lat, r.center_lon,
            r.height_px, r.width_px,
            r.y_m_per_px, r.x_m_per_px,
        )
        if last_lat_lon is not None:
            dist = get_distance_metres(ref_lat, ref_lon, last_lat_lon[0], last_lat_lon[1])
            if dist > max_dist_from_last_m:
                continue
        cluster_median_lat_lon = (ref_lat, ref_lon)
        # Keep matches in this cluster within max_dist_from_cluster_median_m of median (in geo)
        for i in np.where(mask)[0]:
            ref_pt = ref_pts[i]
            ref_lat_i, ref_lon_i = ref_pixel_to_geo(
                ref_pt[0], ref_pt[1],
                r.center_lat, r.center_lon,
                r.height_px, r.width_px,
                r.y_m_per_px, r.x_m_per_px,
            )
            dist_to_median = get_distance_metres(
                ref_lat_i, ref_lon_i,
                cluster_median_lat_lon[0], cluster_median_lat_lon[1],
            )
            if dist_to_median <= max_dist_from_cluster_median_m:
                selected_ref_pts.append(tuple(ref_pts[i]))
                selected_query_pts.append(tuple(query_pts[i]))
        if selected_ref_pts:
            break

    return list(zip(selected_ref_pts, selected_query_pts))
