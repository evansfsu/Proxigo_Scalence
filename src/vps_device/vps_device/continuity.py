"""
Continuity logic: select cluster of matches whose median position is within
a maximum distance of the last predicted position (avoid jumps).
Used inside features.filter_matches_kmeans_continuity; this module holds
shared constants and helpers if needed.
"""

# Continuity thresholds are in config (max_dist_from_last_m, max_dist_from_cluster_median_m).
# This module exists for any future shared helpers, e.g. temporal smoothing.

def select_cluster_by_continuity(
    cluster_median_lat_lon_per_cluster,
    last_lat_lon,
    get_distance_metres,
    max_dist_from_last_m: float,
):
    """
    Given list of (cluster_idx, (lat, lon)) for cluster medians, return the index
    of the cluster whose median is closest to last_lat_lon and within max_dist_from_last_m.
    """
    if last_lat_lon is None:
        return 0  # take first (e.g. largest) cluster
    best_idx = None
    best_dist = float("inf")
    for idx, (lat, lon) in cluster_median_lat_lon_per_cluster:
        d = get_distance_metres(lat, lon, last_lat_lon[0], last_lat_lon[1])
        if d <= max_dist_from_last_m and d < best_dist:
            best_dist = d
            best_idx = idx
    return best_idx if best_idx is not None else 0
