"""
Pixel <-> geographic coordinate transforms.
Uses flat-earth approximation (suitable for small regions). ArduPilot-style distance.
"""

import math
from typing import Tuple

# Earth radius in metres (spherical approximation)
EARTH_RADIUS_M = 6378137.0


def get_distance_metres(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Ground distance in metres between two (lat, lon) points.
    Approximation; accurate for small distances (< ~10 km). From ArduPilot.
    """
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    return math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e5


def ref_pixel_to_geo(
    pixel_x: float,
    pixel_y: float,
    center_lat: float,
    center_lon: float,
    ref_height_px: int,
    ref_width_px: int,
    y_m_per_px: float,
    x_m_per_px: float,
) -> Tuple[float, float]:
    """
    Convert reference image pixel to (lat, lon).
    Pixel (0,0) is top-left; center of image is at (ref_width_px/2, ref_height_px/2).
    """
    # Offset from image center in pixels (y positive down in image)
    dx_px = pixel_x - ref_width_px / 2
    dy_px = pixel_y - ref_height_px / 2

    # Metres (image x -> east, image y -> south in typical north-up ref)
    east_m = dx_px * x_m_per_px
    south_m = -dy_px * y_m_per_px  # pixel y down -> north positive when we subtract
    north_m = -south_m

    # Flat-earth: delta lat = north_m / m_per_deg_lat, delta lon = east_m / m_per_deg_lon
    m_per_deg_lat = EARTH_RADIUS_M * math.pi / 180.0
    m_per_deg_lon = EARTH_RADIUS_M * math.pi / 180.0 * math.cos(math.radians(center_lat))

    lat = center_lat + (north_m / m_per_deg_lat)
    lon = center_lon + (east_m / m_per_deg_lon)
    return lat, lon


def camera_offset_to_geo(
    ref_lat: float,
    ref_lon: float,
    camera_pixel_x: float,
    camera_pixel_y: float,
    cam_center_x_px: float,
    cam_center_y_px: float,
    x_m_per_px: float,
    y_m_per_px: float,
) -> Tuple[float, float]:
    """
    Given a ref feature at (ref_lat, ref_lon), and the matching point in the camera
    image at (camera_pixel_x, camera_pixel_y), compute the camera position (lat, lon).
    Camera view: pixel x right -> east, pixel y down -> south.
    """
    dx_px = camera_pixel_x - cam_center_x_px
    dy_px = camera_pixel_y - cam_center_y_px

    # Camera x right -> east, y down -> south
    east_m = dx_px * x_m_per_px
    south_m = dy_px * y_m_per_px
    north_m = -south_m

    m_per_deg_lat = EARTH_RADIUS_M * math.pi / 180.0
    m_per_deg_lon = EARTH_RADIUS_M * math.pi / 180.0 * math.cos(math.radians(ref_lat))

    # Ref point is where the ground is under the camera's view at that pixel;
    # camera is above that point, so camera lat/lon is same as ref for flat ground.
    # (We are estimating camera horizontal position, not altitude.)
    lat = ref_lat + (north_m / m_per_deg_lat)
    lon = ref_lon + (east_m / m_per_deg_lon)
    return lat, lon


def geo_to_ref_pixel(
    lat: float,
    lon: float,
    center_lat: float,
    center_lon: float,
    ref_height_px: int,
    ref_width_px: int,
    y_m_per_px: float,
    x_m_per_px: float,
) -> Tuple[float, float]:
    """Convert (lat, lon) to reference image pixel (x, y). Inverse of ref_pixel_to_geo."""
    m_per_deg_lat = EARTH_RADIUS_M * math.pi / 180.0
    m_per_deg_lon = EARTH_RADIUS_M * math.pi / 180.0 * math.cos(math.radians(center_lat))
    north_m = (lat - center_lat) * m_per_deg_lat
    east_m = (lon - center_lon) * m_per_deg_lon
    south_m = -north_m
    dx_px = east_m / x_m_per_px
    dy_px = -south_m / y_m_per_px  # north positive -> pixel y negative from center
    pixel_x = ref_width_px / 2 + dx_px
    pixel_y = ref_height_px / 2 + dy_px
    return pixel_x, pixel_y
