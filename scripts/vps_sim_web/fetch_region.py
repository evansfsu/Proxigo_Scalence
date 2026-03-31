"""Fetch satellite imagery for a bbox (Esri World Imagery) and return image + metadata."""

import io
import math
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Optional, Tuple

try:
    from PIL import Image
    import numpy as np
except ImportError:
    Image = None
    np = None


def lat_lon_to_tile(lat: float, lon: float, zoom: int) -> Tuple[int, int]:
    n = 2.0 ** zoom
    x = int((lon + 180.0) / 360.0 * n)
    lat_rad = math.radians(lat)
    y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return x, y


MAX_TILES = 400  # max tiles to fetch; larger areas need more tiles
TILE_TIMEOUT = 12
# Placeholder detection: real satellite tiles have high variance and varied colors; "no data" tiles are gray-ish
# Esri "Map data not yet available" = light gray background + gray text (mean ~200+, std moderate)
PLACEHOLDER_STD_MAX = 48  # below this = solid gray or simple graphic
LIGHT_GRAY_MEAN_MIN = 175  # Esri "no data" tiles are light gray
LIGHT_GRAY_STD_MAX = 58   # still relatively uniform
GRAY_BAND_LO, GRAY_BAND_HI = 105, 150  # mid gray band
GRAY_PIXEL_FRAC_MAX = 0.82
LIGHT_PIXEL_THRESH = 165  # pixel value above this = light gray / white
LIGHT_PIXEL_FRAC_MAX = 0.78  # if this fraction of pixels is light, treat as "no data" tile


def _is_placeholder_tile(img: "Image.Image") -> bool:
    """True if the tile looks like gray/error or Esri 'Map data not yet available'."""
    arr = np.array(img)
    if arr.ndim == 3:
        gray = np.mean(arr, axis=2).astype(np.float64)
    else:
        gray = arr.astype(np.float64)
    mean, std = float(np.mean(gray)), float(np.std(gray))
    # Low variance = solid gray or simple "no data" graphic
    if std < PLACEHOLDER_STD_MAX:
        return True
    # Esri "Map data not yet available" = light gray background (mean 180-230), moderate std
    if mean >= LIGHT_GRAY_MEAN_MIN and std < LIGHT_GRAY_STD_MAX:
        return True
    # Dominant light gray/white pixels (characteristic of "no data" overlay)
    light_frac = np.sum(gray >= LIGHT_PIXEL_THRESH) / gray.size
    if light_frac >= LIGHT_PIXEL_FRAC_MAX:
        return True
    # Dominant mid gray band
    in_band = np.sum((gray >= GRAY_BAND_LO) & (gray <= GRAY_BAND_HI))
    if in_band / gray.size >= GRAY_PIXEL_FRAC_MAX:
        return True
    return False


def _fetch_one_tile(zoom: int, x: int, y: int) -> Tuple[Tuple[int, int], "Image.Image", bool]:
    """Fetch a single tile; returns ((x, y), PIL Image, is_placeholder)."""
    url = f"https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{zoom}/{y}/{x}"
    headers = {"User-Agent": "Mozilla/5.0 (Windows NT 10.0; rv:109.0) Gecko/20100101 Firefox/115.0"}
    for attempt in range(2):
        try:
            req = urllib.request.Request(url, headers=headers)
            with urllib.request.urlopen(req, timeout=TILE_TIMEOUT) as resp:
                img = Image.open(io.BytesIO(resp.read())).convert("RGB")
                is_placeholder = _is_placeholder_tile(img)
                return (x, y), img, is_placeholder
        except Exception:
            if attempt == 0:
                continue
            placeholder = Image.new("RGB", (256, 256), (128, 128, 128))
            return (x, y), placeholder, True
    placeholder = Image.new("RGB", (256, 256), (128, 128, 128))
    return (x, y), placeholder, True


def fetch_imagery_bbox(
    north: float,
    south: float,
    east: float,
    west: float,
    zoom: int = 18,
) -> Tuple[Optional["np.ndarray"], float, float, float]:
    """
    Fetch Esri World Imagery tiles for the bbox (in parallel). Returns (image_rgb, center_lat, center_lon, resolution_m)
    or (None, 0, 0, 0) only if PIL/numpy are not available.
    """
    if Image is None or np is None:
        raise RuntimeError("PIL or numpy not available. Install with: pip install Pillow numpy")
    center_lat = (north + south) / 2
    center_lon = (east + west) / 2
    x_min = y_min = x_max = y_max = 0
    for z in range(zoom, 11, -1):
        x_min, y_min = lat_lon_to_tile(north, west, z)
        x_max, y_max = lat_lon_to_tile(south, east, z)
        x_min, x_max = min(x_min, x_max), max(x_min, x_max)
        y_min, y_max = min(y_min, y_max), max(y_min, y_max)
        ntiles = (x_max - x_min + 1) * (y_max - y_min + 1)
        if ntiles <= MAX_TILES:
            zoom = z
            break
    meters_per_pixel = 156543.03 * math.cos(math.radians(center_lat)) / (2 ** zoom)
    tile_list = [(x, y) for y in range(y_min, y_max + 1) for x in range(x_min, x_max + 1)]
    tiles = {}
    placeholder_count = 0
    with ThreadPoolExecutor(max_workers=min(6, len(tile_list))) as ex:
        futures = {ex.submit(_fetch_one_tile, zoom, x, y): (x, y) for (x, y) in tile_list}
        for fut in as_completed(futures):
            (x, y), img, is_placeholder = fut.result()
            tiles[(x, y)] = img
            if is_placeholder:
                placeholder_count += 1
    # No rejection: always return stitched image so the run can proceed (even if some tiles failed)
    w = (x_max - x_min + 1) * 256
    h = (y_max - y_min + 1) * 256
    combined = Image.new("RGB", (w, h))
    for y in range(y_min, y_max + 1):
        for x in range(x_min, x_max + 1):
            if (x, y) in tiles:
                combined.paste(tiles[(x, y)], ((x - x_min) * 256, (y - y_min) * 256))
    arr = np.array(combined)
    return arr, center_lat, center_lon, meters_per_pixel
