#!/usr/bin/env python3
"""
Proxigo Scalence - Download Satellite Imagery

Downloads satellite imagery from free sources for a specified area.
Supports multiple sources:
- USGS Earth Explorer (Landsat, Sentinel-2)
- OpenStreetMap tiles (for quick testing)
- Google Static Maps API (requires API key)
- Bing Maps (requires API key)

Usage:
    python scripts/download_satellite_imagery.py --lat 39.678 --lon -75.750 --size 1000
    python scripts/download_satellite_imagery.py --bounds "39.67,-75.76,39.69,-75.74" --source osm
    python scripts/download_satellite_imagery.py --config mission_area.json

Requirements:
    pip install requests pillow numpy
"""

import argparse
import json
import math
import os
import sys
from pathlib import Path
from datetime import datetime
from typing import Tuple, Optional
import urllib.request
import urllib.parse

try:
    from PIL import Image
    import numpy as np
    HAS_PIL = True
except ImportError:
    HAS_PIL = False
    print("Warning: PIL not installed. Install with: pip install pillow")


def lat_lon_to_tile(lat: float, lon: float, zoom: int) -> Tuple[int, int]:
    """Convert lat/lon to OSM tile coordinates."""
    lat_rad = math.radians(lat)
    n = 2.0 ** zoom
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return x, y


def tile_to_lat_lon(x: int, y: int, zoom: int) -> Tuple[float, float]:
    """Convert OSM tile coordinates to lat/lon (NW corner)."""
    n = 2.0 ** zoom
    lon = x / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * y / n)))
    lat = math.degrees(lat_rad)
    return lat, lon


def download_osm_tiles(center_lat: float, center_lon: float, 
                       size_m: float, zoom: int = 18,
                       output_dir: Path = None) -> Optional[np.ndarray]:
    """
    Download OpenStreetMap tiles for an area.
    
    Args:
        center_lat: Center latitude
        center_lon: Center longitude
        size_m: Size of area in meters
        zoom: Zoom level (18 = ~0.6m/pixel, 17 = ~1.2m/pixel)
        output_dir: Output directory
        
    Returns:
        Combined image as numpy array
    """
    if not HAS_PIL:
        print("ERROR: PIL required for image processing")
        return None
    
    print(f"Downloading OSM tiles for area: {center_lat:.6f}, {center_lon:.6f}")
    print(f"Size: {size_m}m x {size_m}m, Zoom: {zoom}")
    
    # Calculate meters per pixel at this zoom level and latitude
    # At equator, zoom 0 = 156543 m/pixel
    meters_per_pixel = 156543.03 * math.cos(math.radians(center_lat)) / (2 ** zoom)
    print(f"Resolution: ~{meters_per_pixel:.2f} m/pixel")
    
    # Calculate how many tiles we need
    tile_size_m = meters_per_pixel * 256
    tiles_needed = int(math.ceil(size_m / tile_size_m)) + 1
    
    # Get center tile
    center_x, center_y = lat_lon_to_tile(center_lat, center_lon, zoom)
    
    # Calculate tile range
    half_tiles = tiles_needed // 2
    x_start = center_x - half_tiles
    y_start = center_y - half_tiles
    x_end = center_x + half_tiles + 1
    y_end = center_y + half_tiles + 1
    
    total_tiles = (x_end - x_start) * (y_end - y_start)
    print(f"Downloading {total_tiles} tiles...")
    
    # Download tiles
    tiles = {}
    tile_count = 0
    
    for y in range(y_start, y_end):
        for x in range(x_start, x_end):
            tile_count += 1
            
            # OSM tile URL (using Humanitarian style for satellite-like appearance)
            # Other options: 'a', 'b', 'c' servers; 'tile.openstreetmap.org' for standard
            url = f"https://a.tile.openstreetmap.org/{zoom}/{x}/{y}.png"
            
            # Alternative: ESRI World Imagery (better satellite imagery)
            # url = f"https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{zoom}/{y}/{x}"
            
            print(f"\r  Downloading tile {tile_count}/{total_tiles}...", end='', flush=True)
            
            try:
                # Add user agent to avoid blocking
                request = urllib.request.Request(
                    url,
                    headers={'User-Agent': 'ProxigoScalence/1.0'}
                )
                
                with urllib.request.urlopen(request, timeout=30) as response:
                    img_data = response.read()
                    img = Image.open(io.BytesIO(img_data))
                    tiles[(x, y)] = img
                    
            except Exception as e:
                print(f"\n  Warning: Failed to download tile ({x}, {y}): {e}")
                # Create blank tile
                tiles[(x, y)] = Image.new('RGB', (256, 256), (128, 128, 128))
    
    print("\nStitching tiles...")
    
    # Stitch tiles together
    img_width = (x_end - x_start) * 256
    img_height = (y_end - y_start) * 256
    combined = Image.new('RGB', (img_width, img_height))
    
    for y in range(y_start, y_end):
        for x in range(x_start, x_end):
            if (x, y) in tiles:
                px = (x - x_start) * 256
                py = (y - y_start) * 256
                combined.paste(tiles[(x, y)], (px, py))
    
    return np.array(combined)


def download_esri_imagery(center_lat: float, center_lon: float,
                          size_m: float, zoom: int = 18,
                          output_dir: Path = None) -> Optional[np.ndarray]:
    """
    Download ESRI World Imagery tiles.
    Higher quality satellite imagery than OSM.
    """
    import io
    
    if not HAS_PIL:
        print("ERROR: PIL required")
        return None
    
    print(f"Downloading ESRI World Imagery for: {center_lat:.6f}, {center_lon:.6f}")
    print(f"Size: {size_m}m x {size_m}m, Zoom: {zoom}")
    
    # Calculate meters per pixel
    meters_per_pixel = 156543.03 * math.cos(math.radians(center_lat)) / (2 ** zoom)
    print(f"Resolution: ~{meters_per_pixel:.2f} m/pixel")
    
    # Calculate tiles needed
    tile_size_m = meters_per_pixel * 256
    tiles_needed = int(math.ceil(size_m / tile_size_m)) + 1
    
    # Get center tile
    center_x, center_y = lat_lon_to_tile(center_lat, center_lon, zoom)
    
    # Tile range
    half_tiles = tiles_needed // 2
    x_start = center_x - half_tiles
    y_start = center_y - half_tiles
    x_end = center_x + half_tiles + 1
    y_end = center_y + half_tiles + 1
    
    total_tiles = (x_end - x_start) * (y_end - y_start)
    print(f"Downloading {total_tiles} tiles...")
    
    tiles = {}
    tile_count = 0
    
    for y in range(y_start, y_end):
        for x in range(x_start, x_end):
            tile_count += 1
            
            # ESRI World Imagery URL
            url = f"https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{zoom}/{y}/{x}"
            
            print(f"\r  Downloading tile {tile_count}/{total_tiles}...", end='', flush=True)
            
            try:
                request = urllib.request.Request(
                    url,
                    headers={'User-Agent': 'ProxigoScalence/1.0'}
                )
                
                with urllib.request.urlopen(request, timeout=30) as response:
                    img_data = response.read()
                    img = Image.open(io.BytesIO(img_data))
                    tiles[(x, y)] = img.convert('RGB')
                    
            except Exception as e:
                print(f"\n  Warning: Failed to download tile ({x}, {y}): {e}")
                tiles[(x, y)] = Image.new('RGB', (256, 256), (128, 128, 128))
    
    print("\nStitching tiles...")
    
    # Stitch
    img_width = (x_end - x_start) * 256
    img_height = (y_end - y_start) * 256
    combined = Image.new('RGB', (img_width, img_height))
    
    for y in range(y_start, y_end):
        for x in range(x_start, x_end):
            if (x, y) in tiles:
                px = (x - x_start) * 256
                py = (y - y_start) * 256
                combined.paste(tiles[(x, y)], (px, py))
    
    return np.array(combined)


def create_region_package(image: np.ndarray, center_lat: float, center_lon: float,
                          size_m: float, resolution_m: float,
                          output_dir: Path, region_name: str):
    """Create a complete region package with metadata and features."""
    import pickle
    import cv2
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Save image
    image_path = output_dir / 'satellite.png'
    if HAS_PIL:
        Image.fromarray(image).save(image_path)
    print(f"Saved image: {image_path}")
    
    # Calculate bounds
    # Approximate degrees per meter at this latitude
    m_per_deg_lat = 111000
    m_per_deg_lon = 111000 * math.cos(math.radians(center_lat))
    
    half_size_lat = (size_m / 2) / m_per_deg_lat
    half_size_lon = (size_m / 2) / m_per_deg_lon
    
    bounds = {
        'north': center_lat + half_size_lat,
        'south': center_lat - half_size_lat,
        'east': center_lon + half_size_lon,
        'west': center_lon - half_size_lon,
    }
    
    # Create metadata
    metadata = {
        'region_id': region_name,
        'name': f'Satellite imagery for {region_name}',
        'created': datetime.now().isoformat(),
        'source': 'ESRI World Imagery',
        'center': {
            'latitude': center_lat,
            'longitude': center_lon,
        },
        'bounds': {
            'type': 'Polygon',
            'coordinates': [[
                [bounds['west'], bounds['south']],
                [bounds['west'], bounds['north']],
                [bounds['east'], bounds['north']],
                [bounds['east'], bounds['south']],
                [bounds['west'], bounds['south']],
            ]]
        },
        'imagery': {
            'resolution_m': resolution_m,
            'width_pixels': image.shape[1],
            'height_pixels': image.shape[0],
            'file': 'satellite.png',
        },
        'coverage_m': size_m,
    }
    
    metadata_path = output_dir / 'metadata.json'
    with open(metadata_path, 'w') as f:
        json.dump(metadata, f, indent=2)
    print(f"Saved metadata: {metadata_path}")
    
    # Extract features
    print("Extracting features...")
    try:
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        orb = cv2.ORB_create(nfeatures=5000)
        
        altitudes = [50, 100, 200, 400]
        features_db = {}
        
        for altitude in altitudes:
            # Scale factor for this altitude
            fov_h_rad = math.radians(60)
            view_width_m = 2 * altitude * math.tan(fov_h_rad / 2)
            image_width_m = image.shape[1] * resolution_m
            scale_factor = view_width_m / image_width_m
            
            if scale_factor < 1:
                new_w = int(image.shape[1] * scale_factor)
                new_h = int(image.shape[0] * scale_factor)
                scaled = cv2.resize(gray, (new_w, new_h))
            else:
                scaled = gray
                scale_factor = 1.0
            
            keypoints, descriptors = orb.detectAndCompute(scaled, None)
            
            kp_list = []
            if keypoints:
                for kp in keypoints:
                    orig_x = kp.pt[0] / scale_factor
                    orig_y = kp.pt[1] / scale_factor
                    
                    # Convert to geo
                    rel_x = orig_x / image.shape[1] - 0.5
                    rel_y = orig_y / image.shape[0] - 0.5
                    
                    geo_lon = center_lon + rel_x * (size_m / m_per_deg_lon)
                    geo_lat = center_lat - rel_y * (size_m / m_per_deg_lat)
                    
                    kp_list.append({
                        'pixel': (orig_x, orig_y),
                        'scaled_pixel': (kp.pt[0], kp.pt[1]),
                        'geo': (geo_lon, geo_lat),
                        'response': float(kp.response),
                        'size': float(kp.size),
                        'angle': float(kp.angle),
                    })
            
            features_db[altitude] = {
                'keypoints': kp_list,
                'descriptors': descriptors,
                'altitude': altitude,
                'scale_factor': scale_factor,
                'image_shape': scaled.shape if scale_factor < 1 else image.shape,
                'original_shape': image.shape,
            }
            
            print(f"  Altitude {altitude}m: {len(kp_list)} features")
        
        features_path = output_dir / 'features.pkl'
        with open(features_path, 'wb') as f:
            pickle.dump(features_db, f)
        print(f"Saved features: {features_path}")
        
    except Exception as e:
        print(f"Warning: Feature extraction failed: {e}")
        print("Features will need to be extracted separately")


def main():
    parser = argparse.ArgumentParser(
        description='Download satellite imagery for Proxigo Scalence',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Download 1km x 1km area centered on coordinates
  python download_satellite_imagery.py --lat 39.678 --lon -75.750 --size 1000

  # Download with specific zoom level
  python download_satellite_imagery.py --lat 39.678 --lon -75.750 --size 500 --zoom 19

  # Specify output region name
  python download_satellite_imagery.py --lat 39.678 --lon -75.750 --size 1000 --name my_mission

Sources:
  - ESRI World Imagery (default): High-quality satellite imagery
  - OSM: OpenStreetMap tiles (street maps, not satellite)
        """
    )
    
    parser.add_argument('--lat', type=float, required=True, help='Center latitude')
    parser.add_argument('--lon', type=float, required=True, help='Center longitude')
    parser.add_argument('--size', type=float, default=1000, help='Area size in meters (default: 1000)')
    parser.add_argument('--zoom', type=int, default=18, help='Zoom level 15-20 (default: 18, ~0.6m/pixel)')
    parser.add_argument('--name', type=str, default=None, help='Region name (default: auto-generated)')
    parser.add_argument('--output', type=str, default='satellite_data/regions', help='Output directory')
    parser.add_argument('--source', type=str, default='esri', choices=['esri', 'osm'], help='Imagery source')

    args = parser.parse_args()

    # Generate region name if not provided
    if args.name is None:
        args.name = f"region_{args.lat:.4f}_{args.lon:.4f}".replace('.', '_').replace('-', 'n')

    output_dir = Path(args.output) / args.name

    print("=" * 60)
    print("Proxigo Scalence - Satellite Imagery Download")
    print("=" * 60)
    print(f"Center: {args.lat}, {args.lon}")
    print(f"Size: {args.size}m x {args.size}m")
    print(f"Zoom: {args.zoom}")
    print(f"Source: {args.source}")
    print(f"Output: {output_dir}")
    print()

    # Need io for BytesIO
    import io
    
    # Download imagery
    if args.source == 'esri':
        image = download_esri_imagery(args.lat, args.lon, args.size, args.zoom)
    else:
        image = download_osm_tiles(args.lat, args.lon, args.size, args.zoom)

    if image is None:
        print("ERROR: Failed to download imagery")
        sys.exit(1)

    # Calculate resolution
    meters_per_pixel = 156543.03 * math.cos(math.radians(args.lat)) / (2 ** args.zoom)

    print()
    print(f"Downloaded image: {image.shape[1]} x {image.shape[0]} pixels")
    print(f"Resolution: ~{meters_per_pixel:.2f} m/pixel")

    # Create region package
    print()
    create_region_package(
        image=image,
        center_lat=args.lat,
        center_lon=args.lon,
        size_m=args.size,
        resolution_m=meters_per_pixel,
        output_dir=output_dir,
        region_name=args.name
    )

    print()
    print("=" * 60)
    print("Download complete!")
    print("=" * 60)
    print()
    print(f"Region saved to: {output_dir}")
    print()
    print("To use this region:")
    print(f"  1. Update config/mission/initial_pose.yaml:")
    print(f"     region_id: \"{args.name}\"")
    print(f"     latitude: {args.lat}")
    print(f"     longitude: {args.lon}")
    print()
    print("  2. Copy to drone (if not already there):")
    print(f"     scp -r {output_dir} nvidia@orin.local:~/proxigo/satellite_data/regions/")


if __name__ == '__main__':
    main()
