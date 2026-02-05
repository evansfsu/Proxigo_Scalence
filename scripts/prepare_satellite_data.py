#!/usr/bin/env python3
"""
Proxigo Scalence - Satellite Data Preparation Tool

This script preprocesses satellite imagery for use with the VIO satellite
matching system. It extracts features at multiple altitude scales and
builds a spatial index for efficient runtime matching.

Usage:
    python scripts/prepare_satellite_data.py <geotiff_file> <output_dir> [options]
    
Example:
    python scripts/prepare_satellite_data.py \
        mission_area.tif \
        satellite_data/regions/mission_001 \
        --resolution 0.3 \
        --altitudes 50,100,200,400

Requirements:
    pip install rasterio numpy opencv-contrib-python shapely pyproj pyyaml
"""

import argparse
import json
import pickle
from pathlib import Path
from datetime import datetime

import numpy as np
import cv2
import yaml


def parse_args():
    parser = argparse.ArgumentParser(
        description='Prepare satellite imagery for VIO matching'
    )
    parser.add_argument(
        'input_file',
        type=str,
        help='Input GeoTIFF satellite image'
    )
    parser.add_argument(
        'output_dir',
        type=str,
        help='Output directory for processed data'
    )
    parser.add_argument(
        '--resolution',
        type=float,
        default=0.5,
        help='Ground resolution in meters/pixel (default: 0.5)'
    )
    parser.add_argument(
        '--altitudes',
        type=str,
        default='50,100,200,400',
        help='Comma-separated list of altitude scales in meters (default: 50,100,200,400)'
    )
    parser.add_argument(
        '--features',
        type=int,
        default=5000,
        help='Number of features to extract per scale (default: 5000)'
    )
    parser.add_argument(
        '--region-name',
        type=str,
        default='',
        help='Human-readable region name'
    )
    return parser.parse_args()


def load_geotiff(filepath: str) -> tuple:
    """Load GeoTIFF image with georeferencing info."""
    try:
        import rasterio
        
        with rasterio.open(filepath) as src:
            # Read image data
            image = src.read()
            
            # Get georeferencing
            transform = src.transform
            crs = src.crs
            bounds = src.bounds
            
            # Convert to RGB if multi-band
            if image.shape[0] >= 3:
                image = np.moveaxis(image[:3], 0, -1)
            elif image.shape[0] == 1:
                image = np.repeat(image, 3, axis=0)
                image = np.moveaxis(image, 0, -1)
            
            # Ensure uint8
            if image.dtype != np.uint8:
                image = ((image - image.min()) / (image.max() - image.min()) * 255).astype(np.uint8)
            
            return image, transform, crs, bounds
            
    except ImportError:
        print("Warning: rasterio not installed. Using OpenCV for image loading.")
        print("Georeferencing info will not be available.")
        
        image = cv2.imread(filepath)
        if image is None:
            raise ValueError(f"Could not load image: {filepath}")
        
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Dummy georeferencing (will need manual configuration)
        return image, None, None, None


def pixel_to_geo(pixel_x: float, pixel_y: float, transform) -> tuple:
    """Convert pixel coordinates to geographic coordinates."""
    if transform is None:
        return (pixel_x, pixel_y)  # No transform available
    
    try:
        import rasterio
        lon, lat = rasterio.transform.xy(transform, pixel_y, pixel_x)
        return (lon, lat)
    except:
        return (pixel_x, pixel_y)


def extract_features_at_scale(image: np.ndarray, transform, altitude: float, 
                               resolution: float, num_features: int) -> dict:
    """Extract ORB features for a specific altitude scale."""
    
    # Calculate expected view size at this altitude
    # Assuming ~60 degree horizontal FOV
    fov_h_rad = np.radians(60)
    view_width_m = 2 * altitude * np.tan(fov_h_rad / 2)
    
    # Calculate scale factor
    image_width_m = image.shape[1] * resolution
    scale_factor = view_width_m / image_width_m
    
    # Scale image if needed
    if scale_factor < 1:
        new_width = int(image.shape[1] * scale_factor)
        new_height = int(image.shape[0] * scale_factor)
        scaled_image = cv2.resize(image, (new_width, new_height))
    else:
        scaled_image = image
        scale_factor = 1.0
    
    # Convert to grayscale
    gray = cv2.cvtColor(scaled_image, cv2.COLOR_RGB2GRAY)
    
    # Create ORB detector
    orb = cv2.ORB_create(nfeatures=num_features)
    
    # Detect and compute
    keypoints, descriptors = orb.detectAndCompute(gray, None)
    
    if keypoints is None or len(keypoints) == 0:
        return {
            'keypoints': [],
            'descriptors': None,
            'altitude': altitude,
            'scale_factor': scale_factor,
            'image_shape': scaled_image.shape
        }
    
    # Convert keypoints to serializable format with geo-coordinates
    kp_list = []
    for kp in keypoints:
        # Scale back to original image coordinates
        orig_x = kp.pt[0] / scale_factor
        orig_y = kp.pt[1] / scale_factor
        
        # Get geographic coordinates
        geo = pixel_to_geo(orig_x, orig_y, transform)
        
        kp_list.append({
            'pixel': (orig_x, orig_y),
            'scaled_pixel': (kp.pt[0], kp.pt[1]),
            'geo': geo,
            'response': float(kp.response),
            'size': float(kp.size),
            'angle': float(kp.angle)
        })
    
    return {
        'keypoints': kp_list,
        'descriptors': descriptors,
        'altitude': altitude,
        'scale_factor': scale_factor,
        'image_shape': scaled_image.shape,
        'original_shape': image.shape
    }


def create_metadata(input_file: str, output_dir: Path, resolution: float,
                   crs, bounds, region_name: str) -> dict:
    """Create metadata JSON for the region."""
    
    region_id = output_dir.name
    
    metadata = {
        'region_id': region_id,
        'name': region_name if region_name else region_id,
        'created': datetime.now().isoformat(),
        'source_file': str(Path(input_file).name),
        'imagery': {
            'resolution_m': resolution,
            'file': 'satellite.tif'
        }
    }
    
    if bounds is not None:
        metadata['bounds'] = {
            'type': 'Polygon',
            'coordinates': [[
                [bounds.left, bounds.bottom],
                [bounds.left, bounds.top],
                [bounds.right, bounds.top],
                [bounds.right, bounds.bottom],
                [bounds.left, bounds.bottom]
            ]]
        }
    
    if crs is not None:
        metadata['crs'] = str(crs)
    
    return metadata


def main():
    args = parse_args()
    
    print("=" * 60)
    print("Proxigo Scalence - Satellite Data Preparation")
    print("=" * 60)
    
    # Parse arguments
    input_file = Path(args.input_file)
    output_dir = Path(args.output_dir)
    resolution = args.resolution
    altitudes = [int(a.strip()) for a in args.altitudes.split(',')]
    num_features = args.features
    
    print(f"Input file: {input_file}")
    print(f"Output directory: {output_dir}")
    print(f"Resolution: {resolution} m/px")
    print(f"Altitude scales: {altitudes}")
    print(f"Features per scale: {num_features}")
    print()
    
    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Load image
    print("Loading satellite image...")
    image, transform, crs, bounds = load_geotiff(str(input_file))
    print(f"Image shape: {image.shape}")
    
    if bounds:
        print(f"Bounds: {bounds}")
    
    # Extract features at each altitude scale
    print("\nExtracting features...")
    features_db = {}
    
    for altitude in altitudes:
        print(f"  Altitude {altitude}m... ", end='', flush=True)
        features = extract_features_at_scale(
            image, transform, altitude, resolution, num_features
        )
        features_db[altitude] = features
        print(f"{len(features['keypoints'])} features")
    
    # Save features database
    print("\nSaving features database...")
    features_file = output_dir / 'features.pkl'
    with open(features_file, 'wb') as f:
        pickle.dump(features_db, f)
    print(f"  Saved to: {features_file}")
    
    # Create and save metadata
    print("\nCreating metadata...")
    metadata = create_metadata(
        str(input_file), output_dir, resolution, crs, bounds, args.region_name
    )
    metadata_file = output_dir / 'metadata.json'
    with open(metadata_file, 'w') as f:
        json.dump(metadata, f, indent=2)
    print(f"  Saved to: {metadata_file}")
    
    # Copy/link original image
    print("\nCopying satellite image...")
    import shutil
    dest_image = output_dir / 'satellite.tif'
    shutil.copy2(input_file, dest_image)
    print(f"  Saved to: {dest_image}")
    
    # Summary
    print("\n" + "=" * 60)
    print("Processing complete!")
    print("=" * 60)
    print(f"\nOutput directory: {output_dir}")
    print(f"Files created:")
    for f in output_dir.iterdir():
        size_mb = f.stat().st_size / (1024 * 1024)
        print(f"  - {f.name} ({size_mb:.2f} MB)")
    
    total_features = sum(len(f['keypoints']) for f in features_db.values())
    print(f"\nTotal features extracted: {total_features}")
    
    print("\nTo use this data:")
    print(f"  1. Copy {output_dir} to the drone's /satellite_data/regions/ folder")
    print(f"  2. Update config/mission/initial_pose.yaml with the region_id: {output_dir.name}")


if __name__ == '__main__':
    main()
