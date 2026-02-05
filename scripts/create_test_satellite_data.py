#!/usr/bin/env python3
"""
Proxigo Scalence - Create Synthetic Test Satellite Data

This script creates synthetic satellite test data for development and testing
when you don't have access to real satellite imagery.

Usage:
    python scripts/create_test_satellite_data.py

This will create a test region in satellite_data/regions/test_region/
"""

import json
import pickle
import os
from pathlib import Path
from datetime import datetime

import numpy as np

# Try to import optional dependencies
try:
    import cv2
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False
    print("Warning: OpenCV not installed. Using PIL for image generation.")

try:
    from PIL import Image, ImageDraw, ImageFont
    HAS_PIL = True
except ImportError:
    HAS_PIL = False


def create_synthetic_satellite_image(width: int, height: int) -> np.ndarray:
    """Create a synthetic satellite-like image with features."""
    
    # Create base terrain (brown/green gradient)
    image = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Base color (brown earth tone)
    image[:, :] = [60, 80, 50]  # BGR: brownish-green
    
    # Add noise for texture
    noise = np.random.randint(-20, 20, (height, width, 3), dtype=np.int16)
    image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    
    # Add some "field" patterns (rectangles of different colors)
    np.random.seed(42)  # Reproducible
    for _ in range(15):
        x1 = np.random.randint(0, width - 100)
        y1 = np.random.randint(0, height - 100)
        x2 = x1 + np.random.randint(50, 200)
        y2 = y1 + np.random.randint(50, 200)
        
        # Random field color (various greens and browns)
        colors = [
            [40, 100, 40],   # Dark green
            [60, 120, 60],   # Medium green
            [50, 90, 70],    # Brown-green
            [80, 140, 80],   # Light green
            [70, 70, 50],    # Brown
        ]
        color = colors[np.random.randint(0, len(colors))]
        
        image[y1:y2, x1:x2] = color
    
    # Add "roads" (gray lines)
    # Horizontal roads
    for _ in range(3):
        y = np.random.randint(50, height - 50)
        thickness = np.random.randint(3, 8)
        image[y:y+thickness, :] = [100, 100, 100]
    
    # Vertical roads
    for _ in range(3):
        x = np.random.randint(50, width - 50)
        thickness = np.random.randint(3, 8)
        image[:, x:x+thickness] = [100, 100, 100]
    
    # Add "buildings" (small bright rectangles)
    for _ in range(20):
        x = np.random.randint(0, width - 20)
        y = np.random.randint(0, height - 20)
        w = np.random.randint(5, 20)
        h = np.random.randint(5, 20)
        image[y:y+h, x:x+w] = [180, 180, 180]
    
    # Add some corner markers (for easy visual alignment testing)
    marker_size = 30
    marker_color = [0, 0, 255]  # Red
    
    # Top-left corner
    image[10:10+marker_size, 10:10+marker_size//3] = marker_color
    image[10:10+marker_size//3, 10:10+marker_size] = marker_color
    
    # Top-right corner
    image[10:10+marker_size, width-10-marker_size//3:width-10] = marker_color
    image[10:10+marker_size//3, width-10-marker_size:width-10] = marker_color
    
    # Bottom-left corner
    image[height-10-marker_size:height-10, 10:10+marker_size//3] = marker_color
    image[height-10-marker_size//3:height-10, 10:10+marker_size] = marker_color
    
    # Bottom-right corner
    image[height-10-marker_size:height-10, width-10-marker_size//3:width-10] = marker_color
    image[height-10-marker_size//3:height-10, width-10-marker_size:width-10] = marker_color
    
    return image


def extract_orb_features(image: np.ndarray, num_features: int = 2000) -> dict:
    """Extract ORB features from image."""
    
    if not HAS_OPENCV:
        # Return dummy features if OpenCV not available
        return {
            'keypoints': [],
            'descriptors': None,
            'note': 'OpenCV not available - using dummy features'
        }
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(nfeatures=num_features)
    keypoints, descriptors = orb.detectAndCompute(gray, None)
    
    if keypoints is None:
        return {'keypoints': [], 'descriptors': None}
    
    # Convert keypoints to serializable format
    kp_list = []
    for kp in keypoints:
        kp_list.append({
            'pixel': (float(kp.pt[0]), float(kp.pt[1])),
            'scaled_pixel': (float(kp.pt[0]), float(kp.pt[1])),
            'geo': (
                -75.750456 + (kp.pt[0] / image.shape[1]) * 0.01,  # Longitude
                39.678123 + (kp.pt[1] / image.shape[0]) * 0.01   # Latitude
            ),
            'response': float(kp.response),
            'size': float(kp.size),
            'angle': float(kp.angle)
        })
    
    return {
        'keypoints': kp_list,
        'descriptors': descriptors
    }


def main():
    print("=" * 60)
    print("Proxigo Scalence - Creating Test Satellite Data")
    print("=" * 60)
    
    # Configuration
    image_width = 2000
    image_height = 2000
    resolution_m = 0.5  # meters per pixel
    altitudes = [50, 100, 200, 400]
    
    # Test region location (Delaware, USA)
    center_lat = 39.678123
    center_lon = -75.750456
    
    # Calculate bounds
    # At ~40°N latitude, 1° longitude ≈ 85km, 1° latitude ≈ 111km
    width_m = image_width * resolution_m
    height_m = image_height * resolution_m
    
    delta_lon = width_m / 85000  # degrees
    delta_lat = height_m / 111000  # degrees
    
    # Output directory
    output_dir = Path("satellite_data/regions/test_region")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\nConfiguration:")
    print(f"  Image size: {image_width} x {image_height} pixels")
    print(f"  Resolution: {resolution_m} m/pixel")
    print(f"  Coverage: {width_m} x {height_m} meters")
    print(f"  Center: ({center_lat}, {center_lon})")
    print(f"  Altitude scales: {altitudes}")
    print(f"  Output: {output_dir}")
    
    # Create synthetic satellite image
    print("\n>>> Creating synthetic satellite image...")
    image = create_synthetic_satellite_image(image_width, image_height)
    print(f"    Image shape: {image.shape}")
    
    # Save image
    image_path = output_dir / "satellite_synthetic.png"
    if HAS_OPENCV:
        cv2.imwrite(str(image_path), image)
    elif HAS_PIL:
        # Convert BGR to RGB for PIL
        pil_image = Image.fromarray(image[:, :, ::-1])
        pil_image.save(str(image_path))
    else:
        # Save as raw numpy
        np.save(str(output_dir / "satellite_synthetic.npy"), image)
        print("    Saved as .npy (no image library available)")
    print(f"    Saved to: {image_path}")
    
    # Extract features at each altitude scale
    print("\n>>> Extracting features...")
    features_db = {}
    
    for altitude in altitudes:
        print(f"    Altitude {altitude}m... ", end='', flush=True)
        
        # Calculate scale factor for this altitude
        fov_h_rad = np.radians(60)  # 60 degree horizontal FOV
        view_width_m = 2 * altitude * np.tan(fov_h_rad / 2)
        image_width_m = image_width * resolution_m
        scale_factor = view_width_m / image_width_m
        
        # Scale image
        if HAS_OPENCV and scale_factor < 1:
            new_width = int(image_width * scale_factor)
            new_height = int(image_height * scale_factor)
            scaled_image = cv2.resize(image, (new_width, new_height))
        else:
            scaled_image = image
            scale_factor = 1.0
        
        # Extract features
        features = extract_orb_features(scaled_image)
        features['altitude'] = altitude
        features['scale_factor'] = scale_factor
        features['image_shape'] = scaled_image.shape
        features['original_shape'] = image.shape
        
        features_db[altitude] = features
        print(f"{len(features['keypoints'])} features")
    
    # Save features database
    print("\n>>> Saving features database...")
    features_file = output_dir / "features.pkl"
    with open(features_file, 'wb') as f:
        pickle.dump(features_db, f)
    print(f"    Saved to: {features_file}")
    
    # Create metadata
    print("\n>>> Creating metadata...")
    metadata = {
        'region_id': 'test_region',
        'name': 'Test Region (Synthetic)',
        'created': datetime.now().isoformat(),
        'source_file': 'synthetic',
        'note': 'This is synthetic test data for development',
        'bounds': {
            'type': 'Polygon',
            'coordinates': [[
                [center_lon - delta_lon/2, center_lat - delta_lat/2],
                [center_lon - delta_lon/2, center_lat + delta_lat/2],
                [center_lon + delta_lon/2, center_lat + delta_lat/2],
                [center_lon + delta_lon/2, center_lat - delta_lat/2],
                [center_lon - delta_lon/2, center_lat - delta_lat/2]
            ]]
        },
        'center': {
            'latitude': center_lat,
            'longitude': center_lon
        },
        'imagery': {
            'resolution_m': resolution_m,
            'width_pixels': image_width,
            'height_pixels': image_height,
            'file': 'satellite_synthetic.png'
        },
        'altitudes': altitudes,
        'total_features': sum(len(f['keypoints']) for f in features_db.values())
    }
    
    metadata_file = output_dir / "metadata.json"
    with open(metadata_file, 'w') as f:
        json.dump(metadata, f, indent=2)
    print(f"    Saved to: {metadata_file}")
    
    # Summary
    print("\n" + "=" * 60)
    print("Test satellite data created successfully!")
    print("=" * 60)
    print(f"\nOutput directory: {output_dir.absolute()}")
    print(f"Files created:")
    for f in output_dir.iterdir():
        size_kb = f.stat().st_size / 1024
        print(f"  - {f.name} ({size_kb:.1f} KB)")
    
    total_features = sum(len(f['keypoints']) for f in features_db.values())
    print(f"\nTotal features extracted: {total_features}")
    
    print("\n>>> Updating initial_pose.yaml with test coordinates...")
    
    # Update initial_pose.yaml
    initial_pose_path = Path("config/mission/initial_pose.yaml")
    if initial_pose_path.exists():
        with open(initial_pose_path, 'r') as f:
            content = f.read()
        
        # Update coordinates
        content = content.replace("latitude: 0.0", f"latitude: {center_lat}")
        content = content.replace("longitude: 0.0", f"longitude: {center_lon}")
        content = content.replace('region_id: ""', 'region_id: "test_region"')
        
        with open(initial_pose_path, 'w') as f:
            f.write(content)
        print(f"    Updated {initial_pose_path}")
    
    print("\nYou can now use this test data with the VIO satellite matching system.")
    print("The test region is centered at:")
    print(f"  Latitude:  {center_lat}°")
    print(f"  Longitude: {center_lon}°")


if __name__ == '__main__':
    main()
