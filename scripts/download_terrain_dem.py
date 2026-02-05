#!/usr/bin/env python3
"""
Download DEM (Digital Elevation Model) terrain data and convert to Gazebo heightmap.

Uses USGS National Elevation Dataset (NED) or SRTM data to create terrain.
Output: PNG heightmap compatible with Gazebo Harmonic.
"""

import os
import sys
import math
import struct
import argparse
from pathlib import Path

try:
    import numpy as np
    from PIL import Image
    import requests
except ImportError:
    print("Installing required packages...")
    os.system(f"{sys.executable} -m pip install numpy pillow requests")
    import numpy as np
    from PIL import Image
    import requests


def get_srtm_tile_name(lat: float, lon: float) -> str:
    """Get SRTM tile name for a given coordinate."""
    lat_prefix = 'N' if lat >= 0 else 'S'
    lon_prefix = 'E' if lon >= 0 else 'W'
    lat_val = int(abs(math.floor(lat)))
    lon_val = int(abs(math.floor(lon)))
    return f"{lat_prefix}{lat_val:02d}{lon_prefix}{lon_val:03d}"


def download_opentopography_dem(lat_min: float, lon_min: float, lat_max: float, lon_max: float, 
                                 output_dir: Path, api_key: str = None) -> Path:
    """Download DEM from OpenTopography API (free tier available)."""
    
    print(f"Downloading DEM for region: ({lat_min}, {lon_min}) to ({lat_max}, {lon_max})")
    
    # OpenTopography SRTM GL3 (90m) endpoint - free without API key
    url = "https://portal.opentopography.org/API/globaldem"
    params = {
        "demtype": "SRTMGL3",  # 90m SRTM
        "south": lat_min,
        "north": lat_max,
        "west": lon_min,
        "east": lon_max,
        "outputFormat": "GTiff"
    }
    
    if api_key:
        params["API_Key"] = api_key
    
    output_file = output_dir / "terrain_dem.tif"
    
    try:
        print("Requesting DEM from OpenTopography...")
        response = requests.get(url, params=params, timeout=120)
        
        if response.status_code == 200:
            output_file.write_bytes(response.content)
            print(f"Downloaded DEM to {output_file}")
            return output_file
        else:
            print(f"OpenTopography API error: {response.status_code}")
            print(response.text)
            return None
    except Exception as e:
        print(f"Download failed: {e}")
        return None


def generate_synthetic_terrain(lat_center: float, lon_center: float, 
                                size_km: float, output_dir: Path,
                                resolution: int = 513) -> Path:
    """
    Generate synthetic terrain based on real-world characteristics.
    
    Death Valley characteristics:
    - Low elevation (-86m at Badwater Basin)
    - Mountains on sides (up to 3000m)
    - Salt flats in center
    - Alluvial fans
    """
    
    print(f"Generating synthetic Death Valley terrain...")
    print(f"Center: ({lat_center}, {lon_center})")
    print(f"Size: {size_km} km x {size_km} km")
    print(f"Resolution: {resolution} x {resolution}")
    
    # Create coordinate grids
    x = np.linspace(-1, 1, resolution)
    y = np.linspace(-1, 1, resolution)
    X, Y = np.meshgrid(x, y)
    
    # Distance from center
    R = np.sqrt(X**2 + Y**2)
    
    # Base elevation: valley floor (death valley is below sea level)
    terrain = np.zeros((resolution, resolution), dtype=np.float32)
    
    # Valley floor - mostly flat salt flats
    valley_floor = -50  # meters (simplified, actual is -86m)
    terrain += valley_floor
    
    # Add gentle rolling terrain in the center
    terrain += 10 * np.sin(X * 5) * np.sin(Y * 5)
    
    # Mountain ridges on the sides (Panamint Range to west, Amargosa to east)
    # Western mountains
    west_mountain = 1500 * np.exp(-((X + 0.8)**2) / 0.1) * (1 + 0.3 * np.sin(Y * 10))
    terrain += west_mountain
    
    # Eastern mountains  
    east_mountain = 1200 * np.exp(-((X - 0.8)**2) / 0.15) * (1 + 0.2 * np.sin(Y * 8))
    terrain += east_mountain
    
    # Alluvial fans coming from mountains
    for i in range(5):
        fan_y = -0.8 + i * 0.4
        fan_west = 200 * np.exp(-(((X + 0.5)**2)/0.3 + ((Y - fan_y)**2)/0.1))
        terrain += fan_west
    
    # Add some random noise for texture
    noise = 5 * np.random.randn(resolution, resolution)
    noise = np.clip(noise, -20, 20)
    terrain += noise
    
    # Normalize to 0-65535 for 16-bit PNG (Gazebo heightmap format)
    terrain_min = terrain.min()
    terrain_max = terrain.max()
    
    print(f"Elevation range: {terrain_min:.1f}m to {terrain_max:.1f}m")
    
    # Map to 16-bit range
    terrain_normalized = (terrain - terrain_min) / (terrain_max - terrain_min)
    terrain_16bit = (terrain_normalized * 65535).astype(np.uint16)
    
    # Save as PNG
    output_file = output_dir / "death_valley_heightmap.png"
    img = Image.fromarray(terrain_16bit, mode='I;16')
    img.save(str(output_file))
    
    print(f"Saved heightmap to {output_file}")
    
    # Save metadata for Gazebo
    metadata = {
        "min_elevation": float(terrain_min),
        "max_elevation": float(terrain_max),
        "size_meters": size_km * 1000,
        "resolution": resolution,
        "center_lat": lat_center,
        "center_lon": lon_center
    }
    
    metadata_file = output_dir / "terrain_metadata.txt"
    with open(metadata_file, 'w') as f:
        for key, value in metadata.items():
            f.write(f"{key}: {value}\n")
    
    print(f"Saved metadata to {metadata_file}")
    
    return output_file


def create_gazebo_world(heightmap_path: Path, metadata: dict, output_dir: Path) -> Path:
    """Create a Gazebo world file with the heightmap terrain."""
    
    size = metadata.get("size_meters", 2000)
    min_elev = metadata.get("min_elevation", -100)
    max_elev = metadata.get("max_elevation", 1500)
    height_scale = max_elev - min_elev
    
    world_sdf = f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="death_valley">
    
    <!-- Physics -->
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    
    <!-- Spherical coordinates (Death Valley) -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>36.2329</latitude_deg>
      <longitude_deg>-116.8276</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.8 1</diffuse>
      <specular>0.3 0.3 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <light type="directional" name="sun2">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.3 0.3 0.25 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.5 0.5 -0.9</direction>
    </light>
    
    <!-- Sky -->
    <scene>
      <ambient>0.6 0.6 0.5 1</ambient>
      <background>0.3 0.6 0.9 1</background>
      <shadows>true</shadows>
    </scene>
    
    <!-- Terrain with heightmap -->
    <model name="death_valley_terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="terrain_collision">
          <geometry>
            <heightmap>
              <uri>file:///gazebo_worlds/terrain/death_valley_heightmap.png</uri>
              <size>{size} {size} {height_scale}</size>
              <pos>0 0 {min_elev}</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="terrain_visual">
          <geometry>
            <heightmap>
              <uri>file:///gazebo_worlds/terrain/death_valley_heightmap.png</uri>
              <size>{size} {size} {height_scale}</size>
              <pos>0 0 {min_elev}</pos>
              <texture>
                <diffuse>file:///gazebo_worlds/terrain/death_valley_satellite.jpg</diffuse>
                <normal>file:///gazebo_worlds/terrain/flat_normal.png</normal>
                <size>10</size>
              </texture>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
    
    <!-- Flat ground plane as backup -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>5000 5000</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>5000 5000</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.75 0.6 1</ambient>
            <diffuse>0.8 0.75 0.6 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
'''
    
    output_file = output_dir / "death_valley_terrain.sdf"
    output_file.write_text(world_sdf)
    print(f"Created Gazebo world: {output_file}")
    
    return output_file


def main():
    parser = argparse.ArgumentParser(description="Download/generate terrain for Gazebo simulation")
    parser.add_argument("--lat", type=float, default=36.2329, help="Center latitude")
    parser.add_argument("--lon", type=float, default=-116.8276, help="Center longitude")
    parser.add_argument("--size", type=float, default=2.0, help="Size in km")
    parser.add_argument("--resolution", type=int, default=513, help="Heightmap resolution")
    parser.add_argument("--output", type=str, default="gazebo_worlds/terrain", help="Output directory")
    parser.add_argument("--synthetic", action="store_true", help="Generate synthetic terrain")
    
    args = parser.parse_args()
    
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("TERRAIN DATA GENERATOR FOR GAZEBO")
    print("=" * 60)
    
    # For now, generate synthetic terrain (real DEM download requires API key)
    heightmap = generate_synthetic_terrain(
        args.lat, args.lon, args.size, output_dir, args.resolution
    )
    
    # Read metadata
    metadata = {
        "min_elevation": -100,
        "max_elevation": 1500,
        "size_meters": args.size * 1000,
        "resolution": args.resolution,
        "center_lat": args.lat,
        "center_lon": args.lon
    }
    
    # Create Gazebo world
    create_gazebo_world(heightmap, metadata, output_dir.parent)
    
    print("")
    print("=" * 60)
    print("TERRAIN GENERATION COMPLETE")
    print("=" * 60)
    print(f"Heightmap: {heightmap}")
    print(f"Use with: PX4_GZ_WORLD=death_valley_terrain")


if __name__ == "__main__":
    main()
