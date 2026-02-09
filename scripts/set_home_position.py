#!/usr/bin/env python3
"""
Set PX4 home position and EKF origin to Death Valley (or specified coordinates).
Run this after PX4 SITL starts to fix the "waypoint too far" error.

Usage:
  python3 scripts/set_home_position.py [lat] [lon] [alt]
  
  Or from inside Docker container:
  docker exec px4_sitl python3 /scripts/set_home_position.py
"""

import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("Installing pymavlink...")
    import subprocess
    try:
        subprocess.run([sys.executable, "-m", "pip", "install", "pymavlink", "--quiet"], check=True)
        from pymavlink import mavutil
    except:
        print("ERROR: Could not install pymavlink.")
        print("If running from host, install: pip3 install pymavlink")
        print("Or run from container: docker exec px4_sitl python3 /scripts/set_home_position.py")
        sys.exit(1)

# Default: Death Valley coordinates
HOME_LAT = 36.2329
HOME_LON = -116.8276
HOME_ALT = 0

# Allow override via command line
if len(sys.argv) >= 3:
    HOME_LAT = float(sys.argv[1])
    HOME_LON = float(sys.argv[2])
    if len(sys.argv) >= 4:
        HOME_ALT = float(sys.argv[3])


def connect_mavlink():
    """Connect to MAVLink via available ports."""
    # If running inside container, use localhost
    # If running from host, use forwarded ports
    connections = [
        "udp:127.0.0.1:14540",  # PX4 offboard (works from container)
        "tcp:127.0.0.1:5760",   # TCP (QGC manual, works from host)
        "udp:127.0.0.1:14550",  # UDP (QGC default, works from host)
    ]
    
    for conn in connections:
        try:
            print(f"Trying {conn}...")
            mav = mavutil.mavlink_connection(conn, timeout=5)
            mav.wait_heartbeat(timeout=10)
            print(f"✓ Connected via {conn}")
            return mav
        except Exception as e:
            print(f"  Failed: {e}")
            continue
    
    print("\nERROR: Could not connect to PX4")
    print("Make sure:")
    print("  1. PX4 SITL is running: docker ps | grep px4_sitl")
    print("  2. MAVProxy bridge is running")
    print("  3. Wait a few seconds after starting simulation")
    return None


def set_home_and_ekf_origin(mav, lat, lon, alt=0):
    """Set both home position and EKF origin."""
    print(f"\nSetting home position and EKF origin to:")
    print(f"  Latitude:  {lat}°")
    print(f"  Longitude: {lon}°")
    print(f"  Altitude:  {alt}m")
    
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    alt_mm = int(alt * 1000)
    
    # Set GPS global origin (EKF origin)
    print("\n1. Setting GPS global origin (EKF origin)...")
    mav.mav.set_gps_global_origin_send(
        mav.target_system,
        lat_int, lon_int, alt_mm
    )
    time.sleep(0.5)
    
    # Set home position
    print("2. Setting home position...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # 179
        0,  # Confirmation
        0,  # Use specified location (not current)
        0, 0, 0, 0,  # Unused params
        lat, lon, alt
    )
    time.sleep(0.5)
    
    print("✓ Home position and EKF origin set!")
    print("\nYou can now upload missions in QGroundControl without 'waypoint too far' errors.")


def main():
    print("=" * 60)
    print("PX4 HOME POSITION SETTER")
    print("=" * 60)
    
    mav = connect_mavlink()
    if mav is None:
        sys.exit(1)
    
    set_home_and_ekf_origin(mav, HOME_LAT, HOME_LON, HOME_ALT)
    
    print("\n" + "=" * 60)
    print("SUCCESS!")
    print("=" * 60)
    print(f"\nHome: {HOME_LAT}, {HOME_LON}, {HOME_ALT}m")
    print("\nNext steps:")
    print("  1. In QGroundControl, refresh the map view")
    print("  2. Upload your Death Valley mission")
    print("  3. The waypoints should now be within range")


if __name__ == "__main__":
    main()
