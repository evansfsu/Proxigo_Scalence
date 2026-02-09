#!/usr/bin/env python3
"""
Upload a simple Death Valley mission with all waypoints close together.
This prevents glitching and ensures all points are within a small area.
"""

import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("Installing pymavlink...")
    import subprocess
    subprocess.run([sys.executable, "-m", "pip", "install", "pymavlink", "--quiet"], check=False)
    try:
        from pymavlink import mavutil
    except ImportError:
        print("ERROR: Could not install pymavlink.")
        print("Run from container: docker exec px4_sitl python3 /scripts/upload_simple_mission.py")
        sys.exit(1)

# Death Valley center coordinates
HOME_LAT = 36.2329
HOME_LON = -116.8276
HOME_ALT = 0

# Simple mission - all waypoints within ~200m of home
# Small square pattern around home position
WAYPOINTS = [
    # Home position (waypoint 0)
    (0, 16, 1, 1, 0, 0, 0, 0, HOME_LAT, HOME_LON, 0),
    # Takeoff to 50m
    (3, 22, 0, 1, 0, 0, 0, 0, HOME_LAT, HOME_LON, 50),
    # Small square pattern - all within 200m
    (3, 16, 0, 1, 0, 0, 0, 0, 36.2340, -116.8276, 100),  # North (~120m)
    (3, 16, 0, 1, 0, 0, 0, 0, 36.2340, -116.8260, 100),  # NE (~150m)
    (3, 16, 0, 1, 0, 0, 0, 0, 36.2318, -116.8260, 100),  # SE (~150m)
    (3, 16, 0, 1, 0, 0, 0, 0, 36.2318, -116.8292, 100),  # SW (~150m)
    (3, 16, 0, 1, 0, 0, 0, 0, 36.2340, -116.8292, 100),  # NW (~150m)
    (3, 16, 0, 1, 0, 0, 0, 0, 36.2340, -116.8276, 100),  # Back to North
    # Return to home and land
    (3, 16, 0, 1, 0, 0, 0, 0, HOME_LAT, HOME_LON, 50),   # Return to home at 50m
    (3, 21, 0, 1, 0, 0, 0, 0, HOME_LAT, HOME_LON, 0),    # Land
]


def connect_mavlink():
    """Connect to MAVLink via available ports."""
    connections = [
        "udp:127.0.0.1:14540",  # PX4 offboard (from container)
        "tcp:127.0.0.1:5760",   # TCP (from host)
        "udp:127.0.0.1:14550",  # UDP (from host)
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
    print("Make sure PX4 SITL is running: docker ps | grep px4_sitl")
    return None


def set_vehicle_gps_position(mav, lat, lon, alt=0):
    """Set vehicle's current GPS position using GPS_INPUT message."""
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    
    # GPS_INPUT_IGNORE_FLAGS constants
    IGNORE_VEL_HORIZ = 0x01
    IGNORE_VEL_VERT = 0x02
    IGNORE_SPEED = 0x04
    ignore_flags = IGNORE_VEL_HORIZ | IGNORE_VEL_VERT | IGNORE_SPEED
    
    # Send GPS_INPUT messages to tell PX4 where the vehicle currently is
    for i in range(10):
        try:
            mav.mav.gps_input_send(
                0, 0, ignore_flags, 0, 0, 3,  # time_usec, gps_id, flags, time_week_ms, time_week, fix_type
                lat_int, lon_int, float(alt),  # lat, lon, alt
                1.0, 1.0,  # hdop, vdop
                0.0, 0.0, 0.0,  # vn, ve, vd
                0.0, 0.0, 0.0,  # speed_accuracy, horiz_accuracy, vert_accuracy
                8, 0  # satellites_visible, yaw
            )
        except:
            pass
        time.sleep(0.1)
    time.sleep(0.5)


def set_home_and_ekf_origin(mav, lat, lon, alt=0):
    """Set both home position and EKF origin."""
    print(f"\nSetting home position and EKF origin to: {lat}, {lon}, {alt}m")
    
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    alt_mm = int(alt * 1000)
    
    # Set GPS global origin (EKF origin)
    mav.mav.set_gps_global_origin_send(
        mav.target_system,
        lat_int, lon_int, alt_mm
    )
    time.sleep(0.3)
    
    # Set home position (MAV_CMD_DO_SET_HOME = 179)
    # param1: 0 = use specified location, 1 = use current location
    # param5-7: lat, lon, alt
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,  # Confirmation
        0,  # Use specified location (not current)
        0, 0, 0,  # Unused
        lat, lon, alt
    )
    time.sleep(0.5)
    print("✓ Home position and EKF origin set!")


def upload_mission(mav, waypoints):
    """Upload mission waypoints to PX4."""
    print(f"\nUploading mission with {len(waypoints)} waypoints...")
    
    # Clear existing mission
    print("  Clearing existing mission...")
    mav.mav.mission_clear_all_send(mav.target_system, mav.target_component)
    time.sleep(1.0)  # Give PX4 time to clear
    
    # Send mission count
    print(f"  Sending mission count: {len(waypoints)} waypoints...")
    mav.mav.mission_count_send(
        mav.target_system,
        mav.target_component,
        len(waypoints)
    )
    time.sleep(0.5)
    
    # Send each waypoint when requested (using blocking approach like upload_death_valley_mission.py)
    print("  Waiting for waypoint requests from PX4...")
    for i, wp in enumerate(waypoints):
        # Wait for request (blocking with timeout)
        msg = mav.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], 
                            blocking=True, timeout=5)
        
        if msg is None:
            print(f"  ERROR: No request received for waypoint {i}")
            print("  PX4 may not be ready. Try waiting a few seconds and retry.")
            return False
        
        frame, cmd, cur, auto, p1, p2, p3, p4, lat, lon, alt = wp
        
        # Send waypoint using mission_item_int_send (more reliable)
        mav.mav.mission_item_int_send(
            mav.target_system,
            mav.target_component,
            i,  # seq
            frame,
            cmd,
            cur,
            auto,
            p1, p2, p3, p4,
            int(lat * 1e7),  # lat in 1e7 format
            int(lon * 1e7),  # lon in 1e7 format
            alt
        )
        print(f"  WP{i}: ({lat:.4f}, {lon:.4f}) alt={alt}m")
    
    # Wait for ACK
    print("  Waiting for mission ACK...")
    msg = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if msg and msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print(f"\n✓ Mission uploaded successfully! ({len(waypoints)} waypoints)")
        return True
    else:
        if msg:
            print(f"\n✗ Mission upload failed: ACK type {msg.type}")
        else:
            print("\n✗ Mission upload failed: No ACK received")
        return False


def main():
    print("=" * 60)
    print("SIMPLE DEATH VALLEY MISSION UPLOADER")
    print("=" * 60)
    print(f"\nHome: {HOME_LAT}, {HOME_LON}, {HOME_ALT}m")
    print("Mission: Small square pattern (~200m x 200m)")
    print("All waypoints within Death Valley area\n")
    
    # Connect
    mav = connect_mavlink()
    if mav is None:
        sys.exit(1)
    
    # Wait a moment for PX4 to be fully ready
    print("Waiting for PX4 to be ready...")
    time.sleep(2)
    
    # Set home position first
    set_home_and_ekf_origin(mav, HOME_LAT, HOME_LON, HOME_ALT)
    
    # Also set vehicle's current GPS position (prevents flying across America)
    print("\nSetting vehicle's current GPS position...")
    set_vehicle_gps_position(mav, HOME_LAT, HOME_LON, HOME_ALT)
    
    # Wait a bit more before uploading mission
    print("\nWaiting for EKF to settle...")
    time.sleep(3)
    
    # Upload mission
    success = upload_mission(mav, WAYPOINTS)
    
    if success:
        print("\n" + "=" * 60)
        print("READY TO FLY!")
        print("=" * 60)
        print("\nIn QGroundControl:")
        print("  1. Refresh Plan view - waypoints should appear")
        print("  2. Verify vehicle is at Death Valley location")
        print("  3. Arm the vehicle")
        print("  4. Start mission")
    else:
        print("\nMission upload failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()
