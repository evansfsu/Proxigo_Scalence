#!/usr/bin/env python3
"""
Upload Death Valley survey mission to PX4 via MAVLink.
Also sets the EKF origin to Death Valley coordinates.
"""

import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("Installing pymavlink...")
    import subprocess
    subprocess.run([sys.executable, "-m", "pip", "install", "pymavlink", "--quiet"])
    from pymavlink import mavutil


# Death Valley coordinates
HOME_LAT = 36.2329
HOME_LON = -116.8276
HOME_ALT = 0

# Mission waypoints for Death Valley survey
# Format: (frame, command, current, autocontinue, p1, p2, p3, p4, lat, lon, alt)
WAYPOINTS = [
    # Home position
    (0, 16, 1, 1, 0, 0, 0, 0, HOME_LAT, HOME_LON, 0),
    # Takeoff to 50m
    (3, 22, 0, 1, 0, 0, 0, 0, HOME_LAT, HOME_LON, 50),
    # Survey pattern
    (3, 16, 0, 1, 0, 0, 0, 0, 36.2380, -116.8276, 100),  # North
    (3, 16, 0, 1, 0, 0, 0, 0, 36.2380, -116.8200, 100),  # NE
    (3, 16, 0, 1, 0, 0, 0, 0, 36.2280, -116.8200, 100),  # SE
    (3, 16, 0, 1, 0, 0, 0, 0, 36.2280, -116.8350, 100),  # SW
    (3, 16, 0, 1, 0, 0, 0, 0, 36.2380, -116.8350, 100),  # NW
    # Land
    (3, 21, 0, 1, 0, 0, 0, 0, HOME_LAT, HOME_LON, 0),
]


def connect_mavlink(timeout=30):
    """Connect to MAVLink and wait for heartbeat. Tries multiple ports."""
    
    # Try different connection methods
    connections = [
        "tcp:127.0.0.1:5760",   # TCP forwarded port
        "udp:127.0.0.1:14550",  # UDP standard
        "udp:127.0.0.1:14540",  # UDP offboard
    ]
    
    for conn_string in connections:
        print(f"Trying {conn_string}...")
        try:
            mav = mavutil.mavlink_connection(conn_string, timeout=5)
            msg = mav.wait_heartbeat(timeout=10)
            if msg:
                print(f"Connected! System: {mav.target_system}, Component: {mav.target_component}")
                return mav
        except Exception as e:
            print(f"  Failed: {e}")
            continue
    
    print("ERROR: Could not connect to PX4!")
    print("Make sure the simulation is running: docker ps")
    return None


def set_ekf_origin(mav, lat, lon, alt=0):
    """Set the EKF origin to specified coordinates."""
    print(f"\nSetting EKF origin to: {lat}, {lon}")
    
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    alt_mm = int(alt * 1000)
    
    mav.mav.set_gps_global_origin_send(
        mav.target_system,
        lat_int, lon_int, alt_mm
    )
    
    # Also set home position
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        0,  # Use specified location (not current)
        0, 0, 0,
        lat, lon, alt
    )
    
    time.sleep(0.5)
    print("EKF origin set!")


def upload_mission(mav, waypoints):
    """Upload mission waypoints to PX4."""
    print(f"\nUploading {len(waypoints)} waypoints...")
    
    # Clear existing mission
    mav.mav.mission_clear_all_send(mav.target_system, mav.target_component)
    time.sleep(0.5)
    
    # Send mission count
    mav.mav.mission_count_send(mav.target_system, mav.target_component, len(waypoints))
    
    # Send each waypoint when requested
    for i, wp in enumerate(waypoints):
        # Wait for request
        msg = mav.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], 
                            blocking=True, timeout=5)
        
        if msg is None:
            print(f"ERROR: No request received for waypoint {i}")
            return False
        
        frame, cmd, cur, auto, p1, p2, p3, p4, lat, lon, alt = wp
        
        # Send waypoint
        mav.mav.mission_item_int_send(
            mav.target_system,
            mav.target_component,
            i,  # seq
            frame,
            cmd,
            cur,
            auto,
            p1, p2, p3, p4,
            int(lat * 1e7),
            int(lon * 1e7),
            alt
        )
        print(f"  WP{i}: ({lat:.4f}, {lon:.4f}) alt={alt}m")
    
    # Wait for ACK
    msg = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if msg and msg.type == 0:
        print("\nMission uploaded successfully!")
        return True
    else:
        print(f"\nMission upload failed: {msg}")
        return False


def main():
    print("=" * 50)
    print("DEATH VALLEY MISSION UPLOADER")
    print("=" * 50)
    
    # Connect
    mav = connect_mavlink()
    if mav is None:
        sys.exit(1)
    
    # Set EKF origin
    set_ekf_origin(mav, HOME_LAT, HOME_LON, HOME_ALT)
    
    # Upload mission
    success = upload_mission(mav, WAYPOINTS)
    
    if success:
        print("\n" + "=" * 50)
        print("READY TO FLY!")
        print("=" * 50)
        print(f"\nHome: {HOME_LAT}, {HOME_LON}")
        print("Mission: Death Valley survey pattern")
        print("\nIn QGroundControl:")
        print("  1. Check Plan view - waypoints should appear")
        print("  2. Arm the vehicle")
        print("  3. Start mission")
    else:
        print("\nMission upload failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()
