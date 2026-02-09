#!/usr/bin/env python3
"""
Set PX4 vehicle's CURRENT GPS position (not just home position).
This fixes the issue where the vehicle thinks it's in Zurich/Africa
and flies at light speed to reach Death Valley waypoints.

Usage:
  docker exec px4_sitl python3 /scripts/set_vehicle_position.py [lat] [lon] [alt]
"""

import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not available")
    print("Run from container: docker exec px4_sitl python3 /scripts/set_vehicle_position.py")
    sys.exit(1)

# Default: Death Valley coordinates
LAT = 36.2329
LON = -116.8276
ALT = 0

# Allow override via command line
if len(sys.argv) >= 3:
    LAT = float(sys.argv[1])
    LON = float(sys.argv[2])
    if len(sys.argv) >= 4:
        ALT = float(sys.argv[3])


def connect_mavlink():
    """Connect to MAVLink."""
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
    return None


def set_vehicle_gps_position(mav, lat, lon, alt=0):
    """
    Set vehicle's current GPS position using GPS_INPUT message.
    This tells PX4 where the vehicle currently is (not just where home is).
    """
    print(f"\nSetting vehicle GPS position to: {lat}, {lon}, {alt}m")
    
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    alt_mm = int(alt * 1000)
    
    # Method 1: GPS_INPUT message (simulates GPS receiver)
    # This is the most direct way to set vehicle position
    print("\n1. Sending GPS_INPUT message...")
    # GPS_INPUT_IGNORE_FLAGS constants (from MAVLink spec)
    IGNORE_VEL_HORIZ = 0x01
    IGNORE_VEL_VERT = 0x02
    IGNORE_SPEED = 0x04
    ignore_flags = IGNORE_VEL_HORIZ | IGNORE_VEL_VERT | IGNORE_SPEED
    
    for i in range(15):  # Send multiple times to ensure it's received
        try:
            mav.mav.gps_input_send(
                0,  # time_usec (0 = use system time)
                0,  # gps_id
                ignore_flags,  # ignore_flags
                0,  # time_week_ms
                0,  # time_week
                3,  # fix_type (3 = 3D fix)
                lat_int,  # lat (1e7)
                lon_int,  # lon (1e7)
                float(alt),  # alt (meters)
                1.0,  # hdop (horizontal dilution of precision)
                1.0,  # vdop (vertical dilution of precision)
                0.0,  # vn (velocity north, m/s)
                0.0,  # ve (velocity east, m/s)
                0.0,  # vd (velocity down, m/s)
                0.0,  # speed_accuracy
                0.0,  # horiz_accuracy
                0.0,  # vert_accuracy
                8,  # satellites_visible
                0  # yaw (degrees)
            )
        except Exception as e:
            print(f"  Warning: GPS_INPUT send failed: {e}")
        time.sleep(0.1)
    
    time.sleep(1)
    
    # Method 2: Set GPS global origin (EKF origin)
    print("2. Setting GPS global origin (EKF origin)...")
    mav.mav.set_gps_global_origin_send(
        mav.target_system,
        lat_int, lon_int, alt_mm
    )
    time.sleep(0.5)
    
    # Method 3: Set home position to same location
    print("3. Setting home position...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,  # Confirmation
        0,  # param1: Use specified location (not current)
        0, 0, 0,  # param2-4: unused
        lat, lon, alt  # param5-7: lat, lon, alt
    )
    time.sleep(0.5)
    
    # Method 4: Send a few more GPS_INPUT messages to reinforce
    print("4. Reinforcing GPS position...")
    for i in range(5):
        try:
            mav.mav.gps_input_send(
                0, 0, ignore_flags, 0, 0, 3,
                lat_int, lon_int, float(alt),
                1.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 8, 0
            )
        except:
            pass
        time.sleep(0.2)
    
    print("\n✓ Vehicle GPS position set!")
    print("The vehicle should now think it's at Death Valley.")
    print("Check QGC - the vehicle icon should be at the correct location.")


def main():
    print("=" * 60)
    print("PX4 VEHICLE POSITION SETTER")
    print("=" * 60)
    print(f"\nTarget position: {LAT}, {LON}, {ALT}m")
    print("This sets where the vehicle CURRENTLY is (not just home).\n")
    
    mav = connect_mavlink()
    if mav is None:
        sys.exit(1)
    
    set_vehicle_gps_position(mav, LAT, LON, ALT)
    
    print("\n" + "=" * 60)
    print("SUCCESS!")
    print("=" * 60)
    print("\nThe vehicle should now be positioned at Death Valley.")
    print("If it was flying erratically, it should stop and be at the correct location.")


if __name__ == "__main__":
    main()
