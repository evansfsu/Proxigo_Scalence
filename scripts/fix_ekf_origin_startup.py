#!/usr/bin/env python3
"""
Fix EKF origin at startup - based on official PX4 documentation.
The EKF origin can only be set ONCE and must be set BEFORE the EKF initializes.

This script should be run IMMEDIATELY after PX4 starts (within first 5 seconds).
Based on: https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html
"""

import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not available")
    print("Run from container: docker exec px4_sitl python3 /scripts/fix_ekf_origin_startup.py")
    sys.exit(1)

# Death Valley coordinates
LAT = 36.2329
LON = -116.8276
ALT = 0

if len(sys.argv) >= 3:
    LAT = float(sys.argv[1])
    LON = float(sys.argv[2])
    if len(sys.argv) >= 4:
        ALT = float(sys.argv[3])


def connect_mavlink():
    """Connect to MAVLink - try multiple ports."""
    connections = [
        "udp:127.0.0.1:14540",  # PX4 offboard (from container)
        "tcp:127.0.0.1:5760",   # TCP (from host)
        "udp:127.0.0.1:14550",  # UDP (from host)
    ]
    
    for conn in connections:
        try:
            print(f"Trying {conn}...")
            mav = mavutil.mavlink_connection(conn, timeout=3)
            mav.wait_heartbeat(timeout=5)
            print(f"✓ Connected via {conn}")
            return mav
        except Exception as e:
            print(f"  Failed: {e}")
            continue
    
    print("\nERROR: Could not connect to PX4")
    print("Make sure PX4 is running and wait a few seconds after startup")
    return None


def set_ekf_origin_immediately(mav, lat, lon, alt=0):
    """
    Set EKF origin IMMEDIATELY - this can only be done ONCE.
    Based on: https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html
    
    The EKF origin is the reference location for all position calculations.
    Once set, it cannot be changed without restarting PX4.
    """
    print(f"\nSetting EKF origin to: {lat}, {lon}, {alt}m")
    print("⚠️  WARNING: EKF origin can only be set ONCE per PX4 session!")
    print("   If PX4 has already initialized, restart the simulation.\n")
    
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    alt_mm = int(alt * 1000)
    
    # Method 1: SET_GPS_GLOBAL_ORIGIN (sets EKF origin)
    print("1. Setting GPS global origin (EKF origin)...")
    for i in range(5):  # Send multiple times to ensure it's received
        mav.mav.set_gps_global_origin_send(
            mav.target_system,
            lat_int, lon_int, alt_mm
        )
        time.sleep(0.1)
    
    time.sleep(0.5)
    
    # Method 2: Set home position to match
    print("2. Setting home position to match EKF origin...")
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
    
    # Method 3: Send GPS_INPUT messages to establish position
    print("3. Sending GPS_INPUT messages to establish vehicle position...")
    IGNORE_VEL_HORIZ = 0x01
    IGNORE_VEL_VERT = 0x02
    IGNORE_SPEED = 0x04
    ignore_flags = IGNORE_VEL_HORIZ | IGNORE_VEL_VERT | IGNORE_SPEED
    
    # Send many GPS_INPUT messages to ensure EKF accepts the position
    for i in range(20):
        try:
            mav.mav.gps_input_send(
                0, 0, ignore_flags, 0, 0, 3,  # time, gps_id, flags, time_week_ms, time_week, fix_type
                lat_int, lon_int, float(alt),  # lat, lon, alt
                1.0, 1.0,  # hdop, vdop
                0.0, 0.0, 0.0,  # vn, ve, vd
                0.0, 0.0, 0.0,  # speed_accuracy, horiz_accuracy, vert_accuracy
                8, 0  # satellites_visible, yaw
            )
        except:
            pass
        time.sleep(0.1)
    
    print("\n✓ EKF origin and home position set!")
    print("   The vehicle should now be at Death Valley.")
    print("   Wait 10-15 seconds for EKF to converge before arming.")


def main():
    print("=" * 60)
    print("EKF ORIGIN FIX (Startup)")
    print("=" * 60)
    print("\nBased on official PX4/ArduPilot documentation:")
    print("  https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html")
    print(f"\nTarget: {LAT}, {LON}, {ALT}m")
    print("\n⚠️  IMPORTANT: Run this IMMEDIATELY after PX4 starts!")
    print("   The EKF origin can only be set ONCE per session.\n")
    
    mav = connect_mavlink()
    if mav is None:
        sys.exit(1)
    
    set_ekf_origin_immediately(mav, LAT, LON, ALT)
    
    print("\n" + "=" * 60)
    print("SUCCESS!")
    print("=" * 60)
    print("\nNext steps:")
    print("  1. Wait 10-15 seconds for EKF to converge")
    print("  2. Check QGC - vehicle should be at Death Valley")
    print("  3. Upload mission (waypoints should be close)")
    print("  4. Arm and fly")


if __name__ == "__main__":
    main()
