#!/usr/bin/env python3
"""
Force vehicle position by sending continuous GPS_INPUT messages.
This helps when QGC shows the wrong location (e.g., San Francisco).
Run this and keep it running while QGC is connected.
"""

import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not available")
    print("Run from container: docker exec px4_sitl python3 /scripts/force_vehicle_position.py")
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
    """Connect to MAVLink."""
    connections = [
        "udp:127.0.0.1:14540",
        "tcp:127.0.0.1:5760",
        "udp:127.0.0.1:14550",
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


def send_continuous_gps(mav, lat, lon, alt=0):
    """Send continuous GPS_INPUT messages to keep vehicle position updated."""
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    
    IGNORE_VEL_HORIZ = 0x01
    IGNORE_VEL_VERT = 0x02
    IGNORE_SPEED = 0x04
    ignore_flags = IGNORE_VEL_HORIZ | IGNORE_VEL_VERT | IGNORE_SPEED
    
    print(f"\nSending continuous GPS_INPUT messages...")
    print(f"Position: {lat}, {lon}, {alt}m")
    print(f"Press Ctrl+C to stop\n")
    
    count = 0
    try:
        while True:
            try:
                mav.mav.gps_input_send(
                    0, 0, ignore_flags, 0, 0, 3,
                    lat_int, lon_int, float(alt),
                    1.0, 1.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    8, 0
                )
                count += 1
                if count % 10 == 0:
                    print(f"  Sent {count} GPS messages...", end='\r')
            except Exception as e:
                print(f"\nError sending GPS: {e}")
                break
            time.sleep(0.2)  # Send every 200ms
    except KeyboardInterrupt:
        print(f"\n\nStopped after sending {count} GPS messages")


def main():
    print("=" * 60)
    print("FORCE VEHICLE POSITION (Continuous GPS)")
    print("=" * 60)
    print(f"\nTarget: {LAT}, {LON}, {ALT}m")
    print("This will send GPS messages continuously.")
    print("Keep this running while QGC is connected.\n")
    
    mav = connect_mavlink()
    if mav is None:
        sys.exit(1)
    
    # Initial setup
    lat_int = int(LAT * 1e7)
    lon_int = int(LON * 1e7)
    alt_mm = int(ALT * 1000)
    
    print("\nSetting up initial position...")
    mav.mav.set_gps_global_origin_send(mav.target_system, lat_int, lon_int, alt_mm)
    time.sleep(0.5)
    
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0, 0, 0, 0, 0, 0, 0,
        LAT, LON, ALT
    )
    time.sleep(0.5)
    
    # Start continuous GPS
    send_continuous_gps(mav, LAT, LON, ALT)


if __name__ == "__main__":
    main()
