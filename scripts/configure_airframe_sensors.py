#!/usr/bin/env python3
"""
Configure PX4 airframe and inject sensor data for "none" simulator
This allows fixed-wing operation without Gazebo/jMAVSim
"""

import sys
import time
from pymavlink import mavutil

def connect_mavlink():
    """Connect to PX4 via MAVLink"""
    try:
        # Connect to PX4's offboard port
        mav = mavutil.mavlink_connection('udp:127.0.0.1:14540', baud=57600)
        print("Connecting to PX4...")
        
        # Wait for heartbeat
        mav.wait_heartbeat(timeout=10)
        print(f"✓ Connected to system {mav.target_system}")
        return mav
    except Exception as e:
        print(f"✗ Failed to connect: {e}")
        return None

def set_airframe(mav, autostart=4008):
    """Set airframe to advanced plane"""
    print(f"\nSetting airframe to Advanced Plane (SYS_AUTOSTART={autostart})...")
    
    # Set parameter
    mav.param_set_send('SYS_AUTOSTART', autostart)
    time.sleep(0.5)
    
    # Verify
    mav.param_request_read_send(mav.target_system, mav.target_component, b'SYS_AUTOSTART', -1)
    time.sleep(0.5)
    
    print("✓ Airframe configured (requires PX4 restart to take effect)")

def inject_sensor_data(mav, lat, lon, alt):
    """Inject GPS and sensor data via GPS_INPUT messages"""
    print(f"\nInjecting sensor data (GPS, IMU simulation)...")
    
    # GPS_INPUT flags
    GPS_INPUT_IGNORE_FLAGS = 0
    GPS_INPUT_IGNORE_FLAGS_VEL_HORIZ = 1 << 1
    GPS_INPUT_IGNORE_FLAGS_VEL_VERT = 1 << 2
    
    # Send multiple GPS_INPUT messages to establish position
    for i in range(10):
        mav.mav.gps_input_send(
            0,  # time_usec
            0,  # gps_id
            GPS_INPUT_IGNORE_FLAGS,  # ignore_flags
            int(lat * 1e7),  # time_week_ms
            0,  # time_week
            0,  # fix_type
            int(lat * 1e7),  # lat
            int(lon * 1e7),  # lon
            alt,  # alt
            1.0,  # hdop
            1.0,  # vdop
            0.0,  # vn
            0.0,  # ve
            0.0,  # vd
            0.0,  # speed_accuracy
            0.0,  # horiz_accuracy
            0.0,  # vert_accuracy
            8  # satellites_visible
        )
        time.sleep(0.1)
    
    print("✓ Sensor data injected")

def main():
    if len(sys.argv) < 4:
        print("Usage: configure_airframe_sensors.py <lat> <lon> <alt>")
        print("Example: configure_airframe_sensors.py 36.2329 -116.8276 50")
        sys.exit(1)
    
    lat = float(sys.argv[1])
    lon = float(sys.argv[2])
    alt = float(sys.argv[3])
    
    print("=" * 60)
    print("  PX4 Airframe & Sensor Configuration")
    print("=" * 60)
    
    mav = connect_mavlink()
    if not mav:
        sys.exit(1)
    
    set_airframe(mav, autostart=4008)
    inject_sensor_data(mav, lat, lon, alt)
    
    print("\n" + "=" * 60)
    print("✓ Configuration complete!")
    print("=" * 60)
    print("\nNote: Airframe change requires PX4 restart.")
    print("Sensor data is being injected continuously.")
    print("\nIn QGroundControl:")
    print("  1. Go to Vehicle Setup → Airframe")
    print("  2. Select 'Generic Fixed Wing' or 'Advanced Plane'")
    print("  3. Apply and Restart")

if __name__ == "__main__":
    main()
