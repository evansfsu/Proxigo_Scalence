#!/usr/bin/env python3
"""
Inject sensor data (IMU, Magnetometer, GPS) into PX4 via MAVLink
This allows calibration in QGC even with "none" simulator
"""

import sys
import time
import math
from pymavlink import mavutil

# GPS_INPUT flags
GPS_INPUT_IGNORE_FLAGS = 0
GPS_INPUT_IGNORE_FLAGS_VEL_HORIZ = 1 << 1
GPS_INPUT_IGNORE_FLAGS_VEL_VERT = 1 << 2

def connect_mavlink():
    """Connect to PX4 via MAVLink"""
    try:
        mav = mavutil.mavlink_connection('udp:127.0.0.1:14540', baud=57600)
        print("Connecting to PX4...")
        
        mav.wait_heartbeat(timeout=10)
        print(f"✓ Connected to system {mav.target_system}")
        return mav
    except Exception as e:
        print(f"✗ Failed to connect: {e}")
        return None

def send_gps_data(mav, lat, lon, alt, time_usec=0):
    """Send GPS data via GPS_INPUT message"""
    mav.mav.gps_input_send(
        time_usec,
        0,  # gps_id
        GPS_INPUT_IGNORE_FLAGS,
        int((time.time() * 1000) % (7 * 24 * 3600 * 1000)),  # time_week_ms
        0,  # time_week
        3,  # fix_type (3 = 3D fix)
        int(lat * 1e7),  # lat
        int(lon * 1e7),  # lon
        alt,  # alt
        1.0,  # hdop
        1.0,  # vdop
        0.0,  # vn
        0.0,  # ve
        0.0,  # vd
        0.5,  # speed_accuracy
        0.5,  # horiz_accuracy
        0.5,  # vert_accuracy
        12  # satellites_visible
    )

def send_imu_data(mav, ax=0, ay=0, az=-9.81, gx=0, gy=0, gz=0):
    """Send IMU data via HIGHRES_IMU message"""
    # Get current time in microseconds
    time_usec = int(time.time() * 1e6)
    
    # Send HIGHRES_IMU message
    mav.mav.highres_imu_send(
        time_usec,  # time_usec
        ax, ay, az,  # xacc, yacc, zacc (m/s^2)
        gx, gy, gz,  # xgyro, ygyro, zgyro (rad/s)
        0.0,  # xmag
        0.0,  # ymag
        0.0,  # zmag
        0.0,  # abs_pressure
        0.0,  # diff_pressure
        0.0,  # pressure_alt
        20.0,  # temperature
        0  # fields_updated
    )

def send_magnetometer_data(mav, mx=0.2, my=0.0, mz=0.4):
    """Send magnetometer data via RAW_IMU message"""
    time_usec = int(time.time() * 1e6)
    
    # Send RAW_IMU with magnetometer data
    # Note: RAW_IMU uses integer values, scale appropriately
    # Typical magnetometer range: ±4 Gauss = ±4000 microTesla
    mav.mav.raw_imu_send(
        time_usec,  # time_usec
        int(0),  # xacc (not used for mag)
        int(0),  # yacc
        int(0),  # zacc
        int(0),  # xgyro
        int(0),  # ygyro
        int(0),  # zgyro
        int(mx * 1000),  # xmag (microTesla)
        int(my * 1000),  # ymag
        int(mz * 1000),  # zmag
        0  # id
    )
    
    # Also send via HIGHRES_IMU for better compatibility
    mav.mav.highres_imu_send(
        time_usec,
        0.0, 0.0, -9.81,  # accel (gravity down)
        0.0, 0.0, 0.0,  # gyro
        mx, my, mz,  # mag (Tesla, so 0.2 = 0.2 Tesla = 2000 microTesla)
        0.0,  # abs_pressure
        0.0,  # diff_pressure
        0.0,  # pressure_alt
        20.0,  # temperature
        0x07  # fields_updated (accel + gyro + mag)
    )

def send_barometer_data(mav, pressure=101325.0, temperature=20.0):
    """Send barometer data via SCALED_PRESSURE message"""
    # Use current time in milliseconds, modulo to fit in uint32
    time_boot_ms = int((time.time() * 1000) % 4294967295)
    
    # Temperature in centidegrees (int16: -32768 to 32767)
    # 20°C = 2000 centidegrees
    temp_centidegrees = max(-32768, min(32767, int(temperature * 100)))
    
    try:
        mav.mav.scaled_pressure_send(
            time_boot_ms,  # time_boot_ms (uint32)
            pressure / 100.0,  # press_abs (hPa, float)
            pressure / 100.0,  # press_diff (hPa, float)
            temp_centidegrees,  # temperature (int16, centidegrees)
            0  # temperature_press_diff (int16, centidegrees)
        )
    except Exception as e:
        # Skip barometer if there's an error (not critical)
        pass

def main():
    if len(sys.argv) < 4:
        print("Usage: inject_sensor_data.py <lat> <lon> <alt>")
        print("Example: inject_sensor_data.py 36.2329 -116.8276 50")
        sys.exit(1)
    
    lat = float(sys.argv[1])
    lon = float(sys.argv[2])
    alt = float(sys.argv[3])
    
    print("=" * 60)
    print("  PX4 Sensor Data Injection")
    print("=" * 60)
    print(f"\nInjecting sensor data at: {lat}, {lon}, {alt}m")
    print("This allows QGC to calibrate sensors even with 'none' simulator")
    print("\nPress Ctrl+C to stop")
    print("=" * 60)
    
    mav = connect_mavlink()
    if not mav:
        sys.exit(1)
    
    # Initial GPS position
    print("\nSetting initial GPS position...")
    for i in range(10):
        send_gps_data(mav, lat, lon, alt)
        time.sleep(0.1)
    
    print("✓ GPS position set")
    print("\nInjecting continuous sensor data...")
    print("  - IMU (Accelerometer + Gyroscope)")
    print("  - Magnetometer")
    print("  - Barometer")
    print("  - GPS")
    
    try:
        counter = 0
        while True:
            # Send GPS every 200ms
            send_gps_data(mav, lat, lon, alt)
            
            # Send IMU data (accelerometer + gyroscope) at ~100Hz
            # Simulate level flight: gravity down, no rotation
            send_imu_data(mav, 
                ax=0.0, ay=0.0, az=-9.81,  # Gravity pointing down
                gx=0.0, gy=0.0, gz=0.0     # No rotation
            )
            
            # Send magnetometer data
            # Simulate Earth's magnetic field (varies by location)
            # Death Valley: declination ~12°, inclination ~60°
            # Rough approximation for calibration
            mag_x = 0.2 * math.cos(math.radians(12))  # North component
            mag_y = 0.2 * math.sin(math.radians(12))  # East component  
            mag_z = 0.4  # Down component (inclination)
            send_magnetometer_data(mav, mag_x, mag_y, mag_z)
            
            # Skip barometer for now (not critical for accelerometer/compass calibration)
            # send_barometer_data(mav, pressure=101325.0, temperature=20.0)
            
            time.sleep(0.01)  # ~100Hz update rate
            
            counter += 1
            if counter % 1000 == 0:
                print(f"  Sent {counter} sensor updates...")
                
    except KeyboardInterrupt:
        print("\n\nStopping sensor injection...")
        print("✓ Sensor injection stopped")

if __name__ == "__main__":
    main()
