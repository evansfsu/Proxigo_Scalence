#!/usr/bin/env python3
"""Test QGroundControl connection to PX4 SITL."""
from pymavlink import mavutil
import time

print("Testing QGroundControl connection...")
print("")

# Test UDP 14550 (MAVProxy forward)
print("1. Testing UDP 14550 (MAVProxy -> QGC)...")
try:
    mav1 = mavutil.mavlink_connection('udp:127.0.0.1:14550', timeout=5)
    msg1 = mav1.wait_heartbeat(timeout=10)
    if msg1:
        print(f"   [OK] SUCCESS! Connected to system {mav1.target_system}")
    else:
        print("   [FAIL] No heartbeat received")
except Exception as e:
    print(f"   [FAIL] Error: {e}")

print("")

# Test UDP 18570 (PX4 GCS direct)
print("2. Testing UDP 18570 (PX4 GCS direct)...")
try:
    mav18570 = mavutil.mavlink_connection('udp:127.0.0.1:18570', timeout=5)
    msg18570 = mav18570.wait_heartbeat(timeout=10)
    if msg18570:
        print(f"   [OK] SUCCESS! Connected to system {mav18570.target_system}")
    else:
        print("   [FAIL] No heartbeat received")
except Exception as e:
    print(f"   [FAIL] Error: {e}")

print("")

# Test TCP 5760
print("3. Testing TCP 5760 (QGC manual)...")
try:
    mav2 = mavutil.mavlink_connection('tcp:127.0.0.1:5760', timeout=5)
    msg2 = mav2.wait_heartbeat(timeout=10)
    if msg2:
        print(f"   [OK] SUCCESS! Connected to system {mav2.target_system}")
    else:
        print("   [FAIL] No heartbeat received")
except Exception as e:
    print(f"   [FAIL] Error: {e}")

print("")
print("If all failed, check:")
print("  - Container: docker ps | grep px4_gazebo_plane")
print("  - MAVProxy (uses PX4 port 18570): docker exec px4_gazebo_plane pgrep -f mavproxy")
print("  - In QGC try: UDP Listening Port 14550, or 18570, or TCP 127.0.0.1:5760")
