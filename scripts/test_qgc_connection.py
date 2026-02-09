#!/usr/bin/env python3
"""Test QGroundControl connection to PX4 SITL."""
from pymavlink import mavutil
import time

print("Testing QGroundControl connection...")
print("")

# Test UDP 14550
print("1. Testing UDP 14550 (QGC default)...")
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

# Test TCP 5760
print("2. Testing TCP 5760 (QGC manual)...")
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
print("If both failed, check:")
print("  - PX4 SITL is running: docker ps | grep px4_sitl")
print("  - MAVLink router is running: docker ps | grep mavlink_router")
print("  - Router logs: docker logs mavlink_router")
