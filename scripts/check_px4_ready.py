#!/usr/bin/env python3
"""Check if PX4 is ready to accept missions."""

import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not available")
    sys.exit(1)

def check_px4_ready():
    """Check if PX4 is ready for mission upload."""
    connections = [
        "udp:127.0.0.1:14540",
        "tcp:127.0.0.1:5760",
        "udp:127.0.0.1:14550",
    ]
    
    mav = None
    for conn in connections:
        try:
            mav = mavutil.mavlink_connection(conn, timeout=3)
            mav.wait_heartbeat(timeout=5)
            print(f"✓ Connected via {conn}")
            break
        except:
            continue
    
    if mav is None:
        print("✗ Cannot connect to PX4")
        return False
    
    # Check for HEARTBEAT
    print("Checking PX4 status...")
    msg = mav.recv_match(type='HEARTBEAT', timeout=2)
    if msg:
        print(f"  ✓ Heartbeat received (system: {mav.target_system}, component: {mav.target_component})")
    else:
        print("  ✗ No heartbeat")
        return False
    
    # Check for SYS_STATUS
    msg = mav.recv_match(type='SYS_STATUS', timeout=2)
    if msg:
        print(f"  ✓ System status received")
    else:
        print("  ⚠ No system status (may be normal)")
    
    # Try to request mission count (test if mission system is ready)
    print("  Testing mission system...")
    mav.mav.mission_request_list_send(mav.target_system, mav.target_component)
    
    msg = mav.recv_match(type=['MISSION_COUNT', 'MISSION_ACK'], timeout=3)
    if msg:
        if msg.get_type() == 'MISSION_COUNT':
            print(f"  ✓ Mission system ready (current mission has {msg.count} waypoints)")
        else:
            print(f"  ⚠ Mission system responded with ACK type {msg.type}")
        return True
    else:
        print("  ✗ Mission system not responding")
        print("  ⚠ PX4 may need more time to initialize")
        return False

if __name__ == "__main__":
    if check_px4_ready():
        print("\n✓ PX4 is ready for mission upload")
        sys.exit(0)
    else:
        print("\n✗ PX4 is not ready. Wait a few seconds and try again.")
        sys.exit(1)
