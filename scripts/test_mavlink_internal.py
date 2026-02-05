#!/usr/bin/env python3
"""Test MAVLink connection from inside the container."""
from pymavlink import mavutil
import time

print("Testing MAVLink connection...")

# PX4 SITL MAVLink configuration:
# - GCS port: 18570 (udp)
# - Offboard: local=14580, remote=14540

# Try udpout to connect to PX4's listening port
connections = [
    ("udpout:127.0.0.1:18570", "GCS port"),
    ("udpin:0.0.0.0:14540", "Offboard remote"),
    ("udpout:127.0.0.1:14580", "Offboard local"),
]

for conn_str, desc in connections:
    print(f"\nTrying {desc}: {conn_str}")
    try:
        mav = mavutil.mavlink_connection(conn_str, source_system=255)
        
        # Send a heartbeat to initiate
        mav.mav.heartbeat_send(6, 8, 0, 0, 0)
        time.sleep(0.5)
        
        # Wait for response
        start = time.time()
        while time.time() - start < 5:
            msg = mav.recv_match(blocking=False)
            if msg:
                print(f"  Received: {msg.get_type()}")
                if msg.get_type() == 'HEARTBEAT':
                    print(f"  SUCCESS! Connected via {conn_str}")
                    break
            time.sleep(0.1)
        else:
            print("  No heartbeat received")
            
    except Exception as e:
        print(f"  Error: {e}")
