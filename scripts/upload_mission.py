#!/usr/bin/env python3
"""
Upload Death Valley survey mission to PX4
This script connects to PX4 and uploads a predefined mission.
"""

from pymavlink import mavutil
import time
import sys

# Death Valley coordinates (center: 36.2291, -116.8325)
MISSION_WAYPOINTS = [
    # (lat, lon, alt, command)
    # 0: Home position
    (36.2291, -116.8325, 0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
    # 1: Takeoff
    (36.2291, -116.8325, 50, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF),
    # 2: WP1 - North
    (36.2350, -116.8325, 100, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
    # 3: WP2 - Northeast
    (36.2350, -116.8275, 100, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
    # 4: WP3 - Southeast  
    (36.2230, -116.8275, 100, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
    # 5: WP4 - Southwest
    (36.2230, -116.8375, 100, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
    # 6: WP5 - Northwest
    (36.2350, -116.8375, 100, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
    # 7: Land
    (36.2291, -116.8325, 0, mavutil.mavlink.MAV_CMD_NAV_LAND),
]

def upload_mission(connection_string='udp:127.0.0.1:14540'):
    print(f"Connecting to {connection_string}...")
    m = mavutil.mavlink_connection(connection_string)
    m.wait_heartbeat()
    print(f"Connected to system {m.target_system}, component {m.target_component}")
    
    # Clear existing mission
    print("Clearing existing mission...")
    m.mav.mission_clear_all_send(m.target_system, m.target_component)
    time.sleep(1)
    
    # Send mission count
    count = len(MISSION_WAYPOINTS)
    print(f"Uploading {count} waypoints...")
    m.mav.mission_count_send(
        m.target_system, m.target_component, 
        count, 
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )
    
    # Handle mission item requests
    uploaded = 0
    timeout_count = 0
    
    while uploaded < count and timeout_count < 10:
        msg = m.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], 
                          blocking=True, timeout=5)
        
        if msg is None:
            timeout_count += 1
            print(f"  Timeout waiting for request (attempt {timeout_count})")
            continue
            
        if msg.get_type() == 'MISSION_ACK':
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("Mission upload complete!")
                return True
            else:
                print(f"Mission rejected with code {msg.type}")
                return False
                
        seq = msg.seq
        if seq < count:
            lat, lon, alt, cmd = MISSION_WAYPOINTS[seq]
            
            # Use MISSION_ITEM_INT for better precision
            m.mav.mission_item_int_send(
                m.target_system, m.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                cmd,
                1 if seq == 1 else 0,  # current (set first real waypoint as current)
                1,  # autocontinue
                0, 0, 0, 0,  # params 1-4
                int(lat * 1e7),
                int(lon * 1e7),
                alt
            )
            print(f"  Sent waypoint {seq}: ({lat}, {lon}) @ {alt}m - {cmd}")
            uploaded += 1
            timeout_count = 0
    
    # Wait for final ACK
    msg = m.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if msg and msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("Mission upload SUCCESS!")
        return True
    
    print("Mission upload may have failed")
    return False

def arm_and_start(connection_string='udp:127.0.0.1:14540'):
    print("Connecting for arm/start...")
    m = mavutil.mavlink_connection(connection_string)
    m.wait_heartbeat()
    
    # Set mission mode
    print("Setting AUTO.MISSION mode...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, 1, 4, 0, 0, 0, 0, 0  # AUTO mode
    )
    time.sleep(1)
    
    # Arm
    print("Arming (force)...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21196, 0, 0, 0, 0, 0  # Force arm
    )
    time.sleep(2)
    
    # Start mission
    print("Starting mission...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    
    print("Mission started!")

if __name__ == '__main__':
    conn = sys.argv[1] if len(sys.argv) > 1 else 'udp:127.0.0.1:14540'
    
    if upload_mission(conn):
        print("\nWaiting 3 seconds before starting...")
        time.sleep(3)
        arm_and_start(conn)
    else:
        print("Failed to upload mission")
        sys.exit(1)
