# Fixing "Drone Flying Across America at Light Speed"

## Problem
Even after setting home position correctly, the drone was:
- Flying erratically across the map at maximum speed
- Thinking it was in Zurich/Africa and trying to reach Death Valley waypoints
- Glitching and resetting during flight

## Root Cause
Setting the **home position** tells PX4 where "home" is, but PX4 still thinks the **vehicle's current position** is at the default location (Zurich). When you upload a Death Valley mission, PX4 calculates the distance from Zurich to Death Valley and flies at maximum speed to get there.

## Solution
Set the vehicle's **current GPS position** (not just home position) using GPS_INPUT messages. This tells PX4 where the vehicle currently is.

## Quick Fix

### Option 1: Use the Updated Mission Uploader (Recommended)
The `upload_simple_mission.py` script now automatically sets:
1. Home position
2. EKF origin
3. **Vehicle's current GPS position** ← This is the key fix!

```bash
docker exec px4_sitl python3 /scripts/upload_simple_mission.py
```

### Option 2: Set Vehicle Position Manually
If the vehicle is already flying erratically:

```bash
docker exec px4_sitl python3 /scripts/set_vehicle_position.py
```

This will:
- Send GPS_INPUT messages to set current position
- Set GPS global origin
- Set home position
- Stop the erratic flight behavior

## How It Works

The `set_vehicle_position.py` script uses **GPS_INPUT** MAVLink messages to simulate a GPS receiver reporting the vehicle's current position. This is different from just setting home position:

- **Home Position**: Where the vehicle should return to
- **GPS_INPUT**: Where the vehicle currently is (what we need!)

By sending multiple GPS_INPUT messages, PX4's EKF (Extended Kalman Filter) updates its position estimate to Death Valley, and the vehicle stops trying to fly across the continent.

## Technical Details

The GPS_INPUT message includes:
- Latitude/Longitude (in 1e7 format)
- Altitude
- Fix type (3 = 3D fix)
- Satellite count
- HDOP/VDOP (position accuracy)

We send 10-15 messages to ensure PX4 receives and processes them.

## Verification

After running the script:
1. Check QGC - vehicle icon should be at Death Valley
2. Vehicle should stop flying erratically
3. Mission waypoints should be reachable
4. No more "flying across America" behavior

## If It Still Doesn't Work

1. **Restart the simulation** - Sometimes PX4 needs a fresh start:
   ```bash
   ./scripts/start_simulation.sh --restart
   ```

2. **Run both scripts in sequence**:
   ```bash
   docker exec px4_sitl python3 /scripts/set_vehicle_position.py
   docker exec px4_sitl python3 /scripts/upload_simple_mission.py
   ```

3. **Check PX4 logs** for GPS/EKF errors:
   ```bash
   docker logs px4_sitl | grep -i "gps\|ekf\|position"
   ```

## Prevention

The `upload_simple_mission.py` script now includes vehicle position setting, so future mission uploads should work correctly. The vehicle position is set automatically when you upload a mission.
