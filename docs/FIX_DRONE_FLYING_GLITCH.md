# Fixing Drone Flying Across Map Glitch

## Problem
The drone flies at maximum speed across the map (e.g., from Zurich to Death Valley) instead of staying at the mission location. This happens even when home position is set correctly.

## Root Cause (Based on Official Documentation)

According to [ArduPilot's official documentation](https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html):

1. **EKF Origin Can Only Be Set Once**: The EKF (Extended Kalman Filter) origin is the reference location for all position calculations. **Once set, it cannot be changed** without restarting PX4.

2. **Timing is Critical**: The EKF origin must be set **BEFORE** the EKF initializes (within the first few seconds after PX4 starts).

3. **Home Position ≠ EKF Origin**: Setting the home position is not enough. The EKF origin must also be set to match the vehicle's actual location.

## Solution

### Method 1: Set EKF Origin at Startup (Recommended)

Run this script **IMMEDIATELY** after PX4 starts (within first 5 seconds):

```bash
docker exec px4_sitl python3 /scripts/fix_ekf_origin_startup.py
```

This script:
1. Sets GPS global origin (EKF origin) - can only be done once
2. Sets home position to match
3. Sends GPS_INPUT messages to establish vehicle position
4. Must be run before EKF initializes

### Method 2: Automatic Fix on Startup

The `start_simulation.sh` script now automatically sets the EKF origin immediately after PX4 starts. This ensures it's set before the EKF initializes.

### Method 3: Use MAVProxy Fake GPS Module

Based on [MAVProxy's Fake GPS documentation](https://ardupilot.org/mavproxy/docs/modules/fakegps.html):

1. Set GPS1_TYPE parameter to 14 (MAVLink):
   ```bash
   docker exec px4_sitl bash -c "param set GPS1_TYPE 14"
   ```

2. Use MAVProxy's fakegps module (if available):
   - Load module: `module load fakegps`
   - Right-click on map to set position
   - 10-second EKF realignment delay

### Method 4: Continuous GPS Injection

If the vehicle is already flying erratically, run this to continuously send GPS position:

```bash
docker exec px4_sitl python3 /scripts/force_vehicle_position.py
```

Keep this running while the mission is active.

## Implementation Details

### Why GPS_INPUT Messages?

GPS_INPUT messages simulate a GPS receiver reporting the vehicle's position. This is more reliable than just setting home position because:

1. It updates the EKF's position estimate directly
2. It can be sent continuously to maintain position
3. It works even if EKF origin was set incorrectly

### Why Multiple Methods?

We use multiple methods because:
- **SET_GPS_GLOBAL_ORIGIN**: Sets the EKF origin (one-time, at startup)
- **GPS_INPUT**: Continuously updates vehicle position estimate
- **SET_HOME**: Sets the home position (for return-to-home)

All three must be synchronized to the same location.

## Verification

After setting EKF origin:

1. **Wait 10-15 seconds** for EKF to converge
2. **Check QGC**: Vehicle icon should be at Death Valley
3. **Check EKF status**: Should show valid position estimate
4. **Upload mission**: Waypoints should be close together
5. **No erratic flight**: Vehicle should stay in the mission area

## If It Still Doesn't Work

1. **Restart the simulation**: EKF origin can only be set once per session
   ```bash
   ./scripts/start_simulation.sh --restart
   ```

2. **Check timing**: Make sure the script runs within 5 seconds of PX4 startup

3. **Check PX4 logs**:
   ```bash
   docker logs px4_sitl | grep -i "ekf\|gps\|origin"
   ```

4. **Verify parameters**:
   ```bash
   docker exec px4_sitl bash -c "param show | grep -i gps"
   ```

## References

- [Setting Home and/or EKF origin](https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html)
- [Fake GPS Module](https://ardupilot.org/mavproxy/docs/modules/fakegps.html)
- [PX4 Simulation Debugging](https://docs.px4.io/main/en/debug/simulation_debugging.html)
- [PX4 Missions](https://docs.px4.io/v1.14/en/flying/missions)

## Key Takeaways

1. **EKF origin can only be set ONCE** - must be done at startup
2. **Timing is critical** - set it before EKF initializes
3. **Use multiple methods** - SET_GPS_GLOBAL_ORIGIN + GPS_INPUT + SET_HOME
4. **Wait for convergence** - give EKF 10-15 seconds to settle
5. **If it fails, restart** - you can't change EKF origin mid-session
