# Sensor Calibration with "None" Simulator

## Problem

The "none" simulator doesn't provide sensor data, so QGroundControl shows:
- "Accelerometer needs setup"
- "Compass needs setup"
- "No mags found" error

## Solution: Sensor Data Injection

We inject sensor data via MAVLink messages to simulate sensors, allowing calibration.

## Quick Start

### Option 1: Start with Sensor Injection (Recommended)

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\start_simulation.ps1 -Simulator with-sensors
```

This will:
1. Start PX4 simulation
2. Automatically start sensor data injection
3. Provide all sensors for calibration

### Option 2: Manual Sensor Injection

If simulation is already running:

```bash
# In WSL
wsl -d Ubuntu bash -c "docker exec -d px4_gazebo_plane python3 /scripts/inject_sensor_data.py 36.2329 -116.8276 50"
```

## What Sensors Are Injected

The injection script provides:

1. **Accelerometer** - Via HIGHRES_IMU messages
   - Simulates gravity (9.81 m/s² downward)
   - Allows accelerometer calibration

2. **Gyroscope** - Via HIGHRES_IMU messages
   - Simulates no rotation (for level flight)
   - Allows gyroscope calibration

3. **Magnetometer (Compass)** - Via RAW_IMU and HIGHRES_IMU messages
   - Simulates Earth's magnetic field
   - Death Valley: ~12° declination, ~60° inclination
   - Allows compass calibration

4. **Barometer** - Via SCALED_PRESSURE messages
   - Simulates sea-level pressure (1013.25 hPa)
   - Allows barometer calibration

5. **GPS** - Via GPS_INPUT messages
   - Provides position at Death Valley coordinates
   - Allows GPS calibration/verification

## Calibration Steps in QGC

1. **Open QGroundControl**
2. **Go to Vehicle Setup → Sensors**
3. **Calibrate Accelerometer:**
   - Click "Accelerometer" button
   - Follow on-screen instructions (rotate vehicle)
   - Sensor injection provides data during calibration

4. **Calibrate Compass (Magnetometer):**
   - Click "Compass" button
   - Follow on-screen instructions (rotate vehicle)
   - Sensor injection provides magnetometer data

5. **Verify other sensors:**
   - Barometer should show pressure
   - GPS should show position

## Verify Sensor Injection is Running

```bash
# Check if injection is running
wsl -d Ubuntu bash -c "docker exec px4_gazebo_plane pgrep -f inject_sensor_data"

# View injection logs
wsl -d Ubuntu bash -c "docker exec px4_gazebo_plane tail -20 /tmp/sensor_injection.log"
```

## Troubleshooting

### "No mags found" Error

**Cause:** Sensor injection not running or not providing magnetometer data

**Fix:**
```bash
# Restart sensor injection
wsl -d Ubuntu bash -c "docker exec px4_gazebo_plane pkill -f inject_sensor_data"
wsl -d Ubuntu bash -c "docker exec -d px4_gazebo_plane python3 /scripts/inject_sensor_data.py 36.2329 -116.8276 50"
```

### Accelerometer Calibration Fails

**Cause:** Sensor injection not providing accelerometer data

**Fix:** Ensure injection is running and check logs for errors

### Compass Calibration Shows "No mags found"

**Cause:** Magnetometer data not being sent correctly

**Fix:**
1. Verify injection is running: `docker exec px4_gazebo_plane pgrep -f inject_sensor_data`
2. Check logs: `docker exec px4_gazebo_plane cat /tmp/sensor_injection.log`
3. Restart injection if needed

## Technical Details

### Sensor Data Format

- **IMU Data:** Sent at ~100Hz via HIGHRES_IMU messages
- **Magnetometer:** Sent via both RAW_IMU and HIGHRES_IMU for compatibility
- **GPS:** Sent at ~10Hz via GPS_INPUT messages
- **Barometer:** Sent at ~50Hz via SCALED_PRESSURE messages

### Magnetic Field Simulation

The script simulates Earth's magnetic field for Death Valley:
- **Declination:** ~12° (magnetic north vs true north)
- **Inclination:** ~60° (magnetic field angle)
- **Strength:** ~0.2-0.4 Tesla (typical Earth field)

This allows compass calibration to work properly.

## Notes

- Sensor injection runs continuously in the background
- Data is injected at realistic rates (100Hz for IMU, 10Hz for GPS)
- Calibration data is provided during QGC calibration procedures
- Injection must be running for sensors to be detected
