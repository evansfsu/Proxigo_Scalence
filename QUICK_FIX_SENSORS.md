# Quick Fix: Sensor Calibration

## Problem
QGC shows "Accelerometer and Compass need setup" and "No mags found" error.

## Solution: Sensor Data Injection

Sensor injection is now running! It provides:
- ✅ Accelerometer data (for calibration)
- ✅ Magnetometer/Compass data (fixes "No mags found")
- ✅ Gyroscope data
- ✅ GPS data

## In QGroundControl

1. **Go to Vehicle Setup → Sensors**
2. **Calibrate Accelerometer:**
   - Click "Accelerometer" button
   - Follow instructions (rotate vehicle)
   - Should work now!

3. **Calibrate Compass:**
   - Click "Compass" button  
   - Follow instructions (rotate vehicle)
   - Should work now! (No more "No mags found" error)

## Verify Sensor Injection

```powershell
# Check if running
wsl -d Ubuntu bash -c "docker exec px4_gazebo_plane pgrep -f inject_sensor_data"
```

If not running, start it:
```powershell
wsl -d Ubuntu bash -c "docker exec -d px4_gazebo_plane python3 /scripts/inject_sensor_data.py 36.2329 -116.8276 50"
```

## Airframe Selection

You can select "Generic Flying Wing" in QGC - that's correct for fixed-wing!

The sensor injection provides all the data needed for calibration even though we're using the "none" simulator.
