# Mission Upload Troubleshooting

## Problem: Mission Upload Fails

If mission upload fails with "No request received" or timeout errors, try these solutions:

## Solution 1: Upload via QGroundControl (Recommended)

The easiest way to upload a mission is through QGroundControl:

1. **Connect QGC** to PX4:
   - TCP: `127.0.0.1:5760`
   - Or UDP: `127.0.0.1:14550` (auto-detect)

2. **Load mission file**:
   - Go to **Plan** view
   - Click **File** → **Load Mission**
   - Select: `config/mission/death_valley_simple.plan`

3. **Upload mission**:
   - Click **Upload** button (or press Ctrl+U)
   - Mission should upload successfully

## Solution 2: Wait for PX4 to be Ready

PX4 may need more time to initialize. Wait 30-60 seconds after starting the simulation before uploading.

Check if PX4 is ready:
```bash
docker exec px4_sitl python3 /scripts/check_px4_ready.py
```

## Solution 3: Restart PX4 Mission System

If PX4 is running but not accepting missions, try restarting the mission system:

1. In QGC, go to **Vehicle Setup** → **Parameters**
2. Search for: `MAV_1_CONFIG`
3. Change it temporarily, then change it back
4. This forces PX4 to reinitialize the mission system

## Solution 4: Check PX4 Status

Verify PX4 is actually running:
```bash
docker exec px4_sitl bash -c "pgrep -f px4"
```

Check PX4 logs for errors:
```bash
docker logs px4_sitl | tail -50
```

Look for:
- Mission system errors
- MAVLink connection issues
- EKF initialization problems

## Solution 5: Use QGC Mission Editor

Instead of uploading via script, create the mission in QGC:

1. **Plan** view → Right-click on map
2. **Add Waypoint** at Death Valley location
3. Add multiple waypoints in a small area (~200m x 200m)
4. **Upload** the mission

This ensures the mission is compatible with your PX4 version.

## Solution 6: Check Mission Format

The mission file format must match your PX4 version. If upload fails:

1. Open mission file in QGC
2. QGC will convert it to the correct format
3. Save and upload

## Common Issues

### "No request received"
- **Cause**: PX4 mission system not initialized
- **Fix**: Wait longer, or restart simulation

### "Mission upload timeout"
- **Cause**: PX4 not responding to MAVLink
- **Fix**: Check MAVProxy is running, verify connection

### "Waypoint too far"
- **Cause**: Vehicle position not set correctly
- **Fix**: Run `set_vehicle_position.py` before uploading

### "ACK type not accepted"
- **Cause**: Mission format incompatible
- **Fix**: Use QGC to create/upload mission instead

## Recommended Workflow

1. **Start simulation**: `./scripts/start_simulation.sh --restart`
2. **Wait 30-60 seconds** for PX4 to fully initialize
3. **Connect QGC** and verify connection
4. **Set home position** in QGC (right-click map → "Set Home Here")
5. **Upload mission** via QGC (File → Load Mission → Upload)

This is more reliable than script-based uploads.
