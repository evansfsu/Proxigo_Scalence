# Fixing "Waypoint Too Far" Error

## Problem
When QGroundControl connects to PX4 SITL, the vehicle position defaults to Zurich (PX4's default location). When you upload a Death Valley mission, QGC creates waypoints from Zurich to Death Valley, resulting in:
```
Error: First waypoint too far away: 9248290 m (maximum: 900 m)
```

## Solution
Set the PX4 home position and EKF origin to Death Valley coordinates **before** uploading the mission.

## Quick Fix

### Option 1: Run the script (Recommended)
```bash
# From inside Docker container (pymavlink already installed)
docker exec px4_sitl python3 /scripts/set_home_position.py

# Or from host (requires: pip3 install pymavlink)
python3 scripts/set_home_position.py
```

### Option 2: Use the mission uploader script
The mission uploader automatically sets the home position:
```bash
python3 scripts/upload_death_valley_mission.py
```

### Option 3: Manual in QGroundControl
1. In QGC, go to **Plan** view
2. Right-click on the map at Death Valley location (36.2329, -116.8276)
3. Select **Set Home Here**
4. Then upload your mission

## Automatic Fix
The `start_simulation.sh` script now automatically sets the home position after PX4 starts. If it fails, run Option 1 above.

## Custom Coordinates
To set a different location:
```bash
docker exec px4_sitl python3 /scripts/set_home_position.py [lat] [lon] [alt]
# Example:
docker exec px4_sitl python3 /scripts/set_home_position.py 36.2329 -116.8276 0
```

## Verification
After setting home position:
1. In QGC, check the **Plan** view - the vehicle icon should be at Death Valley
2. The map should center on Death Valley
3. Upload your mission - waypoints should now be within range
