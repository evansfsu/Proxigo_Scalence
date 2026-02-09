# Simple Death Valley Mission

## Problem
When uploading missions, the drone was:
- Starting from a launch point in Africa (wrong home position)
- Glitching/flying erratically across the world map
- Waypoints were too far apart

## Solution
Created a simple mission with:
- All waypoints within ~200m of home position
- Home position set to Death Valley (36.2329, -116.8276)
- Small square pattern for testing

## Mission Details

**Home Position:** 36.2329, -116.8276, 0m (Death Valley)

**Waypoints:**
1. Takeoff to 50m at home
2. North (~120m from home)
3. Northeast (~150m)
4. Southeast (~150m)
5. Southwest (~150m)
6. Northwest (~150m)
7. Return to North
8. Return to home at 50m
9. Land at home

**Pattern Size:** ~200m x 200m square

## Usage

### Option 1: Upload via Script (Recommended)
```bash
# From container (pymavlink already installed)
docker exec px4_sitl python3 /scripts/upload_simple_mission.py

# From host (requires: pip3 install pymavlink)
python3 scripts/upload_simple_mission.py
```

This script:
1. Sets home position to Death Valley
2. Sets EKF origin to Death Valley
3. Uploads the simple mission with all waypoints close together

### Option 2: Upload via QGroundControl
1. In QGC, go to **Plan** view
2. Make sure home position is set to Death Valley (36.2329, -116.8276)
   - If not, right-click on map at Death Valley → **Set Home Here**
3. Load mission file: `config/mission/death_valley_simple.plan`
4. Upload mission

## Verification

After uploading:
1. Check QGC Plan view - all waypoints should be visible in Death Valley area
2. Verify vehicle icon is at Death Valley location (not Africa/Zurich)
3. All waypoints should be within ~200m of each other
4. No "waypoint too far" errors

## Troubleshooting

**If home position is wrong:**
```bash
docker exec px4_sitl python3 /scripts/set_home_position.py
```

**If mission doesn't upload:**
- Make sure PX4 SITL is running: `docker ps | grep px4_sitl`
- Check MAVLink connection: `docker logs px4_sitl | grep -i mavlink`
- Restart simulation: `./scripts/start_simulation.sh --restart`

**If drone still glitches:**
- Verify all waypoints are in Death Valley area in QGC
- Check that home position is correct before arming
- Try a smaller mission pattern (reduce waypoint distances)
