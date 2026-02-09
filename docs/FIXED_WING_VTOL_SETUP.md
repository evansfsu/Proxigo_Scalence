# Fixed-Wing and VTOL Simulation Setup

This guide covers setting up PX4 simulation specifically for **Fixed-Wing** and **VTOL** aircraft.

## Supported Vehicles

### Fixed-Wing
- **Advanced Plane** (SYS_AUTOSTART=4008)
  - Full fixed-wing aerodynamics
  - Mission support
  - Long-range flight capabilities

### VTOL (Vertical Take-Off and Landing)
- **Standard VTOL** (SYS_AUTOSTART=4001)
  - Tiltrotor configuration
  - Vertical takeoff/landing
  - Forward flight mode
  - Transition between modes

## Simulation Modes

### 1. jMAVSim (Recommended)

**Best for:** Fixed-wing and VTOL development with sensors, no 3D visualization

**Fixed-Wing:**
```bash
./scripts/start_simulation_jmavsim.sh
# Or
docker compose -f docker/simulation/docker-compose.jmavsim.yml up
```

**VTOL:**
```bash
./scripts/start_simulation_jmavsim.sh --vehicle vtol
# Or
PX4_VEHICLE=standard_vtol docker compose -f docker/simulation/docker-compose.jmavsim.yml up
```

**Features:**
- ✅ Full sensor simulation (IMU, GPS, Barometer)
- ✅ Proper airframe configuration
- ✅ Position estimates
- ✅ No X11/display required
- ✅ Works reliably on Windows/WSL2

### 2. Gazebo Harmonic

**Best for:** Visual testing, terrain following, 3D visualization

**Fixed-Wing:**
```bash
docker compose -f docker/simulation/docker-compose.gazebo.yml up
```

**VTOL:**
```bash
PX4_VEHICLE=standard_vtol docker compose -f docker/simulation/docker-compose.gazebo.yml up
```

**Features:**
- ✅ Full 3D physics simulation
- ✅ Terrain visualization
- ✅ Camera simulation
- ❌ Requires X11 display (VcXsrv on Windows)
- ❌ More resource intensive

## Airframe Configuration

### Fixed-Wing (Advanced Plane)
- **SYS_AUTOSTART:** 4008
- **Model:** `advanced_plane` or `gz_advanced_plane`
- **Features:**
  - Fixed-wing aerodynamics
  - Mission flight modes
  - Efficient cruise flight

### VTOL (Standard VTOL)
- **SYS_AUTOSTART:** 4001
- **Model:** `standard_vtol` or `gz_standard_vtol`
- **Features:**
  - Vertical takeoff/landing
  - Forward flight
  - Automatic transition

## Quick Start

### Fixed-Wing Development

1. **Start jMAVSim simulation:**
   ```bash
   ./scripts/start_simulation_jmavsim.sh
   ```

2. **Wait for PX4 to initialize** (2-3 minutes first run)

3. **Connect QGroundControl:**
   - Auto-detects on UDP 14550
   - Or manually: TCP 127.0.0.1:5760

4. **Verify airframe:**
   - QGC → Vehicle Setup → Airframe
   - Should show "Generic Fixed Wing" or "Advanced Plane"

### VTOL Development

1. **Start VTOL simulation:**
   ```bash
   ./scripts/start_simulation_jmavsim.sh --vehicle vtol
   ```

2. **Same steps as fixed-wing above**

3. **VTOL-specific features:**
   - QGC will show VTOL flight modes
   - Can test vertical takeoff/landing
   - Transition between modes

## Environment Variables

| Variable | Fixed-Wing | VTOL | Description |
|----------|------------|------|-------------|
| `PX4_VEHICLE` | `advanced_plane` | `standard_vtol` | Vehicle model |
| `SYS_AUTOSTART` | `4008` | `4001` | Airframe ID |
| `PX4_HOME_LAT` | `36.2329` | `36.2329` | Home latitude (Death Valley) |
| `PX4_HOME_LON` | `-116.8276` | `-116.8276` | Home longitude |
| `PX4_HOME_ALT` | `50` | `50` | Home altitude (meters) |

## Mission Planning

Both fixed-wing and VTOL support mission planning in QGroundControl:

1. **Plan mission in QGC:**
   - Fixed-wing: Waypoint missions, survey patterns
   - VTOL: Can include vertical takeoff, waypoints, vertical landing

2. **Upload mission:**
   ```bash
   docker exec px4_jmavsim python3 /scripts/upload_simple_mission.py
   ```

3. **Mission file location:**
   - `config/mission/death_valley_simple.plan`

## Troubleshooting

### Airframe Not Configured

If QGC shows "Choose airframe":
- Verify SYS_AUTOSTART is set correctly
- Check container logs: `docker logs px4_jmavsim | grep SYS_AUTOSTART`
- Restart PX4 with correct airframe

### No Position Estimates

- Make sure you're using jMAVSim or Gazebo (not "none" simulator)
- Check sensors: `docker exec px4_jmavsim param show SENS_*`
- Verify GPS: QGC → Vehicle Setup → Sensors → GPS

### VTOL Not Transitioning

- Verify airframe: `docker exec px4_jmavsim param show SYS_AUTOSTART`
- Should be 4001 for standard VTOL
- Check flight mode in QGC

## Notes

- **Quadcopter models (x500, iris) are NOT supported** - this setup is specifically for fixed-wing and VTOL
- jMAVSim is recommended for daily development (no GUI issues)
- Gazebo is available when 3D visualization is needed
- Both simulators provide full sensor data and airframe configuration
