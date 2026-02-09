# Robust Simulation Options for Fixed-Wing & VTOL

This guide covers modern, robust simulation alternatives to jMAVSim for fixed-wing and VTOL aircraft.

## Why Not jMAVSim?

jMAVSim is outdated and has limitations:
- Limited fixed-wing support
- Less realistic physics
- Maintenance issues
- Not recommended for production use

## Recommended Options

### 1. Gazebo Harmonic Headless (Recommended)

**Best for:** Daily development with full physics, no GUI issues

**Why it's robust:**
- ✅ Full physics simulation
- ✅ Excellent fixed-wing and VTOL support
- ✅ Headless mode (no X11/GUI required)
- ✅ Actively maintained by PX4 team
- ✅ Reliable sensor simulation
- ✅ No bridge timeout issues in headless mode

**Usage:**
```bash
# Fixed-wing (default)
./scripts/start_simulation_gazebo_headless.sh

# VTOL
./scripts/start_simulation_gazebo_headless.sh --vehicle vtol

# Or using Docker Compose
docker compose -f docker/simulation/docker-compose.gazebo-headless.yml up
```

**Features:**
- Full sensor simulation (IMU, GPS, Barometer)
- Proper airframe configuration
- Position estimates
- Realistic aerodynamics
- No X11/display required (headless)
- Works reliably on Windows/WSL2

### 2. FlightGear (Excellent for Fixed-Wing)

**Best for:** Fixed-wing development with realistic flight dynamics

**Why it's robust:**
- ✅ Specifically designed for fixed-wing aircraft
- ✅ Realistic flight dynamics
- ✅ Excellent aerodynamics modeling
- ✅ Supports VTOL configurations
- ✅ Mature and stable
- ✅ No GUI required (can run headless)

**Usage:**
```bash
# Fixed-wing (default)
./scripts/start_simulation_flightgear.sh

# VTOL
./scripts/start_simulation_flightgear.sh --vehicle vtol

# Or using Docker Compose
docker compose -f docker/simulation/docker-compose.flightgear.yml up
```

**Features:**
- Excellent fixed-wing physics
- Realistic aerodynamics
- Full sensor simulation
- Proper airframe configuration
- Position estimates
- VTOL support

### 3. Gazebo Harmonic with GUI (For Visualization)

**Best for:** Visual testing, terrain following, 3D visualization

**Usage:**
```bash
docker compose -f docker/simulation/docker-compose.gazebo.yml up
```

**Note:** Requires VcXsrv/XLaunch on Windows. Use headless version for reliability.

## Comparison

| Feature | Gazebo Headless | FlightGear | Gazebo GUI | None |
|---------|----------------|------------|------------|------|
| **Robustness** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Fixed-Wing** | ✅ Excellent | ✅ Excellent | ✅ Excellent | ❌ |
| **VTOL** | ✅ Excellent | ✅ Good | ✅ Excellent | ❌ |
| **Sensors** | ✅ Full | ✅ Full | ✅ Full | ❌ |
| **Physics** | ✅ Realistic | ✅ Very Realistic | ✅ Realistic | ❌ |
| **GUI Required** | ❌ No | ❌ No | ✅ Yes | ❌ No |
| **X11 Issues** | ❌ None | ❌ None | ⚠️ Possible | ❌ None |
| **Resource Usage** | Medium | Medium | High | Minimal |
| **Maintenance** | ✅ Active | ✅ Stable | ✅ Active | ✅ N/A |

## Quick Start

### For Fixed-Wing Development

**Recommended: Gazebo Headless**
```bash
./scripts/start_simulation_gazebo_headless.sh
```

**Alternative: FlightGear**
```bash
./scripts/start_simulation_flightgear.sh
```

### For VTOL Development

**Recommended: Gazebo Headless**
```bash
./scripts/start_simulation_gazebo_headless.sh --vehicle vtol
```

## Troubleshooting

### Gazebo Headless Bridge Timeout

If you see `ERROR [gz_bridge] Service call timed out`:
- This is normal in headless mode - PX4 will continue without GUI
- Sensors and MAVLink will still work
- Check if PX4 is running: `docker exec px4_gazebo_headless pgrep -f px4`

### FlightGear Not Starting

If FlightGear fails to start:
- Check if FlightGear is installed in the container
- Verify vehicle model: `docker exec px4_flightgear ls /root/PX4-Autopilot/Tools/simulation/flightgear/`
- Check logs: `docker logs px4_flightgear`

### No Sensors/Position Estimates

- Make sure you're using Gazebo or FlightGear (not "none" simulator)
- Verify airframe: `docker exec <container> param show SYS_AUTOSTART`
- Check sensors: `docker exec <container> param show SENS_*`

## Migration from jMAVSim

If you were using jMAVSim:

1. **Stop jMAVSim:**
   ```bash
   docker compose -f docker/simulation/docker-compose.jmavsim.yml down
   ```

2. **Start Gazebo Headless:**
   ```bash
   ./scripts/start_simulation_gazebo_headless.sh
   ```

3. **Verify connection:**
   - QGC should connect automatically
   - Sensors should be available
   - No calibration needed

## Notes

- **jMAVSim is deprecated** - use Gazebo Headless or FlightGear instead
- **Gazebo Headless** is the recommended replacement (most robust)
- **FlightGear** is excellent specifically for fixed-wing
- Both support fixed-wing and VTOL configurations
- Both provide full sensor data and airframe configuration
- Both work reliably without GUI/X11 issues
