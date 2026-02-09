# Proxigo Simulation Quick Start

## ✅ Current Status: QGC Connected!

Your simulation is now running and QGroundControl can connect. However, you're seeing calibration warnings because the "none" simulator doesn't provide sensors.

## 🚀 Solution: Use jMAVSim for Fixed-Wing and VTOL with Sensors

I've created separate Docker configurations for different simulation modes:

### Option 1: jMAVSim (Recommended for Fixed-Wing & VTOL)

**Perfect for:** Fixed-wing and VTOL development with sensors, no 3D visualization needed

```bash
# Start jMAVSim simulation
./scripts/start_simulation_jmavsim.sh

# Or using Docker Compose
docker compose -f docker/simulation/docker-compose.jmavsim.yml up
```

**Features:**
- ✅ Full sensor simulation (IMU, GPS, Barometer)
- ✅ Airframes: Advanced Plane (4008) or Standard VTOL (4001)
- ✅ Position estimates
- ✅ No X11/display required
- ✅ Works reliably on Windows/WSL2

**Usage:**
```bash
# Fixed-wing (default)
./scripts/start_simulation_jmavsim.sh

# VTOL
./scripts/start_simulation_jmavsim.sh --vehicle vtol
```

**First run:** Takes 2-3 minutes to build PX4, then starts quickly.

### Option 2: Gazebo (Full 3D Visualization)

**Perfect for:** Visual testing, terrain following

```bash
docker compose -f docker/simulation/docker-compose.gazebo.yml up
```

**Note:** Requires VcXsrv/XLaunch on Windows. May have bridge timeout issues.

### Option 3: None Simulator (Current - Minimal)

**Perfect for:** MAVLink connectivity testing only

```bash
./scripts/start_simulation_simple.sh
```

**Limitations:**
- ❌ No sensors
- ❌ No airframe configuration
- ❌ No position estimates

## 📋 What Each Mode Provides

| Feature | jMAVSim | Gazebo | None (Current) |
|---------|---------|--------|---------------|
| Sensors | ✅ | ✅ | ❌ |
| Airframe Config | ✅ | ✅ | ❌ |
| Position Estimates | ✅ | ✅ | ❌ |
| 3D Visualization | ❌ | ✅ | ❌ |
| Calibration Needed | ❌ | ❌ | ✅ (no sensors) |

## 🔧 Fixing Your Current Setup

To get sensors and airframe configuration without Gazebo:

1. **Stop current simulation:**
   ```bash
   docker rm -f px4_gazebo_plane
   ```

2. **Start jMAVSim:**
   ```bash
   ./scripts/start_simulation_jmavsim.sh
   ```

3. **Wait 2-3 minutes** for first-time build, then:
   - QGC will connect automatically
   - Sensors will be available
   - Airframe will be configured (Advanced Plane)
   - No calibration needed!

## 📁 File Structure

```
docker/simulation/
├── docker-compose.jmavsim.yml    # jMAVSim (lightweight, sensors)
├── docker-compose.gazebo.yml     # Gazebo (full 3D)
└── docker-compose.simulation.yml # Original (Gazebo)

scripts/
├── start_simulation_jmavsim.sh   # Start jMAVSim
├── start_simulation_simple.sh   # Start "none" simulator
└── start_simulation.sh           # Original (Gazebo)
```

## 🎯 Recommended Workflow

1. **Daily Development:** Use jMAVSim (sensors + airframe, no GUI issues)
2. **Visual Testing:** Use Gazebo when you need 3D visualization
3. **Quick Tests:** Use "none" simulator for MAVLink connectivity only

## 🔍 Troubleshooting

**jMAVSim not starting?**
- First run takes 2-3 minutes to build PX4
- Check logs: `docker logs px4_jmavsim`

**Still seeing calibration warnings?**
- Make sure you're using jMAVSim, not "none" simulator
- Verify airframe: `docker exec px4_jmavsim bash -c "param show SYS_AUTOSTART"`

**QGC not connecting?**
- Check MAVProxy: `docker exec px4_jmavsim pgrep -f mavproxy`
- Test connection: `docker exec px4_jmavsim python3 /scripts/test_qgc_connection.py`
