# Simulation Modes

Proxigo supports multiple simulation modes for different use cases:

## 1. jMAVSim (Recommended for Fixed-Wing)

**Best for:** Fixed-wing development, sensor testing, no 3D visualization needed

**Features:**
- ✅ Full sensor simulation (IMU, GPS, Barometer)
- ✅ Airframe configuration (SYS_AUTOSTART=4008 for advanced plane)
- ✅ Position estimates
- ✅ No X11/display required
- ✅ Lightweight and fast
- ❌ No 3D visualization

**Usage:**
```bash
./scripts/start_simulation_jmavsim.sh
```

**Docker Compose:**
```bash
docker compose -f docker/simulation/docker-compose.jmavsim.yml up
```

## 2. Gazebo Harmonic (Full 3D Visualization)

**Best for:** Visual testing, terrain following, computer vision

**Features:**
- ✅ Full 3D physics simulation
- ✅ Terrain visualization
- ✅ Camera simulation
- ✅ All sensors
- ❌ Requires X11 display (VcXsrv on Windows)
- ❌ More resource intensive
- ❌ Can have bridge timeout issues

**Usage:**
```bash
docker compose -f docker/simulation/docker-compose.gazebo.yml up
```

## 3. None Simulator (Minimal)

**Best for:** MAVLink connectivity testing only

**Features:**
- ✅ Fastest startup
- ✅ No dependencies
- ❌ No sensors
- ❌ No airframe configuration
- ❌ No position estimates

**Usage:**
```bash
./scripts/start_simulation_simple.sh
```

## Comparison

| Feature | jMAVSim | Gazebo | None |
|---------|---------|--------|------|
| Sensors | ✅ | ✅ | ❌ |
| Airframe Config | ✅ | ✅ | ❌ |
| Position Estimates | ✅ | ✅ | ❌ |
| 3D Visualization | ❌ | ✅ | ❌ |
| X11 Required | ❌ | ✅ | ❌ |
| Resource Usage | Low | High | Minimal |
| Reliability | High | Medium | High |

## Recommended Setup

**For Fixed-Wing Development:**
- Use **jMAVSim** for daily development (sensors + airframe, no GUI issues)
- Use **Gazebo** when you need 3D visualization or terrain testing

**For Quick Testing:**
- Use **None** simulator for MAVLink connectivity checks only

## Airframe Configuration

All modes support the advanced plane airframe via `SYS_AUTOSTART=4008`:
- Fixed-wing aircraft
- Full sensor suite
- EKF2 estimator
- Mission support
