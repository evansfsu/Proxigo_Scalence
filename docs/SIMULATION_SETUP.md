# Proxigo UAV Simulation Setup Guide

Cross-platform simulation environment for fixed-wing and VTOL development.

## Supported Platforms

| Platform | Gazebo GUI | QGC Connection | Notes |
|----------|------------|----------------|-------|
| Windows + WSL2 | ✅ | ✅ | Recommended for Windows |
| Linux Native | ✅ | ✅ | Best performance |
| Nvidia Orin Nano | ✅ | ✅ | Production target |

## Quick Start

### Windows (First Time Setup)

```powershell
# 1. Run WSL2 setup (one-time)
.\scripts\setup_wsl.ps1

# 2. Install VcXsrv from https://sourceforge.net/projects/vcxsrv/
#    Start XLaunch with "Disable access control" checked

# 3. Start simulation
.\scripts\Start-Simulation.ps1
```

### Linux / Orin Nano

```bash
# Make script executable
chmod +x scripts/start_simulation.sh

# Start simulation
./scripts/start_simulation.sh
```

## Usage

### Start Simulation

```powershell
# Windows
.\scripts\Start-Simulation.ps1

# Linux
./scripts/start_simulation.sh
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `--vehicle TYPE` | Aircraft type | `advanced_plane` |
| `--location NAME` | Location preset | `death_valley` |
| `--lat LAT` | Custom latitude | - |
| `--lon LON` | Custom longitude | - |
| `--headless` | No GUI | false |
| `--restart` | Restart simulation | - |
| `--stop` | Stop simulation | - |

### Vehicle Types

- `advanced_plane` - Fixed-wing aircraft (default)
- `standard_vtol` - Vertical takeoff and landing
- `x500` - Quadcopter (for testing)

### Location Presets

| Name | Coordinates | Description |
|------|-------------|-------------|
| `death_valley` | 36.2329°N, 116.8276°W | Desert terrain, low altitude |
| `zurich` | 47.3977°N, 8.5456°E | PX4 default, urban |
| `austin` | 30.2672°N, 97.7431°W | Mixed terrain |

### Examples

```powershell
# Start VTOL at Death Valley
.\scripts\Start-Simulation.ps1 -Vehicle standard_vtol

# Start at custom location
.\scripts\Start-Simulation.ps1 -Lat 34.0 -Lon -118.0

# Restart simulation
.\scripts\Start-Simulation.ps1 -Restart

# Stop simulation
.\scripts\Start-Simulation.ps1 -Stop
```

## QGroundControl Connection

QGC should auto-connect. If not:

1. Open QGroundControl
2. Go to **Application Settings → Comm Links**
3. Add new connection:
   - **Type:** UDP
   - **Port:** 14550
4. Or use TCP:
   - **Type:** TCP
   - **Host:** 127.0.0.1
   - **Port:** 5760

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Windows Host                                               │
│  ┌─────────────────┐    ┌─────────────────────────────────┐ │
│  │ QGroundControl  │◄───│  VcXsrv (X11)                   │ │
│  │ UDP:14550       │    │  Display :0                     │ │
│  └────────▲────────┘    └───────────────▲─────────────────┘ │
│           │                             │                   │
├───────────│─────────────────────────────│───────────────────┤
│  WSL2     │                             │                   │
│  ┌────────┴────────────────────────────┴──────────────────┐ │
│  │  Docker                                                │ │
│  │  ┌──────────────────┐    ┌───────────────────────────┐ │ │
│  │  │  PX4 SITL        │◄───│  Gazebo Harmonic          │ │ │
│  │  │  MAVLink         │    │  Physics + Rendering      │ │ │
│  │  │  UDP:18570       │    │  X11 forwarding           │ │ │
│  │  └──────────────────┘    └───────────────────────────┘ │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## Troubleshooting

### "Gazebo bridge error"

**Cause:** X11 display not accessible

**Fix:**
1. Ensure VcXsrv is running with "Disable access control"
2. Check Windows Firewall allows VcXsrv
3. Try: `.\scripts\Start-Simulation.ps1 -Headless`

### "QGC not connecting"

**Fix:**
1. Wait 2-3 minutes for PX4 to fully initialize
2. Try TCP connection: 127.0.0.1:5760
3. Check: `docker logs px4_sitl`

### "First waypoint too far"

**Cause:** Mission location doesn't match simulation home

**Fix:**
1. Use mission file that matches simulation location
2. Or set home position in QGC before uploading mission

### WSL2 Docker Issues

```bash
# In WSL terminal:
# Restart Docker daemon
sudo service docker restart

# Check Docker status
docker info
```

## Deployment to Orin Nano

The same Docker images work on Orin:

```bash
# On Orin Nano
git clone <repo> && cd Proxigo_Scalence
./scripts/start_simulation.sh --vehicle advanced_plane
```

For production, remove simulation and use hardware:
```bash
./scripts/start_hardware.sh  # (separate script for real flight)
```

## File Structure

```
scripts/
├── Start-Simulation.ps1    # Windows launcher
├── start_simulation.sh     # Linux/WSL launcher
├── setup_wsl.ps1           # One-time Windows setup
└── upload_death_valley_mission.py

docker/simulation/
└── docker-compose.simulation.yml

config/mission/
├── death_valley_survey.plan
└── survey_mission.plan
```
