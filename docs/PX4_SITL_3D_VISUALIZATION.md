# PX4 SITL with 3D Visualization on Windows

This guide explains how to run PX4 SITL with Gazebo Harmonic 3D visualization on Windows.

## ✅ Working Configuration

The project includes a pre-built Docker image with:
- **PX4 v1.15.0** with Gazebo Harmonic (new Gazebo)
- **gz_advanced_plane** - Fixed-wing aircraft model
- **Death Valley Salt Flats** as home location (36.2291, -116.8325)

---

## Quick Start

### Step 1: Install & Configure VcXsrv

1. Download from: https://sourceforge.net/projects/vcxsrv/
2. Install with default options
3. Launch "XLaunch" from Start Menu
4. Configure:
   - **Multiple windows** ✓
   - **Display number: 0**
   - **Start no client**
   - **☑ Disable access control** (CRITICAL!)
5. Click Finish

### Step 2: Allow Through Firewall

If prompted, allow VcXsrv through Windows Firewall for both Private and Public networks.

### Step 3: Start Fixed-Wing Simulation

```powershell
# Start PX4 with Gazebo Harmonic (server mode)
docker run -d --name px4_gazebo_plane --network host `
  -e DISPLAY=host.docker.internal:0 `
  -e PX4_HOME_LAT=36.2291 `
  -e PX4_HOME_LON=-116.8325 `
  -e PX4_HOME_ALT=50 `
  -e LIBGL_ALWAYS_SOFTWARE=1 `
  proxigo/px4-gazebo:harmonic `
  bash -c "cd /root/PX4-Autopilot && make px4_sitl gz_advanced_plane"

# Wait 2-3 minutes for build on first run, then start GUI
docker run -d --name gazebo_gui --network host `
  -e DISPLAY=host.docker.internal:0 `
  -e LIBGL_ALWAYS_SOFTWARE=1 `
  proxigo/px4-gazebo:harmonic gz sim -g
```

### Using Docker Compose

```powershell
# Start with 3D visualization
docker compose -f docker-compose.sitl.yml --profile gui up

# Start headless (no GUI)
docker compose -f docker-compose.sitl.yml up px4
```

---

## Available Aircraft Models

### Fixed Wing (Gazebo Harmonic)
- `gz_advanced_plane` - Realistic fixed-wing with full aerodynamics ✅
- `gz_rc_cessna` - RC-style Cessna model
- `gz_standard_plane` - Basic fixed-wing

### VTOL
- `gz_standard_vtol` - QuadPlane (quad + fixed wing)

### Multirotor  
- `gz_x500` - Standard X500 quadcopter

---

## Connecting to QGroundControl

QGroundControl automatically connects to the simulation:

1. Download: https://qgroundcontrol.com/
2. Run QGroundControl
3. It auto-connects to `localhost:14550`
4. You'll see the fixed-wing aircraft telemetry

---

## Connecting Navigation Stack

After PX4 SITL is running:

```powershell
# Start the full navigation stack
docker compose -f docker-compose.sitl.yml up mavros vio_sim satellite fusion
```

The vision pose is published to `/mavros/vision_pose/pose` for PX4's EKF2.

---

## Stopping the Simulation

```powershell
# Stop all containers
docker rm -f px4_gazebo_plane gazebo_gui

# Or with compose
docker compose -f docker-compose.sitl.yml down
```

---

## Troubleshooting

### GUI doesn't appear
1. Verify VcXsrv is running (check system tray)
2. Ensure "Disable access control" is checked
3. Check Windows Firewall allows VcXsrv
4. Restart VcXsrv and try again

### "Could not connect to display"
```powershell
# Test X11 connection
docker run --rm -e DISPLAY=host.docker.internal:0 proxigo/px4-gazebo:harmonic xeyes
```

### Slow performance
The simulation uses software rendering (`LIBGL_ALWAYS_SOFTWARE=1`). 
This is required for Windows/Docker but may be slow on older hardware.

---

## Building the Image (Development)

```powershell
# Rebuild the Gazebo Harmonic image
docker build --no-cache -t proxigo/px4-gazebo:harmonic -f docker/px4_sitl/Dockerfile.gazebo .
```
