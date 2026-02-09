# Quick Start Guide

## Running Simulations from Windows

Since `.sh` files need to run in WSL/Linux, use these PowerShell commands:

### Option 1: Use PowerShell Wrapper (Easiest)

```powershell
# Run the working simulation (none simulator)
.\scripts\start_flightgear.ps1

# Or specify a simulator
.\scripts\start_flightgear.ps1 -Simulator simple
.\scripts\start_flightgear.ps1 -Simulator gazebo-headless
```

### Option 2: Run via WSL Directly

Open WSL terminal and run:
```bash
cd /mnt/c/Users/$USER/OneDrive/Documents/GitHub/Proxigo_Scalence
./scripts/start_simulation_simple.sh
```

## Available Simulators

| Simulator | Command | Status | Sensors |
|-----------|---------|--------|---------|
| **None** (Working) | `.\scripts\start_flightgear.ps1` | ✅ Working | ❌ No |
| Gazebo Headless | `.\scripts\start_flightgear.ps1 -Simulator gazebo-headless` | ⚠️ Bridge timeout | ✅ Yes (if works) |
| FlightGear | N/A | ❌ Not available | N/A |

## Current Working Solution

The **"none" simulator** is currently working and provides:
- ✅ MAVLink connectivity
- ✅ QGC connection
- ✅ MAVProxy forwarding
- ❌ No sensors (calibration warnings in QGC)

## Next Steps

1. **For MAVLink testing:** Use the working "none" simulator
2. **For sensors:** Need to fix Gazebo or find alternative
3. **For development:** Current setup works for connectivity testing

## Troubleshooting

**"Choose an app to open .sh file"**
- Don't double-click .sh files in Windows
- Use the PowerShell wrapper: `.\scripts\start_flightgear.ps1`
- Or run via WSL: `wsl -d Ubuntu bash -c "./scripts/start_simulation_simple.sh"`

**QGC Not Connecting**
- Check container: `wsl -d Ubuntu bash -c "docker ps | grep px4"`
- Check MAVProxy: `wsl -d Ubuntu bash -c "docker exec px4_gazebo_plane pgrep -f mavproxy"`
- Test connection: `wsl -d Ubuntu bash -c "docker exec px4_gazebo_plane python3 /scripts/test_qgc_connection.py"`
