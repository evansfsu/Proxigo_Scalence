# Quick Start - Run Simulation from Windows

## Easy Way (Recommended)

Run this command in PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\start_simulation.ps1
```

This will:
- ✅ Bypass PowerShell execution policy
- ✅ Start the simulation via WSL
- ✅ Set up MAVProxy for QGC connection
- ✅ Configure everything automatically

## Alternative: Direct WSL Command

If you prefer to run directly in WSL:

```bash
wsl -d Ubuntu
cd /mnt/c/Users/$USER/OneDrive/Documents/GitHub/Proxigo_Scalence
./scripts/start_simulation_simple.sh
```

## What Gets Started

- **Container:** `px4_gazebo_plane`
- **Simulator:** "none" (working, provides MAVLink connectivity)
- **MAVProxy:** Forwards to UDP 14550 and TCP 5760
- **QGC Connection:** Auto-detects on UDP 14550

## Connect QGroundControl

1. Open QGroundControl
2. It should auto-detect on UDP port 14550
3. Or manually add: TCP connection to `127.0.0.1:5760`

## Check Status

```powershell
# Check if container is running
wsl -d Ubuntu bash -c "docker ps | grep px4"

# Check MAVLink connection
wsl -d Ubuntu bash -c "docker exec px4_gazebo_plane python3 /scripts/test_qgc_connection.py"

# View logs
wsl -d Ubuntu bash -c "docker logs px4_gazebo_plane"
```

## Stop Simulation

```powershell
wsl -d Ubuntu bash -c "docker rm -f px4_gazebo_plane"
```

## Troubleshooting

**PowerShell Execution Policy Error:**
- Use: `powershell -ExecutionPolicy Bypass -File .\scripts\start_simulation.ps1`
- This bypasses the policy for this script only

**QGC Not Connecting:**
- Wait 30-60 seconds after starting
- Check container: `wsl -d Ubuntu bash -c "docker ps | grep px4"`
- Verify MAVProxy: `wsl -d Ubuntu bash -c "docker exec px4_gazebo_plane pgrep -f mavproxy"`
