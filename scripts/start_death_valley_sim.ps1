# Death Valley UAV Simulation Startup Script
# This script starts PX4 SITL with Gazebo at Death Valley coordinates
#
# Prerequisites:
#   - Docker Desktop running
#   - VcXsrv (XLaunch) running with "Disable access control" checked
#
# Usage:
#   .\scripts\start_death_valley_sim.ps1
#   .\scripts\start_death_valley_sim.ps1 -Headless  # No GUI

param(
    [switch]$Headless
)

$ErrorActionPreference = "Continue"

Write-Host ""
Write-Host "================================================" -ForegroundColor Cyan
Write-Host "  DEATH VALLEY UAV SIMULATION" -ForegroundColor Cyan
Write-Host "  Location: Badwater Basin, Death Valley" -ForegroundColor White
Write-Host "  Coordinates: 36.2329 N, -116.8276 W" -ForegroundColor White
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""

# Configuration
$IMAGE = "proxigo/px4-gazebo:harmonic"
$HOME_LAT = "36.2329"
$HOME_LON = "-116.8276"
$HOME_ALT = "0"
$CONTAINER_NAME = "px4_sim"

# Stop any existing containers
Write-Host "[1/5] Stopping existing containers..." -ForegroundColor Yellow
docker rm -f $CONTAINER_NAME 2>$null | Out-Null
docker rm -f gazebo_gui 2>$null | Out-Null

# Check if VcXsrv is running (for GUI)
if (-not $Headless) {
    $vcxsrv = Get-Process -Name vcxsrv -ErrorAction SilentlyContinue
    if (-not $vcxsrv) {
        Write-Host ""
        Write-Host "WARNING: VcXsrv not detected!" -ForegroundColor Red
        Write-Host "For Gazebo GUI, start XLaunch with:" -ForegroundColor Yellow
        Write-Host "  - Multiple windows, Display 0" -ForegroundColor White
        Write-Host "  - Disable access control (checked)" -ForegroundColor White
        Write-Host ""
        Write-Host "Continuing in headless mode..." -ForegroundColor Yellow
        $Headless = $true
    }
}

Write-Host ""
Write-Host "[2/5] Starting PX4 SITL container..." -ForegroundColor Yellow

# Build docker arguments array
$dockerArgs = @(
    "run", "-d",
    "--name", $CONTAINER_NAME,
    "-p", "14550:14550/udp",
    "-p", "14540:14540/udp",
    "-p", "18570:18570/udp",
    "-p", "5760:5760/tcp",
    "-e", "DISPLAY=host.docker.internal:0",
    "-e", "PX4_HOME_LAT=$HOME_LAT",
    "-e", "PX4_HOME_LON=$HOME_LON",
    "-e", "PX4_HOME_ALT=$HOME_ALT",
    "-e", "HEADLESS=1",
    "-e", "LIBGL_ALWAYS_SOFTWARE=1",
    "-e", "GZ_SIM_RESOURCE_PATH=/root/PX4-Autopilot/Tools/simulation/gz/models:/root/PX4-Autopilot/Tools/simulation/gz/worlds",
    $IMAGE,
    "bash", "-c", "cd /root/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_advanced_plane"
)

# Execute docker command
$result = & docker @dockerArgs 2>&1
if ($LASTEXITCODE -eq 0) {
    $shortId = if ($result.Length -gt 12) { $result.Substring(0, 12) } else { $result }
    Write-Host "  Container started: $shortId..." -ForegroundColor Gray
} else {
    Write-Host "  Docker error: $result" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "[3/5] Waiting for PX4 to initialize..." -ForegroundColor Yellow
Write-Host "  (First run builds firmware - takes 2-3 minutes)" -ForegroundColor Gray

$timeout = 300  # 5 minutes
$elapsed = 0
$ready = $false
$lastProgress = ""

while ($elapsed -lt $timeout -and -not $ready) {
    Start-Sleep -Seconds 10
    $elapsed += 10
    
    # Check if container is still running
    $running = docker ps --filter "name=$CONTAINER_NAME" --format "{{.Names}}" 2>$null
    if (-not $running) {
        Write-Host ""
        Write-Host "Container stopped unexpectedly!" -ForegroundColor Red
        Write-Host "Last logs:" -ForegroundColor Yellow
        docker logs $CONTAINER_NAME 2>&1 | Select-Object -Last 15
        exit 1
    }
    
    # Check logs for ready state
    $logs = docker logs $CONTAINER_NAME 2>&1 | Out-String
    if ($logs -match "pxh>") {
        $ready = $true
        Write-Host "  PX4 shell ready!" -ForegroundColor Green
    } elseif ($logs -match "\[(\d+)/(\d+)\]") {
        $matches = [regex]::Matches($logs, "\[(\d+)/(\d+)\]")
        if ($matches.Count -gt 0) {
            $lastMatch = $matches[$matches.Count - 1]
            $progress = "[$($lastMatch.Groups[1].Value)/$($lastMatch.Groups[2].Value)]"
            if ($progress -ne $lastProgress) {
                Write-Host "  Building $progress ($elapsed sec)" -ForegroundColor Gray
                $lastProgress = $progress
            }
        }
    } else {
        Write-Host "  Initializing... ($elapsed sec)" -ForegroundColor Gray
    }
}

if (-not $ready) {
    Write-Host ""
    Write-Host "Timeout - checking status..." -ForegroundColor Yellow
    $lastLogs = docker logs $CONTAINER_NAME 2>&1 | Out-String
    
    if ($lastLogs -match "pxh>") {
        Write-Host "  PX4 is running!" -ForegroundColor Green
    } elseif ($lastLogs -match "ERROR.*gz_bridge") {
        Write-Host ""
        Write-Host "Gazebo bridge failed - expected on Windows Docker." -ForegroundColor Yellow
        Write-Host "PX4 simulation works, but no 3D visualization." -ForegroundColor Gray
    }
}

Write-Host ""
Write-Host "[4/5] Setting up MAVLink forwarding..." -ForegroundColor Yellow

# Install and start MAVProxy for QGC connection
docker exec $CONTAINER_NAME pip3 install mavproxy pymavlink --quiet 2>$null
docker exec -d $CONTAINER_NAME bash -c "mavproxy.py --master=udp:127.0.0.1:14540 --out=udp:host.docker.internal:14550 --out=tcp:0.0.0.0:5760 2>/dev/null &" 2>$null

Start-Sleep -Seconds 3

Write-Host ""
Write-Host "[5/5] Setting EKF origin to Death Valley..." -ForegroundColor Yellow

# Set EKF origin via Python
python -c @"
import sys, time
try:
    from pymavlink import mavutil
    for port in ['udp:127.0.0.1:14550', 'tcp:127.0.0.1:5760']:
        try:
            mav = mavutil.mavlink_connection(port, timeout=5)
            mav.wait_heartbeat(timeout=10)
            lat, lon = int(36.2329 * 1e7), int(-116.8276 * 1e7)
            mav.mav.set_gps_global_origin_send(mav.target_system, lat, lon, 0)
            time.sleep(0.3)
            mav.mav.command_long_send(mav.target_system, mav.target_component, 179, 0, 0, 0, 0, 0, 36.2329, -116.8276, 0)
            print(f'EKF origin set via {port}')
            break
        except: pass
except Exception as e:
    print(f'Note: EKF origin not set - {e}')
"@ 2>$null

Write-Host ""
Write-Host "================================================" -ForegroundColor Green
Write-Host "  SIMULATION STARTED" -ForegroundColor Green
Write-Host "================================================" -ForegroundColor Green
Write-Host ""
Write-Host "Location: Death Valley (Badwater Basin)" -ForegroundColor Cyan
Write-Host "  Lat: $HOME_LAT N" -ForegroundColor White
Write-Host "  Lon: $HOME_LON W" -ForegroundColor White
Write-Host ""
Write-Host "QGroundControl Connection:" -ForegroundColor Cyan
Write-Host "  UDP: 127.0.0.1:14550 (auto-detect)" -ForegroundColor White
Write-Host "  TCP: 127.0.0.1:5760 (manual add if needed)" -ForegroundColor White
Write-Host ""
Write-Host "Upload Mission:" -ForegroundColor Cyan
Write-Host "  python scripts/upload_death_valley_mission.py" -ForegroundColor Yellow
Write-Host ""
Write-Host "View Logs:" -ForegroundColor Cyan
Write-Host "  docker logs -f $CONTAINER_NAME" -ForegroundColor Gray
Write-Host ""
Write-Host "Stop Simulation:" -ForegroundColor Cyan
Write-Host "  docker rm -f $CONTAINER_NAME" -ForegroundColor Gray
Write-Host ""

# Final status check
$containerStatus = docker ps --filter "name=$CONTAINER_NAME" --format "{{.Status}}"
if ($containerStatus) {
    Write-Host "Container Status: $containerStatus" -ForegroundColor Green
} else {
    Write-Host "Container Status: NOT RUNNING - check logs" -ForegroundColor Red
}
