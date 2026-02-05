# PX4 SITL Simulation Startup Script for Windows
# This script provides a reliable way to start the simulation and connect QGroundControl

param(
    [switch]$Clean,
    [switch]$NoGui
)

Write-Host "================================================" -ForegroundColor Cyan
Write-Host "  PX4 Fixed-Wing Simulation Startup" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""

# Step 1: Stop any existing containers
Write-Host "[1/6] Stopping existing containers..." -ForegroundColor Yellow
docker rm -f px4_gazebo_plane gazebo_gui 2>$null | Out-Null
Start-Sleep -Seconds 2

# Step 2: Start PX4 SITL container
Write-Host "[2/6] Starting PX4 SITL (this takes ~2 minutes on first run)..." -ForegroundColor Yellow
docker run -d `
    --name px4_gazebo_plane `
    -p 5760:5760 `
    -p 14540:14540/udp `
    -p 14550:14550/udp `
    -p 18570:18570/udp `
    -e DISPLAY=host.docker.internal:0 `
    -e PX4_HOME_LAT=36.2291 `
    -e PX4_HOME_LON=-116.8325 `
    -e PX4_HOME_ALT=50 `
    -e LIBGL_ALWAYS_SOFTWARE=1 `
    -v "${PWD}/satellite_data:/satellite_data:ro" `
    proxigo/px4-gazebo:harmonic `
    bash -c "cd /root/PX4-Autopilot && make px4_sitl gz_advanced_plane"

if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Failed to start PX4 container" -ForegroundColor Red
    exit 1
}

# Step 3: Wait for PX4 to be ready
Write-Host "[3/6] Waiting for PX4 to initialize..." -ForegroundColor Yellow
$maxWait = 180  # 3 minutes max
$waited = 0
$ready = $false

while ($waited -lt $maxWait -and -not $ready) {
    Start-Sleep -Seconds 10
    $waited += 10
    $progress = [math]::Round(($waited / $maxWait) * 100)
    Write-Host "  Waiting... ($waited seconds, checking for PX4 shell)" -ForegroundColor Gray
    
    $logs = docker logs --tail 5 px4_gazebo_plane 2>&1
    if ($logs -match "pxh>") {
        $ready = $true
        Write-Host "  PX4 is ready!" -ForegroundColor Green
    }
}

if (-not $ready) {
    Write-Host "WARNING: PX4 may not be fully ready, continuing anyway..." -ForegroundColor Yellow
}

# Step 4: Install and start MAVProxy for QGC connection
Write-Host "[4/6] Setting up MAVLink forwarding for QGroundControl..." -ForegroundColor Yellow
docker exec px4_gazebo_plane pip3 install mavproxy pymavlink --quiet 2>$null

# Start MAVProxy to forward from PX4's offboard port to Windows host
docker exec -d px4_gazebo_plane bash -c "mavproxy.py --master=udp:127.0.0.1:14540 --out=udp:host.docker.internal:14550 --out=tcpin:0.0.0.0:5760 --daemon"
Start-Sleep -Seconds 5

# Verify MAVProxy is running
$mavproxyRunning = docker exec px4_gazebo_plane bash -c "ps aux | grep mavproxy | grep -v grep" 2>&1
if ($mavproxyRunning -match "mavproxy") {
    Write-Host "  MAVProxy forwarding active" -ForegroundColor Green
} else {
    Write-Host "  WARNING: MAVProxy may not be running" -ForegroundColor Yellow
}

# Step 5: Start Gazebo GUI (if not disabled)
if (-not $NoGui) {
    Write-Host "[5/6] Starting Gazebo 3D visualization..." -ForegroundColor Yellow
    docker run -d `
        --name gazebo_gui `
        --network container:px4_gazebo_plane `
        -e DISPLAY=host.docker.internal:0 `
        -e LIBGL_ALWAYS_SOFTWARE=1 `
        -e QT_X11_NO_MITSHM=1 `
        proxigo/px4-gazebo:harmonic `
        gz sim -g
    
    Write-Host "  Gazebo GUI started (check XLaunch window)" -ForegroundColor Green
} else {
    Write-Host "[5/6] Skipping Gazebo GUI (--NoGui specified)" -ForegroundColor Gray
}

# Step 6: Connection instructions
Write-Host "[6/6] Setup complete!" -ForegroundColor Green
Write-Host ""
Write-Host "================================================" -ForegroundColor Green
Write-Host "  QGroundControl Connection Options" -ForegroundColor Green
Write-Host "================================================" -ForegroundColor Green
Write-Host ""
Write-Host "OPTION 1 - UDP (Try First):" -ForegroundColor Yellow
Write-Host "  1. Open QGroundControl" -ForegroundColor White
Write-Host "  2. Q icon -> Application Settings -> Comm Links" -ForegroundColor White
Write-Host "  3. Delete all existing links" -ForegroundColor White
Write-Host "  4. Add new: Type=UDP, Port=14550" -ForegroundColor Cyan
Write-Host "  5. Connect" -ForegroundColor White
Write-Host ""
Write-Host "OPTION 2 - TCP (If UDP fails):" -ForegroundColor Yellow
Write-Host "  1. Add new: Type=TCP, Host=127.0.0.1, Port=5760" -ForegroundColor Cyan
Write-Host ""
Write-Host "Mission file location:" -ForegroundColor Yellow
Write-Host "  $PWD\config\mission\death_valley_survey.plan" -ForegroundColor Cyan
Write-Host ""
Write-Host "To stop simulation: docker rm -f px4_gazebo_plane gazebo_gui" -ForegroundColor Gray
Write-Host ""
