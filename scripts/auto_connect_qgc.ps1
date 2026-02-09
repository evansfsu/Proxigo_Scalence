# Auto-Connect QGroundControl Script
# This script automatically configures QGC to connect to PX4 SITL

Write-Host "================================================" -ForegroundColor Cyan
Write-Host "  QGroundControl Auto-Connect Setup" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""

# Check if QGC is installed
$qgcPath = @(
    "$env:LOCALAPPDATA\Programs\qgroundcontrol\QGroundControl.exe",
    "$env:ProgramFiles\QGroundControl\QGroundControl.exe",
    "${env:ProgramFiles(x86)}\QGroundControl\QGroundControl.exe"
) | Where-Object { Test-Path $_ } | Select-Object -First 1

if (-not $qgcPath) {
    Write-Host "[ERROR] QGroundControl not found!" -ForegroundColor Red
    Write-Host "Please install QGC from: https://qgroundcontrol.com/" -ForegroundColor Yellow
    exit 1
}

Write-Host "[OK] Found QGroundControl at: $qgcPath" -ForegroundColor Green
Write-Host ""

# Check if PX4 is running
$px4Running = docker ps --filter "name=px4_sitl" --format "{{.Names}}" | Select-String "px4_sitl"
if (-not $px4Running) {
    Write-Host "[WARN] PX4 SITL container not running!" -ForegroundColor Yellow
    Write-Host "Starting simulation first..." -ForegroundColor Yellow
    wsl bash -c "cd /mnt/c/Users/$env:USERNAME/OneDrive/Documents/GitHub/Proxigo_Scalence && ./scripts/start_simulation.sh"
    Start-Sleep -Seconds 10
}

# Check if ports are listening
Write-Host "[INFO] Checking connection ports..." -ForegroundColor Gray
$udp14550 = netstat -an | Select-String "14550.*UDP"
$tcp5760 = netstat -an | Select-String "5760.*LISTEN"

if ($udp14550) {
    Write-Host "  ✓ UDP 14550 is listening" -ForegroundColor Green
} else {
    Write-Host "  ✗ UDP 14550 not listening" -ForegroundColor Red
}

if ($tcp5760) {
    Write-Host "  ✓ TCP 5760 is listening" -ForegroundColor Green
} else {
    Write-Host "  ✗ TCP 5760 not listening" -ForegroundColor Red
}

Write-Host ""
Write-Host "================================================" -ForegroundColor Cyan
Write-Host "  QGroundControl Connection Instructions" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "OPTION 1 - UDP Auto-Connect (Recommended):" -ForegroundColor Yellow
Write-Host "  1. Open QGroundControl" -ForegroundColor White
Write-Host "  2. QGC should auto-detect UDP on port 14550" -ForegroundColor White
Write-Host "  3. If not, go to: Application Settings → Comm Links" -ForegroundColor White
Write-Host "  4. Add new connection:" -ForegroundColor White
Write-Host "     - Type: UDP" -ForegroundColor Cyan
Write-Host "     - Listening Port: 14550" -ForegroundColor Cyan
Write-Host "     - Auto Connect: ✓" -ForegroundColor Cyan
Write-Host ""
Write-Host "OPTION 2 - TCP Manual Connection:" -ForegroundColor Yellow
Write-Host "  1. Application Settings → Comm Links → Add" -ForegroundColor White
Write-Host "  2. Type: TCP" -ForegroundColor Cyan
Write-Host "  3. Server Address: 127.0.0.1" -ForegroundColor Cyan
Write-Host "  4. Server Port: 5760" -ForegroundColor Cyan
Write-Host "  5. Auto Connect: ✓" -ForegroundColor Cyan
Write-Host ""
# Configure QGC auto-connect settings
Write-Host "[INFO] Configuring QGC auto-connect..." -ForegroundColor Gray
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$configScript = Join-Path $scriptDir "configure_qgc_autoconnect.ps1"

if (Test-Path $configScript) {
    # Configure UDP connection (primary)
    & $configScript -ConnectionType UDP -Port 14550
    Write-Host ""
    
    # Also configure TCP connection (backup)
    & $configScript -ConnectionType TCP -Port 5760 -ServerHost "127.0.0.1"
    Write-Host ""
} else {
    Write-Host "[WARN] Configuration script not found. Using manual setup." -ForegroundColor Yellow
}

Write-Host "Starting QGroundControl..." -ForegroundColor Yellow
Start-Process $qgcPath

Write-Host ""
Write-Host "[OK] QGroundControl launched with auto-connect configured!" -ForegroundColor Green
Write-Host "     QGC should automatically connect to PX4 SITL." -ForegroundColor Green
Write-Host ""
Write-Host "If connection fails:" -ForegroundColor Yellow
Write-Host "  1. Check PX4 is running: docker ps | grep px4_sitl" -ForegroundColor White
Write-Host "  2. Check MAVProxy: docker exec px4_sitl pgrep -af mavproxy" -ForegroundColor White
Write-Host "  3. Test connection: docker exec px4_sitl python3 /scripts/test_qgc_connection.py" -ForegroundColor White
Write-Host "  4. Re-run config: .\scripts\configure_qgc_autoconnect.ps1" -ForegroundColor White
Write-Host ""
