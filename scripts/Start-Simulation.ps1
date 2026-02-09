<#
.SYNOPSIS
    Proxigo UAV Simulation Launcher for Windows
    
.DESCRIPTION
    Cross-platform simulation launcher that uses WSL2 for Gazebo support.
    Provides single-command startup for QGC + PX4 SITL + Gazebo.
    
.PARAMETER Vehicle
    Vehicle type: advanced_plane (default), standard_vtol, x500
    
.PARAMETER Location
    Location preset: death_valley (default), zurich, austin
    
.PARAMETER Lat
    Custom latitude (overrides Location)
    
.PARAMETER Lon
    Custom longitude
    
.PARAMETER Stop
    Stop the simulation
    
.PARAMETER Restart
    Restart the simulation
    
.PARAMETER Status
    Show simulation status
    
.PARAMETER SetupWSL
    First-time WSL2 setup
    
.EXAMPLE
    .\Start-Simulation.ps1
    
.EXAMPLE
    .\Start-Simulation.ps1 -Vehicle standard_vtol
    
.EXAMPLE
    .\Start-Simulation.ps1 -Restart
#>

param(
    [ValidateSet("advanced_plane", "standard_vtol", "x500")]
    [string]$Vehicle = "advanced_plane",
    
    [ValidateSet("death_valley", "zurich", "austin", "custom")]
    [string]$Location = "death_valley",
    
    [double]$Lat,
    [double]$Lon,
    [double]$Alt = 0,
    
    [switch]$Stop,
    [switch]$Restart,
    [switch]$Status,
    [switch]$SetupWSL,
    [switch]$Headless
)

$ErrorActionPreference = "Continue"

# ============================================================================
# Configuration
# ============================================================================

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectDir = Split-Path -Parent $ScriptDir

# Location presets
$Locations = @{
    "death_valley" = @{ Lat = 36.2329; Lon = -116.8276; Alt = 0 }
    "zurich"       = @{ Lat = 47.3977; Lon = 8.5456; Alt = 488 }
    "austin"       = @{ Lat = 30.2672; Lon = -97.7431; Alt = 150 }
}

# ============================================================================
# Functions
# ============================================================================

function Write-Banner {
    Write-Host ""
    Write-Host "╔══════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
    Write-Host "║           PROXIGO UAV SIMULATION                             ║" -ForegroundColor Cyan
    Write-Host "║           Fixed-Wing & VTOL Development Platform             ║" -ForegroundColor Cyan
    Write-Host "╚══════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
    Write-Host ""
}

function Test-WSL {
    try {
        $wslOutput = wsl --list --quiet 2>&1
        return $LASTEXITCODE -eq 0
    } catch {
        return $false
    }
}

function Test-Docker {
    try {
        $dockerOutput = docker info 2>&1
        return $LASTEXITCODE -eq 0
    } catch {
        return $false
    }
}

function Test-XServer {
    $vcxsrv = Get-Process -Name vcxsrv -ErrorAction SilentlyContinue
    return $null -ne $vcxsrv
}

function Install-WSLSetup {
    Write-Host "[SETUP] Configuring WSL2 for Proxigo simulation..." -ForegroundColor Cyan
    
    # Check if WSL is installed
    if (-not (Test-WSL)) {
        Write-Host "[ERROR] WSL2 not installed. Run: wsl --install" -ForegroundColor Red
        Write-Host "        Then restart and run this script again with -SetupWSL" -ForegroundColor Yellow
        return $false
    }
    
    # Get WSL distro
    $defaultDistro = (wsl --list --quiet | Where-Object { $_ -and $_.Trim() } | Select-Object -First 1).Trim()
    Write-Host "[INFO] Using WSL distro: $defaultDistro" -ForegroundColor Gray
    
    # Install Docker in WSL if not present
    Write-Host "[INFO] Setting up Docker in WSL..." -ForegroundColor Gray
    $bashScript = @'
if ! command -v docker &> /dev/null; then
    echo 'Installing Docker...'
    curl -fsSL https://get.docker.com | sh
    sudo usermod -aG docker $USER
fi

# Start Docker daemon if not running
if ! pgrep -x dockerd > /dev/null; then
    sudo dockerd &
    sleep 3
fi

# Install pymavlink
pip3 install pymavlink --quiet 2>/dev/null || (sudo apt-get install -y python3-pip && pip3 install pymavlink)

echo 'WSL setup complete!'
'@
    wsl -d $defaultDistro bash -c $bashScript
    
    Write-Host "[OK] WSL2 setup complete!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Next steps:" -ForegroundColor Yellow
    Write-Host "  1. Start VcXsrv (XLaunch) with 'Disable access control' checked" -ForegroundColor White
    Write-Host "  2. Run: .\Start-Simulation.ps1" -ForegroundColor White
    
    return $true
}

function Start-XServer {
    if (-not (Test-XServer)) {
        Write-Host "[WARN] VcXsrv not running!" -ForegroundColor Yellow
        Write-Host "       Please start XLaunch with:" -ForegroundColor Yellow
        Write-Host "         - Multiple windows" -ForegroundColor White
        Write-Host "         - Display number: 0" -ForegroundColor White
        Write-Host "         - Start no client" -ForegroundColor White
        Write-Host "         - Disable access control (CHECKED)" -ForegroundColor White
        Write-Host ""
        
        # Try to find and start VcXsrv
        $vcxsrvPath = @(
            "$env:ProgramFiles\VcXsrv\vcxsrv.exe",
            "${env:ProgramFiles(x86)}\VcXsrv\vcxsrv.exe"
        ) | Where-Object { Test-Path $_ } | Select-Object -First 1
        
        if ($vcxsrvPath) {
            Write-Host "[INFO] Found VcXsrv at: $vcxsrvPath" -ForegroundColor Gray
            $response = Read-Host "Start VcXsrv automatically? (y/n)"
            if ($response -eq "y") {
                Start-Process $vcxsrvPath -ArgumentList ":0 -multiwindow -clipboard -ac"
                Start-Sleep -Seconds 2
            }
        }
        
        if (-not (Test-XServer)) {
            if (-not $Headless) {
                Write-Host "[WARN] Continuing without X server - Gazebo GUI may not work" -ForegroundColor Yellow
            }
        }
    } else {
        Write-Host "[OK] VcXsrv is running" -ForegroundColor Green
    }
}

function Get-WindowsHostIP {
    # Get the IP address that WSL can use to reach Windows
    $adapter = Get-NetAdapter | Where-Object { $_.Name -like "*WSL*" -or $_.Name -like "*vEthernet*" } | Select-Object -First 1
    if ($adapter) {
        $ip = (Get-NetIPAddress -InterfaceIndex $adapter.InterfaceIndex -AddressFamily IPv4).IPAddress
        return $ip
    }
    
    # Fallback
    return "host.docker.internal"
}

function Invoke-WSLSimulation {
    param(
        [string]$Action,
        [string]$Vehicle,
        [double]$Lat,
        [double]$Lon,
        [double]$Alt,
        [bool]$Headless
    )
    
    $displayHost = Get-WindowsHostIP
    $headlessFlag = if ($Headless) { "--headless" } else { "" }
    
    # Build command
    $projectPathWSL = "/mnt/c/Users/$env:USERNAME/OneDrive/Documents/GitHub/Proxigo_Scalence"
    $wslCmd = "export DISPLAY=$displayHost`:0; export LIBGL_ALWAYS_SOFTWARE=1; export PX4_HOME_LAT=$Lat; export PX4_HOME_LON=$Lon; export PX4_HOME_ALT=$Alt; export PX4_VEHICLE=$Vehicle; cd $projectPathWSL; chmod +x scripts/start_simulation.sh; ./scripts/start_simulation.sh $Action $headlessFlag --vehicle $Vehicle --lat $Lat --lon $Lon --alt $Alt"
    
    Write-Host "[INFO] Running simulation in WSL2..." -ForegroundColor Gray
    wsl bash -c $wslCmd
}

function Invoke-DirectDocker {
    param(
        [string]$Action,
        [string]$Vehicle,
        [double]$Lat,
        [double]$Lon,
        [double]$Alt
    )
    
    # For Windows Docker Desktop (fallback, less reliable for Gazebo)
    $env:PX4_HOME_LAT = $Lat
    $env:PX4_HOME_LON = $Lon
    $env:PX4_HOME_ALT = $Alt
    $env:PX4_VEHICLE = $Vehicle
    $env:DISPLAY = "host.docker.internal:0"
    $env:LIBGL_ALWAYS_SOFTWARE = "1"
    
    $composeFile = Join-Path $ProjectDir "docker/simulation/docker-compose.simulation.yml"
    
    switch ($Action) {
        "stop" {
            docker compose -f $composeFile down --remove-orphans
            docker rm -f px4_sitl mavlink_router 2>$null
        }
        "start" {
            docker compose -f $composeFile up -d
        }
    }
}

function Show-Status {
    param(
        [double]$Lat,
        [double]$Lon,
        [double]$Alt
    )
    
    Write-Host ""
    Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
    Write-Host "  SIMULATION STATUS" -ForegroundColor Cyan
    Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
    Write-Host ""
    
    Write-Host "Location:" -ForegroundColor White
    Write-Host "  Latitude:  $Lat N" -ForegroundColor Cyan
    Write-Host "  Longitude: $Lon W" -ForegroundColor Cyan
    Write-Host "  Altitude:  $Alt m" -ForegroundColor Cyan
    Write-Host ""
    
    Write-Host "QGroundControl:" -ForegroundColor White
    Write-Host "  1. Open QGroundControl" -ForegroundColor Yellow
    Write-Host "  2. It should auto-connect on UDP 14550" -ForegroundColor Yellow
    Write-Host "  3. Or add TCP connection: 127.0.0.1:5760" -ForegroundColor Yellow
    Write-Host ""
    
    Write-Host "Commands:" -ForegroundColor White
    Write-Host "  Restart:  .\Start-Simulation.ps1 -Restart" -ForegroundColor Gray
    Write-Host "  Stop:     .\Start-Simulation.ps1 -Stop" -ForegroundColor Gray
    Write-Host "  Status:   .\Start-Simulation.ps1 -Status" -ForegroundColor Gray
    Write-Host ""
}

# ============================================================================
# Main
# ============================================================================

Write-Banner

# First-time setup
if ($SetupWSL) {
    Install-WSLSetup
    exit
}

# Resolve location
if ($Lat -and $Lon) {
    $simLat = $Lat
    $simLon = $Lon
    $simAlt = $Alt
} elseif ($Locations.ContainsKey($Location)) {
    $loc = $Locations[$Location]
    $simLat = $loc.Lat
    $simLon = $loc.Lon
    $simAlt = $loc.Alt
} else {
    $simLat = 36.2329
    $simLon = -116.8276
    $simAlt = 0
}

Write-Host "[INFO] Vehicle: $Vehicle" -ForegroundColor Gray
$altStr = "$simAlt" + "m"
Write-Host "[INFO] Location: $Location ($simLat, $simLon, $altStr)" -ForegroundColor Gray
Write-Host ""

# Determine action
$action = "start"
if ($Stop) { $action = "stop" }
elseif ($Restart) { $action = "restart" }
elseif ($Status) { $action = "status" }

# Check environment
if ($action -ne "status") {
    if (-not $Headless) {
        Start-XServer
    }
}

# Execute based on available backend
if (Test-WSL) {
    Write-Host "[INFO] Using WSL2 backend (recommended)" -ForegroundColor Gray
    
    # Convert action to script argument
    $scriptAction = switch ($action) {
        "stop"    { "--stop" }
        "restart" { "--restart" }
        "status"  { "--status" }
        default   { "" }
    }
    
    Invoke-WSLSimulation -Action $scriptAction -Vehicle $Vehicle -Lat $simLat -Lon $simLon -Alt $simAlt -Headless $Headless.IsPresent
} elseif (Test-Docker) {
    Write-Host "[INFO] Using Docker Desktop backend (Gazebo may have issues)" -ForegroundColor Yellow
    
    switch ($action) {
        "stop" {
            Invoke-DirectDocker -Action "stop" -Vehicle $Vehicle -Lat $simLat -Lon $simLon -Alt $simAlt
            Write-Host "[OK] Simulation stopped" -ForegroundColor Green
        }
        "restart" {
            Invoke-DirectDocker -Action "stop" -Vehicle $Vehicle -Lat $simLat -Lon $simLon -Alt $simAlt
            Start-Sleep -Seconds 2
            Invoke-DirectDocker -Action "start" -Vehicle $Vehicle -Lat $simLat -Lon $simLon -Alt $simAlt
        }
        default {
            Invoke-DirectDocker -Action "start" -Vehicle $Vehicle -Lat $simLat -Lon $simLon -Alt $simAlt
        }
    }
}

if ($action -ne "stop") {
    Show-Status -Lat $simLat -Lon $simLon -Alt $simAlt
}
