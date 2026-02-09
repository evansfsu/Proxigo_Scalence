# PowerShell wrapper to start PX4 simulation (bypasses execution policy)
# Usage: powershell -ExecutionPolicy Bypass -File .\scripts\start_simulation.ps1

param(
    [string]$Simulator = "simple"
)

$ErrorActionPreference = "Stop"

Write-Host "================================================" -ForegroundColor Cyan
Write-Host "  Starting PX4 Simulation via WSL" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""

$wslPath = "/mnt/c/Users/$env:USERNAME/OneDrive/Documents/GitHub/Proxigo_Scalence"

switch ($Simulator.ToLower()) {
    "flightgear" {
        Write-Host "Note: FlightGear not available in current PX4 image" -ForegroundColor Yellow
        Write-Host "Using 'simple' simulator instead (working solution)" -ForegroundColor Yellow
        $script = "start_simulation_simple.sh"
    }
    "gazebo-headless" {
        $script = "start_simulation_gazebo_headless.sh"
    }
    "with-sensors" {
        $script = "start_simulation_with_sensors.sh"
        Write-Host "Starting with sensor injection (for calibration)" -ForegroundColor Cyan
    }
    "simple" {
        $script = "start_simulation_simple.sh"
    }
    default {
        $script = "start_simulation_simple.sh"
    }
}

Write-Host "Running simulation script: $script" -ForegroundColor Yellow
Write-Host ""

try {
    wsl -d Ubuntu bash -c "cd $wslPath && chmod +x scripts/$script && ./scripts/$script"
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host ""
        Write-Host "Simulation started!" -ForegroundColor Green
        Write-Host ""
        Write-Host "To check status:" -ForegroundColor Yellow
        Write-Host "  wsl -d Ubuntu bash -c 'docker ps | grep px4'" -ForegroundColor White
        Write-Host ""
        Write-Host "QGroundControl should auto-connect on:" -ForegroundColor Yellow
        Write-Host "  UDP: 127.0.0.1:14550" -ForegroundColor White
        Write-Host "  TCP: 127.0.0.1:5760" -ForegroundColor White
    } else {
        Write-Host ""
        Write-Host "Error starting simulation. Check logs above." -ForegroundColor Red
        exit 1
    }
} catch {
    Write-Host ""
    Write-Host "Error: $_" -ForegroundColor Red
    exit 1
}
