# PowerShell wrapper to run simulation scripts via WSL
# Usage: powershell -ExecutionPolicy Bypass -File .\scripts\start_flightgear.ps1 [simulator]
#   Or: .\scripts\start_flightgear.ps1 [simulator] (if execution policy allows)
#   simulator: flightgear, gazebo-headless, simple (default: simple)

param(
    [string]$Simulator = "simple"
)

Write-Host "================================================" -ForegroundColor Cyan
Write-Host "  Starting PX4 Simulation via WSL" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""

$projectPath = "C:\Users\$env:USERNAME\OneDrive\Documents\GitHub\Proxigo_Scalence"
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
    "simple" {
        $script = "start_simulation_simple.sh"
    }
    default {
        $script = "start_simulation_simple.sh"
    }
}

Write-Host "Running simulation script: $script" -ForegroundColor Yellow
Write-Host ""

wsl -d Ubuntu bash -c "cd $wslPath && chmod +x scripts/$script && ./scripts/$script"

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "Simulation started!" -ForegroundColor Green
    Write-Host ""
    Write-Host "To check status:" -ForegroundColor Yellow
    Write-Host "  wsl -d Ubuntu bash -c 'docker ps | grep px4'" -ForegroundColor White
} else {
    Write-Host ""
    Write-Host "Error starting simulation. Check logs above." -ForegroundColor Red
}
