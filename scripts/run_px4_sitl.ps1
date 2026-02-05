# Run PX4 SITL with Navigation Stack
# Usage: .\scripts\run_px4_sitl.ps1

Write-Host "=============================================" -ForegroundColor Cyan
Write-Host "Proxigo Scalence - PX4 SITL Full Stack" -ForegroundColor Cyan
Write-Host "=============================================" -ForegroundColor Cyan
Write-Host ""

# Check if PX4 container exists and is ready
$status = docker inspect -f '{{.State.Status}}' px4_sitl_build 2>$null

if (-not $status) {
    Write-Host "Error: PX4 SITL not built yet." -ForegroundColor Red
    Write-Host "Run the build first and wait for it to complete." -ForegroundColor Yellow
    exit 1
}

if ($status -eq "running") {
    Write-Host "PX4 is already running!" -ForegroundColor Green
} elseif ($status -eq "exited") {
    $exitCode = docker inspect -f '{{.State.ExitCode}}' px4_sitl_build
    if ($exitCode -eq "0") {
        Write-Host "Starting PX4 SITL..." -ForegroundColor Yellow
        Write-Host ""
        
        # Create a new container from the built image (save it first)
        Write-Host "Creating PX4 SITL image from build..." -ForegroundColor White
        docker commit px4_sitl_build proxigo/px4-sitl:ready
        
        # Remove old container
        docker rm px4_sitl_build
        
        # Start fresh
        Write-Host "Starting PX4 SITL..." -ForegroundColor White
        docker run -d --name px4_sitl_run `
            --network host `
            -e PX4_HOME_LAT=36.2291 `
            -e PX4_HOME_LON=-116.8325 `
            -e PX4_HOME_ALT=50.0 `
            proxigo/px4-sitl:ready `
            bash -c "cd /root/PX4-Autopilot && make px4_sitl_default none_iris"
        
        Write-Host ""
        Write-Host "PX4 SITL starting! Wait ~30 seconds for initialization." -ForegroundColor Green
        Write-Host ""
        Write-Host "View logs: docker logs -f px4_sitl_run" -ForegroundColor White
        Write-Host ""
        Write-Host "Next: Start navigation stack in another terminal:" -ForegroundColor Cyan
        Write-Host "  docker compose -f docker-compose.dev.yml run --rm dev_shell" -ForegroundColor Yellow
    } else {
        Write-Host "Error: PX4 build failed. Check logs." -ForegroundColor Red
        exit 1
    }
} else {
    Write-Host "Unknown state: $status" -ForegroundColor Red
}
