# Monitor PX4 SITL Build Progress
# Usage: .\scripts\monitor_px4_build.ps1

Write-Host "=============================================" -ForegroundColor Cyan
Write-Host "PX4 SITL Build Monitor" -ForegroundColor Cyan
Write-Host "=============================================" -ForegroundColor Cyan
Write-Host ""

# Check if container is running
$status = docker inspect -f '{{.State.Status}}' px4_sitl_build 2>$null

if ($status -eq "running") {
    Write-Host "Status: BUILDING..." -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Latest output:" -ForegroundColor Green
    docker logs --tail 30 px4_sitl_build
    Write-Host ""
    Write-Host "=============================================" -ForegroundColor Cyan
    Write-Host "To follow live: docker logs -f px4_sitl_build" -ForegroundColor White
} elseif ($status -eq "exited") {
    $exitCode = docker inspect -f '{{.State.ExitCode}}' px4_sitl_build
    if ($exitCode -eq "0") {
        Write-Host "Status: BUILD COMPLETE!" -ForegroundColor Green
        Write-Host ""
        Write-Host "PX4 SITL is ready. To start:" -ForegroundColor White
        Write-Host "  docker start -ai px4_sitl_build" -ForegroundColor Yellow
    } else {
        Write-Host "Status: BUILD FAILED (exit code: $exitCode)" -ForegroundColor Red
        Write-Host ""
        Write-Host "Check logs:" -ForegroundColor White
        docker logs --tail 50 px4_sitl_build
    }
} else {
    Write-Host "Status: Container not found" -ForegroundColor Red
    Write-Host ""
    Write-Host "Start build with:" -ForegroundColor White
    Write-Host "  docker compose -f docker-compose.sitl.yml up px4" -ForegroundColor Yellow
}
