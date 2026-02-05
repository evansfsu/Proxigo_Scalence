# Proxigo Scalence - Windows 11 Development Setup
# Run this script in PowerShell as Administrator
#
# Usage: .\scripts\setup_windows.ps1

Write-Host "=============================================="
Write-Host "Proxigo Scalence - Windows Development Setup"
Write-Host "=============================================="
Write-Host ""

# Check if running as administrator
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "WARNING: Some features may require Administrator privileges" -ForegroundColor Yellow
}

# Check WSL2
Write-Host "Checking WSL2..." -NoNewline
try {
    $wslVersion = wsl --version 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Host " OK" -ForegroundColor Green
        Write-Host "  WSL Version info:"
        wsl --version | ForEach-Object { Write-Host "    $_" }
    } else {
        Write-Host " NOT INSTALLED" -ForegroundColor Red
        Write-Host "  Install with: wsl --install" -ForegroundColor Yellow
    }
} catch {
    Write-Host " NOT INSTALLED" -ForegroundColor Red
    Write-Host "  Install with: wsl --install" -ForegroundColor Yellow
}

Write-Host ""

# Check Docker Desktop
Write-Host "Checking Docker Desktop..." -NoNewline
try {
    $dockerVersion = docker --version 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Host " OK" -ForegroundColor Green
        Write-Host "  $dockerVersion"
        
        # Check Docker is running
        $dockerInfo = docker info 2>$null
        if ($LASTEXITCODE -eq 0) {
            Write-Host "  Docker daemon is running" -ForegroundColor Green
        } else {
            Write-Host "  Docker daemon is NOT running - please start Docker Desktop" -ForegroundColor Red
        }
    } else {
        Write-Host " NOT INSTALLED" -ForegroundColor Red
        Write-Host "  Download from: https://www.docker.com/products/docker-desktop" -ForegroundColor Yellow
    }
} catch {
    Write-Host " NOT INSTALLED" -ForegroundColor Red
}

Write-Host ""

# Check Docker Compose
Write-Host "Checking Docker Compose..." -NoNewline
try {
    $composeVersion = docker compose version 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Host " OK" -ForegroundColor Green
        Write-Host "  $composeVersion"
    } else {
        # Try legacy docker-compose
        $composeVersionLegacy = docker-compose --version 2>$null
        if ($LASTEXITCODE -eq 0) {
            Write-Host " OK (legacy)" -ForegroundColor Green
            Write-Host "  $composeVersionLegacy"
        } else {
            Write-Host " NOT INSTALLED" -ForegroundColor Red
        }
    }
} catch {
    Write-Host " NOT INSTALLED" -ForegroundColor Red
}

Write-Host ""

# Check Git
Write-Host "Checking Git..." -NoNewline
try {
    $gitVersion = git --version 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Host " OK" -ForegroundColor Green
        Write-Host "  $gitVersion"
    } else {
        Write-Host " NOT INSTALLED" -ForegroundColor Red
    }
} catch {
    Write-Host " NOT INSTALLED" -ForegroundColor Red
}

Write-Host ""

# Check Python (for satellite data preparation)
Write-Host "Checking Python..." -NoNewline
try {
    $pythonVersion = python --version 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Host " OK" -ForegroundColor Green
        Write-Host "  $pythonVersion"
    } else {
        $python3Version = python3 --version 2>$null
        if ($LASTEXITCODE -eq 0) {
            Write-Host " OK" -ForegroundColor Green
            Write-Host "  $python3Version"
        } else {
            Write-Host " NOT INSTALLED" -ForegroundColor Red
            Write-Host "  Download from: https://www.python.org/downloads/" -ForegroundColor Yellow
        }
    }
} catch {
    Write-Host " NOT INSTALLED" -ForegroundColor Red
}

Write-Host ""
Write-Host "=============================================="
Write-Host "Project Directory Check"
Write-Host "=============================================="

# Check project structure
$projectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Write-Host "Project root: $projectRoot"

$requiredDirs = @("docker", "config", "scripts", "docs", "src", "satellite_data", "logs", "test_data")
foreach ($dir in $requiredDirs) {
    $fullPath = Join-Path $projectRoot $dir
    if (Test-Path $fullPath) {
        Write-Host "  [OK] $dir" -ForegroundColor Green
    } else {
        Write-Host "  [MISSING] $dir" -ForegroundColor Red
        New-Item -ItemType Directory -Path $fullPath -Force | Out-Null
        Write-Host "       Created $dir" -ForegroundColor Yellow
    }
}

Write-Host ""
Write-Host "=============================================="
Write-Host "Next Steps"
Write-Host "=============================================="
Write-Host ""
Write-Host "1. If Docker Desktop is not running, start it now"
Write-Host "2. Build Docker images:"
Write-Host "   docker compose -f docker-compose.dev.yml build" -ForegroundColor Cyan
Write-Host ""
Write-Host "3. Start PX4 SITL simulation:"
Write-Host "   docker compose -f docker-compose.dev.yml --profile simulation up px4_sitl" -ForegroundColor Cyan
Write-Host ""
Write-Host "4. In another terminal, start development shell:"
Write-Host "   docker compose -f docker-compose.dev.yml up dev_shell" -ForegroundColor Cyan
Write-Host ""
