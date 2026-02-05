<#
.SYNOPSIS
    One-time WSL2 setup for Proxigo UAV simulation
    
.DESCRIPTION
    Sets up WSL2 with Docker and all dependencies for running
    PX4 SITL + Gazebo simulation.
    
.EXAMPLE
    .\scripts\setup_wsl.ps1
#>

$ErrorActionPreference = "Stop"

Write-Host ""
Write-Host "╔══════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║        PROXIGO WSL2 SETUP                                    ║" -ForegroundColor Cyan
Write-Host "║        One-time configuration for simulation                 ║" -ForegroundColor Cyan
Write-Host "╚══════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# ============================================================================
# Step 1: Check/Install WSL2
# ============================================================================

Write-Host "[1/5] Checking WSL2..." -ForegroundColor Yellow

try {
    $wslVersion = wsl --version 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw "WSL not installed"
    }
    Write-Host "  WSL2 is installed" -ForegroundColor Green
} catch {
    Write-Host "  Installing WSL2..." -ForegroundColor Cyan
    Write-Host "  This requires administrator privileges and a restart." -ForegroundColor Yellow
    
    # Enable WSL feature
    dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
    dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
    
    Write-Host ""
    Write-Host "  WSL2 features enabled. Please:" -ForegroundColor Yellow
    Write-Host "    1. Restart your computer" -ForegroundColor White
    Write-Host "    2. Run: wsl --install -d Ubuntu" -ForegroundColor White
    Write-Host "    3. Run this script again" -ForegroundColor White
    exit 1
}

# Check for Ubuntu distro
$distros = wsl --list --quiet 2>&1 | Where-Object { $_ -and $_.Trim() }
if (-not ($distros -match "Ubuntu")) {
    Write-Host "  Installing Ubuntu..." -ForegroundColor Cyan
    wsl --install -d Ubuntu
    Write-Host ""
    Write-Host "  Ubuntu installed. Please:" -ForegroundColor Yellow
    Write-Host "    1. Complete Ubuntu setup (create user/password)" -ForegroundColor White
    Write-Host "    2. Run this script again" -ForegroundColor White
    exit 1
}

Write-Host "  Ubuntu distro found" -ForegroundColor Green

# ============================================================================
# Step 2: Install Docker in WSL
# ============================================================================

Write-Host ""
Write-Host "[2/5] Setting up Docker in WSL..." -ForegroundColor Yellow

$dockerScript = @'
#!/bin/bash
set -e

# Update packages
sudo apt-get update -qq

# Install Docker if not present
if ! command -v docker &> /dev/null; then
    echo "Installing Docker..."
    curl -fsSL https://get.docker.com | sudo sh
    sudo usermod -aG docker $USER
    echo "Docker installed"
else
    echo "Docker already installed"
fi

# Install Docker Compose plugin
if ! docker compose version &> /dev/null; then
    echo "Installing Docker Compose..."
    sudo apt-get install -y docker-compose-plugin
fi

# Start Docker daemon
if ! pgrep -x dockerd > /dev/null; then
    echo "Starting Docker daemon..."
    sudo dockerd > /dev/null 2>&1 &
    sleep 3
fi

# Test Docker
docker info > /dev/null 2>&1 && echo "Docker is working!" || echo "Docker test failed"
'@

$dockerScript | wsl bash

Write-Host "  Docker configured in WSL" -ForegroundColor Green

# ============================================================================
# Step 3: Install Python dependencies
# ============================================================================

Write-Host ""
Write-Host "[3/5] Installing Python dependencies..." -ForegroundColor Yellow

wsl bash -c "pip3 install pymavlink --quiet 2>/dev/null || (sudo apt-get install -y python3-pip && pip3 install pymavlink)"
Write-Host "  pymavlink installed" -ForegroundColor Green

# ============================================================================
# Step 4: Configure X11 forwarding
# ============================================================================

Write-Host ""
Write-Host "[4/5] Configuring X11 forwarding..." -ForegroundColor Yellow

# Add DISPLAY to .bashrc if not present
wsl bash -c @'
if ! grep -q "DISPLAY=" ~/.bashrc; then
    WIN_HOST=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')
    echo "export DISPLAY=$WIN_HOST:0" >> ~/.bashrc
    echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
    echo "X11 forwarding configured in .bashrc"
fi
'@

Write-Host "  X11 forwarding configured" -ForegroundColor Green

# ============================================================================
# Step 5: VcXsrv
# ============================================================================

Write-Host ""
Write-Host "[5/5] X Server (VcXsrv)..." -ForegroundColor Yellow

$vcxsrvPath = @(
    "$env:ProgramFiles\VcXsrv\vcxsrv.exe",
    "${env:ProgramFiles(x86)}\VcXsrv\vcxsrv.exe"
) | Where-Object { Test-Path $_ } | Select-Object -First 1

if ($vcxsrvPath) {
    Write-Host "  VcXsrv found at: $vcxsrvPath" -ForegroundColor Green
} else {
    Write-Host "  VcXsrv not found. Please install from:" -ForegroundColor Yellow
    Write-Host "  https://sourceforge.net/projects/vcxsrv/" -ForegroundColor Cyan
}

# ============================================================================
# Complete
# ============================================================================

Write-Host ""
Write-Host "╔══════════════════════════════════════════════════════════════╗" -ForegroundColor Green
Write-Host "║        SETUP COMPLETE!                                       ║" -ForegroundColor Green
Write-Host "╚══════════════════════════════════════════════════════════════╝" -ForegroundColor Green
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Cyan
Write-Host ""
Write-Host "  1. Start VcXsrv (XLaunch) with:" -ForegroundColor White
Write-Host "     - Multiple windows" -ForegroundColor Gray
Write-Host "     - Display number: 0" -ForegroundColor Gray
Write-Host "     - Disable access control (CHECKED)" -ForegroundColor Gray
Write-Host ""
Write-Host "  2. Start simulation:" -ForegroundColor White
Write-Host "     .\scripts\Start-Simulation.ps1" -ForegroundColor Yellow
Write-Host ""
Write-Host "  3. Open QGroundControl" -ForegroundColor White
Write-Host ""
