# Configure QGroundControl Auto-Connect
# Based on QGC container documentation and settings file format
# This script automatically configures QGC to connect to PX4 SITL

param(
    [string]$ConnectionType = "UDP",  # UDP or TCP
    [int]$Port = 14550,                # 14550 for UDP, 5760 for TCP
    [string]$ServerHost = "127.0.0.1"  # Only for TCP (renamed from Host to avoid PowerShell reserved variable)
)

Write-Host "================================================" -ForegroundColor Cyan
Write-Host "  QGroundControl Auto-Connect Configuration" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""

# Find QGC settings directory (QGC uses JSON format)
$qgcSettingsDir = "$env:APPDATA\QGroundControl.org"
$qgcCommLinksFile = Join-Path $qgcSettingsDir "LinkCollection.json"

if (-not (Test-Path $qgcSettingsDir)) {
    Write-Host "[INFO] QGC settings directory not found. Creating..." -ForegroundColor Yellow
    New-Item -ItemType Directory -Path $qgcSettingsDir -Force | Out-Null
}

Write-Host "[INFO] QGC Comm Links: $qgcCommLinksFile" -ForegroundColor Gray

# QGC stores connections in LinkCollection.json (JSON format)
# Read existing connections
$commLinksList = New-Object System.Collections.ArrayList
if (Test-Path $qgcCommLinksFile) {
    try {
        $jsonContent = Get-Content $qgcCommLinksFile -Raw | ConvertFrom-Json
        if ($jsonContent.Links) {
            foreach ($link in $jsonContent.Links) {
                [void]$commLinksList.Add($link)
            }
        }
        Write-Host "[OK] Found existing QGC connections" -ForegroundColor Green
    } catch {
        Write-Host "[WARN] Could not parse existing QGC settings, creating new" -ForegroundColor Yellow
    }
} else {
    Write-Host "[INFO] Creating new QGC connections file" -ForegroundColor Yellow
}

# Configure connection based on type
if ($ConnectionType -eq "UDP") {
    $linkName = "PX4 SITL UDP"
    $linkType = "UDP"
    $linkConfig = @{
        baud = 0
        dynamic = $false
        port = $Port
        host = ""
    }
    Write-Host "[INFO] Configuring UDP connection on port $Port" -ForegroundColor Cyan
} else {
    $linkName = "PX4 SITL TCP"
    $linkType = "TCP"
    $linkConfig = @{
        baud = 0
        dynamic = $false
        port = $Port
        host = $ServerHost
    }
    Write-Host "[INFO] Configuring TCP connection to $ServerHost`:$Port" -ForegroundColor Cyan
}

# Remove existing link with same name if present
for ($i = $commLinksList.Count - 1; $i -ge 0; $i--) {
    if ($commLinksList[$i].name -eq $linkName) {
        $commLinksList.RemoveAt($i)
    }
}

# Create new link object
$newLink = [PSCustomObject]@{
    autoConnect = $true
    highLatency = $false
    name = $linkName
    type = $linkType
    baud = $linkConfig.baud
    dynamic = $linkConfig.dynamic
    port = $linkConfig.port
    host = $linkConfig.host
}

# Add new link
[void]$commLinksList.Add($newLink)

# Create JSON structure
$linkCollection = [PSCustomObject]@{
    Links = $commLinksList.ToArray()
} | ConvertTo-Json -Depth 10

# Write settings file
try {
    Set-Content -Path $qgcCommLinksFile -Value $linkCollection -Encoding UTF8
    Write-Host "[OK] QGC settings configured successfully!" -ForegroundColor Green
} catch {
    Write-Host "[ERROR] Failed to write QGC settings: $_" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "================================================" -ForegroundColor Cyan
Write-Host "  Configuration Complete" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Connection configured:" -ForegroundColor White
Write-Host "  Type: $ConnectionType" -ForegroundColor Cyan
if ($ConnectionType -eq "TCP") {
    Write-Host "  Host: $ServerHost" -ForegroundColor Cyan
}
Write-Host "  Port: $Port" -ForegroundColor Cyan
Write-Host "  Auto Connect: Enabled" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "  1. Open QGroundControl" -ForegroundColor White
Write-Host "  2. QGC should automatically connect to PX4 SITL" -ForegroundColor White
Write-Host "  3. If not, go to: Application Settings → Comm Links" -ForegroundColor White
Write-Host "  4. The connection '$linkName' should be listed with Auto Connect enabled" -ForegroundColor White
Write-Host ""

# Optionally launch QGC
$response = Read-Host "Launch QGroundControl now? (y/n)"
if ($response -eq "y" -or $response -eq "Y") {
    $qgcPath = @(
        "$env:LOCALAPPDATA\Programs\qgroundcontrol\QGroundControl.exe",
        "$env:ProgramFiles\QGroundControl\QGroundControl.exe",
        "${env:ProgramFiles(x86)}\QGroundControl\QGroundControl.exe"
    ) | Where-Object { Test-Path $_ } | Select-Object -First 1
    
    if ($qgcPath) {
        Write-Host "[INFO] Launching QGroundControl..." -ForegroundColor Yellow
        Start-Process $qgcPath
        Write-Host "[OK] QGC launched. It should auto-connect!" -ForegroundColor Green
    } else {
        Write-Host "[WARN] QGroundControl not found. Please launch it manually." -ForegroundColor Yellow
    }
}
