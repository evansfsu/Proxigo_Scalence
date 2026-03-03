$path = Join-Path (Split-Path -Parent $PSScriptRoot) "scripts\Start-Simulation.ps1"
$content = Get-Content -LiteralPath $path -Raw -Encoding UTF8
$content = $content -replace [char]0x201C, [char]0x0022 -replace [char]0x201D, [char]0x0022
Set-Content -LiteralPath $path -Value $content -Encoding UTF8 -NoNewline
Write-Host "Replaced Unicode quotes in Start-Simulation.ps1"
