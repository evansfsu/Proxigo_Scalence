# Check all .ps1 in scripts for parse errors
$base = Split-Path -Parent $PSScriptRoot
$scripts = Get-ChildItem -Path (Join-Path $base "scripts") -Filter "*.ps1" -File
$anyError = $false
foreach ($f in $scripts) {
  $errors = $null
  [void][System.Management.Automation.Language.Parser]::ParseFile($f.FullName, [ref]$null, [ref]$errors)
  if ($errors -and $errors.Count -gt 0) {
    $anyError = $true
    Write-Host "ERROR in $($f.Name):"
    foreach ($e in $errors) { Write-Host "  $($e.ToString())" }
  } else {
    Write-Host "OK: $($f.Name)"
  }
}
if ($anyError) { exit 1 }
exit 0
