# Helper script to run EMG_testing.py using the venv Python directly
# This bypasses the need to activate the venv (which requires PowerShell execution policy)

$venvPython = Join-Path $PSScriptRoot "venv\Scripts\python.exe"
$scriptPath = Join-Path $PSScriptRoot "scripts\EMG_testing.py"

if (-not (Test-Path $venvPython)) {
    Write-Host "Error: Virtual environment not found at $venvPython" -ForegroundColor Red
    Write-Host "Please create the venv first using: python -m venv venv" -ForegroundColor Yellow
    exit 1
}

if (-not (Test-Path $scriptPath)) {
    Write-Host "Error: Script not found at $scriptPath" -ForegroundColor Red
    exit 1
}

Write-Host "Running EMG_testing.py using venv Python..." -ForegroundColor Cyan
Write-Host "Python: $venvPython" -ForegroundColor Gray
Write-Host "Script: $scriptPath" -ForegroundColor Gray
Write-Host ""

& $venvPython $scriptPath
