# PowerShell script to set up Python virtual environment
# Run this script from the project root directory
# Requires Python 3.8-3.12 (3.12 recommended) for TensorFlow compatibility

Write-Host "Setting up Python virtual environment..." -ForegroundColor Green
Write-Host "Note: TensorFlow requires Python 3.8-3.12 (3.12 recommended)" -ForegroundColor Yellow

# Function to find Python 3.12
function Find-Python312 {
    # Try py launcher with 3.12
    try {
        $result = py -3.12 --version 2>&1
        if ($LASTEXITCODE -eq 0) {
            return "py -3.12"
        }
    } catch {}
    
    # Try direct python3.12
    try {
        $result = python3.12 --version 2>&1
        if ($LASTEXITCODE -eq 0) {
            return "python3.12"
        }
    } catch {}
    
    # Check current python version
    try {
        $version = python --version 2>&1
        if ($version -match "3\.(8|9|10|11|12)") {
            Write-Host "Found compatible Python: $version" -ForegroundColor Green
            return "python"
        } else {
            Write-Host "Current Python version: $version" -ForegroundColor Yellow
            Write-Host "This version may not be compatible with TensorFlow." -ForegroundColor Yellow
        }
    } catch {}
    
    return $null
}

# Find compatible Python
$pythonCmd = Find-Python312

if ($null -eq $pythonCmd) {
    Write-Host "`nPython 3.8-3.12 not found!" -ForegroundColor Red
    Write-Host "Attempting to install Python 3.12 via py launcher..." -ForegroundColor Yellow
    py install 3.12
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Python 3.12 installed! Retrying..." -ForegroundColor Green
        $pythonCmd = Find-Python312
        if ($null -eq $pythonCmd) {
            Write-Host "Installation may require a restart. Please:" -ForegroundColor Yellow
            Write-Host "  1. Restart PowerShell" -ForegroundColor Cyan
            Write-Host "  2. Run this script again" -ForegroundColor Cyan
            exit 1
        }
    } else {
        Write-Host "`nPlease install Python 3.12 manually:" -ForegroundColor Yellow
        Write-Host "  1. Download from: https://www.python.org/downloads/" -ForegroundColor Cyan
        Write-Host "  2. Or use: py install 3.12" -ForegroundColor Cyan
        Write-Host "  3. Then run this script again" -ForegroundColor Cyan
        exit 1
    }
}

Write-Host "Using: $pythonCmd" -ForegroundColor Green
if ($pythonCmd -match "py -3.12") {
    $pythonVersion = py -3.12 --version 2>&1
} else {
    $pythonVersion = & $pythonCmd --version 2>&1
}
Write-Host "Python version: $pythonVersion" -ForegroundColor Cyan

# Check if venv already exists
if (Test-Path "venv") {
    Write-Host "`nVirtual environment 'venv' already exists." -ForegroundColor Yellow
    $response = Read-Host "Do you want to recreate it? (y/n)"
    if ($response -eq "y" -or $response -eq "Y") {
        Remove-Item -Recurse -Force venv
        Write-Host "Removed existing virtual environment." -ForegroundColor Yellow
    } else {
        Write-Host "Using existing virtual environment." -ForegroundColor Cyan
        $reuseExisting = $true
    }
}

# Create virtual environment if it doesn't exist
if (-not (Test-Path "venv") -and -not $reuseExisting) {
    Write-Host "`nCreating virtual environment with $pythonCmd..." -ForegroundColor Cyan
    if ($pythonCmd -match "py -3.12") {
        py -3.12 -m venv venv
    } else {
        & $pythonCmd -m venv venv
    }
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Error: Failed to create virtual environment" -ForegroundColor Red
        exit 1
    }
    Write-Host "Virtual environment created successfully!" -ForegroundColor Green
}

# Activate virtual environment
Write-Host "`nActivating virtual environment..." -ForegroundColor Cyan
& .\venv\Scripts\Activate.ps1

# Upgrade pip
Write-Host "Upgrading pip..." -ForegroundColor Cyan
python -m pip install --upgrade pip

# Install dependencies
Write-Host "`nInstalling dependencies from requirements.txt..." -ForegroundColor Cyan
Write-Host "This may take several minutes..." -ForegroundColor Yellow
pip install -r requirements.txt

if ($LASTEXITCODE -eq 0) {
    Write-Host "`nSetup complete! Virtual environment is ready." -ForegroundColor Green
    Write-Host "`nTo activate the environment in the future, run:" -ForegroundColor Yellow
    Write-Host "  .\venv\Scripts\Activate.ps1" -ForegroundColor White
    Write-Host "`nInstalled packages:" -ForegroundColor Cyan
    pip list | Select-String -Pattern "torch|tensorflow|keras|numpy|pandas|matplotlib|pyserial|vpython"
} else {
    Write-Host "`nError: Some packages failed to install. Check the output above." -ForegroundColor Red
    Write-Host "Note: If TensorFlow installation failed, ensure you're using Python 3.8-3.12" -ForegroundColor Yellow
    exit 1
}
