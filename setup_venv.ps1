# PowerShell script to set up Python virtual environment
# Run this script from the project root directory
# Requires Python 3.8-3.12 (3.12 recommended) for TensorFlow compatibility
# Cross-platform: Works on Windows, macOS, and Linux

Write-Host "Setting up Python virtual environment..." -ForegroundColor Green
Write-Host "Note: TensorFlow requires Python 3.8-3.12 (3.12 recommended)" -ForegroundColor Yellow

# Detect OS
# PowerShell Core 6+ has built-in variables, but we also check environment variables for compatibility
$isWindows = if ($IsWindows -ne $null) { $IsWindows } else { $env:OS -match "Windows" -or $PSVersionTable.Platform -match "Win" }
$isMacOS = if ($IsMacOS -ne $null) { $IsMacOS } else { 
    try {
        $uname = & uname 2>&1
        $uname -match "Darwin"
    } catch {
        $false
    }
}
$isLinux = if ($IsLinux -ne $null) { $IsLinux } else {
    try {
        $uname = & uname 2>&1
        $uname -match "Linux"
    } catch {
        $false
    }
}

# Determine activation script path based on OS
if ($isWindows) {
    $activateScript = "venv\Scripts\Activate.ps1"
    $activateCmd = ".\venv\Scripts\Activate.ps1"
} else {
    # macOS/Linux
    $activateScript = "venv/bin/Activate.ps1"
    $activateCmd = "source venv/bin/activate"
}

# Function to find Python 3.12
function Find-Python312 {
    # On Windows, try py launcher first
    if ($isWindows) {
        try {
            $result = py -3.12 --version 2>&1
            if ($LASTEXITCODE -eq 0) {
                return "py -3.12"
            }
        } catch {}
    }
    
    # Try direct python3.12 (works on all platforms)
    try {
        $result = python3.12 --version 2>&1
        if ($LASTEXITCODE -eq 0) {
            return "python3.12"
        }
    } catch {}
    
    # Try python3 (common on macOS/Linux)
    try {
        $version = python3 --version 2>&1
        if ($version -match "3\.(8|9|10|11|12)") {
            Write-Host "Found compatible Python: $version" -ForegroundColor Green
            return "python3"
        }
    } catch {}
    
    # Check current python version (fallback)
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
    if ($isWindows) {
        Write-Host "Attempting to install Python 3.12 via py launcher..." -ForegroundColor Yellow
        try {
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
            }
        } catch {
            Write-Host "Auto-install failed. Please install manually." -ForegroundColor Yellow
        }
    }
    
    if ($null -eq $pythonCmd) {
        Write-Host "`nPlease install Python 3.8-3.12 manually:" -ForegroundColor Yellow
        if ($isWindows) {
            Write-Host "  1. Download from: https://www.python.org/downloads/" -ForegroundColor Cyan
            Write-Host "  2. Or use: py install 3.12" -ForegroundColor Cyan
        } elseif ($isMacOS) {
            Write-Host "  1. Install via Homebrew: brew install python@3.12" -ForegroundColor Cyan
            Write-Host "  2. Or download from: https://www.python.org/downloads/" -ForegroundColor Cyan
        } else {
            Write-Host "  1. Install via your package manager (e.g., apt, yum, pacman)" -ForegroundColor Cyan
            Write-Host "  2. Or download from: https://www.python.org/downloads/" -ForegroundColor Cyan
        }
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
if (Test-Path $activateScript) {
    & $activateScript
} else {
    Write-Host "Warning: Activation script not found at $activateScript" -ForegroundColor Yellow
    Write-Host "You may need to activate manually using: $activateCmd" -ForegroundColor Yellow
}

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
    if ($isWindows) {
        Write-Host "  .\venv\Scripts\Activate.ps1" -ForegroundColor White
    } else {
        Write-Host "  source venv/bin/activate" -ForegroundColor White
        Write-Host "  (or in PowerShell: . venv/bin/Activate.ps1)" -ForegroundColor Gray
    }
    Write-Host "`nInstalled packages:" -ForegroundColor Cyan
    pip list | Select-String -Pattern "torch|tensorflow|keras|numpy|pandas|matplotlib|scipy|pyserial|vpython"
} else {
    Write-Host "`nError: Some packages failed to install. Check the output above." -ForegroundColor Red
    Write-Host "Note: If TensorFlow installation failed, ensure you're using Python 3.8-3.12" -ForegroundColor Yellow
    exit 1
}
