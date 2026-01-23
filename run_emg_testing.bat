@echo off
REM Helper script to run EMG_testing.py using the venv Python directly
REM This bypasses PowerShell execution policy issues

set "VENV_PYTHON=%~dp0venv\Scripts\python.exe"
set "SCRIPT_PATH=%~dp0scripts\EMG_testing.py"

if not exist "%VENV_PYTHON%" (
    echo Error: Virtual environment not found at %VENV_PYTHON%
    echo Please create the venv first using: python -m venv venv
    exit /b 1
)

if not exist "%SCRIPT_PATH%" (
    echo Error: Script not found at %SCRIPT_PATH%
    exit /b 1
)

echo Running EMG_testing.py using venv Python...
echo Python: %VENV_PYTHON%
echo Script: %SCRIPT_PATH%
echo.

"%VENV_PYTHON%" "%SCRIPT_PATH%"
