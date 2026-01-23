#!/usr/bin/env pwsh
# Quick launcher for trial data collection
# Run: .\run_trial.ps1

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host " EMG+IMU Trial Data Collection" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

python trial/setup_trial.py

Read-Host -Prompt "Press Enter to exit"
