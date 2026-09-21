@echo off
title Servo Motor Control System
cd /d "%~dp0"

echo ====================================================
echo   Servo Motor Control System
echo ====================================================
echo.

python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python not found in PATH. Please install Python 3.10+ and add it to PATH.
    pause
    exit /b 1
)

echo Starting Servo Control PC Application...
python main.py

if %errorlevel% neq 0 (
    echo.
    echo Application exited with code: %errorlevel%
    pause
)
