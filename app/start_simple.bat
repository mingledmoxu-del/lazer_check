@echo off
setlocal
cd /d "%~dp0"
title P100S Simple Angle Control

set "PYTHON="

rem Prefer the bundled Python because MSYS2's python.exe may not include pip.
call :try_python "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
call :try_python "D:\ProgramData\miniconda3\python.exe"

for /f "delims=" %%P in ('where python 2^>nul') do call :try_python "%%P"
for /d %%P in ("%LOCALAPPDATA%\Programs\Python\Python3*") do call :try_python "%%~fP\python.exe"

if not defined PYTHON (
    echo [ERROR] No suitable Python was found.
    echo Python 3.10 or newer with pip and Tkinter is required.
    echo MSYS2 Python without pip cannot be used.
    pause
    exit /b 1
)

echo Using Python: %PYTHON%
"%PYTHON%" --version

"%PYTHON%" -c "import serial" >nul 2>nul
if errorlevel 1 (
    echo Installing the small pyserial dependency...
    "%PYTHON%" -m pip install -r requirements.txt
    if errorlevel 1 (
        echo Installation failed. Try:
        echo "%PYTHON%" -m pip install pyserial -i https://pypi.tuna.tsinghua.edu.cn/simple
        pause
        exit /b 1
    )
)

"%PYTHON%" main.py
if errorlevel 1 pause
endlocal
exit /b 0

:try_python
if defined PYTHON exit /b 0
if not exist "%~1" exit /b 0
"%~1" -c "import sys, tkinter; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>nul
if errorlevel 1 exit /b 0
"%~1" -m pip --version >nul 2>nul
if errorlevel 1 exit /b 0
set "PYTHON=%~1"
exit /b 0
