@echo off
setlocal
cd /d "%~dp0"

rem Prefer the Codex bundled Python when this workspace is used on this machine.
set "CODEX_PY=C:\Users\zhang\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%CODEX_PY%" (
    set "PYTHON=%CODEX_PY%"
) else (
    where py >nul 2>nul
    if not errorlevel 1 (
        set "PYTHON=py"
    ) else (
        where python >nul 2>nul
        if not errorlevel 1 (set "PYTHON=python") else (
            echo Python 3.10 or newer was not found.
            echo Install Python, then run: python -m pip install -r requirements.txt
            pause
            exit /b 1
        )
    )
)

"%PYTHON%" -c "import PySide6, serial" >nul 2>nul
if errorlevel 1 (
    echo Required packages are missing: PySide6 and/or pyserial.
    echo Installing from requirements.txt...
    "%PYTHON%" -m pip install -r requirements.txt
    if errorlevel 1 (
        echo Installation failed. Check network access or install the packages manually.
        pause
        exit /b 1
    )
)

"%PYTHON%" main.py
if errorlevel 1 (
    echo.
    echo The upper-computer program exited with an error.
    pause
)
endlocal
