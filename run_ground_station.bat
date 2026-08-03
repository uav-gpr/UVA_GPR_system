@echo off
cd /d "%~dp0"
set "PYTHON_EXE=%~dp0.venv\Scripts\python.exe"

if not exist "%PYTHON_EXE%" (
    echo [Error] Python UI environment was not found.
    echo Expected: %PYTHON_EXE%
    pause
    exit /b 1
)

"%PYTHON_EXE%" uav_gpr_ground_station.py --host 0.0.0.0 --port 8765 --out remote_runs

if errorlevel 1 (
    echo.
    echo [Error] The ground-station UI failed to start.
    pause
)
