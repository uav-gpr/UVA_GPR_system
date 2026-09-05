@echo off
setlocal
title UAV-GPR Ground Station Launcher
cd /d "%~dp0"

set "PYTHON_EXE=%~dp0.venv312\Scripts\python.exe"
if exist "%PYTHON_EXE%" goto python_found

set "PYTHON_EXE=%~dp0.venv\Scripts\python.exe"
if exist "%PYTHON_EXE%" goto python_found

echo [Error] Python UI environment was not found.
echo Expected one of:
echo   %~dp0.venv312\Scripts\python.exe
echo   %~dp0.venv\Scripts\python.exe
echo.
echo Install dependencies before running this launcher.
pause
exit /b 1

:python_found
echo Starting UAV-GPR ground station...
echo Listening: 0.0.0.0:8765
echo Output:    %~dp0remote_runs
echo.

"%PYTHON_EXE%" "%~dp0uav_gpr_ground_station.py" --host 0.0.0.0 --port 8765 --out "%~dp0remote_runs"

if errorlevel 1 (
    echo.
    echo [Error] The ground-station UI failed to start.
    pause
    exit /b 1
)

endlocal
