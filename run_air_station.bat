@echo off
setlocal
title UAV-GPR Air Station Launcher
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
REM 2026-10-15: the old air-side GUI (uav_gpr_qt.py MainWindow) was retired.
REM This launcher now starts the headless black-box service in local mode,
REM sending to a ground station running on this same PC (127.0.0.1:8765).
REM Start run_ground_station.bat as well and control everything from there.
echo Starting UAV-GPR air black-box service (local mode, ground = 127.0.0.1:8765)...
echo Program: %~dp0uav_gpr_air_service.py
echo.

"%PYTHON_EXE%" "%~dp0uav_gpr_air_service.py" --server 127.0.0.1:8765 %*

if errorlevel 1 (
    echo.
    echo [Error] The air black-box service failed to start.
    pause
    exit /b 1
)

endlocal
