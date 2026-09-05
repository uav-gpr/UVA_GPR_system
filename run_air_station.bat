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
echo Starting UAV-GPR air station...
echo Program: %~dp0uav_gpr_qt.py
echo.

"%PYTHON_EXE%" "%~dp0uav_gpr_qt.py"

if errorlevel 1 (
    echo.
    echo [Error] The air-station UI failed to start.
    pause
    exit /b 1
)

endlocal
