@echo off
setlocal
title UAV-GPR Merge Runs
cd /d "%~dp0"

set "PYTHON_EXE=%~dp0.venv312\Scripts\python.exe"
if exist "%PYTHON_EXE%" goto python_found
set "PYTHON_EXE=%~dp0.venv\Scripts\python.exe"
if exist "%PYTHON_EXE%" goto python_found
echo [Error] Python environment was not found (.venv312 or .venv).
pause
exit /b 1

:python_found
rem Drag one or more run folders onto this file.
rem Double-click with no folder: merge every run under remote_runs.
if "%~1"=="" (
    "%PYTHON_EXE%" "%~dp0tools\merge_runs.py" "%~dp0remote_runs"
) else (
    "%PYTHON_EXE%" "%~dp0tools\merge_runs.py" %*
)
echo.
pause
endlocal
