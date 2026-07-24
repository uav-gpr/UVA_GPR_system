@echo off
cd /d "%~dp0"
python uav_gpr_ground_station.py --host 0.0.0.0 --port 8765 --out remote_runs
