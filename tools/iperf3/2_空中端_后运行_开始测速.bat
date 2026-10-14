@echo off
title iperf3 CLIENT - Air PC (run SECOND, Air to Ground)
echo ==============================================
echo  AIR PC (192.168.144.21) - sends to GROUND PC
echo  This measures the radar-data direction.
echo  Make sure script 1 is running on the GROUND PC!
echo  Test runs 30 seconds.
echo ==============================================
cd /d "%~dp0"
iperf3.exe -c 192.168.144.20 -t 30
echo.
echo Read the LAST summary lines:
echo   sender   = what the air PC sent
echo   receiver = what the ground PC got  (USE THIS ONE)
pause
