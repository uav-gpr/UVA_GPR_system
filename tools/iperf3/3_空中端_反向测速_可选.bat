@echo off
title iperf3 CLIENT reverse (optional, Ground to Air)
echo ==============================================
echo  Reverse test: Ground to Air direction.
echo  Make sure script 1 is running on the GROUND PC!
echo ==============================================
cd /d "%~dp0"
iperf3.exe -c 192.168.144.20 -t 30 -R
echo.
echo   sender   = what the ground PC sent
echo   receiver = what the air PC got
pause
