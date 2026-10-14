@echo off
title iperf3 SERVER - Ground PC (run this FIRST)
echo ==============================================
echo  GROUND PC (192.168.144.20) - iperf3 SERVER
echo  Keep this window OPEN.
echo  Then run script 2 on the AIR PC.
echo  If Windows Firewall asks, click ALLOW.
echo ==============================================
cd /d "%~dp0"
rem Auto-clean: close any leftover iperf3 from a previous run (it is our own tool, safe to kill)
taskkill /F /IM iperf3.exe >nul 2>&1
iperf3.exe -s
echo.
echo (Server stopped.)
pause
