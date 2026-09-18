@echo off
chcp 65001 >nul
title 查看本机地面站 IP 地址
cd /d "%~dp0"

echo ======================================================================
echo             UAV-GPR 地面端（当前电脑）IP 地址查询
echo ======================================================================
echo.
echo 正在查询本机可用的 IP 地址...
echo.

powershell -NoProfile -Command "Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -notlike '127.*' -and $_.IPAddress -notlike '169.254.*' } | ForEach-Object { Write-Host ('  >> 网络名称: ' + $_.InterfaceAlias); Write-Host ('  >> IP 地址  : ' + $_.IPAddress -ForegroundColor Green); Write-Host '' }"

echo ======================================================================
echo 【使用说明】：
echo 1. 上面绿色显示的数字就是您本机的 IP 地址（例如 192.168.x.x 或 202.x.x.x）。
echo 2. 在新电脑（空中端）界面顶部的【地面站 IP】框里，填上这个绿色的 IP 数字。
echo 3. 端口保持默认的 8765 不变。
echo ======================================================================
echo.
pause
