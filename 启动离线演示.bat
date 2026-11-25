@echo off
chcp 65001 >nul
title TC264 三环远程调参台（离线虚拟小车演示）
call "%~dp0启动三环调参台.bat" --demo %*
