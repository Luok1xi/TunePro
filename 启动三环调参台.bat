@echo off
chcp 65001 >nul
title TC264 三环远程调参台
where py >nul 2>nul
if %errorlevel%==0 (
  py -3 -X utf8 -u "%~dp0app.py" %*
  goto :done
)
where python >nul 2>nul
if %errorlevel%==0 (
  python -X utf8 -u "%~dp0app.py" %*
  goto :done
)
echo [错误] 未检测到 Python 3，请先安装 Python 3.10+ 并加入 PATH。
:done
if errorlevel 1 pause
