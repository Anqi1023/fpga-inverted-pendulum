@echo off
cd /d "%~dp0"
where py >nul 2>nul
if errorlevel 1 goto python
py -3 -m pip install pyserial==3.5
pause
exit /b
:python
python -m pip install pyserial==3.5
pause
