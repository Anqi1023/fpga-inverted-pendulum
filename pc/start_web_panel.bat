@echo off
cd /d "%~dp0"
where py >nul 2>nul
if errorlevel 1 goto python
py -3 web_panel.py
pause
exit /b
:python
python web_panel.py
pause
