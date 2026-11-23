@echo off
cd /d "%~dp0"
where python >nul 2>nul && (python serve.py) || (py -3 serve.py)
pause
