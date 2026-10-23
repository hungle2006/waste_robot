@echo off
cd /d "%~dp0"
if not defined ARDUINO_PORT if not exist .env set /p ARDUINO_PORT=Nhap cong Arduino (vi du COM3): 
if exist .venv\Scripts\python.exe (
  .venv\Scripts\python.exe main.py
) else (
  python main.py
)
pause
