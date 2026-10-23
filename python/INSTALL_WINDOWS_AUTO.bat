@echo off
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (
  python -m venv .venv
  if errorlevel 1 goto fail
)
.venv\Scripts\python.exe -m pip install -r requirements.txt -r requirements-camera.txt
if errorlevel 1 goto fail
.venv\Scripts\python.exe -c "import serial; assert callable(serial.Serial); print('pyserial OK')"
if errorlevel 1 goto fail
echo Cai dat xong. Sua ARDUINO_PORT trong .env, chay start_windows.bat.
pause
exit /b 0
:fail
echo Cai dat chua thanh cong. Xem loi phia tren.
pause
exit /b 1
