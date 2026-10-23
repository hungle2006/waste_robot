"""Read-only diagnostics: no Arduino connection or motor commands."""
import ctypes
import os
from pathlib import Path
import platform
import sys


def diagnose():
    print('\n=== WASTE ROBOT JETSON NANO - DIAGNOSTICS ===')
    print('Python:', sys.version.split()[0], '|', sys.executable)
    print('OS:', platform.platform(), '| CPU:', platform.machine())
    try:
        libc = ctypes.CDLL('libc.so.6')
        libc.gnu_get_libc_version.restype = ctypes.c_char_p
        print('glibc:', libc.gnu_get_libc_version().decode())
    except (OSError, AttributeError):
        print('glibc: không kiểm tra được')
    for module in ('supabase', 'realtime', 'dotenv', 'httpx', 'fastapi', 'uvicorn', 'serial', 'cv2', 'numpy', 'PIL'):
        try:
            obj = __import__(module)
            print('IMPORT OK:', module, getattr(obj, '__version__', ''))
        except Exception as error:
            print('IMPORT FAIL:', module, type(error).__name__, str(error)[:180])
    for path in ['/dev/video0', '/dev/video1', '/dev/ttyACM0', '/dev/ttyUSB0']:
        print(path, 'có' if Path(path).exists() else 'không thấy')
    print('ARDUINO_PORT:', os.environ.get('ARDUINO_PORT', '/dev/ttyUSB0'))
    print('File .env:', 'có' if Path(__file__).with_name('.env').exists() else 'không có (dùng mặc định)')
    print('Kiểm tra không mở cổng serial và không ra lệnh chuyển động.\n')


def diagnose_livekit():
    """Check current Python's RTC/API and OS compatibility without hardware."""
    import importlib.util
    print('\n=== KIỂM TRA LIVEKIT TRONG PYTHON HIỆN TẠI ===')
    print('Python:', sys.executable)
    print('Version:', sys.version.split()[0], '| CPU:', platform.machine())
    print('Cài gói LiveKit:', 'có' if importlib.util.find_spec('livekit') else 'không thấy')
    libc, version = platform.libc_ver()
    print('libc:', libc, version)
    if libc == 'glibc' and version:
        try:
            if tuple(int(v) for v in version.split('.')[:2]) < (2, 28):
                print('CẢNH BÁO: glibc <2.28 có thể không nạp được LiveKit RTC 1.1.21')
        except ValueError:
            pass
    try:
        from jetson_livekit_camera import livekit_import_check
        ready, reason = livekit_import_check()
    except Exception as exc:
        ready, reason = False, '{}: {}'.format(type(exc).__name__, str(exc)[:200])
    print('RTC + API:', 'OK' if ready else 'CHƯA SẴN SÀNG', '-', reason)
    if ready:
        print('SDK import được; để PHÁT video vẫn cần URL, API key/secret và camera.')
    # main.py may use the separately managed Miniforge environment. Check it as well.
    runtime_python = Path(os.environ.get('WASTE_ROBOT_ENV_HOME', str(Path.home() / '.local' / 'share' / 'waste-robot'))) / 'python310' / 'bin' / 'python'
    runtime_ready = False
    if runtime_python.is_file() and str(runtime_python.resolve()) != str(Path(sys.executable).resolve()):
        import subprocess
        check = subprocess.run([str(runtime_python), '-c',
            'from livekit import rtc, api; assert hasattr(rtc, "Room") and hasattr(api, "AccessToken")'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20)
        runtime_ready = check.returncode == 0
        print('Python riêng của robot:', runtime_python)
        print('LiveKit trong Python riêng:', 'OK' if runtime_ready else 'CHƯA SẴN SÀNG')
    print('Không mở camera, serial, không phát lệnh robot.\n')
    return 0 if (ready or runtime_ready) else 2
