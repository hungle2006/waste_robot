"""Safe, opt-in LiveKit RTC compatibility test for Jetson Nano.

No robot/serial/camera connection is opened. Packages are installed ONLY in a
separate virtual environment, never into the JetPack system Python or the
robot's existing conda environment. Compatible with Python 3.10+.
"""
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

LIVEKIT_VERSION = '1.1.21'
LIVEKIT_API_VERSION = '1.2.2'
RESULT_DIR = Path(os.environ.get(
    'WASTE_ROBOT_ENV_HOME', str(Path.home() / '.local' / 'share' / 'waste-robot')))
TEST_ENV = RESULT_DIR / 'livekit-test'
STATUS_FILE = RESULT_DIR / 'livekit-test-result.json'


def glibc_version():
    libc, version = platform.libc_ver()
    if libc != 'glibc' or not version:
        return None
    try:
        return tuple(int(x) for x in version.split('.')[:2])
    except ValueError:
        return None


def supported_binary_platform():
    """None means unknown, not proven compatible or incompatible."""
    if sys.platform != 'linux':
        return None
    version = glibc_version()
    if version is None:
        return None
    return version >= (2, 28)


def record_status(status, reason):
    """Record only environment diagnostics, never credentials."""
    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    result = {
        'status': status,
        'reason': reason,
        'platform': sys.platform,
        'machine': platform.machine(),
        'glibc': '.'.join(map(str, glibc_version())) if glibc_version() else None,
        'python': platform.python_version(),
        'livekit_version': LIVEKIT_VERSION,
        'livekit_api_version': LIVEKIT_API_VERSION,
        'timestamp': int(time.time()),
        'isolated_env': str(TEST_ENV),
    }
    STATUS_FILE.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf8')


def _run(args):
    return subprocess.run([str(x) for x in args], check=False).returncode


def test_install():
    print('\n=== WASTE ROBOT - THU CAI LIVEKIT (KHONG CHAY ROBOT) ===', flush=True)
    print('Python:', sys.version.split()[0], '\nKien truc:', platform.machine(), flush=True)
    print('glibc:', glibc_version() or 'khong xac dinh', flush=True)
    compatibility = supported_binary_platform()
    if compatibility is False:
        print('CANH BAO: glibc <2.28; LiveKit RTC 1.1.21 khong co wheel phu hop.', flush=True)
        print('Se thu pip voi --only-binary=:all: de bao cao loi thuc te; khong build Rust/C++.', flush=True)
    elif compatibility is True:
        print('He dieu hanh du dieu kien glibc cho wheel LiveKit; van can kiem tra import.', flush=True)
    else:
        print('Chua biet tinh tuong thich glibc; se thu wheel, khong build tu source.', flush=True)

    TEST_ENV.parent.mkdir(parents=True, exist_ok=True)
    interpreter = TEST_ENV / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
    if not interpreter.is_file():
        print('Tao venv RIENG:', TEST_ENV, flush=True)
        code = _run([sys.executable, '-m', 'venv', TEST_ENV])
        if code:
            record_status('venv_failed', 'Không tạo được môi trường thử nghiệm, exit {}'.format(code))
            return 2
    print('Thu cai LiveKit RTC {} + API {} (chi trong venv test)...'.format(
        LIVEKIT_VERSION, LIVEKIT_API_VERSION), flush=True)
    install_code = _run([
        interpreter, '-m', 'pip', 'install', '--disable-pip-version-check',
        '--no-input', '--only-binary=:all:', '--timeout', '25', '--retries', '1',
        'livekit=={}'.format(LIVEKIT_VERSION),
        'livekit-api=={}'.format(LIVEKIT_API_VERSION),
    ])
    if install_code:
        reason = ('Không có LiveKit RTC wheel tương thích với glibc <2.28' if compatibility is False
                  else 'pip không cài được LiveKit; kiểm tra lỗi pip/Internet ở trên')
        print('\nCHUA CAI DUOC: ' + reason, flush=True)
        print('Camera OpenCV, AUTO và phần mềm robot CHUA BI THAY DOI.', flush=True)
        record_status('install_failed', reason)
        return 2
    print('Cai thanh cong. Kiem tra nap thu vien rtc/api...', flush=True)
    probe = _run([interpreter, '-c',
        'from livekit import rtc, api; '
        'assert hasattr(rtc, "Room"); '
        'assert hasattr(api, "AccessToken"); '
        'print("LIVEKIT_RTC_IMPORT_OK + LIVEKIT_API_IMPORT_OK")'])
    if probe:
        print('CAI DUOC NHUNG KHONG IMPORT DUOC: khong bat video live.', flush=True)
        record_status('import_failed', 'Đã cài nhưng rtc/api import thất bại')
        return 3
    print('\nTHANH CONG: Thu vien LiveKit da cai va import trong venv RIENG.', flush=True)
    print('LUU Y: Chua ket noi room, chua phat video, chua bat LiveKit tren robot.', flush=True)
    print('Chay main.py binh thuong van dung camera OpenCV an toan.', flush=True)
    record_status('ready_isolated', 'LiveKit RTC/API import thành công trong venv thử nghiệm')
    return 0


if __name__ == '__main__':
    sys.exit(test_install())
