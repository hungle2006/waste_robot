"""Bootstrap a self-contained Jetson Nano Python runtime before any external import.

Designed to be parseable by the JetPack 4 system Python 3.6. This code never
upgrades system Python, JetPack, CUDA, or apt libraries. First run needs Internet.
"""
import hashlib
import importlib.util
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import urllib.request

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
REQUIREMENTS = HERE / 'requirements-jetson.txt'
MINIFORGE_URL = 'https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh'
ENV_FOLDER = Path(os.environ.get('WASTE_ROBOT_ENV_HOME', str(Path.home() / '.local' / 'share' / 'waste-robot')))
NEEDED = ('httpx', 'dotenv', 'supabase', 'realtime', 'fastapi', 'uvicorn', 'serial', 'cv2', 'numpy', 'PIL')


def is_jetson():
    if os.environ.get('ROBOT_FORCE_JETSON') == '1':
        return True
    if sys.platform != 'linux' or platform.machine().lower() not in ('aarch64', 'arm64'):
        return False
    if Path('/etc/nv_tegra_release').exists():
        return True
    try:
        return 'jetson' in Path('/proc/device-tree/model').read_bytes().lower().decode('utf8', 'ignore')
    except OSError:
        return False


def _log(message):
    print('[Jetson Setup] ' + message, flush=True)


def _run(command, **kwargs):
    _log('Thực hiện: ' + ' '.join(str(x) for x in command[:4]))
    subprocess.run([str(x) for x in command], check=True, **kwargs)


def _python_ok(executable):
    try:
        result = subprocess.run([str(executable), '-c',
            'import sys;assert (3,10)<=sys.version_info[:2]<(3,14)'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
        return result.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def _imports_ok(executable):
    code = 'import ' + ','.join(NEEDED) + '\nimport cv2;assert hasattr(cv2,"grabCut")'
    try:
        result = subprocess.run([str(executable), '-c', code],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=45)
        return result.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def _requirements_hash():
    return hashlib.sha256(REQUIREMENTS.read_bytes()).hexdigest()


def _download_miniforge(destination):
    # Official conda-forge distribution; HTTPS. No third-party mirror is used.
    _log('Tải Miniforge ARM64 chính thức (chỉ lần cài đầu)...')
    request = urllib.request.Request(MINIFORGE_URL, headers={'User-Agent': 'WasteRobot-Jetson-Setup/1.0'})
    with urllib.request.urlopen(request, timeout=120) as response, open(str(destination), 'wb') as handle:
        while True:
            block = response.read(1024 * 1024)
            if not block:
                break
            handle.write(block)
    if destination.stat().st_size < 1000000:
        raise RuntimeError('File cài Miniforge tải về quá nhỏ; kiểm tra kết nối Internet')


def _existing_venv():
    """Reuse a working Jetson .venv (including already installed LiveKit)."""
    if not _python_ok(sys.executable):
        return None
    if sys.prefix == getattr(sys, 'base_prefix', sys.prefix) and not os.environ.get('VIRTUAL_ENV'):
        return None
    if _imports_ok(sys.executable):
        _log('Dùng môi trường .venv hiện có, giữ nguyên LiveKit đã cài: ' + sys.executable)
        return Path(sys.executable)
    _log('Môi trường .venv hiện có thiếu phụ thuộc robot; thử cài gói Python của dự án...')
    try:
        _run([sys.executable, '-m', 'pip', 'install', '--disable-pip-version-check',
              '--prefer-binary', '-r', REQUIREMENTS])
    except subprocess.CalledProcessError:
        _log('Không hoàn thành gói phụ thuộc trong .venv; dùng môi trường Jetson riêng.')
        return None
    if _imports_ok(sys.executable):
        return Path(sys.executable)
    _log('Thiếu OpenCV/NumPy hoặc thư viện native; chuyển sang Miniforge thay vì thay JetPack.')
    return None


def _ensure_optional_livekit(python):
    """Best-effort RTC support in the actual runtime. Never block AUTO startup."""
    mode = os.environ.get('LIVEKIT_PUBLISHER', 'auto').strip().lower()
    if mode in ('0', 'off', 'false', 'no'):
        return
    probe = 'from livekit import rtc,api; assert hasattr(rtc,"Room") and hasattr(api,"AccessToken")'
    try:
        rc = subprocess.run([str(python), '-c', probe], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, timeout=20).returncode
        if rc == 0:
            _log('LiveKit RTC/API đã có trong đúng môi trường chạy main.py.')
            return
    except (OSError, subprocess.SubprocessError):
        pass
    # Published ARM64 LiveKit wheel needs glibc 2.28+, independent of Python.
    try:
        libc = platform.libc_ver()
        glibc = tuple(int(x) for x in libc[1].split('.')[:2])
    except (ValueError, IndexError):
        glibc = (0, 0)
    if platform.system() == 'Linux' and glibc and glibc < (2, 28):
        _log('glibc <2.28: không tự cài LiveKit RTC mới; OpenCV vẫn cấp camera cho AUTO.')
        return
    _log('Cài tùy chọn LiveKit RTC/API vào đúng Python đang chạy, không đụng JetPack...')
    try:
        _run([python, '-m', 'pip', 'install', '--disable-pip-version-check',
              '--only-binary=:all:', 'livekit==1.1.21', 'livekit-api==1.2.2'])
    except subprocess.CalledProcessError:
        _log('Không cài được LiveKit; giữ OpenCV cho AUTO, không dừng chương trình.')
        return
    try:
        if subprocess.run([str(python), '-c', probe], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL, timeout=20).returncode != 0:
            _log('LiveKit đã cài nhưng không import được RTC/API; dùng OpenCV.')
    except (OSError, subprocess.SubprocessError):
        _log('LiveKit không import được; dùng OpenCV.')


def _ensure_jetson_env():
    active = _existing_venv()
    if active:
        _ensure_optional_livekit(active)
        return active
    base = ENV_FOLDER
    conda_root = base / 'miniforge3'
    env = base / 'python310'
    python = env / 'bin' / 'python'
    stamp = env / '.waste_robot_install_stamp'
    base.mkdir(parents=True, exist_ok=True)
    if not _python_ok(python):
        conda = conda_root / 'bin' / 'conda'
        if not conda.exists():
            if conda_root.exists():
                raise RuntimeError('Miniforge cài dở tại {}; xóa thư mục này rồi chạy lại'.format(conda_root))
            with tempfile.TemporaryDirectory(prefix='waste-robot-') as tmp:
                installer = Path(tmp) / 'Miniforge3-Linux-aarch64.sh'
                _download_miniforge(installer)
                _log('Cài môi trường Miniforge riêng trong thư mục người dùng...')
                _run(['bash', installer, '-b', '-p', conda_root])
        _log('Tạo Python 3.10 + OpenCV + NumPy dành cho ARM64 (chỉ lần đầu)...')
        _run([conda, 'create', '-y', '-p', env, 'python=3.10', 'pip', 'numpy<2', 'opencv', 'pillow'])
    if not _python_ok(python):
        raise RuntimeError('Không tạo được Python 3.10 trong môi trường Jetson')
    current = stamp.read_text().strip() if stamp.exists() else ''
    if current != _requirements_hash() or not _imports_ok(python):
        _log('Tự cài các thư viện còn thiếu: Supabase, FastAPI, HTTPX, Serial...')
        _run([python, '-m', 'pip', 'install', '--disable-pip-version-check', '--prefer-binary', '-r', REQUIREMENTS])
        if not _imports_ok(python):
            raise RuntimeError('Một số thư viện vẫn không import được. Kiểm tra log cài đặt phía trên.')
        stamp.write_text(_requirements_hash() + '\n')
    _ensure_optional_livekit(python)
    return python


def _ensure_other_env():
    # Developer Linux/Windows/macOS: create project-local venv, do not use global pip.
    if not ((3, 10) <= sys.version_info[:2] < (3, 14)):
        raise RuntimeError('Máy này cần Python từ 3.10 đến 3.13 để tự cài thư viện.')
    env = HERE / '.venv'
    python = env / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
    if not python.exists():
        _log('Tạo .venv Python riêng cho dự án...')
        _run([sys.executable, '-m', 'venv', env])
    stamp = env / '.waste_robot_install_stamp'
    if (not stamp.exists() or stamp.read_text().strip() != _requirements_hash() or
            not _imports_ok(python)):
        _run([python, '-m', 'pip', 'install', '--disable-pip-version-check', '--prefer-binary',
              '-r', HERE / 'requirements.txt', '-r', HERE / 'requirements-camera.txt'])
        if not _imports_ok(python):
            raise RuntimeError('Cài phụ thuộc không thành công; kiểm tra lỗi pip phía trên.')
        stamp.write_text(_requirements_hash() + '\n')
    return python


def ensure_and_relaunch():
    if os.environ.get('WASTE_ROBOT_SETUP_DONE') == '1':
        return
    if os.environ.get('WASTE_ROBOT_SKIP_SETUP') == '1':
        _log('Bỏ cài tự động theo WASTE_ROBOT_SKIP_SETUP=1')
        return
    jetson = is_jetson()
    try:
        python = _ensure_jetson_env() if jetson else _ensure_other_env()
    except Exception as exc:
        _log('Không cài đặt xong: {}: {}'.format(type(exc).__name__, exc))
        _log('Internet, dung lượng đĩa và quyền ghi HOME là các yêu cầu tối thiểu.')
        raise SystemExit(2)
    os.environ['WASTE_ROBOT_SETUP_DONE'] = '1'
    if jetson:
        # Modern LiveKit binary needs glibc >=2.28; Ubuntu 18.04 is 2.27.
        os.environ.setdefault('JETSON_LOCAL_CAMERA', '1')
        os.environ.setdefault('LIVEKIT_VIEWER', '0')
        os.environ.setdefault('LIVEKIT_PUBLISHER', 'auto')
    _log('Môi trường sẵn sàng: ' + str(python))
    if Path(sys.executable).resolve() == Path(python).resolve():
        return
    os.execv(str(python), [str(python), str(HERE / 'main.py')] + sys.argv[1:])
