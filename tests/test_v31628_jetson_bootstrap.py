"""Pure logic tests: no downloads, no package install, no Arduino movements."""
import asyncio
import os
from pathlib import Path
import sys
from unittest.mock import MagicMock

ROOT = Path(__file__).resolve().parents[1] / 'python'
sys.path.insert(0, str(ROOT))

import jetson_bootstrap as setup
from camera_publisher import run_local_capture


def test_jetsontarget_force_flag(monkeypatch):
    monkeypatch.setenv('ROBOT_FORCE_JETSON', '1')
    assert setup.is_jetson() is True


def test_relaunch_prepares_local_camera_env_without_network(monkeypatch):
    monkeypatch.delenv('WASTE_ROBOT_SETUP_DONE', raising=False)
    monkeypatch.delenv('JETSON_LOCAL_CAMERA', raising=False)
    monkeypatch.delenv('LIVEKIT_VIEWER', raising=False)
    monkeypatch.delenv('LIVEKIT_PUBLISHER', raising=False)
    monkeypatch.setattr(setup, 'is_jetson', lambda: True)
    monkeypatch.setattr(setup, '_ensure_jetson_env', lambda: Path('/tmp/waste-test-env/bin/python'))
    called = {}
    def fake_exec(executable, arguments):
        called.update(executable=executable, arguments=arguments)
    monkeypatch.setattr(setup.os, 'execv', fake_exec)
    setup.ensure_and_relaunch()
    assert called['executable'] == '/tmp/waste-test-env/bin/python'
    assert called['arguments'][1] == str(ROOT / 'main.py')
    assert os.environ['JETSON_LOCAL_CAMERA'] == '1'
    assert os.environ['LIVEKIT_VIEWER'] == '0'
    assert os.environ['LIVEKIT_PUBLISHER'] == 'auto'
    assert os.environ['WASTE_ROBOT_SETUP_DONE'] == '1'


def test_relaunch_skips_if_setup_done(monkeypatch):
    monkeypatch.setenv('WASTE_ROBOT_SETUP_DONE', '1')
    monkeypatch.setattr(setup, '_ensure_jetson_env', lambda: (_ for _ in ()).throw(AssertionError('unexpected install')))
    setup.ensure_and_relaunch()


def test_jetson_requirements_have_no_livekit_or_pip_opencv():
    body = (ROOT / 'requirements-jetson.txt').read_text()
    uncommented = '\n'.join(s for s in body.splitlines() if not s.strip().startswith('#'))
    assert 'supabase==2.32.0' in uncommented
    assert 'livekit' not in uncommented
    assert 'opencv-python' not in uncommented


def test_local_camera_updates_jpeg_without_importing_livekit(monkeypatch):
    import numpy as np
    import camera_publisher
    frame = np.zeros((360, 640, 3), dtype='uint8')
    class FakeCapture:
        def read(self):
            return True, frame
        def release(self):
            pass
    monkeypatch.setattr(camera_publisher, 'open_camera', lambda *args: FakeCapture())
    output = []
    async def run():
        task = asyncio.create_task(run_local_capture(lambda im: output.append(im.copy())))
        for _ in range(30):
            if output:
                break
            await asyncio.sleep(.01)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    asyncio.run(run())
    assert output and output[0].shape == (360, 640, 3)
