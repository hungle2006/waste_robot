"""Display landmarks must track config without entering vision or moving hardware."""
import base64
import copy
import json
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock

import cv2
import numpy as np
import pytest

from camera_guides import CameraGuides, display_frame, guide_layout
from livekit_modal import LiveKitModal
from auto_controller import AutoController
from test_control import robot


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def settings(tmp_path):
    path = tmp_path / 'auto.json'
    c = json.loads((ROOT / 'python/auto_config.json').read_text())
    c['distance']['enabled'] = False  # These tests cover the legacy y2 overlay.
    path.write_text(json.dumps(c))
    return path, c


def test_landmarks_are_taken_from_config(settings):
    _, c = settings
    a = guide_layout(c)
    assert a['target'] == [.5, .5] and a['tolerance'] == [.03, .08]
    assert a['handoff'] == .54 and a['blur'] == .60
    assert a['base_x'] == .5 and a['base_tol'] == .03
    c['visibility'].update(trigger_bottom_norm=.57, blur_bottom_norm=.64)
    c['calibration']['relative_target_norm'] = [.48, .46]
    a = guide_layout(c)
    assert a['handoff'] == .57 and a['blur'] == .64 and a['target'] == [.48, .46]


@pytest.mark.parametrize('width,height', [(640, 480), (1280, 720), (320, 240), (16, 16)])
def test_display_copy_only_and_correct_size(settings, width, height):
    path, _ = settings
    raw = np.full((height, width, 3), 80, dtype=np.uint8)
    before = raw.copy()
    seen = []
    shown = display_frame(raw, lambda image: seen.append(image.copy()), CameraGuides(path))
    assert shown.shape == raw.shape
    assert np.array_equal(raw, before) and np.array_equal(seen[0], before)
    assert not np.shares_memory(raw, shown) and not np.array_equal(raw, shown)


def test_active_config_not_edited_file(settings):
    path, c = settings
    controller = SimpleNamespace(running=True, cfg=copy.deepcopy(c), state={})
    overlay = CameraGuides(path, controller)
    c['visibility']['trigger_bottom_norm'] = .58
    path.write_text(json.dumps(c))
    assert guide_layout(overlay.config())['handoff'] == .54
    controller.running = False
    assert guide_layout(overlay.config())['handoff'] == .58


def test_idle_reload_and_invalid_edit_does_not_show_stale_guides(settings, monkeypatch):
    path, c = settings
    now = [100.]
    monkeypatch.setattr('camera_guides.time.monotonic', lambda: now[0])
    overlay = CameraGuides(path)
    assert guide_layout(overlay.config())['blur'] == .60
    c['visibility']['blur_bottom_norm'] = .65
    path.write_text(json.dumps(c)); now[0] += .6
    assert guide_layout(overlay.config())['blur'] == .65
    path.write_text('{broken'); now[0] += .6
    with pytest.raises(ValueError): overlay.config()
    with pytest.raises(ValueError): overlay.config()
    # A broken visual setting cannot kill the camera or contaminate the raw frame.
    raw = np.zeros((480, 640, 3), np.uint8)
    shown = overlay.render(raw)
    assert shown.shape == raw.shape and not raw.any()


def test_disabled_features_do_not_draw_inactive_landmarks(settings):
    _, c = settings
    c['visibility']['enabled'] = False
    assert guide_layout(c)['handoff'] == c['approach']['stop_bottom_norm']
    assert guide_layout(c)['blur'] is None
    c['approach']['enabled'] = False
    assert guide_layout(c)['handoff'] is None
    c['calibration']['mode'] = 'calibrated'
    assert guide_layout(c) == {'supported': False}


def test_rear_does_not_show_front_guides(settings):
    path, c = settings
    state = {'auto_running': True, 'flow': 'rear', 'state': 'rear_capture'}
    controller = SimpleNamespace(running=True, cfg=c, state=state)
    shown = CameraGuides(path, controller).render(np.full((480, 640, 3), 80, np.uint8))
    assert (shown[200:380] == 80).all()  # No target, handoff, blur or edge marks.


def test_preview_and_calibration_capture_stay_raw_and_no_hardware_write(robot, settings):
    from fastapi.testclient import TestClient
    from calibration_ui import calibration_app
    b, hw = robot
    path, _ = settings
    vision = LiveKitModal('https://app', 'https://modal')
    vision.accept_local_frame(np.full((480, 640, 3), 80, np.uint8))
    vision.frame_at = time.monotonic()
    original = vision.jpeg
    controller = AutoController(b, vision, path, AsyncMock())
    client = TestClient(calibration_app(b, vision, controller))
    # Wait for the fixture's initial SEARCH before taking the command baseline.
    from test_control import wait_for
    wait_for(lambda: not b.busy)
    commands = list(hw.commands)
    assert client.get('/api/preview').content == original
    shown = client.get('/api/preview?guides=true')
    assert shown.status_code == 200 and shown.headers['content-type'] == 'image/jpeg'
    assert shown.content != original
    assert vision.jpeg == original and hw.commands == commands
    assert 'showguides' in client.get('/').text
    assert client.get('/api/preview?guides=false').content == original
    # Emulate an already settled pose and a fresh frame for capture calibration.
    b.motion_finished_at = time.monotonic() - 1
    vision.frame_at = time.monotonic()
    capture = client.get('/api/capture')
    assert capture.status_code == 200
    assert base64.b64decode(capture.json()['image'].split(',', 1)[1]) == original
    assert hw.commands == commands
    vision.frame_at -= 3
    assert client.get('/api/preview?guides=true').status_code == 409


def test_modal_receives_only_original_jpeg(settings, monkeypatch):
    import asyncio
    import httpx
    path, _ = settings
    vision = LiveKitModal('https://app', 'https://modal')
    raw = np.full((480, 640, 3), 80, np.uint8)
    shown = display_frame(raw, vision.accept_local_frame, CameraGuides(path))
    original = vision.jpeg
    sent = []

    async def post(self, url, **kwargs):
        sent.append(kwargs['files']['image'][1])
        return httpx.Response(200, json={'found': False, 'objects': []}, request=httpx.Request('POST', url))

    monkeypatch.setattr(httpx.AsyncClient, 'post', post)
    asyncio.run(vision.detect())
    assert sent == [original]
    decoded = cv2.imdecode(np.frombuffer(original, np.uint8), cv2.IMREAD_COLOR)
    assert np.array_equal(decoded, raw) and not np.array_equal(shown, raw)


def test_invalid_coordinate_cannot_crash_display(settings):
    path, c = settings
    c['visibility']['blur_bottom_norm'] = float('nan')
    path.write_text(json.dumps(c))
    raw = np.zeros((480, 640, 3), np.uint8)
    shown = CameraGuides(path).render(raw)
    assert shown.shape == raw.shape and not raw.any()
