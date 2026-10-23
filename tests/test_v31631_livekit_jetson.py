"""Pure unit tests; no hardware, network, or installed LiveKit required."""
import importlib
import os
import sys
from types import SimpleNamespace
import pytest
import numpy as np

from jetson_livekit_camera import LocalCameraHub, publish_settings, livekit_import_check


def test_hub_delivers_raw_frame_and_only_new_sequences():
    received = []
    hub = LocalCameraHub(lambda f: received.append(f.copy()))
    assert hub.snapshot() is None
    frame = np.zeros((12, 18, 3), dtype=np.uint8)
    hub.accept(frame)
    s, preserved = hub.snapshot()
    assert s == 1 and len(received) == 1
    assert hub.snapshot(after=s) is None
    frame[:] = 255
    assert int(preserved.sum()) == 0  # no alias with mutable capture memory
    hub.accept(frame)
    s2, arr2 = hub.snapshot(after=s)
    assert s2 == 2 and int(arr2.sum()) > 0


def test_hub_fallback_requires_no_livekit(monkeypatch):
    hub = LocalCameraHub(lambda f: None)
    hub.accept(np.zeros((10, 10, 3), dtype=np.uint8))
    assert hub.snapshot()[0] == 1


def test_publish_settings_off_no_secret(monkeypatch):
    monkeypatch.setenv('LIVEKIT_PUBLISHER', '0')
    assert publish_settings('robot_01', {'livekit': {'configured': True}})[0] is None


def test_publish_settings_requires_server_and_keys(monkeypatch):
    monkeypatch.setenv('LIVEKIT_PUBLISHER', 'auto')
    monkeypatch.delenv('LIVEKIT_URL', raising=False)
    assert publish_settings('robot_01', {'livekit': {'configured': True}})[0] is None
    monkeypatch.setenv('LIVEKIT_URL', 'wss://example.livekit.cloud')
    monkeypatch.setenv('LIVEKIT_API_KEY', 'sample')
    monkeypatch.setenv('LIVEKIT_API_SECRET', 'sensitive_test_secret')
    opts, reason = publish_settings('robot_01', {'livekit': {'configured': True, 'trackName': 'front_camera'}})
    assert opts['room'] == 'robot_01' and opts['track'] == 'front_camera'
    assert 'sensitive_test_secret' not in repr(opts) and 'sensitive_test_secret' not in reason


def test_wrong_room_not_allowed(monkeypatch):
    for k,v in {'LIVEKIT_PUBLISHER':'1','LIVEKIT_URL':'wss://example.test',
                'LIVEKIT_API_KEY':'key','LIVEKIT_API_SECRET':'secret','LIVEKIT_ROOM':'another-room'}.items():
        monkeypatch.setenv(k,v)
    opts, reason = publish_settings('robot_01', {'livekit': {'configured': True}})
    assert opts is None and 'robotId' in reason


def test_livekit_check_handles_missing_sdk():
    ok, reason = livekit_import_check()
    assert isinstance(ok,bool) and isinstance(reason,str)


def test_source_is_separate_from_capture():
    import inspect
    from jetson_livekit_camera import run_livekit_from_hub
    src = inspect.getsource(run_livekit_from_hub)
    assert 'VideoCapture' not in src and 'open_camera(' not in src
    assert 'hub.snapshot' in src
