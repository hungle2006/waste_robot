"""COM3 checkpoint and endless SEARCH regression tests (simulation only)."""
import asyncio
from pathlib import Path
from unittest.mock import AsyncMock

import pytest
from auto_checkpoint import AutoCheckpoint
from auto_controller import VisionUnavailable
from test_box_tof_flow import prepared
from test_control import robot


def save(cp, *, stage='auto_search', carrying=False, joints=None, flow='front'):
    return cp.write(state=stage, phase='search', flow=flow,
                    holding_object=carrying, selected_bin=1, label='plastic bottle',
                    cycles=2, rear_cycles=1, drop_failures=0,
                    last_joints=joints)


def test_checkpoint_survives_reload_and_is_atomic(tmp_path):
    cp = AutoCheckpoint(tmp_path / 'robot_params.py')
    search = dict(p1=1300, p10=950, p11=1250, d2=2100, d3=600, p4=500)
    save(cp, joints=search)
    assert AutoCheckpoint(tmp_path / 'robot_params.py').load()['cycles'] == 2
    cp.suspend()
    assert cp.load()['status'] == 'waiting_com_reconnect'
    assert cp.load()['last_commanded_joints'] == search
    cp.clear()
    assert cp.load() is None


def test_recovery_gates_carried_bottle_or_nonsearch_arm(robot, tmp_path):
    b, _ = robot
    _, _, a, _ = prepared(robot, tmp_path)
    cp = AutoCheckpoint(tmp_path / 'params.py')
    a.checkpoint = cp
    search = dict(b.config['poses']['SEARCH'], p4=b.config['lid']['close'])
    save(cp, stage='auto_lift', carrying=True, joints=search)
    assert a.requires_reconnect_inspection(cp.load())
    save(cp, stage='auto_tof_observe', carrying=False, joints=dict(search, p1=2310))
    assert a.requires_reconnect_inspection(cp.load())
    save(cp, stage='auto_search', carrying=False, joints=search)
    assert not a.requires_reconnect_inspection(cp.load())
    save(cp, stage='auto_lower', carrying=False, joints=search)
    assert a.requires_reconnect_inspection(cp.load())  # servo command might be in flight
    save(cp, stage='auto_search', carrying=False, joints={})
    assert a.requires_reconnect_inspection(cp.load())


def test_reconnect_user_click_no_arm_reset(tmp_path):
    from brain import Brain
    from transport import SimTransport
    from robot_params import ROBOT_CONFIG
    hw = SimTransport()
    b = Brain(hw, Path(__file__).resolve().parents[1] / 'python' / 'robot_params.py', simulation=True)
    b.connect(skip_setup=True)
    assert not b.busy
    assert b.connected and not b.stopped
    # Opening and connecting never sends SEARCH servo pulses when recovery
    # requires operator inspection.
    assert not any(cmd.startswith('P 1 ') or cmd.startswith('P 10 ') for cmd in hw.commands)


def test_search_keeps_rotating_beyond_30_without_stopping(robot, tmp_path, monkeypatch):
    async def go():
        _, _, a, _ = prepared(robot, tmp_path)
        a.search_rotations = 0
        a.motor_step = AsyncMock()
        a.wait = AsyncMock()
        monkeypatch.setattr('robot_params.AUTO_SEARCH_TURN_SECONDS', 1.12)
        monkeypatch.setattr('robot_params.AUTO_SEARCH_STOP_SECONDS', 1.0)
        for _ in range(33):
            await a.search_turn_right()
        assert a.search_rotations == 33
        assert a.motor_step.await_count == 66
        assert a.state['scan_group'] == 2
        assert a.state['scan_turn'] == 3
    asyncio.run(go())


def test_camera_offline_does_not_hold_or_rotate_blind(robot, tmp_path):
    async def go():
        _, _, a, _ = prepared(robot, tmp_path)
        a.locked_bin = a.locked_label = None
        a._detect_once = AsyncMock(side_effect=[VisionUnavailable('Camera not ready')]*4 + [asyncio.CancelledError()])
        a.search_turn_right = AsyncMock()
        a.wait = AsyncMock()
        with pytest.raises(asyncio.CancelledError):
            await a.detect()
        assert a.state['state'] == 'auto_acquire'
        assert a.search_turn_right.await_count == 0
    asyncio.run(go())


def test_reconnect_blocks_carrying_without_replaying_release(robot, tmp_path):
    async def go():
        b, hw, a, _ = prepared(robot, tmp_path)
        cp = AutoCheckpoint(tmp_path / 'params.py')
        a.checkpoint = cp
        save(cp, stage='auto_place', carrying=True, joints=dict(b.config['poses']['SEARCH'], p4=500))
        a.start = AsyncMock()
        # Safe to inspect without MOVING the gripper.
        commands_before = len(hw.commands)
        outcome = await a.resume_after_connect()
        assert outcome is False
        assert a.state['state'] == 'auto_reconnect_inspection'
        a.start.assert_not_called()
        assert all(not c.startswith(('P ', 'S ')) for c in hw.commands[commands_before:])
    asyncio.run(go())
