"""V3.16.17: SEARCH-view angle is latched before motion; D2 acts at VL53 gate."""
import asyncio
import copy
import time
from unittest.mock import AsyncMock

import pytest

import operator_control
from auto_controller import CloseRangeContact, NearHold
from box_tof_flow import BoxToFFlow
from grasp_orientation import pulse_for_angle
from test_box_tof_flow import prepared
from test_control import robot


def enable_search_wrist(a):
    a.cfg['grasp_orientation']['enabled'] = True
    a.cfg['grasp_orientation']['calibrated'] = True
    a.search_orientation = None


def search_memory(a, angle=20.):
    return dict(target_id=a.lock_sequence, bin=a.locked_bin,
                label=a.locked_label, source_pose=dict(a.cfg['arm_path']['start']),
                angle_deg=angle, mapped_deg=angle, pulse_us=pulse_for_angle(angle),
                samples=3, captured_at=time.monotonic(), applied=False)


def test_capture_three_new_search_frames_without_moving(robot, tmp_path, monkeypatch):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        enable_search_wrist(a)
        angles = iter([19., 20., 21.])
        monkeypatch.setattr(operator_control, 'estimate_axis',
                            lambda *args: {'angle_deg': next(angles)})
        base = copy.deepcopy(a.target)
        async def camera():
            a.observed_jpeg = b'JPEG-from-SEARCH'
            a.last_candidates = [copy.deepcopy(base)]
            return copy.deepcopy(base)
        a._detect_once = AsyncMock(side_effect=camera)
        a.emit = AsyncMock()
        old_commands = len(hw.commands)
        await a.operator.remember_search_orientation(base)
        memory = a.search_orientation
        assert a._detect_once.await_count == 3
        assert memory['target_id'] == a.lock_sequence
        assert memory['bin'] == 4 and memory['label'] == a.locked_label
        assert memory['pulse_us'] == pulse_for_angle(20.)
        assert memory['applied'] is False
        assert memory['source_pose'] == a.cfg['arm_path']['start']
        assert a.emit.await_args.args[0] == 'auto_orientation_saved'
        assert all(not cmd.startswith(('G ', 'P ', 'S ')) for cmd in hw.commands[old_commands:])
        assert hw.state['joints']['d2'] == 2100
    asyncio.run(run())


def test_capture_rejects_wrong_pose_or_disappearing_object(robot, tmp_path, monkeypatch):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        enable_search_wrist(a)
        monkeypatch.setattr(operator_control, 'estimate_axis',
                            lambda *args: {'angle_deg': 10.})
        hw.state['joints']['p1'] = 1550
        b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
        with pytest.raises(NearHold, match='SEARCH'):
            await a.operator.remember_search_orientation(a.target)
        assert a.search_orientation is None
        hw.state['joints']['p1'] = 1300
        b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
        a._detect_once = AsyncMock(return_value=None)
        with pytest.raises(NearHold, match='Mất/đổi vật'):
            await a.operator.remember_search_orientation(a.target)
        assert a.search_orientation is None
    asyncio.run(run())


def test_vl53_then_rotate_then_lower_with_angle_unchanged(robot, tmp_path):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        enable_search_wrist(a)
        a.search_orientation = search_memory(a, angle=-25.)
        pulse = a.search_orientation['pulse_us']
        f = BoxToFFlow(a)
        f.tof_confirmed = True
        f.tof_confirmed_at = time.monotonic()
        f.tof_pose = a.cfg['arm_path']['start']['p1']
        f.last_obj = copy.deepcopy(a.target)
        f.stage = 'tof_approach'
        a.vision_paused = True
        a.emit = AsyncMock()
        actions = []
        async def fake_move(targets, *_args, **_kwargs):
            actions.append(copy.deepcopy(targets))
            hw.state['joints'].update(targets)
            if hw.state['joints']['p1'] == 2310:
                hw.state['sensors']['D19'] = 0
            b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
        a.move_targets = fake_move
        hw.tof_sample(225)
        b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
        with pytest.raises(CloseRangeContact):
            await f.final_approach()
        assert actions and actions[0] == {'d2': pulse}
        assert len(actions) > 2
        assert all(action.get('d2') == pulse for action in actions[1:])
        assert actions[-1]['p1'] == 2310 and actions[-1]['p10'] == 650
        assert a.search_orientation['applied'] is True
        assert f.rotated_wrist_us == pulse
        assert a.final_authorized == {'target_id':a.lock_sequence,'bin':4,'mode':'box_tof'}
        assert a.vision.calls == 0  # NO new angle images at close-up pose
    asyncio.run(run())


def test_no_wrist_or_lower_without_distance_or_matching_search_memory(robot, tmp_path):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        enable_search_wrist(a)
        a.search_orientation = search_memory(a)
        a.vision_paused = True
        f = BoxToFFlow(a); f.tof_pose = 1300; f.stage = 'tof_approach'
        f.early_ir_at = time.monotonic(); f.last_obj = copy.deepcopy(a.target)
        a.move_targets = AsyncMock()
        with pytest.raises(NearHold, match='VL53 chưa xác nhận'):
            await f.final_approach()
        a.move_targets.assert_not_awaited()
        f.early_ir_at = None
        f.tof_confirmed = True; f.tof_confirmed_at = time.monotonic()
        hw.tof_sample(225)
        b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
        a.search_orientation['target_id'] += 1
        with pytest.raises(NearHold, match='đúng vật'):
            await f.final_approach()
        a.move_targets.assert_not_awaited()
    asyncio.run(run())


def test_invalid_search_axis_does_not_allow_handoff(robot, tmp_path, monkeypatch):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        enable_search_wrist(a)
        monkeypatch.setattr(operator_control, 'estimate_axis',
                            lambda *args: (_ for _ in ()).throw(ValueError('không thấy trục chai')))
        base = copy.deepcopy(a.target)
        async def sample():
            a.observed_jpeg = b'jpeg'; a.last_candidates = [base]
            return copy.deepcopy(base)
        a._detect_once = sample
        with pytest.raises(NearHold, match='không thấy trục chai'):
            await a.operator.remember_search_orientation(base)
        assert a.search_orientation is None
        assert hw.state['joints']['d2'] == 2100
    asyncio.run(run())
