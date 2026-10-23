"""V3.16.19 regression: bounded, camera-verified reverse search after a drop."""
import asyncio
from unittest.mock import AsyncMock

import pytest

from auto_controller import NearHold, AutoFault, drop_recovery_config
from test_box_tof_flow import prepared, raw
from test_control import robot


def _ready_drop(a):
    a.target = None
    a.locked_label = a.locked_bin = None
    a.holding_object = False
    a.vision_paused = False
    a.sensor = lambda pin: False
    a.motor_step = AsyncMock()


def test_drop_reverse_stops_as_soon_as_camera_finds_chai(robot,tmp_path):
    async def run():
        _b, _hw, a, _c = prepared(robot,tmp_path,[None,None,raw()])
        _ready_drop(a)
        a.emit = AsyncMock()
        found=await a.reverse_search_after_drop()
        assert found is True
        assert a.motor_step.await_count == 2
        assert a.vision.calls == 3
        assert all(call.args == ('BACK',) and call.kwargs['duration_ms'] == a.cfg['control']['move_ms']
                   for call in a.motor_step.await_args_list)
        assert all(call.kwargs['settle_ms'] == a.cfg['drop_recovery']['reverse_settle_ms']
                   for call in a.motor_step.await_args_list)
        assert a.target is None  # Fresh classification and orientation in next cycle.
        assert any(call.args[0] == 'auto_drop_target_found' for call in a.emit.await_args_list)
    asyncio.run(run())


def test_drop_reverse_ten_empty_frames_handoffs_to_existing_scan(robot,tmp_path):
    async def run():
        _b, _hw, a, _c = prepared(robot,tmp_path,[None]*11)
        _ready_drop(a)
        a.emit = AsyncMock()
        found = await a.reverse_search_after_drop()
        assert found is False
        assert a.motor_step.await_count == 10
        assert a.vision.calls == 11  # look at SEARCH + look after each BACK
        assert [call.args[0] for call in a.motor_step.await_args_list] == ['BACK']*10
        assert any(call.args[0] == 'auto_drop_reverse_exhausted' for call in a.emit.await_args_list)
        a.drop_search_pending = not found
        a.move_targets = AsyncMock()
        a.sensor = lambda name: False
        from box_tof_flow import BoxToFFlow
        original = BoxToFFlow.run
        async def halt_before_grasp(flow):
            assert a.search_rotations == 1
            assert not a.drop_search_pending
            raise NearHold('test scan transition')
        BoxToFFlow.run = halt_before_grasp
        a.search_turn_right = AsyncMock(side_effect=lambda: setattr(a,'search_rotations', a.search_rotations+1))
        try:
            with pytest.raises(NearHold,match='test scan transition'):
                await a.cycle()
        finally:
            BoxToFFlow.run = original
        a.search_turn_right.assert_awaited_once()
    asyncio.run(run())


def test_drop_reverse_finds_target_before_first_back(robot,tmp_path):
    async def run():
        _b, _hw, a, _c = prepared(robot,tmp_path,[raw()])
        _ready_drop(a)
        assert await a.reverse_search_after_drop() is True
        a.motor_step.assert_not_awaited()
    asyncio.run(run())


def test_drop_reverse_rejects_d19_contact_before_any_motion(robot,tmp_path):
    async def run():
        _b, _hw, a, _c = prepared(robot,tmp_path,[None])
        _ready_drop(a)
        a.sensor=lambda pin: True
        with pytest.raises(NearHold, match='D19'):
            await a.reverse_search_after_drop()
        a.motor_step.assert_not_awaited()
    asyncio.run(run())


def test_drop_reverse_camera_fails_three_times_no_blind_move(robot,tmp_path):
    from auto_controller import VisionUnavailable
    async def run():
        _b, _hw, a, _c = prepared(robot,tmp_path,[None])
        _ready_drop(a)
        a._detect_once=AsyncMock(side_effect=VisionUnavailable('Modal offline'))
        a.wait=AsyncMock()
        with pytest.raises(NearHold,match='lỗi camera/AI 3 lần'):
            await a.reverse_search_after_drop()
        assert a._detect_once.await_count == 3
        a.motor_step.assert_not_awaited()
    asyncio.run(run())


def test_drop_retry_holds_after_second_loss_without_backtracking(robot,tmp_path):
    async def run():
        _b, _hw, a, _c = prepared(robot,tmp_path)
        a.recover_lost_grip=AsyncMock()
        a.reverse_search_after_drop=AsyncMock(return_value=True)
        a.wait=AsyncMock()
        await a.handle_lost_grip('drop1')
        assert a.reverse_search_after_drop.await_count == 1
        assert not a.drop_search_pending
        with pytest.raises(NearHold, match='2 lần'):
            await a.handle_lost_grip('drop2')
        assert a.reverse_search_after_drop.await_count == 1
    asyncio.run(run())


def test_reverse_configuration_and_forward_independence(robot,tmp_path):
    b,_hw,a,_c=prepared(robot,tmp_path)
    assert a.cfg['drop_recovery']['reverse_max_steps']==10
    assert a.cfg['drop_recovery']['reverse_settle_ms']==2000
    assert a.cfg['control']['move_ms']==400
    assert a.cfg['pose_profile']['stages'][0]['forward_ms']==400
    assert a.cfg['box_tof_flow']['max_forward_steps'] is None
    assert drop_recovery_config({})['reverse_max_steps']==10
    with pytest.raises(AutoFault): drop_recovery_config({'reverse_max_steps':11})
    with pytest.raises(AutoFault): drop_recovery_config({'reverse_max_steps':True})
