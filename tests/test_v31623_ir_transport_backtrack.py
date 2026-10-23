"""V3.16.23: IR D19 is a grip/carry sensor, NOT a search motor stop."""
import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock
import pytest
from auto_controller import LostGrip
from test_box_tof_flow import prepared, raw
from test_control import robot


def test_ir_drive_guard_is_stage_specific(robot,tmp_path):
    brain, hw, a, _ = prepared(robot,tmp_path,[None])
    # D19 (gripper IR) must NOT stop forward during SEARCH/approach.
    hw.state['sensors']['D19'] = 0
    brain.set_auto_ir_drive_guard(False,a.token)
    assert brain.snapshot()['hardware']['ir_drive_guard'] is False
    assert brain.smooth_auto_drive('FORWARD',brain.epoch,20,400,160,a.token)>0
    brain.command('XM',brain.epoch,_owner=a.token)
    brain.set_auto_ir_drive_guard(True,a.token)
    with pytest.raises(ValueError,match='IR_BLOCKS_AUTO_DRIVE'):
        brain.smooth_auto_drive('FORWARD',brain.epoch,20,400,160,a.token)
    hw.state['sensors']['D19'] = 1
    hw.state['sensors']['D18'] = 0
    brain.set_auto_ir_drive_guard(False,a.token)
    with pytest.raises(ValueError,match='IR_BLOCKS_AUTO_DRIVE'):
        brain.smooth_auto_drive('FORWARD',brain.epoch,20,400,160,a.token)


def test_loss_waits_full_15_seconds_then_requests_recovery(robot,tmp_path,monkeypatch):
    import auto_controller
    clock=[0.]
    monkeypatch.setattr(auto_controller,'time',SimpleNamespace(monotonic=lambda:clock[0]))
    async def go():
        _brain,_hw,a,_ = prepared(robot,tmp_path,[None])
        a.deadline=1000
        a.sensor=lambda name:False
        a.fresh_hardware=AsyncMock()
        a.emit=AsyncMock()
        async def advance(seconds):clock[0]+=seconds
        a.wait=advance
        with pytest.raises(LostGrip,match='15s'):
            await a.check_grip_or_drop('trong khi đưa vào thùng')
        assert clock[0]>=15
        assert any(c.args[0]=='auto_drop_verify' for c in a.emit.await_args_list)
    asyncio.run(go())


def test_d19_restored_during_check_continues_without_reversing(robot,tmp_path,monkeypatch):
    import auto_controller
    clock=[0.]
    monkeypatch.setattr(auto_controller,'time',SimpleNamespace(monotonic=lambda:clock[0]))
    async def go():
        _brain,_hw,a,_=prepared(robot,tmp_path,[None])
        a.deadline=1000
        a.sensor=lambda pin:clock[0]>=3.0
        a.fresh_hardware=AsyncMock()
        a.emit=AsyncMock()
        async def advance(seconds):clock[0]+=seconds
        a.wait=advance
        await a.check_grip_or_drop('đang nâng')
        assert 3.3<=clock[0]<15
        assert any(c.args[0]=='auto_drop_signal_restored' for c in a.emit.await_args_list)
    asyncio.run(go())


def test_reverse_400ms_2s_and_search_stop_at_first_seen(robot,tmp_path):
    async def go():
        _b,_hw,a,_=prepared(robot,tmp_path,[None,None,raw()])
        a.sensor=lambda pin:False
        a.motor_step=AsyncMock()
        a.emit=AsyncMock()
        assert await a.reverse_search_after_drop() is True
        assert a.motor_step.await_count==2
        assert all(k.args==('BACK',) and k.kwargs['duration_ms']==400
                   and k.kwargs['settle_ms']==2000 for k in a.motor_step.await_args_list)
        assert a.cfg['drop_recovery']['reverse_max_steps']==10
    asyncio.run(go())


def test_pause_servo_job_on_loss_retains_auto_owner(robot,tmp_path):
    import time
    b, hw, a, _=prepared(robot,tmp_path,[None])
    b.auto_targets({'p10':1200},'test holding servo',a.token,step_us=10,step_ms=40)
    assert b.busy
    b.pause_auto_arm_for_drop(a.token)
    assert not b.busy
    assert not b.stopped
    assert b.auto_owner==a.token
    assert hw.state['joints']['p10']<1200  # Partial movement was cancelled.
