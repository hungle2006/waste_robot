"""V3.16.27: D18 priority, MANUAL, conditional carry deposit, AUTO backtrack."""
import asyncio
import time
from unittest.mock import AsyncMock, Mock
import pytest
from test_control import robot, ready
from auto_controller import AutoController, NearHold, RearRequested, load_auto_config
from test_rear import RearVision, rear_settings


def test_manual_drive_allowed_when_d18_armed_but_pending_blocks(robot):
    b, hw = robot
    epoch=ready(b)
    b.enable_rear(True)
    # Even with D18 supervision active, regular MANUAL movement is allowed.
    b.command('DRIVE FORWARD',epoch)
    b.command('XM',b.epoch)
    b._observe_rear(0)
    assert b.rear_requested()
    with pytest.raises(ValueError,match='D18'):
        b.command('GRIP OPEN',b.epoch)
    # Explicit opt-out clears pending, restoring control.
    b.command('AUTO REAR OFF',b.epoch)
    assert not b.rear_requested()
    assert b.rear_manual_opt_out


def test_manual_d18_auto_arms_and_priority_blocks_unknown_grip(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b)
        p=tmp_path/'auto.json';rear_settings(b,p)
        a=AutoController(b,RearVision(b,hw),p,AsyncMock())
        # With no AUTO running, D18 arms automatically in MANUAL.
        await a.service_rear()
        assert b.auto_rear
        # Manual gripper closes without known BIN; do not open/release blindly.
        b.command('GRIP CLOSE', b.epoch)
        while b.busy:await asyncio.sleep(.02)
        hw.state['sensors']['D18']=0
        b.last_hw=b.hw.request('STATUS')
        b._observe_rear(0)
        await a.service_rear()
        assert b.rear_pending and not a.running
        assert a.state['state']=='rear_manual_inspection'
        assert hw.state['joints']['d3']==b.config['grip']['close']
    asyncio.run(run())


def test_auto_while_carrying_defers_d18_until_after_release(robot,tmp_path):
    b,hw=robot;ready(b)
    p=tmp_path/'auto.json';rear_settings(b,p)
    a=AutoController(b,RearVision(b,hw),p,AsyncMock())
    b.enable_rear(True);b._observe_rear(0)
    a.flow='front'
    a.holding_object=True
    assert a.check_rear() is None
    assert b.rear_requested()
    a.holding_object=False
    with pytest.raises(RearRequested):a.check_rear()


def test_post_rear_backtracks_observed_steps_then_found(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor()
        p=tmp_path/'auto.json';rear_settings(b,p)
        a=AutoController(b,RearVision(b,hw),p,AsyncMock())
        a.cfg=load_auto_config(p,b.config);a.token='test-rear-back';b.claim_auto(a.token)
        a.deadline=time.monotonic()+50;a.flow='rear'
        a.fresh_hardware=AsyncMock()
        a._detect_once=AsyncMock(side_effect=[None,None,{'label':'bottle','bin':1}])
        a.motor_step=AsyncMock()
        a.wait=AsyncMock()
        a.emit=AsyncMock()
        b.set_auto_ir_drive_guard=Mock()
        assert await a.post_rear_front_search() is True
        assert a.motor_step.await_count==2
        assert all(c.kwargs['duration_ms']==400 and c.kwargs['settle_ms']==2000 for c in a.motor_step.await_args_list)
        assert a.flow=='front'
    asyncio.run(run())


def test_post_rear_after_ten_empty_images_requests_right_scan(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor()
        p=tmp_path/'auto.json';rear_settings(b,p)
        a=AutoController(b,RearVision(b,hw),p,AsyncMock())
        a.cfg=load_auto_config(p,b.config);a.token='test-empty';b.claim_auto(a.token)
        a.deadline=time.monotonic()+50;a.flow='rear'
        a.fresh_hardware=AsyncMock();a._detect_once=AsyncMock(return_value=None)
        a.motor_step=AsyncMock();a.wait=AsyncMock();a.emit=AsyncMock()
        b.set_auto_ir_drive_guard=Mock()
        assert await a.post_rear_front_search() is False
        assert a.motor_step.await_count == 10
        assert a.drop_search_pending is True
    asyncio.run(run())


def test_manual_deposit_never_opens_unknown_bin(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor()
        p=tmp_path/'auto.json';rear_settings(b,p)
        a=AutoController(b,RearVision(b,hw),p,AsyncMock())
        a.cfg=load_auto_config(p,b.config);a.token='test-deposit';b.claim_auto(a.token)
        a.deadline=time.monotonic()+50
        n=len(hw.commands)
        with pytest.raises(NearHold,match='BIN'):
            await a.deposit_manual_carry_before_rear(None)
        assert not any(x=='S 3 600' for x in hw.commands[n:])
    asyncio.run(run())


def test_manual_known_grip_is_deposited_before_rear_observation(robot,tmp_path):
    async def run():
        from test_auto import finish
        b, hw = robot
        ready(b);b.start_monitor()
        p=tmp_path/'auto.json';rear_settings(b,p)
        a=AutoController(b,RearVision(b,hw),p,AsyncMock())
        a.operator.held_bin=1
        b.command('GRIP CLOSE',b.epoch)
        while b.busy:await asyncio.sleep(.02)
        hw.state['sensors']['D19']=0
        hw.state['sensors']['D18']=0
        b.enable_rear(True)
        b.last_hw=b.hw.request('STATUS')
        b._observe_rear(0)
        release_log_start=len(hw.commands)
        async def sensors():
            released=False
            while True:
                if 'S 3 600' in hw.commands[release_log_start:] and not released:
                    hw.state['sensors']['D19']=1;released=True
                if a.state.get('state')=='rear_wait_drop':
                    hw.state['sensors']['D18']=1
                    return
                await asyncio.sleep(.01)
        watcher=asyncio.create_task(sensors())
        await a.service_rear()
        try:
            await asyncio.wait_for(finish(a,timeout=35),timeout=37)
            await asyncio.wait_for(watcher,timeout=2)
        except Exception:
            print('DEBUG',a.state,'BRAIN',b.job,'TAIL',hw.commands[-35:])
            watcher.cancel()
            await a.stop()
            raise
        assert a.state['rear_cycles']==1,a.state
        assert a.operator.held_bin is None
        log=hw.commands
        gripclose=log.index('S 3 2300')
        release=log.index('S 3 600',gripclose)
        rear=log.index('S 2 500',release)
        assert gripclose < release < rear
        assert not b.stopped
    asyncio.run(run())


def test_v31634_manual_observer_keeps_control_when_d18_detects(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b)
        p=tmp_path/'auto.json';rear_settings(b,p)
        a=AutoController(b,RearVision(b,hw),p,AsyncMock())
        await a.operator.set_mechanics(False)
        await a.operator.start_observing()
        assert a.running and a.state['phase']=='observe'
        hw.state['sensors']['D18']=0
        b._observe_rear(0)
        a.start=AsyncMock()
        await a.service_rear()
        assert not a._resume_observing_after_rear
        assert a.running
        assert not b.rear_pending
        a.start.assert_not_called()
        await a.stop()
    asyncio.run(run())
