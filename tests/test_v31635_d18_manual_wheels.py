"""V3.16.35: real AUTO ownership handover + D18 LOW + cloud/manual wheel commands."""
import asyncio
from unittest.mock import AsyncMock
import pytest

from test_control import robot, ready
from test_rear import RearVision, rear_settings
from auto_controller import AutoController
from realtime_bridge import RealtimeService


async def auto_running_for_handover(a):
    """Use an inert AUTO task instead of moving simulated hardware in the test."""
    async def wait_until_cancelled(rear_only=False):
        await asyncio.Event().wait()
    a.run = wait_until_cancelled
    await a.start()
    assert a.running
    assert a.brain.auto_owner is not None


def test_auto_toggle_off_gives_manual_wheels_while_d18_is_low(robot,tmp_path):
    async def scenario():
        b, hw = robot
        ready(b)
        cfg=tmp_path/'auto.json';rear_settings(b,cfg)
        vision=RearVision(b, hw)
        vision.prefer_local=True
        a=AutoController(b,vision,cfg,AsyncMock())
        await auto_running_for_handover(a)
        assert b.auto_rear  # AUTO will handle rear D18 while physically in control.

        # The UI's red MECHANICS OFF may be used while AUTO is in progress.
        await a.operator.action({'command':'MECHANICS OFF','epoch':b.epoch})
        assert a.operator.mechanics_enabled is False
        assert b.manual_mechanics_mode is True
        assert b.auto_owner is None
        assert a.running and a.state.get('phase')=='observe'
        assert not b.stopped

        hw.state['sensors']['D18']=0
        b.last_hw=hw.request('STATUS')
        b._observe_rear(0)
        await a.service_rear()
        assert not b.rear_pending and not b.rear_handling
        snapshot=a.operator.snapshot()
        assert snapshot['manual_wheels_available'] is True
        assert snapshot['d18_detected'] is True
        assert snapshot['auto_running'] is True  # Observation continues; not a motor owner.
        assert b.snapshot()['auto_running'] is False  # Hardware has no AUTO owner.

        b.command('DRIVE FORWARD',b.epoch)
        assert any(x.startswith('C ') for x in hw.commands), hw.commands[-10:]
        assert any(hw.state['motors']), hw.state['motors']
        b.command('XM',b.epoch)
        assert hw.state['motors']==[0,0,0,0]
        assert not b.stopped
        await a.stop()
        assert b.stopped
        with pytest.raises(ValueError,match='dừng|Cho phép|Đang'):
            b.command('DRIVE FORWARD',b.epoch)
    asyncio.run(scenario())


def test_vercel_manual_mode_is_handover_not_stop(robot,tmp_path):
    async def scenario():
        b,hw=robot;ready(b)
        cfg=tmp_path/'auto.json';rear_settings(b,cfg)
        vision=RearVision(b,hw);vision.prefer_local=True
        a=AutoController(b,vision,cfg,AsyncMock())
        await auto_running_for_handover(a)
        service=RealtimeService(b,'robot-test',None,vision)
        service.auto=a
        await service.mode({'action':'manual'})
        assert not b.stopped and b.auto_owner is None and b.manual_mechanics_mode
        hw.state['sensors']['D18']=0
        b._observe_rear(0)
        b.command('DRIVE RIGHT',b.epoch)
        assert any(hw.state['motors'])
        b.command('XM',b.epoch)
        await a.stop()
    asyncio.run(scenario())


def test_manual_flag_never_overrides_a_real_stop(robot,tmp_path):
    async def scenario():
        b,hw=robot;ready(b)
        cfg=tmp_path/'auto.json';rear_settings(b,cfg)
        a=AutoController(b,RearVision(b,hw),cfg,AsyncMock())
        await a.operator.set_mechanics(False)
        b.stop()
        assert b.stopped
        assert a.operator.snapshot()['manual_wheels_available'] is False
        with pytest.raises(ValueError): b.command('DRIVE BACK',b.epoch)
    asyncio.run(scenario())
