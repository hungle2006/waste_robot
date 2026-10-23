"""V3.16.34 D18 does not preempt MANUAL wheels or arm, but AUTO still prioritizes it."""
import asyncio
from pathlib import Path
from unittest.mock import AsyncMock
from test_control import robot, ready, wait_for
from auto_controller import AutoController
from test_rear import RearVision, rear_settings


def test_manual_red_allows_drive_and_servo_with_d18_low(robot,tmp_path):
    async def test():
        b, hw = robot
        ready(b)
        p = tmp_path/'auto.json'; rear_settings(b,p)
        a = AutoController(b,RearVision(b,hw),p,AsyncMock())
        await a.operator.set_mechanics(False)
        assert b.manual_mechanics_mode and not b.auto_rear
        # Sensor remains observable; no automatic XM or rear task is created.
        hw.state['sensors']['D18']=0
        b.last_hw=b.hw.request('STATUS')
        n=len(hw.commands)
        b._observe_rear(0)
        await a.service_rear()
        assert not b.rear_pending and not b.rear_handling
        assert 'XM' not in hw.commands[n:]
        state=b.snapshot()
        assert state['manual_mechanics_mode'] is True
        assert state['hardware']['sensors']['D18']==0
        b.command('DRIVE FORWARD',b.epoch,speed=20)
        assert any(hw.state['motors'])
        # Polling D18 while driving must not cancel the manual wheel lease.
        n=len(hw.commands)
        b._observe_rear(0)
        await a.service_rear()
        assert 'XM' not in hw.commands[n:]
        assert any(hw.state['motors'])
        b.command('XM',b.epoch)
        b.command('GRIP OPEN',b.epoch)
        wait_for(lambda:not b.busy)
        assert hw.state['joints']['d3']==b.config['grip']['open']
    asyncio.run(test())


def test_return_to_auto_restores_d18_priority(robot,tmp_path):
    async def test():
        b,hw=robot;ready(b)
        p=tmp_path/'auto.json';rear_settings(b,p)
        a=AutoController(b,RearVision(b,hw),p,AsyncMock())
        await a.operator.set_mechanics(False)
        await a.operator.set_mechanics(True)
        assert not b.manual_mechanics_mode
        await a.service_rear()
        assert b.auto_rear
        b._observe_rear(0)
        assert b.rear_requested()
        try:
            b.command('DRIVE FORWARD',b.epoch)
        except ValueError as ex:
            assert 'D18' in str(ex)
        else:
            raise AssertionError('AUTO rear priority must not allow manual drive')
    asyncio.run(test())


def test_firmware_manual_c_drive_does_not_gate_d18_but_smooth_auto_does():
    text=(Path(__file__).resolve().parents[1]/'arduino/robot_control_v3/robot_control_v3.ino').read_text()
    manual=text.split('if(cmd[0]==\'C\'||cmd[0]==\'D\'){',1)[1].split('return;',1)[0]
    assert 'stable[2]==LOW' not in manual
    smooth=text.split('void updateSmooth(unsigned long now){',1)[1].split('void handle(char *line){',1)[0]
    assert 'stable[2]==LOW' in smooth
    assert 'if(tofBlocksDrive(now))' in smooth


def test_control_ui_d18_does_not_disable_manual_drive():
    ui=(Path(__file__).resolve().parents[1]/'web/robot-control.html').read_text()
    assert 'next.auto_rear&&!next.manual_mechanics_mode' in ui
    assert 'state.auto_rear&&!state.manual_mechanics_mode' in ui
