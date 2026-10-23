"""AUTO continues finite pulses without ToF; only a valid window starts grasp."""
import asyncio
import copy
import time
from unittest.mock import AsyncMock

import pytest
from test_control import robot
from test_box_tof_flow import prepared
from arm_path import targets_at
from auto_controller import AutoFault, NearHold
from box_tof_flow import BoxToFFlow


def publish(b, hw, kind):
    hw.tof_sample(225, valid=False)
    if kind == 'missing':
        hw.state['tof'].update(present=False, mm=None, seq=0, age_ms=None)
    b.last_hw=hw.request('STATUS'); b.updated=time.monotonic()


@pytest.mark.parametrize('kind',['invalid','missing'])
def test_camera_forward_completes_pulse_without_valid_range(robot,tmp_path,kind):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        publish(b,hw,kind);b.start_monitor()
        f=BoxToFFlow(a);f.last_action='FORWARD';f.last_obj=copy.deepcopy(a.target)
        n=len(hw.commands);started=time.monotonic()
        await f.camera_forward(1300,200,0)
        assert time.monotonic()-started >= .2
        assert any(c.startswith('G ') for c in hw.commands[n:])
        assert not f.tof_confirmed and not hw.profile
        assert not any(c.startswith(('P ','S ')) for c in hw.commands[n:])
        assert not any(v.args[0]['state']=='auto_tof_retry' for v in a.report.call_args_list)
    asyncio.run(run())


@pytest.mark.parametrize('kind',['invalid','missing'])
def test_sensor_phase_moves_then_hands_off_on_first_valid_window(robot,tmp_path,kind):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],2050))
        publish(b,hw,kind);b.start_monitor()
        f=BoxToFFlow(a);f.last_obj=copy.deepcopy(a.target)
        f.final_approach=AsyncMock(return_value='grasp')
        async def distance_arrives():
            while not any(c.startswith('G ') for c in hw.commands):await asyncio.sleep(.01)
            assert not f.tof_confirmed
            await asyncio.sleep(.07)
            hw.tof_sample(225)
        later=asyncio.create_task(distance_arrives())
        try:
            assert await asyncio.wait_for(f.sensor_approach(),3)=='grasp'
            await later
            assert f.tof_confirmed and a.state['distance_ready']
            f.final_approach.assert_awaited_once()
            assert not hw.profile and not any(hw.state['motors'])
        finally:
            later.cancel();await asyncio.gather(later,return_exceptions=True)
    asyncio.run(run())


@pytest.mark.parametrize('reading',['invalid','unchanged_far'])
def test_no_ready_range_never_authorizes_grasp_and_pulse_limit_still_applies(robot,tmp_path,reading):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],2050))
        publish(b,hw,'invalid')
        if reading=='unchanged_far':
            hw.tof_sample(800);hw.tof_at=time.monotonic()-5
            b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        b.start_monitor()
        f=BoxToFFlow(a);f.last_obj=copy.deepcopy(a.target)
        f.c['max_tof_pulses']=2;f.final_approach=AsyncMock()
        with pytest.raises(NearHold,match='Hết số nhịp'):
            await f.sensor_approach()
        assert sum(c.startswith('G ') for c in hw.commands)==2
        f.final_approach.assert_not_awaited();assert not f.tof_confirmed
    asyncio.run(run())


def test_old_firmware_cannot_silently_use_new_auto_policy(robot,tmp_path):
    b,hw,a,_=prepared(robot,tmp_path)
    hw.state.pop('tof_auto_continue');b.last_hw=hw.request('STATUS')
    with pytest.raises(ValueError,match='V3.16.4'):
        b.tof_guard(True,a.token,ignore_invalid=True)
    assert not any(c.startswith('TF AUTO') for c in hw.commands)


def test_stop_during_invalid_range_drive_cannot_issue_late_motion(robot,tmp_path):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],2050))
        publish(b,hw,'invalid');b.start_monitor()
        f=BoxToFFlow(a);f.last_obj=copy.deepcopy(a.target)
        task=asyncio.create_task(f.sensor_approach())
        while not any(c.startswith('G ') for c in hw.commands):await asyncio.sleep(.01)
        a.task=task;await a.stop(emergency=True)
        result=(await asyncio.gather(task,return_exceptions=True))[0]
        assert isinstance(result,(AutoFault,asyncio.CancelledError)) and b.stopped
        n=len(hw.commands);await asyncio.sleep(.15)
        assert not any(c.startswith(('G ','P ','S ')) for c in hw.commands[n:])
        assert not any(hw.state['motors'])
    asyncio.run(run())
