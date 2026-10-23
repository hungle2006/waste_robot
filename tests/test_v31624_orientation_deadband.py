"""AUTO D2 deadband: |mapped angle| < 10° must preserve 2100 µs."""
import asyncio
import copy
import time
from unittest.mock import AsyncMock

import pytest

import operator_control
from grasp_orientation import auto_pulse_for_angle, pulse_for_angle
from box_tof_flow import BoxToFFlow
from test_box_tof_flow import prepared
from test_control import robot


@pytest.mark.parametrize('angle,pulse', [
    (0.,2100), (1.,2100), (-1.,2100), (9.99,2100), (-9.99,2100),
    (10.,pulse_for_angle(10.)), (-10.,pulse_for_angle(-10.)),
    (20.,pulse_for_angle(20.)), (-45.,1300), (75.,2500),
])
def test_auto_deadband_cutoff(angle,pulse):
    assert auto_pulse_for_angle(angle,10.0) == pulse


def test_manual_wrist_pulse_not_deadbanded():
    assert pulse_for_angle(5.) != 2100
    assert auto_pulse_for_angle(5.) == 2100


@pytest.mark.parametrize('measured,expected_pulse,is_deadband', [
    (9.,2100,True), (-9.,2100,True), (10.,pulse_for_angle(10.),False),
    (-10.,pulse_for_angle(-10.),False), (75.,2500,False),
])
def test_search_remembers_cutoff_and_vl53_applies(robot,tmp_path,monkeypatch,
                                                 measured,expected_pulse,is_deadband):
    async def run():
        b,hw,a,_ = prepared(robot,tmp_path)
        a.cfg['grasp_orientation'].update(enabled=True,calibrated=True,deadband_deg=10.0)
        obj=copy.deepcopy(a.target)
        async def frame():
            a.observed_jpeg=b'jpeg'
            a.last_candidates=[copy.deepcopy(obj)]
            return copy.deepcopy(obj)
        a._detect_once=AsyncMock(side_effect=frame)
        a.emit=AsyncMock()
        monkeypatch.setattr(operator_control,'estimate_axis',lambda *_:{'angle_deg':measured})
        await a.operator.remember_search_orientation(obj)
        memory=a.search_orientation
        assert memory['pulse_us']==expected_pulse
        assert memory['deadband_applied']==is_deadband
        assert memory['commanded_deg']==pytest.approx(0.0 if is_deadband else min(measured,60.0),abs=.2)
        assert hw.state['joints']['d2']==2100
        f=BoxToFFlow(a)
        f.tof_confirmed=True
        f.tof_confirmed_at=time.monotonic()
        f.tof_pose=a.cfg['arm_path']['start']['p1']
        f.stage='tof_approach'
        hw.tof_sample(225)
        b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        f.stationary_pose=lambda p: (a.guard(),p)
        motions=[]
        async def mock_move(targets,*args,**kwargs):
            motions.append(dict(targets))
            hw.state['joints'].update(targets)
            b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        a.move_targets=mock_move
        await f.rotate_saved_grip()
        assert f.rotated_wrist_us==expected_pulse
        assert memory['applied'] is True
        if is_deadband:
            assert not motions  # D2 already at 2100, no command to turn
        else:
            assert motions==[{'d2':expected_pulse}]
    asyncio.run(run())
