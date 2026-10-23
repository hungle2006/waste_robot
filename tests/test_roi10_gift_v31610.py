"""Requested additions to the uploaded V3.16.4 baseline."""
import asyncio
from pathlib import Path
import pytest
from test_control import robot, ready, wait_for
from test_box_tof_flow import prepared
from gift_control import GIFT_PIN, GIFT_PROTOCOL, load_gift_params
from config_io import read_config
from auto_controller import load_auto_config

ROOT=Path(__file__).resolve().parents[1]


def test_default_gift_pulses_grip_and_uploaded_distance_window(robot):
    b,hw=robot;ready(b)
    cfg=load_gift_params(ROOT/'python/gift_params.py')
    assert GIFT_PIN==4 and GIFT_PROTOCOL==3
    assert cfg['GIFT_OPEN_US']==600 and cfg['GIFT_CLOSE_US']==2500
    for name in ('robot_params.py','robot_config.json'):
        assert read_config(ROOT/'python'/name,'robot')['grip']=={'open':600,'close':2300}
    auto=load_auto_config(ROOT/'python/robot_params.py',b.config)
    assert auto['box_tof_flow']['tof_min_mm']==200
    assert auto['box_tof_flow']['tof_max_mm']==250
    assert hw.state['tof']['roi_size_configured']==10
    n=len(hw.commands);b.command('GIFT DISPENSE',b.epoch)
    wait_for(lambda:not b.gift.state['busy'])
    assert [c for c in hw.commands[n:] if c.startswith('S 4 ')]==['S 4 600','S 4 2500']
    assert hw.state['joints']['d4']==2500 and b.gift.state['phase']=='done'
    b.command('GRIP CLOSE',b.epoch);wait_for(lambda:not b.busy)
    assert hw.state['joints']['d3']==2300


@pytest.mark.parametrize('roi',[None,8,16])
def test_old_roi_firmware_rejected_before_auto_drive(robot,tmp_path,roi):
    b,hw,a,_=prepared(robot,tmp_path)
    if roi is None:hw.state['tof'].pop('roi_size_configured')
    else:hw.state['tof']['roi_size_configured']=roi
    b.last_hw=hw.request('STATUS');n=len(hw.commands)
    with pytest.raises(ValueError,match='ROI 10x10'):
        b.tof_guard(True,a.token,ignore_invalid=True)
    assert not any(c.startswith(('TF AUTO','G ')) for c in hw.commands[n:])


def test_correct_roi_keeps_continue_on_invalid_policy(robot,tmp_path):
    b,hw,a,_=prepared(robot,tmp_path)
    hw.state['tof'].update(present=False,valid=False)
    b.last_hw=hw.request('STATUS')
    b.tof_guard(True,a.token,ignore_invalid=True)
    assert hw.commands[-2]=='TF AUTO 200 250'
    assert hw.state['tof']['guard'] and hw.state['tof']['invalid_policy']=='continue'
