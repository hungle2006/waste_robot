"""Mega digital D4 gift output must remain distinct from PCA9685 channel P4."""
import pytest
from test_control import robot, ready, wait_for
from test_gift_v3167 import writes, params


def test_gift_d4_and_lid_p4_are_independent(robot,tmp_path):
    b,hw=robot;ready(b)
    assert b.snapshot()['gift']['pin']==4
    assert 'd22' not in hw.state['joints']
    b.command('LID OPEN',b.epoch);wait_for(lambda:not b.busy)
    lid=hw.state['joints']['p4']
    assert hw.state['joints']['d4']==2500
    params(b,tmp_path,opened=600,closed=2500,hold=20)
    b.gift.grant_after_rear('RECYCLE')
    n=len(hw.commands);b.command('GIFT DISPENSE',b.epoch)
    wait_for(lambda:not b.gift.state['busy'])
    assert writes(hw,n)==['S 4 600','S 4 2500']
    assert hw.state['joints']['p4']==lid
    assert not any(c.startswith('S 22 ') for c in hw.commands)
    with pytest.raises(ValueError):hw.request('S 22 600')


def test_old_d22_firmware_protocol_cannot_enable_d4(robot):
    b,hw=robot;hw.state['gift_protocol']=2
    b.last_hw=hw.request('STATUS');n=len(hw.commands)
    assert not b.snapshot()['gift']['available']
    with pytest.raises(ValueError,match='V3.16.11'):b.command('GIFT DISPENSE',b.epoch)
    assert not writes(hw,n)
