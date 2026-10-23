"""Firmware compatibility: never bypass missing IG; safe recovery for IG 0."""
import pytest
from test_control import robot, ready


def test_rearm_d19_guard_when_firmware_supports_ig(robot):
    brain, hw = robot
    ready(brain)
    hw.request('IG 0')
    brain.last_hw = hw.request('STATUS')
    assert brain.snapshot()['hardware']['ir_drive_guard'] is False
    brain.ensure_d19_guard_for_auto()
    assert brain.snapshot()['hardware']['ir_drive_guard'] is True
    assert 'IG 1' in hw.commands
    assert brain.snapshot()['hardware']['motors'] == [0,0,0,0]


def test_missing_ig_capability_never_falls_back_to_unsafe_auto(robot):
    brain, hw = robot
    ready(brain)
    brain.last_hw.pop('ir_drive_guard',None)
    before = len(hw.commands)
    with pytest.raises(ValueError,match='không báo IG'):
        brain.ensure_d19_guard_for_auto()
    assert len(hw.commands) == before


def test_rearm_cannot_bypass_hardware_stop(robot):
    brain, hw = robot
    ready(brain)
    brain.stop()
    with pytest.raises(ValueError,match='STOP'):
        brain.ensure_d19_guard_for_auto()
    assert hw.state['stopped'] is True
