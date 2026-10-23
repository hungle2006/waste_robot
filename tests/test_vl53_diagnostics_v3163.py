import asyncio
import time
from unittest.mock import AsyncMock

import pytest

from test_control import robot
from test_box_tof_flow import prepared
from box_tof_flow import BoxToFFlow, tof_sample
from auto_controller import NearHold, AutoFault


def invalid_range(b, hw, status=2, mm=280):
    hw.tof_sample(mm, valid=False)
    hw.state['tof']['status'] = status
    b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()


def test_invalid_reading_reports_raw_diagnostic_fields(robot, tmp_path):
    b, hw, a, _ = prepared(robot, tmp_path)
    invalid_range(b, hw)
    with pytest.raises(ValueError) as error:
        tof_sample(b.snapshot(), a.cfg['box_tof_flow'], check_age=False)
    message = str(error.value)
    assert 'valid=False' in message and 'status=2' in message
    assert 'SignalFail' in message and 'mm=280' in message and 'seq=' in message

