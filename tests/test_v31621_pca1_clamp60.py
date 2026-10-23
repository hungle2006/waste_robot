"""V3.16.21: PCA > 1 threshold, reachable right wrist clamp."""
import asyncio
import copy
from unittest.mock import AsyncMock

import cv2
import numpy as np
import pytest
import operator_control
from grasp_orientation import angle_for_pulse, estimate_axis, pulse_for_angle
from test_box_tof_flow import prepared
from test_control import robot


def test_clamp_right_above_60_but_preserve_left_calibration():
    assert pulse_for_angle(61.) == 2500
    assert pulse_for_angle(75.) == 2500
    assert pulse_for_angle(89.) == 2500
    assert pulse_for_angle(60.) == 2500
    assert pulse_for_angle(-45.) == 1300
    assert pulse_for_angle(-90.) == 500
    with pytest.raises(ValueError):
        pulse_for_angle(float('nan'))


@pytest.mark.parametrize('ratio,accepted', [(1.0, False), (1.0001, True), (1.2, True)])
def test_pca_ratio_must_be_strictly_greater_than_one(monkeypatch, ratio, accepted):
    # The same segmented sample with patched PCA eigenvalues tests the strict
    # comparison independently of GrabCut's exact numerical output.
    image = np.full((300, 300, 3), 230, np.uint8)
    cv2.rectangle(image, (137, 55), (163, 245), (30, 100, 40), -1)
    encoded = cv2.imencode('.jpg', image)[1].tobytes()
    original = cv2.PCACompute2

    def pca_with_ratio(*args, **kwargs):
        mean, vec, values = original(*args, **kwargs)
        values = np.array(values, copy=True)
        values[1, 0] = values[0, 0] / ratio
        return mean, vec, values

    monkeypatch.setattr(cv2, 'PCACompute2', pca_with_ratio)
    if not accepted:
        with pytest.raises(ValueError, match='PCA=.*<= 1.0000'):
            estimate_axis(encoded, [132, 50, 168, 250])
    else:
        result = estimate_axis(encoded, [132, 50, 168, 250])
        assert result['elongation'] == pytest.approx(ratio)
        assert abs(result['angle_deg']) < 4


def test_search_right_75_records_limited_command_without_moving(robot, tmp_path, monkeypatch):
    async def run():
        _b, hw, a, _ = prepared(robot, tmp_path)
        a.cfg['grasp_orientation'].update(enabled=True, calibrated=True)
        obj = copy.deepcopy(a.target)
        async def sample():
            a.observed_jpeg = b'jpeg'
            a.last_candidates = [copy.deepcopy(obj)]
            return copy.deepcopy(obj)
        a._detect_once = AsyncMock(side_effect=sample)
        a.emit = AsyncMock()
        monkeypatch.setattr(operator_control, 'estimate_axis',
                            lambda *args: {'angle_deg': 75.})
        await a.operator.remember_search_orientation(obj)
        m = a.search_orientation
        assert m['angle_deg'] == pytest.approx(75)
        assert m['mapped_deg'] == pytest.approx(75)
        assert m['commanded_deg'] == pytest.approx(60)
        assert m['angle_limited'] is True
        assert m['pulse_us'] == 2500
        assert m['applied'] is False
        assert hw.state['joints']['d2'] == 2100  # Still waiting for VL53
        assert 'GIỚI HẠN về +60°' in a.emit.await_args.args[1]
    asyncio.run(run())
