"""V3.16.20: noisy SEARCH segmentation gets bounded retries, never blind wrist."""
import asyncio
import copy
from unittest.mock import AsyncMock

import pytest
import operator_control
from auto_controller import NearHold
from test_box_tof_flow import prepared
from test_control import robot


def _frames(a):
    obj = copy.deepcopy(a.target)
    async def take():
        a.observed_jpeg = b'jpeg'
        a.last_candidates = [copy.deepcopy(obj)]
        return copy.deepcopy(obj)
    return obj, take


def test_search_recovers_from_one_round_segmentation(robot, tmp_path, monkeypatch):
    async def go():
        b, hw, a, _ = prepared(robot, tmp_path)
        a.cfg['grasp_orientation'].update(enabled=True, calibrated=True)
        obj, take = _frames(a)
        a._detect_once = AsyncMock(side_effect=take)
        a.emit = AsyncMock()
        a.wait = AsyncMock()
        angles = iter([None, 15.0, 16.0, 14.0])
        def calculate(*_):
            angle = next(angles)
            if angle is None: raise ValueError('Vật gần tròn hoặc hướng không rõ')
            return {'angle_deg': angle}
        monkeypatch.setattr(operator_control, 'estimate_axis', calculate)
        await a.operator.remember_search_orientation(obj)
        assert a._detect_once.await_count == 4
        assert a.search_orientation['samples'] == 3
        assert a.search_orientation['applied'] is False
        assert hw.state['joints']['d2'] == 2100
        assert any(c.args[0] == 'auto_orientation_retry' for c in a.emit.await_args_list)
    asyncio.run(go())


def test_search_still_holds_after_nine_failed_frames(robot, tmp_path, monkeypatch):
    async def go():
        b, hw, a, _ = prepared(robot, tmp_path)
        a.cfg['grasp_orientation'].update(enabled=True, calibrated=True)
        obj, take = _frames(a)
        a._detect_once = AsyncMock(side_effect=take)
        a.emit = AsyncMock()
        a.wait = AsyncMock()
        def cannot_estimate(*_):
            raise ValueError('Vật gần tròn hoặc hướng không rõ')
        monkeypatch.setattr(operator_control, 'estimate_axis', cannot_estimate)
        with pytest.raises(NearHold, match='sau 9 ảnh'):
            await a.operator.remember_search_orientation(obj)
        assert a._detect_once.await_count == 9
        assert a.search_orientation is None
        assert hw.state['joints']['d2'] == 2100
        assert sum(c.args[0]=='auto_orientation_retry' for c in a.emit.await_args_list)==9
    asyncio.run(go())
