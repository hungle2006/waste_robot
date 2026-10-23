"""Inclusive 20–25 cm window through the real AUTO approach state machine."""
import asyncio
import copy
import time
from unittest.mock import AsyncMock
import pytest
from test_control import robot
from test_box_tof_flow import prepared
from arm_path import targets_at
from auto_controller import NearHold, load_auto_config
from box_tof_flow import BoxToFFlow, DEFAULTS, checked_tof_flow
from config_io import read_config
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_all_config_sources_agree():
    robot_cfg = read_config(ROOT/'python/robot_params.py', 'robot')
    current = load_auto_config(ROOT/'python/robot_params.py', robot_cfg)
    assert (current['box_tof_flow']['tof_min_mm'], current['box_tof_flow']['tof_max_mm']) == (200, 250)
    # The historical JSON predates this flow; missing window fields use DEFAULTS.
    legacy = read_config(ROOT/'python/auto_config.json')
    cfg = checked_tof_flow(legacy.get('box_tof_flow', {}), current)
    assert (cfg['tof_min_mm'], cfg['tof_max_mm']) == (200, 250)
    assert (DEFAULTS['tof_min_mm'], DEFAULTS['tof_max_mm']) == (200, 250)


@pytest.mark.parametrize('mm', [200, 225, 250])
def test_inclusive_window_authorizes_grasp(robot, tmp_path, mm):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'], 2050))
        hw.tof_sample(mm)
        b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
        f = BoxToFFlow(a); f.last_obj = copy.deepcopy(a.target)
        f.final_approach = AsyncMock(return_value='grasp')
        assert await f.sensor_approach() == 'grasp'
        f.final_approach.assert_awaited_once()
        assert a.state['distance_ready'] and a.state['tof_mm'] == mm
        assert '20–25' in a.state['message']
    asyncio.run(run())


@pytest.mark.parametrize('mm', [199, 251])
def test_outside_window_never_authorizes_grasp(robot, tmp_path, mm):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'], 2050))
        hw.tof_sample(mm)
        b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
        f = BoxToFFlow(a); f.last_obj = copy.deepcopy(a.target)
        f.final_approach = AsyncMock()
        f.c['max_tof_pulses'] = 1
        f.paused_pulse = AsyncMock()
        a.wait = AsyncMock()
        with pytest.raises(NearHold):
            await f.sensor_approach()
        f.final_approach.assert_not_awaited()
        assert not f.tof_confirmed
        assert f.paused_pulse.await_count == (1 if mm == 251 else 0)
    asyncio.run(run())
