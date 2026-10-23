"""Regression tests: 30 rightward search sweeps, D19 loss, D18 reward gate."""
import asyncio
import time
from unittest.mock import AsyncMock

import pytest

from auto_controller import LostGrip
from test_control import robot, ready, wait_for
from test_box_tof_flow import prepared


def test_gift_requires_completed_rear_deposit_is_single_use_and_expires(robot, tmp_path):
    b,hw=robot
    ready(b)
    assert not b.snapshot()['gift']['eligible']
    with pytest.raises(ValueError, match='D18'):
        b.command('GIFT DISPENSE', b.epoch)
    b.gift.grant_after_rear(4)  # rear_cycle calls only after D18 clear and lid close
    assert b.snapshot()['gift']['eligible']
    assert 0 < b.snapshot()['gift']['remaining_seconds'] <= 120
    b.gift.claim_until = time.monotonic()-1
    assert not b.snapshot()['gift']['eligible']
    with pytest.raises(ValueError, match='quá 2 phút'):
        b.command('GIFT DISPENSE', b.epoch)
    b.gift.grant_after_rear(2)
    b.command('GIFT DISPENSE', b.epoch)
    wait_for(lambda: not b.gift.state['busy'])
    assert not b.snapshot()['gift']['eligible']
    with pytest.raises(ValueError, match='D18'):
        b.command('GIFT DISPENSE', b.epoch)
    b.gift.grant_after_rear(1)
    assert b.snapshot()['gift']['eligible']
    b.stop()
    assert not b.snapshot()['gift']['eligible']


def test_forward_limits_disabled_but_lateral_and_time_gates_remain(robot, tmp_path):
    _,_,a,c=prepared(robot,tmp_path)
    assert a.cfg['box_tof_flow']['max_forward_steps'] is None
    assert a.cfg['box_tof_flow']['max_tof_pulses'] is None
    from pathlib import Path
    from config_io import read_config
    live=read_config(Path(__file__).resolve().parents[1]/'python/robot_params.py')
    assert live['box_tof_flow']['max_final_pulses'] is None
    assert a.cfg['box_tof_flow']['max_lateral_steps']==25
    assert a.cfg['box_tof_flow']['tof_approach_timeout_ms']==45000 or a.cfg['box_tof_flow']['tof_approach_timeout_ms']==5000
    assert a.cfg['box_tof_flow']['final_timeout_ms']==2000
    assert all(s['max_forward_steps'] is None for s in a.cfg['pose_profile']['stages'])


def test_scan_30_right_rotations_then_continues_without_forward(robot, tmp_path, monkeypatch):
    async def go():
        _,_,a,_=prepared(robot,tmp_path)
        a.locked_bin=None
        a.locked_label=None
        a.search_rotations=0
        a.motor_step=AsyncMock()
        a.wait=AsyncMock()
        monkeypatch.setattr('robot_params.AUTO_SEARCH_TURN_SECONDS',1.12) # 2 guarded pulses each
        monkeypatch.setattr('robot_params.AUTO_SEARCH_STOP_SECONDS',1.0)
        for k in range(30):
            await a.search_turn_right()
            assert a.search_rotations==k+1
        assert a.motor_step.await_count == 60
        assert all(x.args[0]=='TURN_RIGHT' and x.kwargs['duration_ms']==400 for x in a.motor_step.await_args_list)
        await a.search_turn_right()
        assert a.search_rotations == 31
        assert a.state['scan_group'] == 2
        assert a.motor_step.await_count == 62
    asyncio.run(go())


def test_d19_loss_retracts_without_claiming_success(robot,tmp_path):
    async def go():
        b,hw,a,_=prepared(robot,tmp_path)
        a.sensor=lambda pin: False
        a.wait=AsyncMock()
        a.fresh_hardware=AsyncMock()
        with pytest.raises(LostGrip):
            await a.check_grip_or_drop('sau khi nâng')
        await a.recover_lost_grip('D19 mất vật')
        assert a.state['state']=='auto_drop_recovered'
        assert not b.snapshot()['gift']['eligible']
        assert hw.state['joints']['p4']==b.config['lid']['close']
        assert hw.state['joints']['d3']==b.config['grip']['open']
        assert a.locked_bin is None and a.selected_bin is None
    asyncio.run(go())
