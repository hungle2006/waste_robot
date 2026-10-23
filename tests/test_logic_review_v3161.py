import asyncio
import time

import pytest

from test_control import robot
from test_operator_v316 import manual
from arm_path import targets_at


def test_hold_preserves_bin_of_manually_grasped_object(robot, tmp_path):
    async def run():
        b, hw, a, _ = manual(robot, tmp_path)
        a.operator.mechanics_enabled = False
        a.operator.held_bin = 1  # The manual grasp selected this bin.
        a.selected_bin = 4      # Left over from an earlier AUTO cycle.
        hw.state['joints']['d3'] = b.config['grip']['close']
        hw.state['sensors']['D19'] = 0
        b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
        await a.operator.action({'command': 'HOLD', 'epoch': b.epoch})
        assert a.operator.held_bin == 1
    asyncio.run(run())


def test_auto_handover_replaces_old_manual_bin_with_current_auto_bin(robot, tmp_path):
    async def run():
        b, hw, a, _ = manual(robot, tmp_path)
        a.operator.held_bin = 1
        a.selected_bin = 4
        b.claim_auto(a.token)
        hw.state['joints']['d3'] = b.config['grip']['close']
        hw.state['sensors']['D19'] = 0
        b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
        await a.operator.action({'command': 'HOLD', 'epoch': b.epoch})
        assert a.operator.held_bin == 4
    asyncio.run(run())


@pytest.mark.parametrize('mechanics_enabled', [False, True])
def test_start_during_manual_assist_does_not_replace_owner_token(robot, tmp_path, mechanics_enabled):
    async def run():
        b, hw, a, _ = manual(robot, tmp_path)
        a.operator.mechanics_enabled = mechanics_enabled
        a.token = 'manual-assist-owner'; b.claim_auto(a.token)
        a.operator.assist = asyncio.create_task(asyncio.sleep(20))
        n = len(hw.commands)
        try:
            with pytest.raises(ValueError, match='gắp/thả'):
                await a.operator.action({'command': 'AUTO START', 'epoch': b.epoch})
            assert a.token == b.auto_owner == 'manual-assist-owner'
            assert not a.operator.assist.done()
            assert not a.running
            assert len(hw.commands) == n
        finally:
            await a.operator.cancel_assist()
    asyncio.run(run())


def test_ai_grasp_rejects_closed_gripper_before_claim_or_motion(robot, tmp_path):
    async def run():
        b, hw, a, _ = manual(robot, tmp_path)
        a.operator.mechanics_enabled = False
        hw.state['joints'].update(targets_at(a.cfg['arm_path'], 2310), d3=b.config['grip']['close'])
        hw.state['sensors']['D19'] = 0
        b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
        a.target = {'bin': 4}; a.locked_bin = 4
        a.state['target_fresh'] = True; a.operator.observation_at = time.monotonic()
        n = len(hw.commands)
        try:
            with pytest.raises(ValueError, match='Mở kẹp'):
                await a.operator.action({'command': 'GRASP', 'epoch': b.epoch})
            assert b.auto_owner is None and a.operator.assist is None
            assert len(hw.commands) == n
        finally:
            await a.operator.cancel_assist()
    asyncio.run(run())


def test_failed_handover_does_not_leave_transition_permanently_locked(robot, tmp_path, monkeypatch):
    async def run():
        b, hw, a, _ = manual(robot, tmp_path)
        def failed_detach():
            raise ConnectionError('USB không xác nhận XM')
        monkeypatch.setattr(b, 'detach_auto', failed_detach)
        with pytest.raises(ConnectionError):
            await a.operator.action({'command': 'MECHANICS OFF', 'epoch': b.epoch})
        assert not a.operator.transitioning
        assert a.operator.mechanics_enabled  # No successful OFF ACK.
        assert a.cancelled                  # No automatic retry of motion.
    asyncio.run(run())
