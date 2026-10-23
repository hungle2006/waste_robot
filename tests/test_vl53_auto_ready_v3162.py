"""Auto uses valid distance, then latches handoff independently of ToF age."""
import asyncio
import copy
import time
from unittest.mock import AsyncMock

import pytest

from test_control import robot
from test_box_tof_flow import prepared, full_demo
from test_auto import finish
from arm_path import targets_at
from auto_controller import CloseRangeContact, AutoFault, NearHold
from box_tof_flow import BoxToFFlow, tof_sample


def cached_range(b, hw, mm=225, age_s=5):
    hw.tof_sample(mm)
    hw.tof_at = time.monotonic() - age_s
    b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()


@pytest.mark.parametrize('age_ms', [300, 5000, None])
def test_auto_range_ignores_age_but_default_manual_reader_stays_strict(robot, tmp_path, age_ms):
    b, hw, a, _ = prepared(robot, tmp_path)
    cached_range(b, hw)
    s = b.snapshot(); s['hardware']['tof']['age_ms'] = age_ms
    with pytest.raises(ValueError):
        tof_sample(s, a.cfg['box_tof_flow'])
    assert tof_sample(s, a.cfg['box_tof_flow'], check_age=False)[1] == 225


def test_auto_accepts_one_in_range_sample_without_new_sequence_or_window_streak(robot, tmp_path):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'], 2050))
        cached_range(b, hw)
        f = BoxToFFlow(a); f.last_obj = copy.deepcopy(a.target)
        f.final_approach = AsyncMock(return_value='grasp')
        n = len(hw.commands)
        assert await f.sensor_approach() == 'grasp'
        assert f.tof_confirmed and not hw.state['tof']['guard']
        f.final_approach.assert_awaited_once()
        assert a.state['distance_ready'] and a.state['tof_votes'] == 1
        assert not any(c.startswith(('P ', 'S ', 'G ')) for c in hw.commands[n:])
    asyncio.run(run())


def test_auto_handover_does_not_expire_while_reporting_or_disabling_tof(robot, tmp_path):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'], 2050))
        cached_range(b, hw)
        f = BoxToFFlow(a); f.last_obj = copy.deepcopy(a.target)
        original_final = f.final_approach
        moves = []
        async def move(targets, *args, **kwargs):
            moves.append(dict(targets)); hw.state['joints'].update(targets)
            hw.state['sensors']['D19'] = 0 if hw.state['joints']['p1'] == 2310 else 1
            b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
        async def delayed_final():
            f.tof_confirmed_at -= 30
            hw.tof_at = time.monotonic() - 100
            hw.state['tof']['valid'] = False  # The handoff was already confirmed.
            b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
            return await original_final()
        a.move_targets = move; f.final_approach = delayed_final
        with pytest.raises(CloseRangeContact):
            await f.sensor_approach()
        assert moves and moves[-1]['p1'] == 2310 and moves[-1]['p10'] == 650
        assert f.stage == 'final_ir'
    asyncio.run(run())


def test_auto_checks_ready_distance_before_waiting_for_another_camera_action(robot, tmp_path):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        cached_range(b, hw)
        a.detect = AsyncMock(return_value=copy.deepcopy(a.target))
        a.prepare_bin = AsyncMock()
        f = BoxToFFlow(a)
        f.observe = AsyncMock(side_effect=AssertionError('Should hand off before another image'))
        f.final_approach = AsyncMock(side_effect=CloseRangeContact(copy.deepcopy(a.target)))
        with pytest.raises(CloseRangeContact):
            await f.run()
        assert f.tof_pose == 1300 and f.tof_confirmed
        f.observe.assert_not_awaited()
    asyncio.run(run())


def test_cached_far_reading_recovers_when_range_arrives_after_old_fault_deadline(robot, tmp_path):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'], 2050))
        cached_range(b, hw, mm=800)
        f = BoxToFFlow(a); f.last_obj = copy.deepcopy(a.target)
        f.final_approach = AsyncMock(return_value='grasp')
        async def range_arrives():
            await asyncio.sleep(1.2)  # Fixture's previous fault deadline is 1 second.
            cached_range(b, hw, mm=225)
        later = asyncio.create_task(range_arrives())
        try:
            assert await asyncio.wait_for(f.sensor_approach(), 4) == 'grasp'
            assert f.tof_confirmed
        finally:
            later.cancel(); await asyncio.gather(later, return_exceptions=True)
    asyncio.run(run())


def test_stop_after_distance_confirmation_still_prevents_arm_motion(robot, tmp_path):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'], 2050))
        cached_range(b, hw)
        f = BoxToFFlow(a); f.last_obj = copy.deepcopy(a.target)
        original_final = f.final_approach
        async def stop_before_final():
            b.stop()
            return await original_final()
        f.final_approach = stop_before_final
        with pytest.raises(AutoFault):
            await f.sensor_approach()
        stop = max(i for i, c in enumerate(hw.commands) if c == 'STOP')
        assert not any(c.startswith(('P ', 'S ', 'G ')) for c in hw.commands[stop + 1:])
        assert b.stopped
    asyncio.run(run())


def test_enough_distance_still_requires_d19_before_gripping(robot, tmp_path):
    async def run():
        b, hw, a, _ = prepared(robot, tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'], 2050))
        cached_range(b, hw)
        f = BoxToFFlow(a); f.last_obj = copy.deepcopy(a.target)
        async def move(targets, *args, **kwargs):
            hw.state['joints'].update(targets)
            b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
        a.move_targets = move
        n = len(hw.commands)
        with pytest.raises(NearHold, match='D19'):
            await f.sensor_approach()
        assert f.tof_confirmed and f.stage == 'final_ir'
        assert hw.state['joints']['d3'] == b.config['grip']['open']
        assert not any(c.startswith('S 3 ') for c in hw.commands[n:])
    asyncio.run(run())


def test_complete_auto_grasp_lift_release_with_old_in_range_tof_metadata(tmp_path):
    async def run():
        b, hw, a, vision = full_demo(tmp_path)
        request = hw.request
        def old_age(command):
            result = request(command)
            if command == 'STATUS' and 200 <= result['tof']['mm'] <= 250:
                result['tof']['age_ms'] = 5000
            return result
        hw.request = old_age
        try:
            await a.start(); await finish(a, timeout=45)
            assert a.state['state'] == 'auto_done' and hw.released, a.state
            assert hw.state['joints']['d3'] == b.config['grip']['open']
            events = [c.args[0] for c in a.report.call_args_list]
            ready = [e for e in events if e['state'] == 'auto_distance_ready']
            assert len(ready) == 1 and ready[0]['tof_votes'] == 1
            assert not any(e['state'] == 'auto_hold' for e in events)
            sent = hw.commands
            assert sent.index('P 1 2310') < sent.index('S 3 2300') < sent.index('P 4 1600')
        finally:
            if a.running: await a.stop()
            b.close()
    asyncio.run(run())
