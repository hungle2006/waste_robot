"""D19 authorizes contact only after the complete final arm trajectory."""
import asyncio
import copy
import time
import pytest
from test_control import robot
from test_box_tof_flow import prepared, full_demo
from test_auto import finish
from arm_path import targets_at
from box_tof_flow import BoxToFFlow
from auto_controller import AutoFault, NearHold, CloseRangeContact, RearRequested


def setup_flow(robot, tmp_path, *, start=2050, ir=False):
    b, hw, a, _ = prepared(robot, tmp_path)
    hw.state['joints'].update(targets_at(a.cfg['arm_path'], start))
    hw.state['sensors']['D19'] = 0 if ir else 1
    hw.tof_sample(225)
    b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
    f = BoxToFFlow(a); f.last_obj = copy.deepcopy(a.target)
    moves=[]
    async def move(targets, *args, **kwargs):
        assert not any(hw.state['motors'])
        assert hw.state['joints']['d3'] == b.config['grip']['open']
        moves.append(copy.deepcopy(targets))
        hw.state['joints'].update(targets)
        b.last_hw = hw.request('STATUS'); b.updated = time.monotonic()
    a.move_targets = move
    sensor = a.sensor
    checks=[]
    def final_only(name):
        if name == 'D19':
            assert hw.state['joints']['p1'] == a.cfg['arm_path']['end']['p1']
            assert moves and moves[-1] == targets_at(a.cfg['arm_path'], 2310)
            checks.append(hw.state['joints']['p1'])
        return sensor(name)
    a.sensor = final_only
    return b, hw, a, f, moves, checks


@pytest.mark.parametrize('activation', ['before_descent', 'during_descent'])
def test_early_ir_does_not_interrupt_final_descent(robot, tmp_path, activation):
    async def run():
        b,hw,a,f,moves,checks=setup_flow(robot,tmp_path,ir=activation=='before_descent')
        move=a.move_targets
        async def activate(targets,*args,**kwargs):
            await move(targets,*args,**kwargs)
            if activation=='during_descent':
                hw.state['sensors']['D19']=0
                b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        a.move_targets=activate
        with pytest.raises(CloseRangeContact):await f.sensor_approach()
        assert moves[-1] == targets_at(a.cfg['arm_path'],2310)
        assert checks and a.state['ir_check_enabled'] is True
        assert hw.state['joints']['d3']==600
        assert not any(c.startswith('G ') for c in hw.commands)
    asyncio.run(run())


def test_ready_range_and_early_ir_enters_descent_before_camera(robot,tmp_path):
    async def run():
        b,hw,a,f,moves,checks=setup_flow(robot,tmp_path,start=1300,ir=True)
        async def detect():return copy.deepcopy(a.target)
        async def prepare(_):pass
        a.detect=detect;a.prepare_bin=prepare
        with pytest.raises(CloseRangeContact):await f.run()
        assert moves[-1]==targets_at(a.cfg['arm_path'],2310) and checks
    asyncio.run(run())


@pytest.mark.parametrize('detect_after', [1,2,None])
def test_missing_ir_pulses_only_at_final_pose(robot,tmp_path,detect_after):
    async def run():
        b,hw,a,f,moves,checks=setup_flow(robot,tmp_path)
        pulses=[];f.c['max_final_pulses']=2
        async def pulse(p1,actual_ms,tof_gate,**kwargs):
            assert p1==2310 and moves[-1]==targets_at(a.cfg['arm_path'],2310)
            assert a.state['ir_check_enabled'] is True
            assert actual_ms==160 and tof_gate is False
            pulses.append(actual_ms)
            if len(pulses)==detect_after:
                hw.state['sensors']['D19']=0
                b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        f.paused_pulse=pulse
        expected=NearHold if detect_after is None else CloseRangeContact
        with pytest.raises(expected):await f.sensor_approach()
        assert len(pulses)==(detect_after or 2)
        assert hw.state['joints']['d3']==600
    asyncio.run(run())


@pytest.mark.parametrize('interruption',['stop','rear'])
def test_descent_retains_stop_and_d18(robot,tmp_path,interruption):
    async def run():
        b,hw,a,f,moves,checks=setup_flow(robot,tmp_path,ir=True)
        move=a.move_targets
        async def interrupt(targets,*args,**kwargs):
            await move(targets,*args,**kwargs)
            if interruption=='stop':b.stop()
            else:
                hw.state['sensors']['D18']=0;b.auto_rear=True;b.rear_pending=True
        a.move_targets=interrupt
        with pytest.raises(AutoFault if interruption=='stop' else RearRequested):
            await f.sensor_approach()
        assert len(moves)==1 and not checks
        assert not any(c.startswith('G ') for c in hw.commands)
    asyncio.run(run())


def test_complete_cycle_with_transient_ir_during_real_arm_jobs(tmp_path):
    async def run():
        b,hw,a,vision=full_demo(tmp_path)
        request=hw.request;early=[]
        def transient(command):
            result=request(command)
            if command=='STATUS' and 2050<hw.state['joints'].get('p1',0)<2310 and not hw.held:
                early.append(hw.state['joints']['p1'])
                hw.state['sensors']['D19']=0
                result['sensors']['D19']=0
            return result
        hw.request=transient
        try:
            await a.start();await finish(a,timeout=45)
            assert early and a.state['state']=='auto_done' and hw.released
            assert hw.commands.index('P 1 2310')<hw.commands.index('S 3 2300')
            assert hw.ir_pulses==2  # Transient early IR did not authorize closing.
            events=[call.args[0] for call in a.report.call_args_list]
            assert not any(e['state']=='auto_hold' for e in events)
            order=[e['state'] for e in events]
            assert order.index('auto_tof_final_pose')<order.index('auto_tof_ir_wait')<order.index('auto_tof_ir_ready')<order.index('auto_grip')
        finally:
            if a.running:await a.stop()
            b.close()
    asyncio.run(run())
