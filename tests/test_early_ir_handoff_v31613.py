"""Early D19 stops wheels and completes descent; final live IR closes only later."""
import asyncio
import copy
import time
from unittest.mock import AsyncMock
import pytest
from test_control import robot
from test_box_tof_flow import prepared, full_demo
from test_auto import finish
from box_tof_flow import BoxToFFlow
from auto_controller import CloseRangeContact, AutoFault, NearHold
from pose_profile import reference_at
from arm_path import targets_at


def setup(robot,tmp_path,start=1300,ir=True,mm=800):
    b,hw,a,_=prepared(robot,tmp_path)
    hw.state['joints'].update(targets_at(a.cfg['arm_path'],start))
    hw.state['sensors']['D19']=0 if ir else 1
    hw.tof_sample(mm);b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
    f=BoxToFFlow(a);moves=[]
    a.detect=AsyncMock(return_value=copy.deepcopy(a.target))
    a.prepare_bin=AsyncMock()
    async def move(targets,*args,**kwargs):
        assert not any(hw.state['motors']) and not b.snapshot()['drive']['active']
        assert hw.state['joints']['d3']==600
        moves.append(copy.deepcopy(targets));hw.state['joints'].update(targets)
        b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
    a.move_targets=move
    return b,hw,a,f,moves


@pytest.mark.parametrize('start,mm',[(1300,800),(1550,800),(2050,800),(2050,199)])
def test_early_ir_far_tof_finishes_all_arm_steps(robot,tmp_path,start,mm):
    async def run():
        b,hw,a,f,moves=setup(robot,tmp_path,start=start,mm=mm)
        with pytest.raises(CloseRangeContact):await f.run()
        assert f.early_ir_at is not None and not f.tof_confirmed
        assert moves[-1]==targets_at(a.cfg['arm_path'],2310)
        assert a.state['grasp_trigger']=='early_ir'
        assert not a.state.get('distance_ready',False)
        assert hw.state['joints']['d3']==600
        assert not any(c.startswith('G ') for c in hw.commands)
    asyncio.run(run())


def test_transient_early_ir_cannot_authorize_grasp(robot,tmp_path):
    async def run():
        b,hw,a,f,moves=setup(robot,tmp_path)
        move=a.move_targets
        async def clear(targets,*args,**kwargs):
            await move(targets,*args,**kwargs)
            hw.state['sensors']['D19']=1
            b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        a.move_targets=clear
        f.c['max_final_pulses']=2;pulses=[]
        async def pulse(p1,actual_ms,tof_gate,**kwargs):
            assert p1==2310 and moves[-1]==targets_at(a.cfg['arm_path'],2310)
            assert actual_ms==160 and tof_gate is False
            pulses.append(actual_ms)
        f.paused_pulse=pulse
        with pytest.raises(NearHold,match='D19'):await f.run()
        assert pulses==[160,160] and hw.state['joints']['d3']==600
    asyncio.run(run())


@pytest.mark.parametrize('phase',['lower','forward','lateral'])
def test_early_ir_during_camera_action_reaches_final_pose(robot,tmp_path,phase):
    async def run():
        b,hw,a,f,moves=setup(robot,tmp_path,ir=False)
        ref=reference_at(a.cfg['pose_profile'],1300)
        obj=copy.deepcopy(a.target);cx=ref['target_x_norm']
        bottom=ref['target_bottom_norm']
        if phase=='forward':bottom-=.2
        if phase=='lateral':cx=.85
        obj['bbox_norm']=[cx-.08,bottom-.2,cx+.08,bottom]
        f.observe=AsyncMock(return_value=obj)
        if phase=='lower':
            move=a.move_targets
            async def activate(targets,*args,**kwargs):
                await move(targets,*args,**kwargs)
                hw.state['sensors']['D19']=0
                b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
            a.move_targets=activate
        else:
            request=hw.request
            def activate(command):
                result=request(command)
                if command.startswith('G '):hw.state['sensors']['D19']=0
                return result
            hw.request=activate
        with pytest.raises(CloseRangeContact):await asyncio.wait_for(f.run(),5)
        assert moves[-1]==targets_at(a.cfg['arm_path'],2310)
        assert f.early_ir_at is not None and not any(hw.state['motors'])
        assert hw.profile is None
    asyncio.run(run())


def test_pending_camera_request_cancelled_before_early_descent(robot,tmp_path):
    async def run():
        b,hw,a,f,moves=setup(robot,tmp_path,ir=False)
        f.last_action='FORWARD';cancelled=asyncio.Event()
        async def detect(*args,**kwargs):
            hw.state['sensors']['D19']=0
            b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
            try:await asyncio.Event().wait()
            finally:cancelled.set()
        a._detect_once=detect
        move=a.move_targets
        async def after_cancel(targets,*args,**kwargs):
            assert cancelled.is_set()
            await move(targets,*args,**kwargs)
        a.move_targets=after_cancel
        with pytest.raises(CloseRangeContact):await asyncio.wait_for(f.run(),5)
        assert moves[-1]==targets_at(a.cfg['arm_path'],2310)
    asyncio.run(run())


def test_no_locked_target_cannot_start_early_descent(robot,tmp_path):
    async def run():
        b,hw,a,f,moves=setup(robot,tmp_path)
        a.detect=AsyncMock(return_value=None)
        with pytest.raises(NearHold):await f.run()
        assert not moves
    asyncio.run(run())


def test_d18_only_wheel_rejection_is_not_mistaken_for_early_d19(robot,tmp_path):
    async def run():
        b,hw,a,f,moves=setup(robot,tmp_path,ir=False)
        hw.state['sensors']['D18']=0
        b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        ref=reference_at(a.cfg['pose_profile'],1300)
        obj=copy.deepcopy(a.target);bottom=ref['target_bottom_norm']
        obj['bbox_norm']=[.77,bottom-.2,.93,bottom]
        f.observe=AsyncMock(return_value=obj)
        with pytest.raises(NearHold):await f.run()
        assert not moves and f.early_ir_at is None
    asyncio.run(run())


def test_stop_before_early_handoff_prevents_lowering(robot,tmp_path):
    async def run():
        b,hw,a,f,moves=setup(robot,tmp_path)
        f.last_obj=copy.deepcopy(a.target);b.stop()
        with pytest.raises(AutoFault):await f.early_ir_handoff()
        assert not moves
    asyncio.run(run())


def test_full_cycle_ir_active_at_search_then_clears_before_final(tmp_path):
    async def run():
        b,hw,a,vision=full_demo(tmp_path);request=hw.request;early=[]
        def signal(command):
            result=request(command)
            if command=='STATUS' and hw.state['joints'].get('p1',0)<2310 and not hw.held:
                early.append(hw.state['joints']['p1'])
                hw.state['sensors']['D19']=0;result['sensors']['D19']=0
            return result
        hw.request=signal
        try:
            await a.start();await finish(a,timeout=45)
            assert early and a.state['state']=='auto_done' and hw.released
            assert hw.commands.index('P 1 2310')<hw.commands.index('S 3 2300')
            assert hw.ir_pulses==2
            events=[call.args[0] for call in a.report.call_args_list]
            assert any(e['state']=='auto_early_ir_lower' for e in events)
            assert not any(e['state']=='auto_hold' for e in events)
            assert not any(e['state']=='auto_distance_ready' for e in events)
        finally:
            if a.running:await a.stop()
            b.close()
    asyncio.run(run())
