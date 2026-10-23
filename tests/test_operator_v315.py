import asyncio, math, time
from unittest.mock import AsyncMock
import cv2
import numpy as np
import pytest
from test_control import robot, ready, wait_for
from test_box_tof_flow import prepared, raw
from test_realtime import packet
from auto_controller import NearHold
from realtime_bridge import RealtimeService
from grasp_orientation import axis_angle, pulse_for_angle, mean_axis, estimate_axis
from arm_path import targets_at

@pytest.mark.parametrize('angle,pulse',[(-90,500),(-45,1300),(0,2100),(30,2300),(60,2500),(90,500),(180,2100)])
def test_user_calibration(angle,pulse): assert pulse_for_angle(angle)==pulse

@pytest.mark.parametrize('angle',[61,75,89.5])
def test_positive_angles_above_servo_limit_are_clamped(angle):
    assert pulse_for_angle(angle)==2500

@pytest.mark.parametrize('angle',[float('nan'),True])
def test_invalid_angles(angle):
    with pytest.raises(ValueError): pulse_for_angle(angle)

def test_axis_without_head_tail_and_circular_average():
    assert axis_angle(60,-200)==pytest.approx(16.699244)
    assert axis_angle(-60,200)==pytest.approx(16.699244)
    assert abs(mean_axis([89,-89,90]))==pytest.approx(90)
    with pytest.raises(ValueError): mean_axis([-30,0,30])

@pytest.mark.parametrize('angle',[-50,-20,0,30,55])
def test_axis_from_actual_pixels(angle):
    image=np.full((300,300,3),230,np.uint8)
    dx,dy=math.sin(math.radians(angle)),-math.cos(math.radians(angle))
    c=np.array([150,150]);v=np.array([dx,dy])*80;n=np.array([-dy,dx])*15
    poly=np.int32([c-v-n,c-v+n,c+v+n,c+v-n]);cv2.fillConvexPoly(image,poly,(30,100,40))
    x,y,w,h=cv2.boundingRect(poly);jpeg=cv2.imencode('.jpg',image)[1].tobytes()
    r=estimate_axis(jpeg,[x-3,y-3,x+w+3,y+h+3])
    assert abs((r['angle_deg']-angle+90)%180-90)<2

def test_round_and_clipped_do_not_produce_a_direction():
    image=np.full((200,200,3),230,np.uint8);cv2.circle(image,(100,100),35,(30,70,120),-1)
    jpeg=cv2.imencode('.jpg',image)[1].tobytes()
    with pytest.raises(ValueError,match='không trùng trục dài'):estimate_axis(jpeg,[60,60,140,140])
    with pytest.raises(ValueError,match='cắt'):estimate_axis(jpeg,[0,10,100,120])

def test_red_observation_has_no_motion_writes_and_allows_manual(robot,tmp_path):
    async def run():
        b,hw,a,c=prepared(robot,tmp_path,[raw()]*12);b.release_auto(a.token)
        await a.operator.set_mechanics(False);hw.commands.clear();await a.start();await asyncio.sleep(.7)
        assert a.running and not a.state['mechanics_enabled']
        assert not [x for x in hw.commands if x!='STATUS']
        b.command(None,b.epoch,'d2',2300);await asyncio.to_thread(wait_for,lambda:not b.busy)
        assert hw.state['joints']['d2']==2300 and a.running
        await a.stop()
    asyncio.run(run())

def test_detach_cancels_mid_ramp_and_old_owner(robot):
    b,hw=robot;ready(b);b.claim_auto('old');b.auto_targets({'p1':2400},'moving','old',step_us=10,step_ms=20)
    time.sleep(.06);b.detach_auto();at=len(hw.commands);time.sleep(.08)
    assert not b.busy and b.auto_owner is None and not b.stopped
    assert hw.state['motors']==[0,0,0,0]
    assert not any(x.startswith(('P ','S ','G ','C ')) for x in hw.commands[at:])
    with pytest.raises(ValueError):b.auto_targets({'p1':2000},'late','old')

def test_green_refuses_held_object(robot,tmp_path):
    async def run():
        b,hw,a,c=prepared(robot,tmp_path);b.release_auto(a.token);await a.operator.set_mechanics(False)
        hw.state['joints']['d3']=b.config['grip']['close'];b.last_hw=hw.request('STATUS');count=len(hw.commands)
        with pytest.raises(ValueError,match='Thả vật'):await a.operator.set_mechanics(True)
        assert len(hw.commands)==count
    asyncio.run(run())

def test_range_three_distinct_samples_then_stale(robot,tmp_path):
    b,hw,a,c=prepared(robot,tmp_path);op=a.operator;op.mechanics_enabled=False
    for i in range(3):
        hw.tof_sample(280);b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        assert op.range_sample(b.snapshot())[1]==(i==2)
        op.range_sample(b.snapshot());assert op.votes==i+1
    assert op.snapshot()['distance_ready']
    b.updated-=1;op.range_sample(b.snapshot());assert not op.snapshot()['distance_ready']
    hw.tof_sample(240);b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
    assert not op.range_sample(b.snapshot())[1]

def test_lift_preserves_rotated_wrist(robot,tmp_path):
    async def run():
        b,hw,a,c=prepared(robot,tmp_path);hw.state['joints'].update(targets_at(a.cfg['arm_path'],2310),d2=1300)
        hw.state['sensors']['D19']=0;b.last_hw=hw.request('STATUS');b.updated=time.monotonic();moves=[]
        async def fake(targets,*args,**kwargs):
            moves.append(dict(targets));hw.state['joints'].update(targets);b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        a.move_targets=fake;await a.retract_command_path('lift',require_hold=True)
        assert moves and all(x['d2']==1300 for x in moves)
    asyncio.run(run())

def test_s1_before_s2(robot,tmp_path):
    async def run():
        b,hw,a,c=prepared(robot,tmp_path);b.start_monitor();hw.commands.clear()
        await a.move_targets({'p1':1350,'p10':935},'order',settle_ms=0,gentle=True)
        cmds=[x for x in hw.commands if x.startswith('P ')]
        assert cmds.index('P 10 935')<cmds.index('P 1 1350')
    asyncio.run(run())

def service_for(b,a):
    s=RealtimeService(b,'robot_01',type('Channel',(),{'send_broadcast':AsyncMock()})());s.auto=a;s.online=True;return s

def test_expired_stop_does_not_cancel_auto(robot,tmp_path):
    async def run():
        b,hw,a,c=prepared(robot,tmp_path);s=service_for(b,a);a.stop=AsyncMock()
        p=packet(s.dispatcher,1,body={'command':'STOP','epoch':b.epoch});p['issued_at']=0;n=len(hw.commands)
        await s.command(p);a.stop.assert_not_awaited();assert len(hw.commands)==n
        assert not s.channel.send_broadcast.call_args.args[1]['ok']
    asyncio.run(run())

def test_operator_ack_and_deduplication(robot,tmp_path):
    async def run():
        b,hw,a,c=prepared(robot,tmp_path);b.release_auto(a.token);s=service_for(b,a)
        p=packet(s.dispatcher,1,'operator',{'command':'MECHANICS OFF','epoch':b.epoch})
        await s.command(p);assert not a.operator.mechanics_enabled
        n=len(hw.commands);await s.command(p);assert len(hw.commands)==n
        assert s.channel.send_broadcast.call_args.args[1]['duplicate']
    asyncio.run(run())

def test_xm_in_red_keeps_camera_auto_running(robot,tmp_path):
    async def run():
        b,hw,a,c=prepared(robot,tmp_path,[raw()]*5);b.release_auto(a.token)
        await a.operator.set_mechanics(False);await a.start();s=service_for(b,a)
        await s.command(packet(s.dispatcher,1,body={'command':'XM','epoch':b.epoch}))
        assert a.running and not b.stopped
        await a.stop()
    asyncio.run(run())

def test_uncalibrated_rotation_sends_nothing(robot,tmp_path):
    async def run():
        b,hw,a,c=prepared(robot,tmp_path);a.cfg['grasp_orientation']={'enabled':True,'calibrated':False};n=len(hw.commands)
        with pytest.raises(NearHold,match='hiệu chuẩn'):await a.operator.rotate_for_grasp(a.target)
        assert len(hw.commands)==n
    asyncio.run(run())

def test_grasp_then_release_complete_with_live_simulated_sensors(robot,tmp_path):
    async def run():
        b,hw,a,c=prepared(robot,tmp_path);b.release_auto(a.token)
        await a.operator.set_mechanics(False)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],2310),d2=1300)
        hw.state['sensors']['D19']=0;b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        b.start_monitor();a.operator.observation_at=time.monotonic();a.state['target_fresh']=True
        closed=False
        async def sensors():
            nonlocal closed
            while True:
                if hw.state['joints'].get('d3')==b.config['grip']['close']:closed=True
                hw.state['sensors']['D19']=1 if closed and hw.state['joints'].get('d3')==b.config['grip']['open'] else 0
                await asyncio.sleep(.02)
        sensor=asyncio.create_task(sensors())
        try:
            await a.operator.action({'command':'GRASP','epoch':b.epoch})
            await asyncio.wait_for(a.operator.assist,30)
            assert a.state['state']=='manual_grasp_done',a.state
            assert hw.state['joints']['d2']==1300
            assert hw.state['joints']['p1']==1300
            assert hw.state['joints']['d3']==b.config['grip']['close']
            await a.operator.action({'command':'RELEASE','epoch':b.epoch})
            await asyncio.wait_for(a.operator.assist,30)
            assert a.state['state']=='manual_release_done',a.state
            assert hw.state['joints']['p4']==b.config['lid']['close']
            assert hw.state['joints']['d3']==b.config['grip']['open']
            assert a.operator.held_bin is None
        finally:
            sensor.cancel();await asyncio.gather(sensor,return_exceptions=True)
            await a.stop()
    asyncio.run(run())

def test_production_recovery_requires_current_visible_target():
    from test_box_tof_flow import PARAMS
    from config_io import read_config
    assert read_config(PARAMS)['box_tof_flow']['require_visible_target'] is True

def test_stale_target_cannot_authorize_manual_grasp(robot,tmp_path):
    async def run():
        b,hw,a,c=prepared(robot,tmp_path);b.release_auto(a.token)
        await a.operator.set_mechanics(False)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],2310))
        b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        a.operator.observation_at=time.monotonic();a.state['target_fresh']=False
        n=len(hw.commands)
        with pytest.raises(ValueError,match='khóa'):await a.operator.action({'command':'GRASP','epoch':b.epoch})
        assert len(hw.commands)==n
    asyncio.run(run())

def test_local_panel_requires_token_before_any_motion(robot,tmp_path):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from operator_ui import mount_operator
    b,hw,a,c=prepared(robot,tmp_path);app=FastAPI();mount_operator(app,b,a)
    with TestClient(app) as client:
        assert client.get('/control').status_code==200
        n=len(hw.commands)
        assert client.post('/operator/action',json={'action':'command','command':'STOP','epoch':b.epoch}).status_code==403
        assert len(hw.commands)==n

def test_running_auto_to_red_holds_servo_pulses(robot,tmp_path):
    async def run():
        b,hw,a,c=prepared(robot,tmp_path,[raw()]*10)
        async def active_auto():
            try:
                while True:await a.wait(.03)
            except asyncio.CancelledError:raise
            except Exception:await asyncio.to_thread(b.stop)
            finally:b.release_auto(a.token)
        a.task=asyncio.create_task(active_auto());await asyncio.sleep(.02);n=len(hw.commands)
        await a.operator.set_mechanics(False);await asyncio.sleep(.1)
        assert a.running and not b.stopped
        assert 'STOP' not in hw.commands[n:]
        assert hw.state['joints']
        await a.stop()
    asyncio.run(run())

def test_handover_of_held_object_and_release_at_place(robot,tmp_path):
    async def run():
        b,hw,a,c=prepared(robot,tmp_path)
        hw.state['joints'].update(b.config['poses']['PLACE'],d3=b.config['grip']['close'],p9=b.config['bins']['OTHER'],p4=b.config['lid']['open'])
        hw.state['sensors']['D19']=0;b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        a.selected_bin=4
        await a.operator.set_mechanics(False)
        assert a.operator.held_bin==4
        a.token='release';b.claim_auto(a.token);a.cancelled=False;a.deadline=time.monotonic()+30
        moves=[]
        async def move(targets,*args,**kwargs):
            moves.append(dict(targets));hw.state['joints'].update(targets)
            if targets.get('d3')==b.config['grip']['open']:hw.state['sensors']['D19']=1
            b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        a.move_targets=move;a.confirm_hold=AsyncMock()
        await a.operator.run_assist('RELEASE')
        assert a.state['state']=='manual_release_done',a.state
        assert moves[0]=={'p4':b.config['lid']['open']}
        assert not any('p9' in m for m in moves)
        open_index=next(i for i,m in enumerate(moves) if 'd3' in m)
        close_index=next(i for i,m in enumerate(moves) if m.get('p4')==b.config['lid']['close'])
        assert open_index<close_index
    asyncio.run(run())
