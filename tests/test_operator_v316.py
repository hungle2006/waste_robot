import asyncio
import time
import pytest
from test_control import robot, wait_for
from test_box_tof_flow import prepared
from grasp_orientation import angle_for_pulse, pulse_for_angle
from arm_path import targets_at
from test_operator_v315 import service_for
from test_realtime import packet


def manual(robot,tmp_path):
    b,hw,a,c=prepared(robot,tmp_path)
    b.release_auto(a.token)
    a.target=None;a.locked_bin=None;a.locked_label=None
    return b,hw,a,c


def ready_range(b,hw,op,mm=225):
    for _ in range(3):
        hw.tof_sample(mm);b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        op.poll_sensors()
    return op.snapshot()


@pytest.mark.parametrize('angle',[-90,-80,-45,-10,0,10,30,60])
def test_inverse_calibration_is_piecewise_not_a_single_scale(angle):
    assert angle_for_pulse(pulse_for_angle(angle))==pytest.approx(angle,abs=.08)


@pytest.mark.parametrize('pulse',[499,2501,True,None,float('nan')])
def test_unknown_wrist_position_is_rejected(pulse):
    with pytest.raises(ValueError):angle_for_pulse(pulse)


def test_wrist_steps_accumulate_from_ack_and_do_not_wrap_right_limit(robot,tmp_path):
    async def run():
        b,hw,a,_=manual(robot,tmp_path)
        for i in range(1,7):
            await a.operator.action({'command':'WRIST RIGHT','epoch':b.epoch})
            await asyncio.to_thread(wait_for,lambda:not b.busy)
            assert hw.state['joints']['d2']==pulse_for_angle(i*10)
        n=len(hw.commands)
        with pytest.raises(ValueError,match='giới hạn'):await a.operator.action({'command':'WRIST RIGHT','epoch':b.epoch})
        assert len(hw.commands)==n
        for i in range(5,-1,-1):
            await a.operator.action({'command':'WRIST LEFT','epoch':b.epoch})
            await asyncio.to_thread(wait_for,lambda:not b.busy)
            assert hw.state['joints']['d2']==pulse_for_angle(i*10)
        await a.operator.action({'command':'WRIST LEFT','epoch':b.epoch})
        await asyncio.to_thread(wait_for,lambda:not b.busy)
        assert hw.state['joints']['d2']==1922
        await a.operator.action({'command':'WRIST CENTER','epoch':b.epoch})
        await asyncio.to_thread(wait_for,lambda:not b.busy)
        assert hw.state['joints']['d2']==2100
    asyncio.run(run())


def test_wrist_never_races_busy_auto_or_stale_epoch(robot,tmp_path):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        n=len(hw.commands)
        with pytest.raises(ValueError,match='AUTO'):await a.operator.action({'command':'WRIST RIGHT','epoch':b.epoch})
        assert len(hw.commands)==n
        b.release_auto(a.token)
        with pytest.raises(ValueError,match='cũ'):await a.operator.action({'command':'WRIST RIGHT','epoch':b.epoch-1})
        b.command(None,b.epoch,'d2',500)
        with pytest.raises(ValueError,match='Đang chạy'):await a.operator.action({'command':'WRIST RIGHT','epoch':b.epoch})
    asyncio.run(run())


def test_manual_range_readiness_needs_new_samples_and_resets_after_motion(robot,tmp_path):
    b,hw,a,_=manual(robot,tmp_path)
    assert not a.operator.snapshot()['sensor_grasp_ready']
    s=ready_range(b,hw,a.operator)
    assert s['operator_protocol']==2 and s['sensor_grasp_ready']
    assert a.target is None and s['sensor_distance_ready']
    for _ in range(4):a.operator.poll_sensors()
    assert a.operator.votes==3
    hw.state['joints']['d2']=2300;b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
    assert not a.operator.snapshot()['sensor_grasp_ready']
    assert ready_range(b,hw,a.operator)['sensor_grasp_ready']
    b.updated-=1
    assert not a.operator.snapshot()['sensor_grasp_ready']


@pytest.mark.parametrize('failure',['range','pose','stopped','moving','closed','rear'])
def test_sensor_grasp_rejects_unready_without_moving(robot,tmp_path,failure):
    async def run():
        b,hw,a,_=manual(robot,tmp_path)
        ready_range(b,hw,a.operator)
        if failure=='range':hw.tof_sample(350)
        if failure=='pose':hw.state['joints']['p10']=1200
        if failure=='stopped':b.stop()
        if failure=='moving':hw.state['motors']=[10,10,10,10]
        if failure=='closed':hw.state['joints']['d3']=b.config['grip']['close']
        if failure=='rear':b.enable_rear(True)
        if failure!='stopped':b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        n=len(hw.commands)
        with pytest.raises((ValueError,ConnectionError)):await a.operator.action({'command':'GRASP SENSOR','epoch':b.epoch})
        assert len(hw.commands)==n
        assert not a.operator.snapshot()['sensor_grasp_ready']
    asyncio.run(run())


def test_sensor_grasp_and_release_without_ai_keep_manual_wrist(robot,tmp_path):
    async def run():
        b,hw,a,_=manual(robot,tmp_path)
        a.vision=None
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],2050),d2=2300)
        b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        assert ready_range(b,hw,a.operator)['sensor_grasp_ready']
        b.start_monitor();closed=False
        async def sensors():
            nonlocal closed
            while True:
                joints=hw.state['joints']
                if joints.get('d3')==b.config['grip']['close']:closed=True
                hw.state['sensors']['D19']=0 if (joints.get('p1')==2310 or closed) and not (closed and joints.get('d3')==b.config['grip']['open']) else 1
                hw.tof_sample(225)
                await asyncio.sleep(.02)
        sensor=asyncio.create_task(sensors());hw.commands.clear()
        try:
            await a.operator.action({'command':'GRASP SENSOR','epoch':b.epoch})
            await asyncio.wait_for(a.operator.assist,40)
            assert a.state['state']=='manual_grasp_done',a.state
            assert hw.state['joints']['d2']==2300
            assert hw.state['joints']['d3']==b.config['grip']['close']
            assert not [cmd for cmd in hw.commands if cmd.startswith('S 2 ') and cmd!='S 2 2300']
            assert a.operator.snapshot()['release_ready']
            await a.operator.action({'command':'RELEASE','epoch':b.epoch})
            await asyncio.wait_for(a.operator.assist,40)
            assert a.state['state']=='manual_release_done',a.state
            assert hw.state['joints']['p4']==b.config['lid']['close']
            assert a.operator.held_bin is None
        finally:
            sensor.cancel();await asyncio.gather(sensor,return_exceptions=True)
            await a.stop()
    asyncio.run(run())


def test_hold_cancels_manual_assist_and_keeps_pulses(robot,tmp_path):
    async def run():
        b,hw,a,_=manual(robot,tmp_path)
        a.operator.mechanics_enabled=False
        a.operator.assist=asyncio.create_task(asyncio.sleep(10))
        b.command(None,b.epoch,'d2',500)
        await asyncio.sleep(.04)
        await a.operator.action({'command':'HOLD','epoch':b.epoch})
        at=len(hw.commands);p=dict(hw.state['joints'])
        await asyncio.sleep(.1)
        assert not b.stopped and not b.busy and a.operator.assist.done()
        assert hw.state['joints']==p
        assert not [x for x in hw.commands[at:] if x!='STATUS']
    asyncio.run(run())


def test_lift_open_gripper_preserves_angle(robot,tmp_path):
    async def run():
        b,hw,a,_=manual(robot,tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],1400),d2=1300)
        b.last_hw=hw.request('STATUS');b.updated=time.monotonic();b.start_monitor()
        await a.operator.action({'command':'LIFT','epoch':b.epoch})
        await asyncio.wait_for(a.operator.assist,20)
        assert a.state['state']=='manual_lift_done',a.state
        assert hw.state['joints']['p1']==1300 and hw.state['joints']['d2']==1300
    asyncio.run(run())


def test_wrist_packet_duplicate_is_only_one_step(robot,tmp_path):
    async def run():
        b,hw,a,_=manual(robot,tmp_path);s=service_for(b,a)
        p=packet(s.dispatcher,1,'operator',{'command':'WRIST RIGHT','epoch':b.epoch})
        await s.command(p);await asyncio.to_thread(wait_for,lambda:not b.busy)
        await s.command(p)
        assert hw.state['joints']['d2']==2167
    asyncio.run(run())


def test_service_collects_manual_distance_without_camera_or_auto(robot,tmp_path):
    async def run():
        b,hw,a,_=manual(robot,tmp_path);a.vision=None
        service=service_for(b,a);service.online=False
        task=asyncio.create_task(service.run())
        try:
            for _ in range(7):
                hw.tof_sample(225);b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
                await asyncio.sleep(.06)
            assert a.operator.snapshot()['sensor_grasp_ready']
            assert not a.running and b.auto_owner is None
        finally:
            task.cancel();await asyncio.gather(task,return_exceptions=True)
    asyncio.run(run())


def test_sensor_grasp_reads_usb_ir_again_before_lowering(robot,tmp_path):
    async def run():
        b,hw,a,_=manual(robot,tmp_path)
        ready_range(b,hw,a.operator)
        assert b.last_hw['sensors']['D19']==1
        hw.state['sensors']['D19']=0  # Arrived after the previous 200ms background poll.
        n=len(hw.commands)
        await a.operator.action({'command':'GRASP SENSOR','epoch':b.epoch})
        await asyncio.wait_for(a.operator.assist,5)
        assert a.state['state']=='manual_assist_error'
        assert 'trước tư thế cuối' in a.state['message']
        assert not any(x.startswith(('P ','S ','C ','G ')) for x in hw.commands[n:])
    asyncio.run(run())


def test_sensor_grasp_cannot_claim_after_epoch_changes_during_validation(robot,tmp_path,monkeypatch):
    async def run():
        import auto_controller
        b,hw,a,_=manual(robot,tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],2310))
        hw.state['sensors']['D19']=0;b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        old=b.epoch;load=auto_controller.load_auto_config
        def changed_epoch(*args,**kwargs):
            value=load(*args,**kwargs);b.epoch+=1;return value
        monkeypatch.setattr(auto_controller,'load_auto_config',changed_epoch)
        n=len(hw.commands)
        with pytest.raises(ValueError,match='cũ'):await a.operator.action({'command':'GRASP SENSOR','epoch':old})
        assert b.auto_owner is None and a.operator.assist is None
        assert a.operator.held_bin is None
        assert len(hw.commands)==n
    asyncio.run(run())
