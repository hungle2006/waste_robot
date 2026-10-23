"""V3.14.1: physical approach decisions, early range handoff and late vision."""
import asyncio
import copy
import json
import time
from unittest.mock import AsyncMock

import pytest

from test_control import robot, wait_for
from test_auto import finish
from test_target_lock import obj as tracked, until
from test_box_tof_flow import prepared, raw, config, PARAMS
from auto_controller import AutoController, AutoFault, NearHold, RearRequested
from box_tof_flow import BoxToFFlow
from arm_path import targets_at
from brain import Brain
from config_io import read_config
from transport import SimTransport


@pytest.mark.parametrize('box,expected',[
    ([0,.3,.3,.7],'LEFT'),
    ([.7,.3,1,.7],'RIGHT'),
    ([0,.3,1,.7],'both_sides'),
    ([.515,.4,.525,.42],'FORWARD'),
    ([.8,.4,.81,.42],'RIGHT'),
])
def test_clipped_or_small_correct_lock_has_recovery_action(robot,tmp_path,box,expected):
    async def run():
        _,_,a,_=prepared(robot,tmp_path)
        a.detect=AsyncMock(return_value=copy.deepcopy(a.target));a.prepare_bin=AsyncMock()
        f=BoxToFFlow(a);f.observe=AsyncMock(return_value=dict(raw(box),**tracked(box,'can',4)))
        actions=[]
        async def stop(action):actions.append(action);raise NearHold('test complete')
        f.box_drive=stop;f.sensor_approach=stop
        with pytest.raises(NearHold,match='test complete'):await f.run()
        assert actions==[expected]
    asyncio.run(run())


def stream_range(b,hw,mm=225,freeze=False):
    if not freeze:hw.tof_sample(mm)
    original=hw.request
    def request(command):
        if command=='STATUS' and not freeze:
            if hw.tof_at is None or time.monotonic()-hw.tof_at>=.02:
                hw.tof_sample(mm)
        return original(command)
    hw.request=request
    b.last_hw=hw.request('STATUS');b.updated=time.monotonic();b.start_monitor()


@pytest.mark.parametrize('p1,reason,mm',[
    (1300,'both_sides',200),(1550,'both_sides',250),
    (1800,'forward_range',225),(2000,'forward_range',225),
    (2050,'handoff',225),
])
def test_range_confirmation_can_start_at_current_stored_pose(robot,tmp_path,p1,reason,mm):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],p1))
        f=BoxToFFlow(a);f.last_obj=copy.deepcopy(a.target);f.last_action='FORWARD'
        if reason=='both_sides':f.last_obj['bbox_norm']=[0,.3,1,.7]
        stream_range(b,hw,mm)
        async def final():
            assert f.tof_confirmed and f.tof_pose==p1 and a.vision_paused
            assert not any(hw.state['motors']) and not hw.state['tof']['guard']
            raise NearHold('confirmed')
        f.final_approach=final
        n=len(hw.commands)
        with pytest.raises(NearHold,match='confirmed'):await f.sensor_approach(reason)
        votes=[e.args[0] for e in a.report.call_args_list if e.args[0]['state']=='auto_tof_observe']
        assert votes and votes[-1]['tof_votes']==1
        assert not any(s.startswith(('P ','S ','G ')) for s in hw.commands[n:])
        assert hw.state['joints']['p1']==p1
        assert a.vision.calls==0 and a.selected_bin==4
    asyncio.run(run())


@pytest.mark.parametrize('kind',['below','invalid'])
def test_early_range_does_not_bypass_invalid_or_unconfirmed_samples(robot,tmp_path,kind):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        f=BoxToFFlow(a);f.last_obj=copy.deepcopy(a.target);f.last_obj['bbox_norm']=[0,.3,1,.7]
        hw.tof_sample(190 if kind=='below' else 225,valid=kind!='invalid')
        original=hw.request
        def request(command):
            if command=='STATUS':hw.tof_sample(190 if kind=='below' else 225,valid=kind!='invalid')
            return original(command)
        hw.request=request;b.start_monitor()
        b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        n=len(hw.commands)
        with pytest.raises(NearHold):await f.sensor_approach('both_sides')
        assert not f.tof_confirmed and a.final_authorized is None
        assert not any(s.startswith(('P ','S ')) for s in hw.commands[n:])
        assert any(s.startswith('G ') for s in hw.commands[n:]) == (kind == 'invalid')
        if f.tof_guard_armed:b.tof_guard(False,a.token)
    asyncio.run(run())


def test_pending_detector_is_cancelled_before_range_handoff(robot,tmp_path):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        started=asyncio.Event();cancelled=asyncio.Event()
        async def detect():
            started.set()
            try:await asyncio.Event().wait()
            finally:cancelled.set()
        a._detect_once=detect
        f=BoxToFFlow(a);f.last_action='FORWARD';f.last_obj=copy.deepcopy(a.target)
        async def sensor(reason):
            assert reason=='forward_range' and cancelled.is_set()
            raise NearHold('range handoff')
        f.sensor_approach=sensor
        task=asyncio.create_task(f.observe())
        await started.wait()
        hw.tof_sample(225);b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        with pytest.raises(NearHold,match='range handoff'):await asyncio.wait_for(task,1)
        assert cancelled.is_set()
    asyncio.run(run())


@pytest.mark.parametrize('when',['before_G','during_G'])
def test_camera_forward_brakes_when_range_arrives_during_command_or_pulse(robot,tmp_path,when):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        original=hw.request;sent_at=None;blocked=[]
        def request(command):
            nonlocal sent_at
            if command.startswith('G '):
                sent_at=time.monotonic()
                if when=='before_G':hw.tof_sample(225)
            if command=='STATUS' and (hw.tof_at is None or time.monotonic()-hw.tof_at>=.02):
                close=sent_at is not None and (when=='before_G' or time.monotonic()-sent_at>=.04)
                hw.tof_sample(225 if close else 800)
            try:return original(command)
            except ValueError as exc:
                blocked.append(str(exc));raise
        hw.request=request;b.last_hw=hw.request('STATUS');b.updated=time.monotonic();b.start_monitor()
        f=BoxToFFlow(a);f.last_obj=copy.deepcopy(a.target)
        async def sensor(reason):
            assert reason=='forward_range' and f.last_action=='FORWARD'
            assert not any(hw.state['motors']) and hw.profile is None
            assert time.monotonic()-sent_at<.5
            raise NearHold('range brake succeeded')
        f.sensor_approach=sensor
        n=len(hw.commands)
        with pytest.raises(NearHold,match='range brake succeeded'):await f.box_drive('FORWARD')
        assert not any(s.startswith(('P ','S ')) for s in hw.commands[n:])
        assert not hw.state['tof']['guard']
        if when=='before_G':assert 'TOF_BLOCKS_AUTO_DRIVE' in blocked
    asyncio.run(run())


@pytest.mark.parametrize('when',['before_G','during_G'])
def test_invalid_range_allows_camera_pulse_without_gripping(robot,tmp_path,when):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        original=hw.request;sent=False
        def request(command):
            nonlocal sent
            if command.startswith('G '):sent=True
            if command=='STATUS':hw.tof_sample(800,valid=not (when=='before_G' or sent))
            return original(command)
        hw.request=request;b.last_hw=hw.request('STATUS');b.updated=time.monotonic();b.start_monitor()
        f=BoxToFFlow(a);f.last_obj=copy.deepcopy(a.target);n=len(hw.commands)
        await f.box_drive('FORWARD')
        assert not f.tof_confirmed and a.final_authorized is None
        assert not any(s.startswith(('P ','S ')) for s in hw.commands[n:])
        assert not any(hw.state['motors']) and hw.profile is None and not hw.state['tof']['guard']
        assert any(s.startswith('G ') for s in hw.commands[n:])
    asyncio.run(run())


@pytest.mark.parametrize('last',['LOWER','LEFT','RIGHT'])
def test_passive_range_does_not_shortcut_non_forward_actions(robot,tmp_path,last):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path,[None]*3)
        hw.tof_sample(225);b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        f=BoxToFFlow(a);f.last_action=last;f.sensor_approach=AsyncMock()
        assert await f.observe() is None
        assert a.vision.calls==3
        f.sensor_approach.assert_not_called()
    asyncio.run(run())


@pytest.mark.parametrize('reason',['handoff','both_sides','forward_range','distance_ready','unknown'])
def test_unauthorized_early_entry_cannot_move_hand_or_wheels(robot,tmp_path,reason):
    async def run():
        _,hw,a,_=prepared(robot,tmp_path)
        f=BoxToFFlow(a);f.last_obj=copy.deepcopy(a.target)
        n=len(hw.commands)
        with pytest.raises(AutoFault):await f.sensor_approach(reason)
        assert not any(s.startswith(('G ','P ','S ')) for s in hw.commands[n:])
    asyncio.run(run())


class RecoveryTransport(SimTransport):
    def __init__(self,grip):
        super().__init__();self.grip=grip
        self.held=self.released=False
        self.early_pulses=self.ir_pulses=0
        self.tof_sample(900)

    def request(self,command):
        j=self.state['joints']
        if command.startswith('G 100 0 0 '):
            if j.get('p1',0)<2050:self.early_pulses+=1
            elif j.get('p1')==2310:self.ir_pulses+=1
        if command=='STATUS':
            if j.get('d3')==self.grip['close']:self.held=True
            if self.held and j.get('d3')==self.grip['open']:self.released=True
            active=(self.held or (j.get('p1')==2310 and self.ir_pulses>=2)) and not self.released
            self.state['sensors']['D19']=0 if active else 1
            if self.tof_at is None or time.monotonic()-self.tof_at>=.06:
                self.tof_sample(max(225,900-310*self.early_pulses))
        return super().request(command)


class RecoveryVision:
    jpeg=b'synthetic-recovery-frame';frame_at=0.
    def __init__(self,hw,kind):self.hw=hw;self.kind=kind;self.calls=0;self.missing=0
    async def detect(self,fresh_after=None):
        self.calls+=1;self.frame_at=time.monotonic()
        if self.hw.state['joints']['p1']>=2050:raise AssertionError('Late detector call')
        if self.kind=='both':box=[0,.30,1,.65]
        else:box=[.42,.15,.62,.45]
        if self.kind=='missing' and self.hw.early_pulses:
            self.missing+=1;objects=[]
        else:objects=[raw(box)]
        return {'found':bool(objects),'objects':objects,'_frame_at':self.frame_at,
                '_frame_size':[640,360],'_frame_jpeg':self.jpeg}


@pytest.mark.parametrize('kind',['both','missing'])
def test_early_full_cycle_reaches_grip_and_frozen_bin_without_more_vision(tmp_path,kind):
    async def run():
        r=read_config(PARAMS,'robot');r.update(step_us=100,step_ms=20,settle_ms=0)
        c=config();p=tmp_path/'robot.json';q=tmp_path/'auto.json'
        p.write_text(json.dumps(r));q.write_text(json.dumps(c))
        hw=RecoveryTransport(r['grip']);b=Brain(hw,p,True);b.connect();b.start_monitor()
        wait_for(lambda:b.snapshot()['fresh'] and not b.busy)
        v=RecoveryVision(hw,kind);a=AutoController(b,v,q,AsyncMock())
        try:
            await a.start();await finish(a,timeout=50)
            assert a.state['state']=='auto_done' and hw.released,a.state
            events=[call.args[0] for call in a.report.call_args_list]
            entry=[e for e in events if e['state']=='auto_tof_handoff']
            assert len(entry)==1 and entry[0]['s2_us']==1300
            assert entry[0]['tof_entry_reason']==('both_sides' if kind=='both' else 'forward_range')
            assert hw.early_pulses==3 and hw.ir_pulses==2
            assert 'P 1 2310' in hw.commands and 'P 10 650' in hw.commands
            assert all(e['bin']==4 for e in events if e['state']=='auto_grip')
            if kind=='missing':
                assert v.missing==6  # Two groups of three missed frames before the nearer window.
                assert 'G 100 0 0 20 160 240' in hw.commands
                assert 'G 100 0 0 20 80 0' in hw.commands
            else:assert not any(s=='G 100 0 0 20 160 240' for s in hw.commands)
            assert all(s.split()[1:4]==['100','0','0'] for s in hw.commands if s.startswith('G '))
        finally:
            if a.running:await a.stop()
            b.close()
    asyncio.run(run())


@pytest.mark.parametrize('kind',['stop','rear'])
def test_early_sensor_phase_obeys_stop_and_rear_interrupt(robot,tmp_path,kind):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        stream_range(b,hw,800);b.enable_rear(True)
        f=BoxToFFlow(a);f.last_obj=copy.deepcopy(a.target);f.last_obj['bbox_norm']=[0,.3,1,.7]
        task=asyncio.create_task(f.sensor_approach('both_sides'))
        await until(lambda:any(s.startswith('G ') for s in hw.commands))
        if kind=='stop':
            a.task=task;await a.stop(emergency=True)
            assert task.cancelled() and b.stopped
        else:
            hw.state['sensors']['D18']=0
            with pytest.raises(RearRequested):await asyncio.wait_for(task,2)
        n=len(hw.commands);await asyncio.sleep(.15)
        assert not any(s.startswith(('G ','P ','S ')) for s in hw.commands[n:])
        assert not any(hw.state['motors'])
        if b.auto_owner==a.token:b.tof_guard(False,a.token)
    asyncio.run(run())
