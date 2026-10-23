"""V3.14: rolling identity, 1/3 fresh frames, finite retries, ToF + IR gates."""
import asyncio
import copy
import json
import time
from pathlib import Path
from unittest.mock import AsyncMock

import pytest

from test_control import robot, ready, wait_for
from test_auto import finish
from test_target_lock import until, obj as tracked
from auto_controller import (AutoController, load_auto_config, NearHold,
                             CloseRangeContact, RearRequested)
from box_tof_flow import (BoxToFFlow, DEFAULTS, checked_tof_flow,
                          nominal_for_actual, vertical_edge, tof_sample)
from config_io import read_config
from arm_path import targets_at
from target_lock import associate_rolling
from demo_auto import ToFDemoTransport, ToFDemoVision
from brain import Brain

PARAMS=Path(__file__).resolve().parents[1]/'python/robot_params.py'


def config():
    c=read_config(PARAMS)
    c['box_tof_flow']['require_visible_target']=False  # legacy recovery scenarios
    c['grasp_orientation']['enabled']=False  # These legacy tests predate SEARCH-angle capture.
    c['control'].update(settle_ms=100,repeat=False,max_cycle_s=120)
    c['target_lock'].update(continuous_auto=False,retry_ms=100)
    c['motion']['arm_step_ms']=20
    c['box_tof_flow'].update(retry_ms=100,max_final_pulses=3,final_timeout_ms=2000,
                            tof_fault_timeout_ms=1000,tof_approach_timeout_ms=5000)
    for s in c['pose_profile']['stages']:s.update(settle_ms=100,arm_step_ms=20)
    return c


class SequenceVision:
    jpeg=b'fresh-synthetic-frame'
    frame_at=0.
    def __init__(self,sequence):self.sequence=list(sequence);self.calls=0
    async def detect(self,fresh_after=None):
        self.calls+=1
        item=self.sequence.pop(0) if self.sequence else None
        self.frame_at=time.monotonic()
        objects=[] if item is None else item if isinstance(item,list) else [item]
        return {'found':bool(objects),'objects':objects,'_frame_size':[640,360],
                '_frame_at':self.frame_at,'_frame_jpeg':self.jpeg}


def raw(box=(.42,.3,.62,.65),label='can',bin=4):
    return dict(label=label,bin=bin,bbox=[v*s for v,s in zip(box,[640,360,640,360])],_appearance=[1.,0.])


def prepared(robot,tmp_path,sequence=()):
    b,hw=robot;ready(b)
    c=config();p=tmp_path/'tof.json';p.write_text(json.dumps(c))
    hw.tof_sample(800)
    b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
    a=AutoController(b,SequenceVision(sequence),p,AsyncMock())
    a.cfg=load_auto_config(p,b.config);a.token='tof-test';b.claim_auto(a.token)
    a.deadline=time.monotonic()+120;a.flow='front'
    a.lock_sequence=1;a.locked_bin=a.selected_bin=4;a.locked_label='can'
    a.target=dict(raw(),**tracked([.42,.3,.62,.65],'can',4))
    a.lock_frame_size=[640,360];a.lock_last_seen=time.monotonic()
    return b,hw,a,c


def test_defaults_keep_sensor_flow_exact_pose_and_actual_timing():
    c=load_auto_config(PARAMS,read_config(PARAMS,'robot'))
    assert c['box_tof_flow']['enabled']
    assert not c['distance']['enabled']
    assert c['box_tof_flow']['missing_frames']==3
    assert targets_at(c['arm_path'],2050)['p10']==727
    assert targets_at(c['arm_path'],2310)['p10']==650
    assert nominal_for_actual(160,160)==80
    assert all(s['forward_ms']==400 for s in c['pose_profile']['stages'])


@pytest.mark.parametrize('key,value',[
    ('enabled',1),('missing_frames',1),('step_us',51),('handoff_p10_us',728),
    ('tof_min_mm',300),('tof_confirm_samples',1),('tof_max_age_ms',1000),
    ('slow_pulse_ms',101),('edge_margin_norm',float('nan')),('max_recovery_actions',0),
])
def test_bad_settings_rejected(key,value):
    c=config();f=copy.deepcopy(c['box_tof_flow']);f[key]=value
    with pytest.raises(ValueError):checked_tof_flow(f,c)


@pytest.mark.parametrize('key',['distance'])
def test_floor_distance_and_sensor_flow_cannot_both_be_enabled(key):
    c=config();c[key]['enabled']=True
    with pytest.raises(ValueError,match='tắt'):checked_tof_flow(c['box_tof_flow'],c)


@pytest.mark.parametrize('box,expected',[
    ([.3,.1,.7,.8],None),([.2,.01,.8,.8],'top'),
    ([.3,.2,.7,.99],'bottom'),([.2,0,.8,1],'top+bottom'),
])
def test_edge_is_not_a_size_or_occlusion_classifier(box,expected):
    assert vertical_edge(box,8/360)==expected


def test_rolling_box_growth_is_relative_to_latest_not_original_and_ambiguous_waits():
    c=config();f=c['box_tof_flow'];lock=c['target_lock']
    start=ref=tracked([.48,.45,.52,.55],appearance=[1.,0.])
    for box in ([.46,.42,.54,.58],[.43,.36,.57,.64],[.38,.26,.62,.74],[.3,.1,.7,.9]):
        new=tracked(box,appearance=[1.,0.])
        assert associate_rolling(ref,[new],lock,f)[0]==0
        ref=new
    assert associate_rolling(start,[ref],lock,f)[0] is None
    assert associate_rolling(ref,[ref,copy.deepcopy(ref)],lock,f)[0] is None
    assert associate_rolling(start,[tracked([.8,.1,.95,.3],'can',4)],lock,f)[0] is None


def test_one_correct_frame_updates_box_and_keeps_locked_label_bin(robot,tmp_path):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path,[raw((.40,.28,.64,.69),'another name',2)])
        n=len(hw.commands);flow=BoxToFFlow(a)
        found=await flow.observe()
        assert a.vision.calls==1 and found['bbox_norm']==pytest.approx([.40,.28,.64,.69])
        assert a.target['bbox_norm']==found['bbox_norm']
        assert (found['label'],found['bin'],found['observed_bin'])==('can',4,2)
        assert not any(s.startswith(('G ','P ','S ')) for s in hw.commands[n:])
    asyncio.run(run())


def test_misses_then_one_correct_frame_succeeds_without_extra_confirmation(robot,tmp_path):
    async def run():
        _,_,a,_=prepared(robot,tmp_path,[None,raw((.8,.1,.95,.3)),raw()])
        result=await BoxToFFlow(a).observe()
        assert result and a.vision.calls==3
        assert [call.args[0]['missing_frames'] for call in a.report.call_args_list
                if call.args[0]['state']=='auto_tof_missing']==[1,2]
    asyncio.run(run())


def test_three_wrong_or_missing_frames_keep_reference_and_return_no_live_box(robot,tmp_path):
    async def run():
        _,_,a,_=prepared(robot,tmp_path,[raw((.8,.1,.95,.3)),None,None,raw()])
        old=copy.deepcopy(a.target)
        assert await BoxToFFlow(a).observe() is None
        assert a.vision.calls==3 and a.target==old
    asyncio.run(run())


def test_network_failures_do_not_count_as_three_negative_frames(robot,tmp_path):
    from auto_controller import VisionUnavailable
    async def run():
        _,hw,a,_=prepared(robot,tmp_path)
        a._detect_once=AsyncMock(side_effect=VisionUnavailable('network unavailable'))
        n=len(hw.commands)
        with pytest.raises(NearHold,match='kết quả mới'):await BoxToFFlow(a).observe()
        assert not any(s.startswith(('G ','P ','S ')) for s in hw.commands[n:])
    asyncio.run(run())


@pytest.mark.parametrize('last_action',['FORWARD','LOWER','LEFT'])
def test_three_misses_repeat_exact_last_action_and_stop_between_checks(robot,tmp_path,last_action):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path,[None]*6)
        a.detect=AsyncMock(return_value=copy.deepcopy(a.target))
        a.prepare_bin=AsyncMock()
        flow=BoxToFFlow(a);flow.last_action=last_action
        actions=[]
        async def act(*args):
            actions.append((last_action,a.vision.calls))
            if len(actions)==2:raise NearHold('end test')
        flow.lower=act;flow.box_drive=act
        with pytest.raises(NearHold,match='end test'):await flow.run()
        assert actions==[(last_action,3),(last_action,6)]
        assert all(call.args[0].get('missing_frames') in (1,2,3)
                   for call in a.report.call_args_list if call.args[0]['state']=='auto_tof_missing')
    asyncio.run(run())


def test_persistent_loss_has_finite_repeat_budget(robot,tmp_path):
    async def run():
        _,_,a,_=prepared(robot,tmp_path,[None]*12)
        a.detect=AsyncMock(return_value=copy.deepcopy(a.target));a.prepare_bin=AsyncMock()
        flow=BoxToFFlow(a);flow.last_action='FORWARD';flow.c['max_recovery_actions']=2
        flow.box_drive=AsyncMock()
        with pytest.raises(NearHold,match='Hết số lần'):await flow.run()
        assert a.vision.calls==9 and flow.box_drive.await_count==2
        assert all(call.args==('FORWARD',) for call in flow.box_drive.call_args_list)
    asyncio.run(run())


@pytest.mark.parametrize('case,box,expected',[
    ('above',[.42,.1,.62,.45],'FORWARD'),
    ('inside',[.42,.3,.62,.575],'LOWER'),
    ('below',[.42,.4,.62,.8],'LOWER'),
    ('top_clip',[.42,0,.62,.8],'FORWARD'),
    ('bottom_clip',[.42,.3,.62,.99],'FORWARD'),
])
def test_box_decisions_use_current_frame_and_edge_overrides_lowering(robot,tmp_path,case,box,expected):
    async def run():
        _,_,a,_=prepared(robot,tmp_path)
        a.detect=AsyncMock(return_value=copy.deepcopy(a.target));a.prepare_bin=AsyncMock()
        flow=BoxToFFlow(a)
        entry=dict(raw(box),**tracked(box,'can',4))
        flow.observe=AsyncMock(return_value=entry)
        actions=[]
        async def drive(move):actions.append(move);raise NearHold('end test')
        async def lower():actions.append('LOWER');raise NearHold('end test')
        flow.box_drive=drive;flow.lower=lower
        with pytest.raises(NearHold,match='end test'):await flow.run()
        assert actions==[expected]
    asyncio.run(run())


def sample_snapshot():
    return {'age_ms':20,'hardware':{'tof_protocol':1,'tof':{
        'present':True,'valid':True,'mm':225,'seq':4,'status':0,'age_ms':40,'window_count':3}}}


@pytest.mark.parametrize('field,value',[
    ('valid',False),('mm',0),('mm',float('nan')),('seq',0),('status',4),
    ('age_ms',240),('age_ms',None),('present',False),
])
def test_bad_tof_sample_never_authorizes_motion(field,value):
    s=sample_snapshot();s['hardware']['tof'][field]=value
    with pytest.raises(ValueError):tof_sample(s,DEFAULTS)


def test_valid_tof_sample_converts_no_camera_or_focal_units():
    assert tof_sample(sample_snapshot(),DEFAULTS)==(4,225)


def test_old_firmware_or_absent_sensor_refused_before_auto_movement(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);c=config();p=tmp_path/'tof.json';p.write_text(json.dumps(c))
        b.last_hw['tof_protocol']=0
        a=AutoController(b,SequenceVision([]),p,AsyncMock());n=len(hw.commands)
        await a.start()
        assert a.state['state']=='auto_not_ready' and not a.running
        assert not any(s.startswith(('G ','P ','S ')) for s in hw.commands[n:])
    asyncio.run(run())


def test_final_pose_cannot_be_called_without_confirmed_tof_distance(robot,tmp_path):
    async def run():
        _,hw,a,_=prepared(robot,tmp_path);flow=BoxToFFlow(a);flow.last_obj=copy.deepcopy(a.target)
        n=len(hw.commands)
        with pytest.raises(ValueError,match='Chưa đủ'):await flow.final_approach()
        assert not any(s.startswith(('P ','S ','G ')) for s in hw.commands[n:])
    asyncio.run(run())


def full_demo(tmp_path):
    r=read_config(PARAMS,'robot');r.update(step_us=100,step_ms=20,settle_ms=0)
    c=config()
    # Legacy ToF demo camera deliberately blocks inference from P1=2050 onward.
    # Its regression covers the sensor-only mode, not AUTO D2-angle capture.
    c['grasp_orientation']['enabled']=False
    p=tmp_path/'r.json';p.write_text(json.dumps(r));q=tmp_path/'a.json';q.write_text(json.dumps(c))
    hw=ToFDemoTransport(2310,r['grip']);b=Brain(hw,p,True);b.connect();b.start_monitor()
    wait_for(lambda:b.snapshot()['fresh'] and not b.busy)
    v=ToFDemoVision(hw,c['pose_profile']);a=AutoController(b,v,q,AsyncMock())
    return b,hw,a,v


def test_complete_cycle_no_vision_from2050_to_grip_release(tmp_path):
    async def run():
        b,hw,a,v=full_demo(tmp_path)
        try:
            await a.start();await finish(a,timeout=45)
            assert a.state['state']=='auto_done' and hw.released,a.state
            assert max(v.calls)<2050
            assert hw.slow_pulses==3 and hw.ir_pulses==2
            sent=hw.commands
            assert 'P 1 2050' in sent and 'P 10 727' in sent
            assert 'P 1 2310' in sent and 'P 10 650' in sent
            assert not any(s.startswith(('G -','G 0 0 ','C -')) for s in sent)
            assert 'TF AUTO 200 250' in sent and 'TF OFF' in sent
            assert sent.index('P 4 1600')<sent.index('P 1 2310')<sent.index('S 3 2300')
            events=[v.args[0] for v in a.report.call_args_list]
            obs=[e for e in events if e['state']=='auto_tof_observe' and 200<=e['tof_mm']<=250]
            assert obs and obs[-1]['tof_votes']==1
            assert all(e['bin']==4 and e['label']=='can' for e in events if e['state']=='auto_grip')
        finally:
            if a.running:await a.stop()
            b.close()
    asyncio.run(run())


@pytest.mark.parametrize('kind',['below','invalid'])
def test_tof_fault_or_below_window_never_lowers_final_or_grips(robot,tmp_path,kind):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],2050))
        original=hw.request
        def read(command):
            hw.tof_sample(190 if kind=='below' else 225,valid=kind!='invalid')
            return original(command)
        hw.request=read;b.last_hw=hw.request('STATUS');b.updated=time.monotonic();b.start_monitor()
        flow=BoxToFFlow(a);flow.last_obj=copy.deepcopy(a.target)
        try:
            with pytest.raises(NearHold):await flow.sensor_approach()
            assert hw.state['joints']['p1']==2050
            assert 'S 3 2300' not in hw.commands and 'P 1 2100' not in hw.commands
            assert not flow.tof_confirmed
        finally:
            if b.auto_owner==a.token:b.tof_guard(False,a.token)
    asyncio.run(run())


def test_firmware_sim_tof_guard_brakes_and_blocks_further_pulse(robot,tmp_path):
    b,hw,a,_=prepared(robot,tmp_path)
    b.tof_guard(True,a.token,200,250)
    hw.request('G 100 0 0 20 80 0')
    hw.tof_sample(225)
    state=hw.request('STATUS')
    assert state['motors']==[0,0,0,0] and hw.profile is None
    with pytest.raises(ValueError,match='TOF_BLOCKS'):hw.request('G 100 0 0 20 80 0')
    b.tof_guard(False,a.token)


def test_camera_guides_show_new_flow_and_leave_raw_frame_clean(monkeypatch):
    import cv2
    import numpy as np
    from types import SimpleNamespace
    from camera_guides import CameraGuides
    c=config();labels=[];original=cv2.putText
    def put(img,text,*args,**kwargs):labels.append(text);return original(img,text,*args,**kwargs)
    monkeypatch.setattr(cv2,'putText',put)
    controller=SimpleNamespace(running=True,cfg=c,state={'auto_running':True,'phase':'box_follow','s2_us':1300})
    raw_image=np.zeros((360,640,3),np.uint8)
    shown=CameraGuides(PARAMS,controller).render(raw_image)
    assert not raw_image.any() and shown.any()
    assert any('MATCH 1 FRAME / MISS 3 FRAMES' in s for s in labels)
    controller.state.update(phase='tof_approach',s2_us=2050,tof_mm=225)
    labels.clear();CameraGuides(PARAMS,controller).render(raw_image)
    assert any('VISION OFF / VL53L1X ONLY' in s for s in labels)
    assert not any('VL53 22.5cm' in s for s in labels)
    controller.state.update(tof_valid=True)
    labels.clear();CameraGuides(PARAMS,controller).render(raw_image)
    assert any('VL53 22.5cm' in s for s in labels)


@pytest.mark.parametrize('kind',['stop','rear'])
def test_stop_and_d18_interrupt_sensor_phase_without_late_motion(robot,tmp_path,kind):
    async def run():
        b,hw,a,_=prepared(robot,tmp_path)
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],2050))
        original=hw.request
        def read(command):
            if command=='STATUS':hw.tof_sample(800)
            return original(command)
        hw.request=read;b.last_hw=hw.request('STATUS');b.updated=time.monotonic();b.start_monitor();b.enable_rear(True)
        flow=BoxToFFlow(a);flow.last_obj=copy.deepcopy(a.target)
        task=asyncio.create_task(flow.sensor_approach())
        await until(lambda:any(s.startswith('G ') for s in hw.commands))
        if kind=='stop':
            a.task=task;await a.stop(emergency=True)
            assert task.cancelled() and b.stopped
        else:
            hw.state['sensors']['D18']=0
            with pytest.raises(RearRequested):await asyncio.wait_for(task,2)
        n=len(hw.commands);await asyncio.sleep(.15)
        assert not any(s.startswith(('P ','S ','G ')) for s in hw.commands[n:])
        assert not any(hw.state['motors'])
        if b.auto_owner==a.token:b.tof_guard(False,a.token)
    asyncio.run(run())
