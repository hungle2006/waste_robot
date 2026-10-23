"""V3.12: measured lens references, piecewise PWM and observed wheel corrections."""
import asyncio
import copy
import json
import time
from pathlib import Path
import pytest
from test_control import robot
from test_v39 import setup_follow
from test_auto import finish
from test_confirmed_grasp import reference
from config_io import read_config
from auto_controller import load_auto_config,AutoFault
from arm_path import targets_at,validate_path
from pose_profile import reference_at,view_problem,search_lens_estimate

PARAMS=Path(__file__).resolve().parents[1]/'python/robot_params.py'


def test_five_measured_pairs_are_exact_and_lens_is_separate_from_s2():
    cfg=load_auto_config(PARAMS,read_config(PARAMS,'robot'));path=cfg['arm_path']
    expected=[(1300,950,75),(1550,876,45),(1800,801,24),(2050,727,12),(2310,650,5)]
    for p1,p10,lens in expected:
        assert targets_at(path,p1)['p10']==p10
        assert reference_at(cfg['pose_profile'],p1)['lens_cm']==lens
    assert targets_at(path,1500)['p10']==891
    assert targets_at(path,1800)['p10']==801
    assert targets_at(path,1900)['p10']==771
    assert cfg['distance']['enabled'] is False
    assert cfg['confirmed_grasp']['measured_forward_cm']==27
    assert cfg['confirmed_grasp']['measured_lens_cm']==5
    stages=cfg['pose_profile']['stages']
    assert [s['image_no'] for s in stages]==[2,4,7,5,6]
    assert len(cfg['pose_profile']['search_samples'])==3
    assert stages[-1]['wheels_enabled'] is False


def test_search_lookup_is_bounded_empirical_and_only_at_search():
    cfg=read_config(PARAMS);samples=cfg['pose_profile']['search_samples']
    for sample in samples:
        box=[x/s for x,s in zip(sample['bbox_px'],[1920,1080,1920,1080])]
        assert search_lens_estimate({'bbox_norm':box},samples,1300)==pytest.approx(sample['lens_cm'])
        assert search_lens_estimate({'bbox_norm':box},samples,1550) is None
    assert search_lens_estimate({'bbox_norm':[.49,.05,.54,.15]},samples,1300) is None


@pytest.mark.parametrize('field,value',[
    ('max_wheel_p1_us',2310),('top_clip_from_p1',1299),('stable_frames',1),
    ('min_progress_norm',float('nan')),
])
def test_bad_profile_fails_before_hardware(field,value,tmp_path):
    c=read_config(PARAMS);c['pose_profile'][field]=value
    p=tmp_path/'bad.json';p.write_text(json.dumps(c))
    with pytest.raises(AutoFault):load_auto_config(p,read_config(PARAMS,'robot'))


def test_invalid_knots_and_unmatched_camera_references_are_rejected(tmp_path):
    c=read_config(PARAMS);r=read_config(PARAMS,'robot')
    path=copy.deepcopy(c['arm_path']);path['knots'][2]['p1']=1550
    with pytest.raises(ValueError):validate_path(path,r,2310)
    c['pose_profile']['stages'][2]['p10']=802
    p=tmp_path/'bad.json';p.write_text(json.dumps(c))
    with pytest.raises(AutoFault,match='P10'):load_auto_config(p,r)


def test_top_clipping_is_pose_specific_and_never_allows_side_or_bottom_loss():
    p=read_config(PARAMS)['pose_profile'];obj={'bbox_norm':[.25,0,.75,.82]}
    assert view_problem(obj,p,1550)
    assert view_problem(obj,p,2050) is None
    assert view_problem({'bbox_norm':[0,0,.75,.82]},p,2310)
    assert view_problem({'bbox_norm':[.25,0,.75,.98]},p,2310)


def setup_measured(robot,tmp_path,mode):
    b,hw,a,_,world,c=setup_follow(robot,tmp_path,contact=1350 if mode=='early_ir' else None if mode=='no_ir' else 1400)
    c['target_lock']['enabled']=False
    c['control']['s2_step_us']=c['motion']['arm_step_us']=c['visibility']['min_step_us']=50
    c['calibration']['grasp_final_s2_us']=1400
    start={j:b.config['poses']['SEARCH'][j] for j in ('p1','p10','p11','d2')}
    middle=dict(start,p1=1350,p10=936);end=dict(start,p1=1400,p10=920)
    c['arm_path']={'start':start,'end':end,'knots':[start,middle,end]}
    c['confirmed_grasp']=reference(c['arm_path'])
    c['confirmed_grasp']['measured_lens_cm']=5
    p=copy.deepcopy(read_config(PARAMS)['pose_profile']);p.update(
        top_clip_from_p1=1350,max_wheel_p1_us=1375,search_samples=[],max_arm_feature_jump_norm=.4)
    boxes=[[864,360,1056,594],[816,324,1104,648],[480,0,1440,864]]
    stages=[]
    for i,(pose,box) in enumerate(zip([start,middle,end],boxes)):
        stage=copy.deepcopy(p['stages'][i]);stage.update(p1=pose['p1'],p10=pose['p10'],bbox_px=box,
            forward_ms=60,back_ms=60,lateral_ms=60,settle_ms=100,arm_step_ms=20,
            max_forward_steps=6,max_back_steps=3,max_lateral_steps=3,wheels_enabled=i<2)
        stages.append(stage)
    p['stages']=stages;c['pose_profile']=p;a.path.write_text(json.dumps(c))
    class Vision:
        jpeg=b'image';frame_at=0
        async def detect(self,fresh_after=None):
            self.frame_at=time.monotonic();pulse=hw.state['joints']['p1']
            box=list(reference_at(p,pulse)['bbox_norm'])
            fw=sum(x.startswith('G 100 0 0 ') for x in hw.commands)
            back=sum(x.startswith('G -100 0 0 ') for x in hw.commands)
            if pulse==1300:
                if mode in ('forward','stop','stuck','wrong'):
                    bottom=.42+(0 if mode=='stuck' else -.04*fw if mode=='wrong' else .08*fw)
                    box[1]-=box[3]-bottom;box[3]=bottom
                if mode=='back':
                    bottom=.69-.08*back;box[1]+=bottom-box[3];box[3]=bottom
            if mode=='lost' and pulse>=1350:
                return {'found':False,'object':None,'_frame_at':self.frame_at,'_frame_size':[640,360]}
            if mode=='side_clip' and pulse>=1350:box[0]=0
            if mode=='final_mismatch' and pulse==1400:box[3]=.93
            return {'found':True,'object':{'label':'bottle','bin':1,
                'bbox':[x*s for x,s in zip(box,[640,360,640,360])]},
                '_frame_at':self.frame_at,'_frame_size':[640,360]}
    a.vision=Vision()
    return b,hw,a,world,c


@pytest.mark.parametrize('mode,expected',[
    ('aligned','auto_done'),('forward','auto_done'),('back','auto_done'),
    ('stuck','auto_near_hold'),('wrong','auto_near_hold'),('lost','auto_near_hold'),
    ('early_ir','auto_near_hold'),('no_ir','auto_near_hold'),
    ('side_clip','auto_near_hold'),('final_mismatch','auto_near_hold'),
])
def test_measured_profile_feedback_and_ir_gate(robot,tmp_path,mode,expected):
    async def run():
        b,hw,a,world,c=setup_measured(robot,tmp_path,mode);n=len(hw.commands)
        await a.start();await finish(a,timeout=25)
        assert a.state['state']==expected,a.state
        sent=hw.commands[n:];drives=[x for x in sent if x.startswith('G ')]
        if mode=='forward':assert len(drives)==1 and drives[0].startswith('G 100 0 0 ')
        if mode=='back':assert len(drives)==1 and drives[0].startswith('G -100 0 0 ')
        if mode=='stuck':assert len(drives)==4 and 'P 1 1350' not in sent
        if mode=='wrong':assert len(drives)==1 and 'P 1 1350' not in sent
        if expected=='auto_done':
            assert world.released and 'P 10 936' in sent
            grip=sent.index('S 3 2300');assert sent.index('P 1 1400')<grip
            assert 'P 10 936' in sent[grip:] and 'P 10 950' in sent[grip:]
            assert not any(x.startswith('G ') for x in sent[sent.index('P 1 1400'):])
        else:assert 'S 3 2300' not in sent
    asyncio.run(run())


def test_stop_during_new_profile_wheel_pulse_never_restarts(robot,tmp_path):
    from test_target_lock import until
    async def run():
        b,hw,a,_,_=setup_measured(robot,tmp_path,'stop')
        await a.start();await until(lambda:any(x.startswith('G ') for x in hw.commands))
        await a.stop(emergency=True);n=len(hw.commands);await asyncio.sleep(.2)
        assert b.stopped and hw.state['motors']==[0,0,0,0]
        assert not any(x.startswith(('G ','C ','P ','S ')) for x in hw.commands[n:])
    asyncio.run(run())


def test_d18_preempts_new_profile_and_retracts_both_joints(robot,tmp_path):
    from test_target_lock import until
    async def run():
        b,hw,a,_,_=setup_measured(robot,tmp_path,'aligned')
        original=a.vision.detect;front_wait=asyncio.Event();rear_seen=asyncio.Event();blocked=asyncio.Event()
        async def detect(fresh_after=None):
            if a.flow=='rear':
                rear_seen.set();await blocked.wait()
            if hw.state['joints']['p1']==1350:
                front_wait.set();await blocked.wait()
            return await original(fresh_after)
        a.vision.detect=detect;n=len(hw.commands)
        await a.start();await asyncio.wait_for(front_wait.wait(),10)
        hw.state['sensors']['D18']=0
        await asyncio.wait_for(rear_seen.wait(),12)
        sent=hw.commands[n:]
        lowered=sent.index('P 10 936');retracted=sent.index('P 10 950',lowered+1)
        assert lowered<retracted<sent.index('P 10 1630',retracted+1)
        assert 'P 1 1400' not in sent and 'S 3 2300' not in sent
        assert a.flow=='rear' and not any(hw.state['motors'])
        await a.stop(emergency=True)
    asyncio.run(run())


def test_new_guides_are_pose_specific_and_leave_vision_raw(monkeypatch):
    import cv2
    import numpy as np
    from types import SimpleNamespace
    from camera_guides import CameraGuides
    cfg=read_config(PARAMS)
    cfg['box_tof_flow']['enabled']=False  # Explicitly test the legacy V3.12 overlay.
    labels=[];original=cv2.putText
    def put(img,text,*args,**kwargs):
        labels.append(text);return original(img,text,*args,**kwargs)
    monkeypatch.setattr(cv2,'putText',put)
    controller=SimpleNamespace(running=True,cfg=cfg,state={'auto_running':True,'s2_us':2050})
    raw=np.zeros((360,640,3),np.uint8);shown=CameraGuides(PARAMS,controller).render(raw)
    assert shown.any() and not raw.any()
    assert any('POSE 4' in t for t in labels)
    assert any('TOP CLIP ALLOWED' in t for t in labels)
    assert not any('BLUR EDGE' in t for t in labels)
