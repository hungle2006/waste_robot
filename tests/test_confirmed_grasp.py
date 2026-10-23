import asyncio
import copy
import json
from pathlib import Path
import pytest
from test_control import robot
from test_v39 import setup_follow
from test_auto import finish
from confirmed_grasp import checked_reference,matches_close_view
from config_io import read_config


def reference(path):
    return dict(enabled=True,pose_us=path['end'],measured_forward_cm=27.,
                target_x_norm=.5,x_tolerance_norm=.06,min_rw=.2,min_rh=.25,
                max_rw=.8,side_margin_norm=.02,stable_frames=2)


def obj(box):return {'bbox_norm':box}


def test_partial_top_requires_exact_pose_and_horizontal_visibility():
    path={'start':dict(p1=1300,p10=950,p11=1250,d2=2100),
          'end':dict(p1=2310,p10=650,p11=1250,d2=2100)}
    c=checked_reference(reference(path),path);j=path['end']
    assert matches_close_view(obj([.25,0,.75,.82]),j,c)
    assert not matches_close_view(obj([.25,0,.75,.82]),dict(j,p1=2300),c)
    assert not matches_close_view(obj([0,0,.75,.82]),j,c)
    assert not matches_close_view(obj([.25,0,.75,1.]),j,c)
    assert not matches_close_view(obj([.47,.4,.53,.5]),j,c)
    bad=copy.deepcopy(c);bad['pose_us']['p10']=700
    with pytest.raises(ValueError):checked_reference(bad,path)


@pytest.mark.parametrize('contact,early_clip,unstable,expected',[
    (1400,False,False,'auto_done'),
    (1350,False,False,'auto_near_hold'),
    (None,False,False,'auto_near_hold'),
    (1400,True,False,'auto_near_hold'),
    (1400,False,True,'auto_near_hold'),
])
def test_fresh_close_view_and_ir_are_both_required(robot,tmp_path,contact,early_clip,unstable,expected):
    async def run():
        b,hw,a,_,world,c=setup_follow(robot,tmp_path,contact=contact)
        c['target_lock']['enabled']=False
        c['control']['s2_step_us']=c['motion']['arm_step_us']=c['visibility']['min_step_us']=50
        c['calibration']['grasp_final_s2_us']=1400
        start={j:b.config['poses']['SEARCH'][j] for j in ('p1','p10','p11','d2')}
        c['arm_path']={'start':start,'end':dict(start,p1=1400,p10=920)}
        c['confirmed_grasp']=reference(c['arm_path'])
        c['approach'].update(handoff_min_rw=.08,handoff_min_rh=.12)
        a.path.write_text(json.dumps(c))
        class Vision:
            jpeg=b'image';frame_at=0;final_calls=0
            async def detect(self,fresh_after=None):
                import time
                self.frame_at=time.monotonic();p=hw.state['joints']['p1']
                if p==1300:box=[.48,.514,.52,.594]  # Far/small; handoff is for visibility only.
                elif p==1400 or early_clip:
                    self.final_calls+=1
                    box=[.25,0,.75,.82]
                    if unstable and self.final_calls>1:box=[.40,0,.90,.82]
                else:box=[.44,.42,.56,.54]
                return {'found':True,'object':{'label':'bottle','bin':1,'bbox':[
                    value*size for value,size in zip(box,[640,360,640,360])]},
                    '_frame_at':self.frame_at,'_frame_size':[640,360]}
        a.vision=Vision();n=len(hw.commands)
        await a.start();await finish(a,timeout=20)
        assert a.state['state']==expected,a.state
        sent=hw.commands[n:]
        assert 'P 1 1350' in sent
        assert not any(x.startswith(('G ','C ')) for x in sent[sent.index('P 1 1350'):])
        if expected=='auto_done':
            assert 'S 3 2300' in sent and world.released
            events=[x.args[0] for x in a.report.call_args_list]
            confirmed=next(x for x in events if x['state']=='auto_grasp_pose_confirmed')
            assert confirmed['distance_cm'] is None and confirmed['reference_forward_cm']==27
        else:assert 'S 3 2300' not in sent
    asyncio.run(run())


def test_confirmed_image_does_not_enable_search_distance():
    p=Path(__file__).resolve().parents[1]/'python/robot_params.py'
    c=read_config(p)
    assert c['confirmed_grasp']['pose_us']['p1']==2310
    assert c['confirmed_grasp']['pose_us']['p10']==650
    assert c['distance']['enabled'] is False


def test_terminal_guides_match_partial_view_gate_and_never_annotate_raw(tmp_path,monkeypatch):
    import cv2
    import numpy as np
    from types import SimpleNamespace
    from camera_guides import CameraGuides
    p=Path(__file__).resolve().parents[1]/'python/robot_params.py'
    cfg=read_config(p)
    cfg['box_tof_flow']['enabled']=False
    labels=[];original=cv2.putText
    def put(image,text,*args,**kwargs):
        labels.append(text);return original(image,text,*args,**kwargs)
    monkeypatch.setattr(cv2,'putText',put)
    state=dict(auto_running=True,state='auto_near_wait',phase='grasp_contact')
    controller=SimpleNamespace(running=True,cfg=cfg,state=state)
    raw=np.zeros((360,640,3),np.uint8)
    shown=CameraGuides(p,controller).render(raw)
    assert shown.any() and not raw.any()
    assert any('TOP CLIP ALLOWED' in t for t in labels)
    assert not any('BLUR EDGE' in t or 'ARM TARGET' in t for t in labels)


@pytest.mark.parametrize('stuck,limit,expect',[(False,1350,'auto_done'),(True,1350,'auto_near_hold'),(False,1300,'auto_done')])
def test_alternating_base_and_arm_requires_fresh_progress_and_stops_before_final(robot,tmp_path,stuck,limit,expect):
    async def run():
        b,hw,a,_,world,c=setup_follow(robot,tmp_path,contact=1400)
        c['target_lock']['enabled']=False
        c['control']['s2_step_us']=c['motion']['arm_step_us']=c['visibility']['min_step_us']=50
        c['calibration']['grasp_final_s2_us']=1400
        start={j:b.config['poses']['SEARCH'][j] for j in ('p1','p10','p11','d2')}
        c['arm_path']={'start':start,'end':dict(start,p1=1400,p10=920)}
        c['confirmed_grasp']=reference(c['arm_path'])
        c['confirmed_grasp']['base_follow']=dict(enabled=True,trigger_bottom_norm=.48,
                pulse_ms=60,max_p1_us=limit,max_steps=6,max_no_progress=4)
        a.path.write_text(json.dumps(c))
        class Vision:
            jpeg=b'image';frame_at=0
            async def detect(self,fresh_after=None):
                import time
                self.frame_at=time.monotonic();p=hw.state['joints']['p1']
                steps=sum(x.startswith('G 100 0 0 ') for x in hw.commands)
                if p==1300:box=[.48,.514,.52,.594]
                elif p==1350:
                    bottom=.44 if stuck or not steps else .52
                    box=[.44,bottom-.10,.56,bottom]
                else:box=[.25,0,.75,.82]
                return {'found':True,'object':{'label':'bottle','bin':1,'bbox':[
                    v*s for v,s in zip(box,[640,360,640,360])]},
                    '_frame_at':self.frame_at,'_frame_size':[640,360]}
        a.vision=Vision();n=len(hw.commands)
        await a.start();await finish(a,timeout=20)
        assert a.state['state']==expect,a.state
        sent=hw.commands[n:];first=sent.index('P 1 1350')
        drives=[i for i,x in enumerate(sent) if x.startswith('G 100 0 0 ')]
        if limit==1300:assert not drives
        elif stuck:
            assert len(drives)==4 and 'S 3 2300' not in sent and 'P 1 1400' not in sent
        else:
            assert len(drives)==1 and first<drives[0]<sent.index('P 1 1400')
            assert not any(x.startswith('G ') for x in sent[sent.index('P 1 1400'):])
            assert world.released
    asyncio.run(run())
