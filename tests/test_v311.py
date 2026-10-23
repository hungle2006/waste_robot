import asyncio
import copy
import json
from pathlib import Path
from unittest.mock import AsyncMock

import pytest

from test_control import robot
from arm_path import targets_at, pulse_steps, validate_path
from auto_controller import AutoController, load_auto_config
from config_io import read_config, read_namespace
from floor_distance import distance_config, measure_floor
from paper_distance import create_reference, paper_position
from record_grasp_reference import record, select_reference_point
from test_v39 import setup_follow
from test_auto import finish

PARAMETERS=Path(__file__).resolve().parents[1]/'python/robot_params.py'


def test_python_is_authoritative_and_reloads_without_pyc(tmp_path):
    path=tmp_path/'params.py'
    path.write_text("ROBOT_CONFIG={'x':1}\nAUTO_CONFIG={'x':2}\n")
    assert read_config(path)=={'x':2}
    assert read_config(path,'robot')=={'x':1}
    path.write_text("ROBOT_CONFIG={'x':1}\nAUTO_CONFIG={'x':3}\n")
    assert read_config(path)=={'x':3}


def test_delivered_parameters_start_without_missing_calibration():
    robot=read_config(PARAMETERS,'robot');cfg=load_auto_config(PARAMETERS,robot)
    assert cfg['distance']['stop_forward_cm']==27
    assert cfg['distance']['enabled'] is False  # Never manufacture the missing USB correspondence.
    assert cfg['motion']['arm_step_us']==cfg['visibility']['min_step_us']==50
    path=cfg['arm_path'];steps=pulse_steps(1300,2310,50)
    assert len(steps)==21 and steps[-2:]==[2300,2310]
    assert targets_at(path,2310)=={'p1':2310,'p10':650,'p11':1250,'d2':2100}
    values=[targets_at(path,p) for p in [1300,*steps]]
    assert all(a['p10']>=b['p10'] for a,b in zip(values,values[1:]))
    assert targets_at(path,1300)['p10']==950
    bad=copy.deepcopy(path);bad['end']['p11']=1300
    with pytest.raises(ValueError):validate_path(bad,robot,2310)


def test_recovered_paper_maps_corners_and_not_stretched_frames():
    p=read_namespace(PARAMETERS)
    for xy,expected in zip(p['PAPER_CORNERS_PX'],[(0,0),(17,0),(17,17),(0,17)]):
        result=paper_position([xy[0]/1920,xy[1]/1080],p,[640,360])
        assert result==pytest.approx(expected,abs=1e-5)
    with pytest.raises(ValueError,match='tỷ lệ'):
        paper_position([.5,.5],p,[640,480])


def test_usb_anchor_is_required_then_restores_distance_vs_54_percent():
    p=read_namespace(PARAMETERS);r=p['ROBOT_CONFIG'];anchor=[.5,.70]
    ref=create_reference(p,anchor,[640,360])
    cfg=distance_config(dict(p['AUTO_CONFIG']['distance'],enabled=True,calibration=ref),r,p['CAMERA_INDEX'])
    pose=ref['search_pose']
    at=measure_floor({'bbox_norm':[.47,.62,.53,.70]},cfg,pose,[640,360])
    far=measure_floor({'bbox_norm':[.47,.514,.53,.594]},cfg,pose,[640,360])
    assert at['forward_cm']==pytest.approx(27)
    assert at['lateral_cm']==pytest.approx(0,abs=1e-8)
    assert at['close'] and not at['too_close']
    assert not far['close'] and far['forward_cm']>27
    assert at['quality']=='preliminary' and ref['independent_validation'] is False
    # A hand-edited derived matrix does not overwrite the retained point measurements.
    ref['matrix_norm_to_cm'][0][2]+=999
    rebuilt=distance_config(dict(cfg,calibration=ref),r,p['CAMERA_INDEX'])
    assert measure_floor({'bbox_norm':[.47,.62,.53,.7]},rebuilt,pose,[640,360])['lateral_cm']==pytest.approx(0,abs=1e-8)


def test_reference_capture_requires_three_consistent_unambiguous_raw_frames():
    import numpy as np
    class Camera:
        released=False
        def read(self):return True,np.zeros((360,640,3),np.uint8)
        def release(self):self.released=True
    async def run():
        p=read_namespace(PARAMETERS);cam=Camera()
        async def detector(image):
            assert image.startswith(b'\xff\xd8')
            return {'found':True,'object':{'label':'bottle','bin':1,'bbox':[300,210,340,252]}}
        result=await record(p,lambda *_:cam,detector)
        assert result['capture_count']==3 and cam.released
        assert result['anchor_norm']==pytest.approx([.5,.7])
        with pytest.raises(ValueError,match='đúng một'):
            select_reference_point({'found':False,'object':None},[640,360])
    asyncio.run(run())


def test_two_joint_path_grips_lifts_reverse_then_opens_lid(robot,tmp_path):
    async def run():
        b,hw,a,v,world,c=setup_follow(robot,tmp_path,contact=1400)
        c['target_lock']['enabled']=False
        c['motion']['arm_step_us']=c['visibility']['min_step_us']=c['control']['s2_step_us']=50
        c['calibration']['grasp_final_s2_us']=1400
        c['visibility']['min_bottom_norm']=.05
        start={j:b.config['poses']['SEARCH'][j] for j in ('p1','p10','p11','d2')}
        c['arm_path']={'start':start,'end':dict(start,p1=1400,p10=920)}
        a.path.write_text(json.dumps(c));n=len(hw.commands)
        await a.start();await finish(a,timeout=25)
        assert a.state['state']=='auto_done',a.state
        sent=hw.commands[n:];close=sent.index('S 3 2300')
        assert sent.index('P 1 1350')<sent.index('P 1 1400')<close
        assert 'P 10 935' in sent[:close] and 'P 10 920' in sent[:close]
        assert 'P 10 935' in sent[close:] and 'P 10 950' in sent[close:]
        assert not any(x.startswith(('G ','C ')) for x in sent[sent.index('P 1 1350'):])
        assert close<sent.index('P 4 1600')
        assert world.released
    asyncio.run(run())


def test_py_pose_save_never_overwrites_source(robot,tmp_path):
    b,hw=robot
    from test_control import ready
    ready(b);path=tmp_path/'params.py';path.write_text(PARAMETERS.read_text())
    b.path=path;before=path.read_bytes();n=len(hw.commands)
    with pytest.raises(ValueError,match='robot_params.py'):
        b.save_pose('SEARCH',{'p1':1400},b.epoch)
    assert path.read_bytes()==before and len(hw.commands)==n


def test_log_case_small_box_does_not_trigger_arm_at_54_percent(robot,tmp_path):
    class SmallVision:
        jpeg=b'image'
        frame_at=0
        async def detect(self,fresh_after=None):
            import time
            self.frame_at=time.monotonic()
            return {'found':True,'object':{'label':'bottle','bin':1,
                    'bbox':[.48*640,.514*480,.52*640,.594*480]},
                    '_frame_at':self.frame_at,'_frame_size':[640,480]}
    async def run():
        b,hw,a,_,_,c=setup_follow(robot,tmp_path,contact=1900)
        c['target_lock']['enabled']=False
        c['approach'].update(handoff_min_rw=.08,handoff_min_rh=.12,max_forward_steps=1)
        a.path.write_text(json.dumps(c));a.vision=SmallVision();n=len(hw.commands)
        await a.start();await finish(a,timeout=15)
        assert a.state['state']=='auto_near_hold',a.state
        sent=hw.commands[n:]
        assert any(x.startswith('G 100 0 0 ') for x in sent)
        assert not any(x.startswith('P 1 ') and int(x.split()[2])>1300 for x in sent)
        assert 'S 3 2300' not in sent
    asyncio.run(run())


def test_d18_reverses_both_joints_before_rear_rotation(robot,tmp_path):
    from test_rear import rear_settings,RearVision,queue_rear,clear_after_open
    from test_control import ready,wait_for
    async def run():
        b,hw=robot;ready(b);b.start_monitor();path=tmp_path/'rear_path.json'
        c=rear_settings(b,path)
        start={j:b.config['poses']['SEARCH'][j] for j in ('p1','p10','p11','d2')}
        c['arm_path']={'start':start,'end':dict(start,p1=1400,p10=920)}
        c['control']['s2_step_us']=50;c['motion']['arm_step_us']=50
        path.write_text(json.dumps(c))
        # Simulate D18 arriving at the end of an observed front arm step.
        b.command(None,b.epoch,'p1',1400);wait_for(lambda:not b.busy)
        b.command(None,b.epoch,'p10',920);wait_for(lambda:not b.busy)
        wait_for(lambda:b.last_hw['joints']['p10']==920)
        a=AutoController(b,RearVision(b,hw),path,AsyncMock());queue_rear(b,hw)
        n=len(hw.commands);clear=asyncio.create_task(clear_after_open(b,hw))
        await a.service_rear();await finish(a);await clear
        assert a.state['state']=='rear_idle',a.state
        sent=hw.commands[n:]
        assert sent.index('P 10 935')<sent.index('P 10 950')<sent.index('P 10 1630')
        assert sent.index('P 1 1300')<sent.index('P 10 1630')
        assert 'S 3 2300' not in sent
    asyncio.run(run())
