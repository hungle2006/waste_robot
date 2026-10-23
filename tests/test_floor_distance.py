import asyncio
import copy
import json
import math
import re
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from floor_distance import DEFAULTS, distance_config, fit_calibration, measure_floor
from auto_motion import MOTION_DEFAULTS, VISIBILITY_DEFAULTS, checked_settings, visible_step
from auto_controller import load_auto_config, AutoFault
from test_control import robot, wait_for
from test_v39 import setup_follow
from test_auto import finish


def points():
    # Independent synthetic test geometry: X=100*(x-.5), Y=100*(1-y).
    xy = [(.1,.2),(.9,.2),(.9,.8),(.1,.8),(.5,.4),(.5,.6)]
    return [{'pixel':[x*640,y*480],'floor_cm':[(x-.5)*100,(1-y)*100],
             'kind':'fit' if i<4 else 'check'} for i,(x,y) in enumerate(xy)]


def model(b, stop=30):
    search={j:b.config['poses']['SEARCH'][j] for j in ('p1','p10','p11','d2')}
    cal=fit_calibration(points(),[640,480],search,0,2)
    return distance_config(dict(DEFAULTS,enabled=True,stop_forward_cm=stop,calibration=cal),b.config,0)


def object_at(bottom): return {'bbox_norm':[.47,bottom-.08,.53,bottom]}


def test_calibration_independent_checks_and_distance(robot):
    b,_=robot;c=model(b);pose=c['calibration']['search_pose']
    far=measure_floor(object_at(.594),c,pose,[640,480])
    near=measure_floor(object_at(.70),c,pose,[640,480])
    assert far['forward_cm']==pytest.approx(40.6,abs=.01) and not far['close']
    assert near['forward_cm']==pytest.approx(30,abs=.01) and near['close'] and not near['too_close']
    assert near['uncertainty_cm']>2
    assert measure_floor(object_at(.70),c,pose,[1280,960])['forward_cm']==pytest.approx(30,abs=.01)


def test_distance_refuses_bad_geometry_and_unobserved_region(robot):
    b,_=robot;c=model(b);pose=c['calibration']['search_pose']
    with pytest.raises(ValueError,match='Tỷ lệ'):measure_floor(object_at(.7),c,pose,[640,360])
    with pytest.raises(ValueError,match='SEARCH'):measure_floor(object_at(.7),c,dict(pose,p1=1350),[640,480])
    with pytest.raises(ValueError,match='ngoài vùng FIT'):measure_floor(object_at(.9),c,pose,[640,480])
    bad=points();bad[-1]['floor_cm'][1]+=10
    with pytest.raises(ValueError,match='Sai số'):fit_calibration(bad,[640,480],pose,0,2)
    bad=points();bad[-1]=dict(bad[0],kind='check')
    with pytest.raises(ValueError,match='khác FIT'):fit_calibration(bad,[640,480],pose,0,2)
    with pytest.raises(ValueError,match='4 điểm FIT'):fit_calibration(points()[:4],[640,480],pose,0,2)


def test_no_fabricated_distance_or_old_paper_fallback(robot):
    b,_=robot
    with pytest.raises(ValueError,match='stop_forward_cm'):distance_config(dict(DEFAULTS,enabled=True),b.config,0)
    with pytest.raises(ValueError,match='Chưa hiệu chuẩn'):distance_config(dict(DEFAULTS,enabled=True,stop_forward_cm=30),b.config,0)
    c=model(b)
    with pytest.raises(ValueError,match='Camera index'):distance_config(c,b.config,1)
    c['calibration']['matrix_norm_to_cm'][0][2]+=999
    assert distance_config(c,b.config,0)['calibration']['matrix_norm_to_cm'][0][2]==pytest.approx(-50,abs=.01)


def test_fifty_us_is_a_real_observation_step_even_near_blur():
    motion=checked_settings(dict(MOTION_DEFAULTS,arm_step_us=50),MOTION_DEFAULTS)
    vis=checked_settings(dict(VISIBILITY_DEFAULTS,min_step_us=50),VISIBILITY_DEFAULTS)
    assert visible_step(object_at(.594),1300,2900,None,motion,vis)==50
    assert visible_step(object_at(.54),2880,2900,None,motion,vis)==20  # Last step stops exactly at the cap.


def test_migration_preserves_custom_settings_and_backup(tmp_path):
    from upgrade_v310 import upgrade
    original={'control':{'speed':35,'s2_step_us':20},'motion':{'arm_step_us':8},
              'visibility':{'blur_bottom_norm':.65,'min_step_us':5},
              'distance':{'enabled':False,'stop_forward_cm':30}}
    path=tmp_path/'auto.json';path.write_text(json.dumps(original))
    backup=upgrade(path);c=json.loads(path.read_text())
    assert c['control']['speed']==35 and c['visibility']['blur_bottom_norm']==.65
    assert c['motion']['arm_step_us']==c['visibility']['min_step_us']==50
    assert c['distance']['enabled'] and c['distance']['stop_forward_cm']==30
    assert json.loads(backup.read_text())==original
    upgrade(path);assert json.loads(backup.read_text())==original


def test_metric_overlay_hides_legacy_54_percent_line(robot,tmp_path):
    from camera_guides import CameraGuides,guide_layout
    from pathlib import Path
    b,_=robot;c=json.loads((Path(__file__).resolve().parents[1]/'python/auto_config.json').read_text())
    c['distance']=model(b)
    assert guide_layout(c)['handoff'] is None
    path=tmp_path/'auto.json';path.write_text(json.dumps(c))
    state={'auto_running':True,'state':'auto_base_observe','phase':'base_approach','distance_cm':40.6,'distance_uncertainty_cm':2.4}
    controller=SimpleNamespace(running=True,cfg=c,state=state)
    raw=np.zeros((480,640,3),np.uint8)
    shown=CameraGuides(path,controller).render(raw)
    assert shown.shape==raw.shape and not raw.any() and shown.any()


class MetricVision:
    prefer_local=True
    def __init__(self,b,hw):self.b,self.hw=b,hw;self.jpeg=b'image';self.frame_at=time.monotonic()
    async def detect(self,fresh_after=None):
        self.frame_at=time.monotonic();p=self.hw.state['joints']['p1']
        forward=sum(cmd.startswith('G 100 0 0 ') for cmd in self.hw.commands)
        bottom=[.594,.65,.70][min(forward,2)] if p==1300 else .54
        return {'found':True,'object':{'label':'bottle','bin':1,'bbox':[.47*640,(bottom-.08)*480,.53*640,bottom*480]},
                '_frame_at':self.frame_at,'_frame_size':[640,480]}


def test_distance_not_y2_controls_handoff_and_arm_moves_50(robot,tmp_path):
    async def run():
        b,hw,a,_,world,c=setup_follow(robot,tmp_path,contact=1350)
        c['distance']=model(b);c['visibility'].update(blur_bottom_norm=.9,min_step_us=50)
        c['motion']['arm_step_us']=50;c['calibration']['grasp_final_s2_us']=1400
        c['target_lock']['enabled']=False
        a.path.write_text(json.dumps(c));a.vision=MetricVision(b,hw);n=len(hw.commands)
        await a.start();await finish(a,timeout=20)
        assert a.state['state']=='auto_done',a.state
        sent=hw.commands[n:]
        assert sum(x.startswith('G 100 0 0 ') for x in sent)==2
        first=sent.index('P 1 1350')
        assert not any(x.startswith('G ') for x in sent[first:])
        observed=[call.args[0] for call in a.report.call_args_list if call.args[0]['state']=='auto_base_observe']
        assert observed[0]['bottom_norm']>.54 and not observed[0]['approach_close']
        assert observed[0]['distance_cm']==pytest.approx(40.6,abs=.01)
        assert observed[-1]['approach_close']
        assert a.state['distance_cm'] is None
    asyncio.run(run())


def test_distance_ui_checks_before_writing_and_never_moves_robot(robot,tmp_path,monkeypatch):
    from calibration_ui import calibration_app
    from fastapi.testclient import TestClient
    from livekit_modal import LiveKitModal
    from auto_controller import AutoController
    from unittest.mock import AsyncMock
    b,hw=robot;wait_for(lambda:not b.busy)
    path=tmp_path/'auto.json'
    from pathlib import Path
    path.write_text((Path(__file__).resolve().parents[1]/'python/auto_config.json').read_text())
    v=LiveKitModal('https://app','https://modal');v.prefer_local=True
    v.accept_local_frame(np.zeros((480,640,3),np.uint8));b.motion_finished_at=time.monotonic()-1
    original_snapshot=b.snapshot
    def fresh_simulated_snapshot():
        b.updated=time.monotonic();v.frame_at=time.monotonic()
        return original_snapshot()
    monkeypatch.setattr(b,'snapshot',fresh_simulated_snapshot)
    a=AutoController(b,v,path,AsyncMock());client=TestClient(calibration_app(b,v,a))
    page=client.get('/').text;token=re.search("const token='([^']+)'",page).group(1)
    shot=client.get('/api/capture');assert shot.status_code==200
    payload={'capture_id':shot.json()['id'],'points':points(),'stop_forward_cm':30,
             'tolerance_cm':3,'max_error_cm':2,'confirm_floor':True}
    n=len(hw.commands)
    assert client.post('/api/distance',json=payload).status_code==403
    result=client.post('/api/distance',headers={'x-cal-token':token},json=payload)
    assert result.status_code==200,result.text
    assert len(hw.commands)==n
    assert load_auto_config(path,b.config)['distance']['enabled']
    b.claim_auto('test')
    assert client.post('/api/distance',headers={'x-cal-token':token},json=payload).status_code==409
    b.release_auto('test')


def test_rear_only_does_not_require_front_distance_calibration(robot,tmp_path):
    from test_rear import rear_settings,RearVision,queue_rear,clear_after_open
    from auto_controller import AutoController
    from unittest.mock import AsyncMock
    async def run():
        b,hw=robot;b.start_monitor();path=tmp_path/'auto.json';c=rear_settings(b,path)
        c['distance']=dict(DEFAULTS,enabled=True)  # Intentionally no front metric calibration.
        path.write_text(json.dumps(c));a=AutoController(b,RearVision(b,hw),path,AsyncMock())
        queue_rear(b,hw);clear=asyncio.create_task(clear_after_open(b,hw))
        await a.service_rear();await finish(a);await clear
        assert a.state['state']=='rear_idle' and a.state['rear_cycles']==1,a.state
    asyncio.run(run())
