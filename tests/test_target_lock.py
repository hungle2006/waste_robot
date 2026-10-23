"""V3.8 association, uncertain observations, lifecycle and raw-video separation."""
import asyncio, copy, json, time
from pathlib import Path
from unittest.mock import AsyncMock
import pytest
from test_control import robot
from test_approach import configured, ApproachVision
from test_auto import finish
from target_lock import DEFAULTS, associate, lock_config, attach_appearance
from auto_controller import load_auto_config
from livekit_modal import parse_detection, LiveKitModal


def obj(box, label='bottle', bin=1, appearance=None):
    x1,y1,x2,y2=box
    d={'bbox_norm':box,'center_norm':[(x1+x2)/2,(y1+y2)/2], 'label':label,'bin':bin}
    if appearance is not None:d['_appearance']=appearance
    return d


def test_unordered_two_boxes_keep_same_geometry_despite_label_and_bin():
    old=obj([.4,.4,.6,.6])
    current=[obj([.7,.4,.8,.6]),obj([.41,.4,.61,.6],'battery',3)]
    assert associate(old,current,DEFAULTS)[0]==1
    assert associate(old,current[::-1],DEFAULTS)[0]==0


def test_two_similar_boxes_are_ambiguous():
    old=obj([.4,.4,.6,.6])
    assert associate(old,[obj([.39,.4,.59,.6]),obj([.41,.4,.61,.6])],DEFAULTS)[0] is None


def test_disjoint_box_cannot_replace_target_without_appearance():
    old=obj([.4,.4,.45,.5])
    assert associate(old,[obj([.5,.4,.55,.5])],DEFAULTS)[0] is None


def test_appearance_supports_camera_shift_but_still_bounds_it():
    old=obj([.4,.4,.45,.5],appearance=[1.,0.])
    assert associate(old,[obj([.5,.4,.55,.5],appearance=[1.,0.])],DEFAULTS)[0]==0
    assert associate(old,[obj([.75,.4,.8,.5],appearance=[1.,0.])],DEFAULTS)[0] is None
    assert associate(old,[obj([.4,.4,.45,.5],appearance=[0.,1.])],DEFAULTS)[0] is None


@pytest.mark.parametrize('field,value',[('confirm_frames',1),('retry_ms',0),('reference_max_age_s',float('nan')),('continuous_auto',1)])
def test_bad_lock_config_is_rejected(field,value):
    with pytest.raises(ValueError):lock_config({field:value})


def test_parser_supports_list_and_legacy():
    a={'label':'bottle','bin':1,'bbox':[10,10,20,20]}
    b={'label':'can','bin':4,'bbox':[30,10,40,20]}
    assert len(parse_detection({'found':True,'objects':[a,b]})['objects'])==2
    assert parse_detection({'found':True,'object':a})['objects']==[a]
    assert parse_detection({'found':False,'objects':[]})['object'] is None
    with pytest.raises(ValueError):parse_detection({'found':True,'objects':[]})


class LockVision(ApproachVision):
    def __init__(self,b,hw):
        super().__init__(b,hw)
        self.vanish=0;self.recoveries=0;self.two=True;self.ambiguous=False
        self.overlay=None
    def clear_overlay(self):self.overlay=None
    def set_overlay(self,objects,selected,captured,frame_size,state,target_id,locked_bin,max_age):
        self.overlay={'objects':copy.deepcopy(objects),'selected':selected,'state':state}
    async def detect(self,fresh_after=None):
        r=await super().detect(fresh_after)
        current=r['object'];current['_appearance']=[1.,0.]
        forward=self.calls[-1][1]
        if forward:
            current['label']='different name';current['bin']=3
        if forward==1 and self.vanish:
            self.vanish-=1;self.recoveries+=1
            r.update(found=False,object=None,objects=[]);return r
        objects=[current]
        if self.two:
            objects.insert(0,{'label':'other','bin':2,'bbox':[10,80,40,130],'_appearance':[0.,1.]})
        if forward and self.ambiguous:objects=[current,copy.deepcopy(current)]
        r['objects']=objects
        return r


def lock_setup(robot,tmp_path,continuous=False):
    b,hw=robot;p=tmp_path/'lock.json'
    a,_,world,c=configured(b,hw,p)
    c['target_lock']=dict(DEFAULTS,enabled=True,continuous_auto=continuous,retry_ms=100)
    p.write_text(json.dumps(c));v=LockVision(b,hw);a.vision=v
    return b,hw,a,v,world,c


async def until(predicate,timeout=8):
    end=time.monotonic()+timeout
    while not predicate() and time.monotonic()<end:await asyncio.sleep(.02)
    assert predicate(),'wait timed out'


def test_two_boxes_labels_and_bin_changes_do_not_abort_or_change_selected_bin(robot,tmp_path):
    async def run():
        b,hw,a,v,world,c=lock_setup(robot,tmp_path)
        await a.start();await finish(a)
        assert a.state['state']=='auto_done',a.state
        assert world.released and a.state['cycles']==1
        observations=[x.args[0] for x in a.report.call_args_list if x.args[0]['state']=='auto_target_observe']
        assert any(x['observed_bin']==3 and x['bin']==1 for x in observations)
        assert any(len(x['detections'])==2 and x['detections'][1]['selected'] for x in observations)
        assert any(x['detections'][1]['bin']==3 and x['detections'][1]['label']=='different name' for x in observations)
        assert 'P 9 1450' not in hw.commands
    asyncio.run(run())


def test_temporary_loss_holds_without_start_then_automatically_resumes(robot,tmp_path):
    async def run():
        b,hw,a,v,world,c=lock_setup(robot,tmp_path);v.vanish=3
        await a.start();await until(lambda:v.recoveries>=1)
        assert a.running and a.state['auto_running'] and not b.stopped and b.auto_owner==a.token
        n=len(hw.commands);await asyncio.sleep(.15)
        assert not any(x.startswith(('C ','P ','S ')) for x in hw.commands[n:])
        await finish(a)
        assert a.state['state']=='auto_done' and world.released
    asyncio.run(run())


def test_permanent_loss_preserves_old_target_and_stop_cancels_pending_request(robot,tmp_path):
    async def run():
        b,hw,a,v,_,_=lock_setup(robot,tmp_path);v.vanish=1000
        await a.start();await until(lambda:v.recoveries>=2)
        target=copy.deepcopy(a.target);n=len(hw.commands)
        await asyncio.sleep(.25)
        assert a.running and a.state['auto_running'] and not b.stopped
        assert a.target==target and not any(x.startswith(('C ','P ','S ')) for x in hw.commands[n:])
        v.base_delay=asyncio.Event()
        await asyncio.sleep(.15);await a.stop();n=len(hw.commands)
        v.base_delay.set();await asyncio.sleep(.1)
        assert not b.auto_owner and not a.running
        assert not any(x.startswith(('C ','P ','S ')) for x in hw.commands[n:])
    asyncio.run(run())


def test_ambiguous_two_boxes_wait_without_gripping(robot,tmp_path):
    async def run():
        b,hw,a,v,_,_=lock_setup(robot,tmp_path);v.ambiguous=True
        await a.start();await until(lambda:a.state['state']=='auto_reacquire' and 'Hai box' in a.state['message'])
        assert a.running and b.auto_owner==a.token and not b.stopped
        n=len(hw.commands);await asyncio.sleep(.2)
        assert not any(x.startswith(('C ','P ','S ')) for x in hw.commands[n:])
        assert 'S 3 2300' not in hw.commands
        await a.stop()
    asyncio.run(run())


def test_limit_holds_auto_ownership_without_repeating_motion(robot,tmp_path):
    async def run():
        b,hw,a,v,_,c=lock_setup(robot,tmp_path)
        c['approach']['max_forward_steps']=1;a.path.write_text(json.dumps(c))
        await a.start();await until(lambda:a.state['state']=='auto_hold')
        n=len(hw.commands);await asyncio.sleep(.2)
        assert a.running and a.state['auto_running'] and b.auto_owner==a.token and not b.stopped
        assert not any(x.startswith(('C ','P ','S ')) for x in hw.commands[n:])
        await a.stop()
    asyncio.run(run())


def test_continuous_auto_stays_running_after_success_until_stop(robot,tmp_path):
    async def run():
        b,hw,a,v,_,_=lock_setup(robot,tmp_path,continuous=True)
        await a.start();await until(lambda:a.state['cycles']>=1,timeout=12)
        assert a.running and b.auto_owner==a.token and a.state['auto_running']
        await a.stop()
    asyncio.run(run())


def test_reference_expiry_never_selects_another_object(robot,tmp_path):
    b,hw,a,v,_,_=lock_setup(robot,tmp_path)
    a.cfg=load_auto_config(a.path,b.config);a.target=obj([.4,.4,.6,.6])
    a.lock_frame_size=[640,480];a.lock_last_seen=time.monotonic()-31
    r={'found':True,'object':{'label':'bottle','bin':1,'bbox':[256,192,384,288]},'_frame_size':[640,480]}
    a.frame_after=time.monotonic();a.observed_frame_size=[640,480]
    assert a.select_locked_target(r) is None
    assert 'quá cũ' in a.lock_reason


def test_camera_cache_is_raw_and_no_box_renderer_exists():
    cv2=pytest.importorskip('cv2');import numpy as np
    v=LiveKitModal('https://app','https://modal')
    raw=np.zeros((100,200,3),np.uint8);raw[:]=(20,50,180)
    v.accept_local_frame(raw);jpeg=v.jpeg
    objects=[{'label':'bottle','bin':1,'bbox':[30,40,80,90]}]
    attach_appearance(jpeg,objects,[200,100])
    assert len(objects[0]['_appearance'])==128 and v.jpeg==jpeg
    v.set_overlay(objects,0,time.monotonic(),[200,100],'LOCK',1,1,2)
    assert not hasattr(v,'draw_overlay') and not hasattr(v,'draw_camera')
    assert not hasattr(v,'detection_snapshot')
    v.clear_overlay();assert v.jpeg==jpeg


def test_local_box_preview_and_lock_settings_are_readable_and_edits_are_guarded(robot,tmp_path):
    import re
    from fastapi.testclient import TestClient
    from calibration_ui import calibration_app
    b,hw,a,_,_,c=lock_setup(robot,tmp_path)
    v=LiveKitModal('https://app','https://modal');a.vision=v
    cv2=pytest.importorskip('cv2');import numpy as np
    v.accept_local_frame(np.zeros((480,640,3),np.uint8))
    v.set_overlay([{'label':'bottle','bin':1,'bbox':[100,100,180,250]}],0,
                  time.monotonic(),[640,480],'LOCK',7,1,2)
    client=TestClient(calibration_app(b,v,a));n=len(hw.commands)
    page=client.get('/').text;token=re.search("const token='([^']+)'",page).group(1)
    boxes=client.get('/api/boxes').json()
    assert boxes['target_id']==7 and boxes['objects'][0]['bbox']==[100,100,180,250]
    assert not boxes['stale'] and boxes['selected']==0
    response=client.get('/api/preview')
    assert response.status_code==200 and response.headers['content-type']=='image/jpeg'
    assert client.post('/api/target-lock',json=c['target_lock']).status_code==403
    headers={'X-Cal-Token':token}
    assert client.post('/api/target-lock',json=dict(c['target_lock'],confirm_frames=1),headers=headers).status_code==400
    assert client.post('/api/target-lock',json=c['target_lock'],headers=headers).status_code==200
    assert not any(x.startswith(('C ','P ','S ')) for x in hw.commands[n:])
    b.claim_auto('test-owner')
    assert client.post('/api/target-lock',json=c['target_lock'],headers=headers).status_code==409
    b.release_auto('test-owner')


def test_transient_model_network_failure_waits_in_auto_then_recovers(robot,tmp_path):
    import httpx
    async def run():
        b,hw,a,v,world,c=lock_setup(robot,tmp_path)
        detect=v.detect;failed=False
        async def flaky(fresh_after=None):
            nonlocal failed
            if any(cmd.startswith('C 100 0 0 ') for cmd in hw.commands) and not failed:
                failed=True
                raise httpx.ConnectError('offline')
            return await detect(fresh_after)
        v.detect=flaky
        await a.start();await finish(a)
        assert failed and a.state['state']=='auto_done' and world.released
        statuses=[call.args[0] for call in a.report.call_args_list]
        assert any(s['state']=='auto_reacquire' and 'Modal' in s['message'] and s['auto_running'] for s in statuses)
    asyncio.run(run())
