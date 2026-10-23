"""Camera scale limits must end visual servoing before any further approach."""
import asyncio
import copy
import json
import re
import time
from unittest.mock import AsyncMock

import pytest

from test_control import robot,ready,wait_for
from test_auto import relative_config,calibrated,finish
from auto_controller import AutoController,AutoFault,NearHold,near_config,image_extent,load_auto_config,NEAR_DEFAULTS


class NearVision:
    def __init__(self,b,hw,bbox=None):
        self.b,self.hw=b,hw
        self.jpeg=b'frame';self.frame_at=time.monotonic();self.calls=0
        self.bbox=bbox or (lambda pulse: [128,220,512,260] if pulse>=1320 else [300,220,340,260])
        self.label='plastic bottle';self.bin=1
        self.on_capture=None

    async def detect(self,fresh_after=None):
        self.calls+=1
        pulse=self.hw.state['joints']['p1']
        bbox=self.bbox(pulse)
        if self.on_capture:self.on_capture(self,pulse)
        self.frame_at=time.monotonic()
        return {'found':True,'object':{'label':self.label,'bin':self.bin,'bbox':bbox},
                '_frame_at':self.frame_at,'_frame_size':[640,480]}


class ContactWorld:
    """Update IR on STATUS from simulated servo positions, not inference calls."""
    def __init__(self,b,hw,contact_at=1320):
        self.b,self.hw,self.contact_at=b,hw,contact_at
        self.held=False;self.released=False
        self.force=None
        self.original=hw.request
        hw.request=self.request

    def request(self,command):
        if command=='STATUS':
            joints=self.hw.state['joints']
            if joints.get('d3')==self.b.config['grip']['close']:self.held=True
            if self.held and joints.get('d3')==self.b.config['grip']['open']:self.released=True
            contact=(self.contact_at is not None and joints.get('p1',0)>=self.contact_at)
            active=(self.held or contact) and not self.released
            level=0 if active else 1
            if self.force is not None:level=self.force()
            self.hw.state['sensors']['D19']=level
        return self.original(command)


def setup_near(b,hw,path,bbox=None,contact_at=1320,mode='relative'):
    ready(b)
    cfg=(relative_config if mode=='relative' else calibrated)(b,path,final=1360)
    cfg['control']['settle_ms']=250  # Longer than the real 200 ms sensor polling interval.
    cfg['calibration'].update(contact_sensor='D19',hold_sensor='D19')
    cfg['near'].update(wait_contact_ms=700,contact_confirm_ms=200)
    path.write_text(json.dumps(cfg))
    world=ContactWorld(b,hw,contact_at)
    stamp=time.monotonic();b.start_monitor()
    wait_for(lambda:b.updated>stamp)
    vision=NearVision(b,hw,bbox)
    a=AutoController(b,vision,path,AsyncMock())
    return a,vision,world,cfg


@pytest.mark.parametrize('bbox,reason',[
    ([128,220,512,260],'rw'),
    ([0,220,260,260],'edge'),
    ([380,220,640,260],'edge'),
    ([300,0,340,220],'edge'),
    ([300,260,340,480],'edge'),
])
@pytest.mark.parametrize('mode',['relative','calibrated'])
def test_scale_or_any_edge_switches_to_ir_without_more_lowering(robot,tmp_path,bbox,reason,mode):
    async def run():
        b,hw=robot
        a,v,world,cfg=setup_near(b,hw,tmp_path/'auto.json',
                               bbox=lambda pulse:bbox if pulse>=1320 else [300,220,340,260],mode=mode)
        n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_done',a.state
        assert a.state['cycles']==1 and v.calls==4
        sent=hw.commands[n:]
        assert 'P 1 1320' in sent and 'P 1 1340' not in sent and 'P 1 1360' not in sent
        assert not any(cmd.startswith('C ') for cmd in sent)
        states=[call.args[0] for call in a.report.call_args_list]
        near=next(item for item in states if item['state']=='auto_near_wait')
        assert reason in near['near_reasons'] and near['vision_mode']=='ir_only'
        paused=next(i for i,x in enumerate(states) if x['state']=='auto_near_wait')
        assert not any(x['state']=='auto_align' for x in states[paused+1:])
        assert sent.index('P 9 2350') < sent.index('S 3 2300') < sent.index('P 4 1600')
        close=sent.index('S 3 2300');assert sent.index('S 3 600',close)>sent.index('P 4 1600')
        assert world.released
    asyncio.run(run())


def test_no_ir_holds_servo_pulses_disables_d18_and_never_grips(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=setup_near(b,hw,tmp_path/'auto.json',contact_at=None)
        n=len(hw.commands);await a.start();await finish(a)
        assert a.state['state']=='auto_near_hold' and 'Hết thời gian chờ IR' in a.state['message']
        assert not b.stopped and not b.auto_owner and not b.auto_rear
        assert hw.state['joints']['p1']==1320 and hw.state['motors']==[0,0,0,0]
        sent=hw.commands[n:]
        assert 'S 3 2300' not in sent and 'P 4 1600' not in sent and 'STOP' not in sent
        assert 'P 1 1340' not in sent and v.calls==4
        # A held task cannot be restarted by an incoming D18 event.
        hw.state['sensors']['D18']=0
        await asyncio.sleep(.3);await a.service_rear()
        assert a.state['state']=='auto_near_hold' and not a.running
    asyncio.run(run())


def test_large_object_in_search_without_alignment_never_sorts_or_grips(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=setup_near(b,hw,tmp_path/'auto.json',bbox=lambda pulse:[128,220,512,260])
        n=len(hw.commands);await a.start();await finish(a)
        assert a.state['state']=='auto_near_hold' and 'chưa có lần căn ổn định' in a.state['message']
        sent=hw.commands[n:]
        assert not any(cmd.startswith(('C ','P 9 ')) for cmd in sent)
        assert 'P 1 1320' not in sent and 'S 3 2300' not in sent and v.calls==1
    asyncio.run(run())


def test_recent_unaligned_frame_cannot_use_an_older_alignment(robot,tmp_path):
    async def run():
        b,hw=robot
        def shape(pulse):return [380,220,420,260] if pulse>=1320 else [300,220,340,260]
        a,v,_,_=setup_near(b,hw,tmp_path/'auto.json',bbox=shape,contact_at=None)
        original=v.detect
        async def next_after_drive(fresh_after=None):
            if any(cmd.startswith('C ') for cmd in hw.commands):v.bbox=lambda pulse:[128,220,512,260]
            return await original(fresh_after)
        v.detect=next_after_drive
        n=len(hw.commands);await a.start();await finish(a)
        assert a.state['state']=='auto_near_hold' and 'chưa có lần căn ổn định' in a.state['message']
        assert 'S 3 2300' not in hw.commands[n:]
    asyncio.run(run())


def test_stale_alignment_does_not_allow_grip(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=setup_near(b,hw,tmp_path/'auto.json')
        def stale(vision,pulse):
            if pulse>=1320:a.aligned_anchor['at']=time.monotonic()-31
        v.on_capture=stale
        n=len(hw.commands);await a.start();await finish(a)
        assert a.state['state']=='auto_near_hold' and 'quá cũ' in a.state['message']
        assert 'S 3 2300' not in hw.commands[n:]
    asyncio.run(run())


def test_changed_label_at_cutoff_still_stops_instead_of_using_ir(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=setup_near(b,hw,tmp_path/'auto.json')
        v.on_capture=lambda vision,pulse:setattr(vision,'label','different object') if pulse>=1320 else None
        n=len(hw.commands);await a.start();await finish(a)
        assert a.state['state']=='auto_error' and b.stopped
        assert 'đổi mục tiêu' in a.state['message'] and 'S 3 2300' not in hw.commands[n:]
    asyncio.run(run())


def test_ir_requires_continuous_confirmation_without_more_images(robot,tmp_path):
    async def run():
        b,hw=robot;p=tmp_path/'auto.json';a,v,world,cfg=setup_near(b,hw,p,contact_at=None)
        cfg['near']['contact_confirm_ms']=300;p.write_text(json.dumps(cfg))
        entered=None
        original=a.report
        async def report(state):
            nonlocal entered
            if state['state']=='auto_near_wait':entered=time.monotonic()
            await original(state)
        a.report=report
        samples=0
        def flicker():
            nonlocal samples
            if entered is None:return 1
            samples+=1
            return 0 if samples%2 else 1
        world.force=flicker
        n=len(hw.commands);await a.start();await finish(a)
        assert a.state['state']=='auto_near_hold' and v.calls==4
        assert 'S 3 2300' not in hw.commands[n:]
    asyncio.run(run())


def test_delayed_ir_contact_can_grip_while_no_more_images_are_sent(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,world,_=setup_near(b,hw,tmp_path/'auto.json',contact_at=None)
        entered=None
        async def report(state):
            nonlocal entered
            if state['state']=='auto_near_wait':entered=time.monotonic()
        a.report=report
        def delayed():
            if world.released:return 1
            return 0 if entered is not None and time.monotonic()-entered>=.15 else 1
        world.force=delayed
        await a.start();await finish(a)
        assert a.state['state']=='auto_done' and v.calls==4,a.state
    asyncio.run(run())


def test_stop_during_ir_wait_never_closes_grip_or_resumes_servo(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=setup_near(b,hw,tmp_path/'auto.json',contact_at=None)
        entered=asyncio.Event()
        async def report(state):
            if state['state']=='auto_near_wait':entered.set()
        a.report=report
        await a.start();await asyncio.wait_for(entered.wait(),5)
        await a.stop(emergency=True);n=len(hw.commands)
        await asyncio.sleep(.3)
        assert b.stopped and not b.auto_owner and a.state['state']=='auto_stopped'
        assert not any(cmd.startswith(('P ','S ','C ')) for cmd in hw.commands[n:])
        assert v.calls==4
    asyncio.run(run())


def test_rear_request_during_ir_wait_preempts_front_without_gripping(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=setup_near(b,hw,tmp_path/'auto.json',contact_at=None)
        entered=asyncio.Event();rear_started=asyncio.Event()
        async def report(state):
            if state['state']=='auto_near_wait':entered.set()
            if state['state']=='rear_look':rear_started.set()
        a.report=report
        await a.start();await asyncio.wait_for(entered.wait(),5)
        hw.state['sensors']['D18']=0
        await asyncio.wait_for(rear_started.wait(),2)
        assert 'S 3 2300' not in hw.commands
        await a.stop()
    asyncio.run(run())


@pytest.mark.parametrize('rw,expected',[(.59,False),(.6,True),(.8,True)])
@pytest.mark.parametrize('frame',[[640,480],[1280,720]])
def test_rw_threshold_is_resolution_independent(rw,expected,frame):
    w,h=frame;half=w*rw/2
    extent=image_extent([w/2-half,h*.4,w/2+half,h*.6],frame,NEAR_DEFAULTS)
    assert ('rw' in extent['limit_reasons']) is expected


@pytest.mark.parametrize('bad',[
    {'stop_rw':True},{'stop_rw':float('nan')},{'stop_rw':1.1},
    {'edge_margin_norm':-.1},{'contact_confirm_ms':100},
    {'wait_contact_ms':300,'contact_confirm_ms':300},{'unknown':1},
])
def test_invalid_near_config_is_rejected(bad):
    with pytest.raises(AutoFault):near_config(dict(NEAR_DEFAULTS,**bad))


def test_old_config_uses_near_defaults_and_disabled_limit_stays_disabled(robot,tmp_path):
    b,hw=robot;ready(b);p=tmp_path/'auto.json';cfg=relative_config(b,p)
    cfg.pop('near');p.write_text(json.dumps(cfg))
    assert load_auto_config(p,b.config)['near']==NEAR_DEFAULTS
    disabled=dict(NEAR_DEFAULTS,enabled=False)
    assert not image_extent([0,0,640,480],[640,480],disabled)['limit_reasons']


def test_local_near_settings_validate_and_save_without_motion(robot,tmp_path):
    from fastapi.testclient import TestClient
    from calibration_ui import calibration_app
    b,hw=robot;ready(b);p=tmp_path/'auto.json';relative_config(b,p)
    a=AutoController(b,None,p,AsyncMock());client=TestClient(calibration_app(b,None,a))
    page=client.get('/').text
    token=re.search("const token='([^']+)'",page).group(1);headers={'X-Cal-Token':token}
    n=len(hw.commands);values=dict(NEAR_DEFAULTS,stop_rw=.55,edge_margin_norm=0)
    assert client.post('/api/near',json=values).status_code==403
    response=client.post('/api/near',json=values,headers=headers)
    assert response.status_code==200 and json.loads(p.read_text())['near']==values
    assert client.get('/api/config').json()['near']==values
    values['stop_rw']=1.2
    assert client.post('/api/near',json=values,headers=headers).status_code==400
    assert json.loads(p.read_text())['near']['stop_rw']==.55 and len(hw.commands)==n
