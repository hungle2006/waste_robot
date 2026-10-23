"""V3.7: a centered far target must cause forward motion BEFORE any arm lowering."""
import asyncio,json,re,time
from unittest.mock import AsyncMock
import pytest
from test_control import robot,ready,wait_for
from test_auto import relative_config,finish
from test_near import ContactWorld
from auto_controller import AutoController,AutoFault,approach_config,tracking_config,APPROACH_DEFAULTS,NEAR_DEFAULTS,TRACKING_DEFAULTS


class ApproachVision:
    def __init__(self,b,hw):
        self.b,self.hw=b,hw;self.jpeg=b'frame';self.frame_at=time.monotonic();self.calls=[]
        self.stuck=False;self.offset=0.;self.lost=False;self.relabel=False;self.bin_change=False
        self.arm_drift=False;self.huge=False;self.close_by_size=False;self.base_delay=None
        self.rejected_label=False
    async def detect(self,fresh_after=None):
        joints=self.hw.state['joints'];pulse=joints['p1']
        forward=sum(cmd.startswith('C 100 0 0 ') for cmd in self.hw.commands)
        lateral=sum(cmd.startswith(('C 0 100 0 ','C 0 -100 0 ')) for cmd in self.hw.commands)
        if self.base_delay and forward:await self.base_delay.wait()
        previous=self.calls[-1] if self.calls else None
        self.calls.append((pulse,forward,lateral))
        self.frame_at=time.monotonic()
        if self.lost and forward:return {'found':False,'object':None,'_frame_at':self.frame_at,'_frame_size':[640,480]}
        if pulse==1300:
            bottom=.5 if self.stuck else min(.5+.10*forward,.8)
            cy=bottom-.04;half=.03
            if self.close_by_size:cy=.5;half=.18+.035*forward
            cx=.5+(self.offset if not lateral else 0)
        else:
            cy={1320:.70,1340:.60,1360:.50}.get(pulse,.5);half=.03
            if self.close_by_size:cy={1320:.52,1340:.51,1360:.50}.get(pulse,.5)
            cx=.60 if self.arm_drift else .5
        if self.huge:half=.32
        box=[(cx-half)*640,(cy-.04)*480,(cx+half)*640,(cy+.04)*480]
        label='bottle' if self.relabel and previous==(pulse,forward,lateral) else 'plastic bottle'
        if self.rejected_label and forward:label='other object';box=[420,160,500,240]
        return {'found':True,'object':{'label':label,'bin':2 if self.bin_change and forward else 1,'bbox':box},
                '_frame_at':self.frame_at,'_frame_size':[640,480]}


def configured(b,hw,path,contact=1360):
    ready(b);c=relative_config(b,path,final=1360)
    c['approach']=dict(APPROACH_DEFAULTS,enabled=True)
    c['tracking']=dict(TRACKING_DEFAULTS,allow_label_variation=True)
    c['calibration'].update(contact_sensor='D19',hold_sensor='D19',relative_tolerance_norm=[.03,.08])
    c['control']['settle_ms']=250
    c['near'].update(wait_contact_ms=700,contact_confirm_ms=200)
    path.write_text(json.dumps(c));world=ContactWorld(b,hw,contact)
    stamp=time.monotonic();b.start_monitor();wait_for(lambda:b.updated>stamp)
    v=ApproachVision(b,hw);a=AutoController(b,v,path,AsyncMock())
    return a,v,world,c


def test_centered_far_target_moves_forward_before_lowering_then_releases(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,world,c=configured(b,hw,tmp_path/'auto.json');n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_done',a.state
        sent=hw.commands[n:];first_lower=sent.index('P 1 1320')
        forward=[i for i,cmd in enumerate(sent) if cmd.startswith('C 100 0 0 ')]
        assert len(forward)==3 and max(forward)<first_lower
        assert not any(cmd.startswith('C ') for cmd in sent[first_lower:])
        assert 'P 1 1340' in sent and 'P 1 1360' in sent and 'P 1 1380' not in sent
        assert sent.index('P 9 2350')<min(forward)
        assert sent.index('P 4 1600')>sent.index('S 3 2300')
        assert a.state['cycles']==1 and world.released
        assert all(pulse==1300 for pulse,fw,lat in v.calls if fw<3)
    asyncio.run(run())


def test_lateral_alignment_happens_before_forward_drive(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=configured(b,hw,tmp_path/'auto.json');v.offset=.07;n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_done',a.state
        sent=hw.commands[n:]
        assert next(i for i,s in enumerate(sent) if s.startswith('C 0 100 0 ')) < next(i for i,s in enumerate(sent) if s.startswith('C 100 0 0 '))
    asyncio.run(run())


def test_size_limit_can_stop_forward_before_bottom_line(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=configured(b,hw,tmp_path/'auto.json');v.close_by_size=True;n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_done',a.state
        sent=hw.commands[n:]
        assert len([s for s in sent if s.startswith('C 100 0 0 ')])==1
        assert 'P 1 1320' in sent
    asyncio.run(run())


def test_bin_is_not_changed_when_same_object_gets_a_different_name(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=configured(b,hw,tmp_path/'auto.json');v.relabel=True
        await a.start();await finish(a)
        assert a.state['state']=='auto_done',a.state
        statuses=[call.args[0]['state'] for call in a.report.call_args_list]
        assert 'auto_track_label' in statuses
    asyncio.run(run())


@pytest.mark.parametrize('field',['bin_change','rejected_label'])
def test_changed_bin_or_unrelated_bbox_does_not_continue_approach(robot,tmp_path,field):
    async def run():
        b,hw=robot;a,v,_,_=configured(b,hw,tmp_path/'auto.json');setattr(v,field,True);n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_error' and b.stopped
        assert 'P 1 1320' not in hw.commands[n:] and 'S 3 2300' not in hw.commands[n:]
    asyncio.run(run())


def test_no_visual_forward_progress_holds_search_instead_of_lowering(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=configured(b,hw,tmp_path/'auto.json');v.stuck=True;n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_near_hold' and 'chưa gần lên' in a.state['message']
        sent=hw.commands[n:]
        assert len([s for s in sent if s.startswith('C 100 0 0 ')])==5
        assert 'P 1 1320' not in sent and hw.state['joints']['p1']==1300
        assert not b.stopped and not b.auto_rear
    asyncio.run(run())


def test_forward_step_limit_holds_without_starting_arm(robot,tmp_path):
    async def run():
        b,hw=robot;p=tmp_path/'auto.json';a,v,_,c=configured(b,hw,p)
        c['approach']['max_forward_steps']=2;p.write_text(json.dumps(c));n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_near_hold' and 'hết số nhịp tiến' in a.state['message']
        assert 'P 1 1320' not in hw.commands[n:]
    asyncio.run(run())


def test_lost_object_stops_base_and_does_not_lower(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=configured(b,hw,tmp_path/'auto.json');v.lost=True;n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_near_hold' and 'Mất vật' in a.state['message']
        assert 'P 1 1320' not in hw.commands[n:] and hw.state['motors']==[0,0,0,0]
    asyncio.run(run())


def test_arm_stops_at_configured_limit_with_no_ir(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=configured(b,hw,tmp_path/'auto.json',contact=None);n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_near_hold',a.state
        sent=hw.commands[n:]
        assert hw.state['joints']['p1']==1360 and 'P 1 1380' not in sent
        assert 'S 3 2300' not in sent
        assert not any(s.startswith('C ') for s in sent[sent.index('P 1 1320'):])
    asyncio.run(run())


def test_arm_lateral_drift_holds_without_moving_wheels(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=configured(b,hw,tmp_path/'auto.json');v.arm_drift=True;n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_near_hold' and 'lệch ngang' in a.state['message']
        sent=hw.commands[n:];lower=sent.index('P 1 1320')
        assert not any(s.startswith('C ') for s in sent[lower:]) and 'P 1 1340' not in sent
    asyncio.run(run())


def test_large_search_image_does_not_trigger_blind_forward_motion(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=configured(b,hw,tmp_path/'auto.json');v.huge=True;n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_near_hold'
        assert not any(s.startswith('C ') for s in hw.commands[n:]) and 'P 1 1320' not in hw.commands[n:]
    asyncio.run(run())


def test_stop_during_base_inference_cannot_restart_wheels(robot,tmp_path):
    async def run():
        b,hw=robot;a,v,_,_=configured(b,hw,tmp_path/'auto.json');v.base_delay=asyncio.Event()
        await a.start()
        end=time.monotonic()+5
        while not any(s.startswith('C 100 0 0 ') for s in hw.commands) and time.monotonic()<end:await asyncio.sleep(.02)
        assert any(s.startswith('C 100 0 0 ') for s in hw.commands)
        await a.stop(emergency=True);n=len(hw.commands);v.base_delay.set();await asyncio.sleep(.2)
        assert not any(s.startswith(('C ','P ','S ')) for s in hw.commands[n:])
    asyncio.run(run())


@pytest.mark.parametrize('bad',[
    {'stop_bottom_norm':1.1},{'forward_ms':True},{'max_forward_steps':0},
    {'stop_rw':.6},{'x_tolerance_norm':float('nan')},{'enabled':'yes'},
])
def test_bad_approach_settings_rejected(bad):
    c=dict(APPROACH_DEFAULTS,enabled=True);c.update(bad)
    with pytest.raises(AutoFault):approach_config(c,NEAR_DEFAULTS)


def test_approach_ui_save_does_not_move_hardware(robot,tmp_path):
    from fastapi.testclient import TestClient
    from calibration_ui import calibration_app
    b,hw=robot;p=tmp_path/'auto.json';a,v,_,c=configured(b,hw,p)
    client=TestClient(calibration_app(b,v,a));token=re.search("const token='([^']+)'",client.get('/').text).group(1)
    body={'approach':c['approach'],'tracking':c['tracking'],'speed':20};n=len(hw.commands)
    assert client.post('/api/approach',json=body).status_code==403
    assert client.post('/api/approach',json=body,headers={'X-Cal-Token':token}).status_code==200
    assert client.get('/api/config').json()['approach']['enabled']
    body['approach']['stop_rw']=.6
    assert client.post('/api/approach',json=body,headers={'X-Cal-Token':token}).status_code==400
    assert all(command=='STATUS' for command in hw.commands[n:])  # Background telemetry is not motion.
