import asyncio,copy,json,time
from pathlib import Path
from unittest.mock import AsyncMock
import pytest
from test_control import robot,ready,wait_for
from auto_controller import AutoController,AutoFault,load_auto_config,choose_direction,infer_image_direction,sensor_pin
from realtime_bridge import RealtimeService


def calibrated(b,path,final=1340):
    cfg=json.loads((Path(__file__).resolve().parents[1]/'python/auto_config.json').read_text())
    cal=cfg['calibration'];cal.update(mode='calibrated',frozen_joints_us={j:b.config['poses']['SEARCH'][j] for j in ('p10','p11','d2')},
        positive_error_x_move='RIGHT',positive_error_y_move='FORWARD',contact_sensor='D15',hold_sensor='D16',
        grasp_final_s2_us=final,stages=[{'s2_us':1300,'target_px':[320,240],'tolerance_px':[16,16]},
                                     {'s2_us':final,'target_px':[320,240],'tolerance_px':[16,16]}])
    cfg['control'].update(settle_ms=100,repeat=False,s2_step_us=20)
    # These tests exercise the V3.6/recorded-stage path; V3.7 has separate approach tests.
    cfg['approach']['enabled']=False
    cfg['tracking']['allow_label_variation']=False
    cfg['target_lock']['enabled']=False
    cfg['motion']['enabled']=False
    cfg['visibility']['enabled']=False
    cfg['distance']['enabled']=False  # Legacy image-only path; metric mode has separate tests.
    cal['relative_tolerance_norm']=[.08,.08]
    path.write_text(json.dumps(cfg));return cfg


def relative_config(b,path,final=1340):
    cfg=calibrated(b,path,final)
    cfg['calibration'].update(mode='relative',stages=[],positive_error_x_move=None,positive_error_y_move=None)
    path.write_text(json.dumps(cfg));return cfg


class Vision:
    def __init__(self,b,hw,final=1340):
        self.b,self.hw,self.final=b,hw,final
        self.jpeg=b'frame';self.frame_at=time.monotonic();self.calls=0
        self.missing_at=None;self.bad_bin=None;self.delay=None;self.shift_first=False
    async def detect(self,fresh_after=None):
        self.calls+=1
        if self.delay: await self.delay.wait()
        stamp=max(time.monotonic(),(fresh_after or 0)+.00001)
        self.frame_at=stamp
        pulse=self.b.last_hw.get('joints',{}).get('p1',1300)
        # Fixture IR: proximity at final pose; held object after closing; free after opening.
        self.hw.state['sensors']['D15']=0 if pulse==self.final else 1
        self.b.last_hw['sensors']['D15']=self.hw.state['sensors']['D15']
        if self.missing_at==self.calls:return {'found':False,'object':None,'_frame_at':stamp,'_frame_size':[640,480]}
        shift=60 if self.shift_first and not any(cmd.startswith('C ') for cmd in self.hw.commands) else 0
        return {'found':True,'object':{'label':'plastic bottle','bin':self.bad_bin or 1,'category':'recyclable',
            'bbox':[300+shift,220,340+shift,260],'center':[320+shift,240]},'_frame_at':stamp,'_frame_size':[640,480]}


def hold_monitor(b,hw):
    closed=b.last_hw.get('joints',{}).get('d3')==b.config['grip']['close']
    hw.state['sensors']['D16']=0 if closed else 1
    return closed

async def feed_sensors(b,hw):
    while True:
        hold_monitor(b,hw)
        await asyncio.sleep(.01)

async def finish(a,timeout=15):
    await asyncio.wait_for(asyncio.shield(a.task),timeout)


def test_incomplete_calibration_never_moves(robot,tmp_path):
    b,hw=robot;ready(b)
    p=tmp_path/'auto.json';c=json.loads((Path(__file__).resolve().parents[1]/'python/auto_config.json').read_text())
    c['calibration']['mode']='calibrated';c['calibration']['positive_error_x_move']=None;c['calibration']['positive_error_y_move']=None
    p.write_text(json.dumps(c))
    a=AutoController(b,None,p,AsyncMock());n=len(hw.commands)
    asyncio.run(a.start())
    assert a.state['state']=='auto_not_ready' and 'hiệu chuẩn' in a.state['message']
    assert len(hw.commands)==n and not b.auto_owner


def test_valid_config_and_changed_search_rejected(robot,tmp_path):
    b,_=robot;ready(b);p=tmp_path/'auto.json';c=calibrated(b,p)
    assert load_auto_config(p,b.config)['calibration']['grasp_final_s2_us']==1340
    c['calibration']['frozen_joints_us']['p10']=1000;p.write_text(json.dumps(c))
    with pytest.raises(AutoFault,match='SEARCH'):load_auto_config(p,b.config)


def test_image_axes_and_deadband():
    stage={'target_px':[320,240],'tolerance_px':[16,16]}
    cal={'positive_error_x_move':'LEFT','positive_error_y_move':'BACK'}
    assert choose_direction([330,245],stage,cal)[0] is None
    assert choose_direction([370,240],stage,cal)[0]=='LEFT'
    assert choose_direction([270,240],stage,cal)[0]=='RIGHT'
    assert choose_direction([320,280],stage,cal)[0]=='BACK'


def test_whole_cycle_s2_only_approach_and_correct_bin(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';calibrated(b,p)
        vision=Vision(b,hw);a=AutoController(b,vision,p,AsyncMock())
        sensor_task=asyncio.create_task(feed_sensors(b,hw))
        n=len(hw.commands)
        await a.start();await finish(a)
        sensor_task.cancel();await asyncio.gather(sensor_task,return_exceptions=True)
        assert a.state['state']=='auto_done',a.state
        assert a.state['cycles']==1 and not b.auto_owner
        sent=hw.commands[n:]
        assert 'P 9 2350' in sent and 'P 4 1600' in sent
        gripclose=sent.index('S 3 2300')
        assert sent.index('P 9 2350') < sent.index('P 1 1320') < gripclose < sent.index('P 4 1600')
        assert sent.index('P 4 1600') < sent.index('S 3 600',gripclose)
        assert not any(x.startswith(('P 10 ','P 11 ','S 2 ')) for x in sent[sent.index('P 1 1320'):gripclose])
        assert b.last_hw['joints']['p1']==1300
        assert vision.calls>=8  # Each lowering step receives new detections.
        assert hw.state['motors']==[0,0,0,0]
    asyncio.run(run())


def test_lost_target_stops_without_grip(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';calibrated(b,p)
        v=Vision(b,hw);v.missing_at=3
        a=AutoController(b,v,p,AsyncMock());n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_error' and b.stopped
        assert 'S 3 2300' not in hw.commands[n:]
    asyncio.run(run())


def test_cancel_pending_modal_has_no_late_motion(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';calibrated(b,p)
        v=Vision(b,hw);v.delay=asyncio.Event()
        a=AutoController(b,v,p,AsyncMock());await a.start()
        end=time.monotonic()+3
        while not v.calls and time.monotonic()<end: await asyncio.sleep(.02)
        await a.stop(emergency=True);n=len(hw.commands)
        v.delay.set();await asyncio.sleep(.1)
        assert not any(x.startswith(('P ','S ','C ')) for x in hw.commands[n:])
        assert b.stopped and a.emergency_latched
    asyncio.run(run())


def test_duplicate_start_and_manual_exclusion(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';calibrated(b,p)
        v=Vision(b,hw);v.delay=asyncio.Event();a=AutoController(b,v,p,AsyncMock())
        await a.start();task=a.task;await a.start();assert a.task is task
        with pytest.raises(ValueError,match='AUTO'):b.command(None,b.epoch,'p10',1000)
        with pytest.raises(ValueError,match='AUTO'):b.save_pose('SEARCH',{'p1':1300},b.epoch)
        await a.stop()
    asyncio.run(run())


def test_actual_legacy_start_event_and_disconnect(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);p=tmp_path/'auto.json';calibrated(b,p)
        ch=type('Channel',(),{'send_broadcast':AsyncMock()})()
        svc=RealtimeService(b,'robot_01',ch);svc.online=True
        svc.auto=AutoController(b,Vision(b,hw),p,svc.auto_report)
        svc.on_command({'payload':{'robot_id':'robot_01','type':'mode','action':'auto_start','ts':int(time.time()*1000)}})
        await asyncio.sleep(.02)
        assert svc.auto.running
        await svc.disconnected();assert b.stopped and not svc.auto.running
    asyncio.run(run())


def test_stale_auto_start_event_rejected(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);p=tmp_path/'auto.json';calibrated(b,p)
        ch=type('Channel',(),{'send_broadcast':AsyncMock()})();svc=RealtimeService(b,'robot_01',ch);svc.online=True
        svc.auto=AutoController(b,Vision(b,hw),p,svc.auto_report)
        await svc.mode({'action':'auto_start','ts':0})
        assert not svc.auto.running
    asyncio.run(run())


def test_calibration_page_records_actual_pulses_without_hardware_motion(robot,tmp_path):
    from fastapi.testclient import TestClient
    from calibration_ui import calibration_app
    import re
    b,hw=robot;ready(b);p=tmp_path/'auto.json';calibrated(b,p)
    c=json.loads(p.read_text());c['calibration']['stages']=[];c['calibration']['frozen_joints_us']={};p.write_text(json.dumps(c))
    v=Vision(b,hw);v.frame_size=[640,480];v.frame_at=time.monotonic()
    b.motion_finished_at=time.monotonic()-1
    a=AutoController(b,v,p,AsyncMock());client=TestClient(calibration_app(b,v,a))
    token=re.search("const token='([^']+)'",client.get('/').text).group(1)
    n=len(hw.commands)
    shot=client.get('/api/capture').json()
    result=client.post('/api/stage',json={'capture_id':shot['id'],'target_px':[320,240],'tolerance_px':[16,16]},headers={'X-Cal-Token':token})
    assert result.status_code==200 and len(hw.commands)==n
    saved=json.loads(p.read_text())['calibration']
    assert saved['stages'][0]['s2_us']==1300
    assert saved['frozen_joints_us']=={'p10':950,'p11':1250,'d2':2100}
    assert client.post('/api/reset',json={}).status_code==403


def test_changed_resolution_and_stale_result_cannot_move(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';calibrated(b,p)
        v=Vision(b,hw);a=AutoController(b,v,p,AsyncMock())
        original=v.detect
        async def wrong(fresh_after=None):
            r=await original(fresh_after=fresh_after);r['_frame_size']=[1920,1080];return r
        v.detect=wrong
        n=len(hw.commands);await a.start();await finish(a)
        assert a.state['state']=='auto_error' and 'Kích thước' in a.state['message']
        assert not any(x.startswith('C ') for x in hw.commands[n:])
    asyncio.run(run())


def test_shifted_target_drives_then_stops_before_lowering(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';calibrated(b,p)
        v=Vision(b,hw);v.shift_first=True;a=AutoController(b,v,p,AsyncMock());n=len(hw.commands)
        st=asyncio.create_task(feed_sensors(b,hw));await a.start();await finish(a)
        st.cancel();await asyncio.gather(st,return_exceptions=True)
        sent=hw.commands[n:]
        drive=next(i for i,x in enumerate(sent) if x.startswith('C '))
        lower=sent.index('P 1 1320')
        assert 'XM' in sent[drive+1:lower] and a.state['state']=='auto_done'
    asyncio.run(run())


def test_no_contact_at_final_prevents_grip(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';calibrated(b,p)
        v=Vision(b,hw,final=1999);a=AutoController(b,v,p,AsyncMock());n=len(hw.commands)
        await a.start();await finish(a)
        assert 'IR chưa thấy' in a.state['message']
        assert 'S 3 2300' not in hw.commands[n:]
    asyncio.run(run())


def test_no_hold_confirmation_keeps_lid_closed_and_prevents_release(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';calibrated(b,p)
        a=AutoController(b,Vision(b,hw),p,AsyncMock());n=len(hw.commands)
        await a.start();await finish(a)
        assert 'chưa xác nhận' in a.state['message']
        assert 'P 9 2350' in hw.commands[n:]  # Classification now selects a bin before pickup.
        assert 'P 4 1600' not in hw.commands[n:]
        close=hw.commands[n:].index('S 3 2300')
        assert 'S 3 600' not in hw.commands[n:][close+1:]
    asyncio.run(run())


def test_deferred_start_after_stop_is_rejected(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);p=tmp_path/'auto.json';calibrated(b,p)
        ch=type('Channel',(),{'send_broadcast':AsyncMock()})();svc=RealtimeService(b,'robot_01',ch);svc.online=True
        svc.auto=AutoController(b,Vision(b,hw),p,svc.auto_report)
        issued=int(time.time()*1000)-100
        await svc.mode({'action':'auto_stop'})
        await svc.mode({'action':'auto_start','ts':issued})
        assert not svc.auto.running and b.stopped
    asyncio.run(run())


def test_explicit_connect_clears_emergency_before_first_usb_connection(robot):
    from test_realtime import packet
    async def run():
        b,hw=robot;ready(b)
        ch=type('Channel',(),{'send_broadcast':AsyncMock()})();svc=RealtimeService(b,'robot_01',ch);svc.online=True
        svc.auto=AutoController(b,None,Path('unused.json'),svc.auto_report)
        svc.auto.emergency_latched=True
        await svc.command(packet(svc.dispatcher,1,'connect'))
        assert not svc.auto.emergency_latched
    asyncio.run(run())


def test_relative_points_and_user_search_pulses(robot,tmp_path):
    b,_=robot;ready(b);p=tmp_path/'auto.json';c=calibrated(b,p)
    source=json.loads((Path(__file__).resolve().parents[1]/'python/auto_config.json').read_text())
    assert source['calibration']['contact_sensor']==source['calibration']['hold_sensor']=='D19'
    assert source['calibration']['frozen_joints_us']=={'p10':950,'p11':1250,'d2':2100}
    assert source['measurements_reference_only']['search_pulses_us']==b.config['poses']['SEARCH']
    c['calibration'].update(contact_sensor='D19',hold_sensor='D19',stages=[
        {'s2_us':1300,'target_norm':[.45,.6],'tolerance_norm':[.025,.035]},
        {'s2_us':1340,'target_norm':[.5,.5],'tolerance_norm':[.025,.035]}])
    p.write_text(json.dumps(c));cfg=load_auto_config(p,b.config)
    assert cfg['calibration']['stages'][0]['target_px']==[288,288]
    assert cfg['calibration']['stages'][0]['tolerance_px']==pytest.approx([16,16.8])
    assert sensor_pin('D19')=='D19' and sensor_pin('P15') is None
    assert sensor_pin('PCA15') is None
    assert json.loads(p.read_text())['calibration']['stages'][0].get('target_px') is None
    c['calibration']['stages'][0]['target_norm']=[1.1,.5];p.write_text(json.dumps(c))
    with pytest.raises(AutoFault,match='tương đối'):load_auto_config(p,b.config)


@pytest.mark.parametrize('command,after,expected',[
    ('RIGHT',[290,240],'RIGHT'),('RIGHT',[350,240],'LEFT'),
    ('FORWARD',[320,220],'FORWARD'),('FORWARD',[320,260],'BACK')])
def test_manual_trial_establishes_correct_image_compensation(command,after,expected):
    assert infer_image_direction([320,240],after,command,[640,480])[0]==expected
    with pytest.raises(AutoFault,match='1%'):
        infer_image_direction([320,240],[321,241],command,[640,480])


def test_direction_capture_pair_and_relative_record_never_send_motion(robot,tmp_path):
    from fastapi.testclient import TestClient
    from calibration_ui import calibration_app
    import re
    b,hw=robot;ready(b);p=tmp_path/'auto.json';calibrated(b,p)
    v=Vision(b,hw);v.frame_size=[640,480];b.motion_finished_at=time.monotonic()-1
    a=AutoController(b,v,p,AsyncMock());client=TestClient(calibration_app(b,v,a))
    token=re.search("const token='([^']+)'",client.get('/').text).group(1);headers={'X-Cal-Token':token}
    # This test exercises read-only calibration, using fresh simulated sensor/frame
    # timestamps for each capture; it does not run the background USB monitor.
    b.updated=v.frame_at=time.monotonic()
    n=len(hw.commands);response=client.get('/api/capture')
    assert response.status_code==200,response.json()
    before=response.json()
    body={'before_id':before['id'],'after_id':before['id'],'before_px':[320,240],
          'after_px':[280,240],'command':'RIGHT'}
    assert client.post('/api/direction',json=body,headers=headers).status_code==409
    b.updated=v.frame_at=time.monotonic();response=client.get('/api/capture')
    assert response.status_code==200,response.json()
    after=response.json();body['after_id']=after['id']
    result=client.post('/api/direction',json=body,headers=headers)
    assert result.status_code==200 and result.json()['move']=='RIGHT'
    saved=json.loads(p.read_text())['calibration']
    assert saved['direction_trials']['positive_error_x_move']['delta_norm']==[-.0625,0]
    response=client.post('/api/stage',json={'capture_id':after['id'],
        'target_norm':[.4375,.5],'tolerance_norm':[.025,.035]},headers=headers)
    assert response.status_code==200
    saved=json.loads(p.read_text())['calibration']
    assert saved['stages'][0]['target_norm']==[.4375,.5]
    assert len(hw.commands)==n
    assert client.post('/api/direction',json=body).status_code==403


def test_trial_rejects_changed_arm_pose(robot,tmp_path):
    from fastapi.testclient import TestClient
    from calibration_ui import calibration_app
    import re
    b,hw=robot;ready(b);p=tmp_path/'auto.json';calibrated(b,p)
    v=Vision(b,hw);v.frame_size=[640,480];b.motion_finished_at=time.monotonic()-1
    a=AutoController(b,v,p,AsyncMock());client=TestClient(calibration_app(b,v,a))
    token=re.search("const token='([^']+)'",client.get('/').text).group(1)
    before=client.get('/api/capture').json()
    b.last_hw['joints']['p1']=1320;v.frame_at=time.monotonic();after=client.get('/api/capture').json()
    result=client.post('/api/direction',json={'before_id':before['id'],'after_id':after['id'],
        'before_px':[320,240],'after_px':[280,240],'command':'RIGHT'},headers={'X-Cal-Token':token})
    assert result.status_code==409 and 'Giữ nguyên tay' in result.json()['detail']


@pytest.mark.parametrize('mode',['calibrated','relative'])
def test_single_d19_presence_through_lift_and_release(robot,tmp_path,mode):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';c=(relative_config if mode=='relative' else calibrated)(b,p)
        c['calibration'].update(contact_sensor='D19',hold_sensor='D19')
        p.write_text(json.dumps(c))
        def update():
            joints=hw.state['joints']
            present=joints.get('p1')==1340 or joints.get('d3')==2300
            hw.state['sensors']['D19']=0 if present else 1
            b.last_hw['sensors']['D19']=hw.state['sensors']['D19']
        class SingleVision(Vision):
            async def detect(self,fresh_after=None):
                result=await super().detect(fresh_after=fresh_after);update();return result
        async def feed():
            while True: update();await asyncio.sleep(.01)
        v=SingleVision(b,hw);a=AutoController(b,v,p,AsyncMock());feed_task=asyncio.create_task(feed())
        await a.start();await finish(a);feed_task.cancel();await asyncio.gather(feed_task,return_exceptions=True)
        assert a.state['state']=='auto_done',a.state
        assert hw.state['sensors']['D16']==1  # No mandatory second sensor.
        assert hw.state['sensors']['D19']==1 and a.state['cycles']==1
    asyncio.run(run())


def test_relative_anchor_path_interpolates_during_actual_lowering(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';c=calibrated(b,p)
        c['calibration']['stages']=[
            {'s2_us':1300,'target_norm':[.45,.6],'tolerance_norm':[.025,.035]},
            {'s2_us':1340,'target_norm':[.5,.5],'tolerance_norm':[.025,.035]}]
        p.write_text(json.dumps(c));v=Vision(b,hw);a=AutoController(b,v,p,AsyncMock())
        async def follow(fresh_after=None):
            result=await Vision.detect(v,fresh_after=fresh_after)
            fraction=(b.last_hw['joints']['p1']-1300)/40
            x=640*(.45+fraction*.05);y=480*(.6-fraction*.1)
            result['object']['bbox']=[x-10,y-10,x+10,y+10];return result
        v.detect=follow;feed_task=asyncio.create_task(feed_sensors(b,hw))
        await a.start();await finish(a);feed_task.cancel();await asyncio.gather(feed_task,return_exceptions=True)
        assert a.state['state']=='auto_done',a.state
        assert not any(cmd.startswith('C ') for cmd in hw.commands)
    asyncio.run(run())


def test_relative_default_ready_without_measured_points_or_directions(robot,tmp_path):
    b,_=robot;ready(b)
    p=tmp_path/'auto.json';p.write_bytes((Path(__file__).resolve().parents[1]/'python/auto_config.json').read_bytes())
    with pytest.raises(AutoFault,match='stop_forward_cm'):
        load_auto_config(p,b.config)  # V3.10 does not pretend an unmeasured cm goal is ready.
    legacy=json.loads(p.read_text());legacy['distance']['enabled']=False;p.write_text(json.dumps(legacy))
    cfg=load_auto_config(p,b.config)
    assert cfg['calibration']['mode']=='relative'
    assert cfg['calibration']['stages'][0]['target_px']==[320,240]
    assert cfg['calibration']['stages'][0]['tolerance_px']==pytest.approx([19.2,38.4])
    assert json.loads(p.read_text())['calibration']['stages']==[]  # Runtime references aren't measured data.
    b.config['poses']['SEARCH']['p10']=1000
    assert load_auto_config(p,b.config)['calibration']['frozen_joints_us']['p10']==1000


def test_relative_ir_stops_lowering_before_reference_limit(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';relative_config(b,p,final=1360)
        v=Vision(b,hw,final=1320);a=AutoController(b,v,p,AsyncMock());n=len(hw.commands)
        sensor_task=asyncio.create_task(feed_sensors(b,hw))
        await a.start();await finish(a);sensor_task.cancel();await asyncio.gather(sensor_task,return_exceptions=True)
        assert a.state['state']=='auto_done',a.state
        sent=hw.commands[n:];close=sent.index('S 3 2300')
        assert 'P 1 1320' in sent[:close] and 'P 1 1340' not in sent[:close] and 'P 1 1360' not in sent[:close]
        assert not any(x.startswith(('P 10 ','P 11 ','S 2 ')) for x in sent[sent.index('P 1 1320'):close])
    asyncio.run(run())


@pytest.mark.parametrize('axis,first,corrected',[(0,'C 0 100 0','C 0 -100 0'),(1,'C -100 0 0','C 100 0 0')])
def test_relative_feedback_corrects_reversed_image_direction(robot,tmp_path,axis,first,corrected):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';relative_config(b,p)
        v=Vision(b,hw);original=v.detect
        async def reversed_response(fresh_after=None):
            r=await original(fresh_after=fresh_after)
            values=[.65,.70,.62,.55]
            moves=sum(cmd.startswith('C ') for cmd in hw.commands[n:])
            pos=values[min(moves,len(values)-1)]
            xy=[320,240];xy[axis]=pos*[640,480][axis]
            x,y=xy;r['object']['bbox']=[x-10,y-10,x+10,y+10];return r
        v.detect=reversed_response;a=AutoController(b,v,p,AsyncMock());n=len(hw.commands)
        sensor_task=asyncio.create_task(feed_sensors(b,hw))
        await a.start();await finish(a);sensor_task.cancel();await asyncio.gather(sensor_task,return_exceptions=True)
        assert a.state['state']=='auto_done',a.state
        drives=[cmd for cmd in hw.commands[n:] if cmd.startswith('C ')]
        assert drives[0].startswith(first) and drives[1].startswith(corrected)
        assert len(drives)==3 and hw.state['motors']==[0,0,0,0]
    asyncio.run(run())


def test_relative_no_image_response_stops_before_lowering_or_grip(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';relative_config(b,p)
        v=Vision(b,hw);original=v.detect
        async def stuck(fresh_after=None):
            r=await original(fresh_after=fresh_after);r['object']['bbox']=[438,230,458,250];return r
        v.detect=stuck;a=AutoController(b,v,p,AsyncMock());n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_error' and 'phản hồi ảnh' in a.state['message']
        sent=hw.commands[n:]
        assert len([cmd for cmd in sent if cmd.startswith('C ')])==3
        assert 'P 1 1320' not in sent and 'S 3 2300' not in sent and b.stopped
    asyncio.run(run())


def test_relative_resolution_changes_keep_normalized_target(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';relative_config(b,p)
        v=Vision(b,hw);original=v.detect
        async def resized(fresh_after=None):
            r=await original(fresh_after=fresh_after)
            w,h=([640,480] if v.calls<3 else [1280,720])
            r['_frame_size']=[w,h];r['object']['bbox']=[w*.48,h*.48,w*.52,h*.52];return r
        v.detect=resized;a=AutoController(b,v,p,AsyncMock());n=len(hw.commands)
        sensor_task=asyncio.create_task(feed_sensors(b,hw))
        await a.start();await finish(a);sensor_task.cancel();await asyncio.gather(sensor_task,return_exceptions=True)
        assert a.state['state']=='auto_done',a.state
        assert not any(cmd.startswith('C ') for cmd in hw.commands[n:])
    asyncio.run(run())


def test_relative_ir_outside_reference_does_not_drive_or_grip(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';relative_config(b,p)
        v=Vision(b,hw,final=1320);original=v.detect
        async def offset(fresh_after=None):
            r=await original(fresh_after=fresh_after)
            if b.last_hw['joints']['p1']==1320:r['object']['bbox']=[422,230,442,250]
            return r
        v.detect=offset;a=AutoController(b,v,p,AsyncMock());n=len(hw.commands)
        await a.start();await finish(a)
        assert a.state['state']=='auto_error' and 'IR đã phát hiện' in a.state['message']
        assert 'S 3 2300' not in hw.commands[n:] and not any(x.startswith('C ') for x in hw.commands[n:])
    asyncio.run(run())


def test_relative_limit_without_ir_never_closes_gripper(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';relative_config(b,p)
        a=AutoController(b,Vision(b,hw,final=1999),p,AsyncMock());n=len(hw.commands)
        await a.start();await finish(a)
        assert 'giới hạn hạ tương đối' in a.state['message'] and a.state['state']=='auto_error'
        assert 'S 3 2300' not in hw.commands[n:] and 'P 1 1360' not in hw.commands[n:]
    asyncio.run(run())


def test_relative_settings_save_without_recording_or_hardware_motion(robot,tmp_path):
    from fastapi.testclient import TestClient
    from calibration_ui import calibration_app
    import re
    b,hw=robot;ready(b);p=tmp_path/'auto.json';relative_config(b,p)
    a=AutoController(b,None,p,AsyncMock());client=TestClient(calibration_app(b,None,a))
    token=re.search("const token='([^']+)'",client.get('/').text).group(1);headers={'X-Cal-Token':token}
    n=len(hw.commands)
    body={'target_norm':[.5,.6],'tolerance_norm':[.08,.1],'grasp_final_s2_us':1340,'repeat':False}
    assert client.post('/api/relative',json=body,headers=headers).status_code==200
    saved=json.loads(p.read_text());assert saved['calibration']['mode']=='relative'
    assert saved['calibration']['stages']==[] and saved['calibration']['relative_target_norm']==[.5,.6]
    assert 'không cần ghi' in client.get('/api/config').json()['readiness']
    assert len(hw.commands)==n
    assert client.post('/api/relative',json=body).status_code==403
    body['target_norm']=[1.1,.5]
    assert client.post('/api/relative',json=body,headers=headers).status_code==400
