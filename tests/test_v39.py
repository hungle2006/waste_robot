"""Smooth finite wheel profiles, no rendering, and observed S2 visibility guards."""
import asyncio,json,re,subprocess,time
from pathlib import Path
from unittest.mock import AsyncMock
import pytest
from test_control import robot
from test_approach import configured
from test_auto import finish
from test_target_lock import until
from auto_controller import AutoController
from auto_motion import MOTION_DEFAULTS,VISIBILITY_DEFAULTS,checked_settings,visible_step
from target_lock import DEFAULTS
from mecanum import smooth_pwm,smooth_timing
from livekit_modal import LiveKitModal


@pytest.mark.parametrize('power',[1,20,80,255])
def test_profile_is_monotonic_up_down_and_zero_at_end(power):
    values=[smooth_pwm(power,t,120,0) for t in range(241)]
    assert values[0]==0 and values[120]==power and values[-1]==0
    assert values[:121]==sorted(values[:121]) and values[120:]==sorted(values[120:],reverse=True)
    assert abs(sum(values)-power*120)<=power*2


@pytest.mark.parametrize('duration,cap',[(60,160),(100,160),(120,160),(400,160)])
def test_profile_preserves_nominal_pulse_area_without_extending_cruise(duration,cap):
    ramp,hold,total=smooth_timing(duration,cap)
    assert ramp+hold==duration and total==duration+ramp and total<=800


def test_actual_sketch_profile_and_safety_compile_on_host(tmp_path):
    root=Path(__file__).resolve().parent/'firmware_host';binary=tmp_path/'firmware'
    subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-I',str(root),str(root/'harness.cpp'),'-o',str(binary)],check=True,capture_output=True,text=True)
    subprocess.run([str(binary)],check=True,capture_output=True,text=True)


class FollowVision:
    def __init__(self,b,hw):
        self.b,self.hw=b,hw;self.jpeg=b'image';self.frame_at=time.monotonic();self.calls=[]
        self.bad=False;self.lost=False;self.stuck_arm=False;self.drift=False;self.obscured=False
    async def detect(self,fresh_after=None):
        pulse=self.hw.state['joints']['p1']
        forward=sum(x.startswith('G 100 0 0 ') for x in self.hw.commands)
        self.calls.append((pulse,forward));self.frame_at=time.monotonic()
        result={'_frame_at':self.frame_at,'_frame_size':[640,480]}
        if self.lost and pulse>1300:return dict(result,found=False,object=None)
        bottom=min(.46+.04*forward,.54)
        if self.obscured:bottom=.63
        if pulse>1300:
            bottom=.54 if self.stuck_arm else .54-.0008*(pulse-1300)
            if self.bad:bottom=.54+.01*(pulse-1300)
        cx=.62 if self.drift and pulse>1300 else .5
        obj={'label':'bottle','bin':1,'bbox':[(cx-.03)*640,(bottom-.08)*480,(cx+.03)*640,bottom*480],
             '_appearance':[1.,0.]}
        return dict(result,found=True,object=obj)


def setup_follow(robot,tmp_path,contact=1332):
    b,hw=robot;p=tmp_path/'v39.json'
    a,_,world,c=configured(b,hw,p,contact=contact)
    c['motion']=dict(MOTION_DEFAULTS,enabled=True)
    c['visibility']=dict(VISIBILITY_DEFAULTS,enabled=True)
    c['target_lock']=dict(DEFAULTS,enabled=True,continuous_auto=False,retry_ms=100)
    p.write_text(json.dumps(c));v=FollowVision(b,hw);a.vision=v
    return b,hw,a,v,world,c


def test_smooth_forward_early_handoff_small_arm_steps_then_ir_release(robot,tmp_path):
    async def run():
        b,hw,a,v,world,c=setup_follow(robot,tmp_path);n=len(hw.commands)
        await a.start();await finish(a,timeout=20)
        assert a.state['state']=='auto_done' and world.released,a.state
        sent=hw.commands[n:];profiles=[x for x in sent if x.startswith('G ')]
        assert profiles==['G 100 0 0 20 120 0','G 100 0 0 20 60 0']
        first=sent.index('P 1 1308')
        assert not any(x.startswith(('C ','G ')) for x in sent[first:])
        lower=[int(x.split()[2]) for x in sent[:sent.index('S 3 2300')] if x.startswith('P 1 ') and int(x.split()[2])>1300]
        assert lower==[1308,1316,1324,1332] and max(lower)<1360
        assert sent.index('P 4 1600')>sent.index('S 3 2300')
        assert any(call.args[0]['state']=='auto_arm_follow' for call in a.report.call_args_list)
    asyncio.run(run())


def test_wrong_camera_response_retracts_one_observed_step_and_holds(robot,tmp_path):
    async def run():
        b,hw,a,v,_,_=setup_follow(robot,tmp_path,contact=1900);v.bad=True
        await a.start();await until(lambda:a.state['state']=='auto_hold')
        assert hw.state['joints']['p1']==1300 and a.running
        assert 'P 1 1308' in hw.commands and 'P 1 1316' not in hw.commands
        assert 'S 3 2300' not in hw.commands
        await a.stop()
    asyncio.run(run())


@pytest.mark.parametrize('mode',['lost','drift','stuck_arm','obscured'])
def test_unsafe_view_never_continues_blind_lowering(robot,tmp_path,mode):
    async def run():
        b,hw,a,v,_,_=setup_follow(robot,tmp_path,contact=1900);setattr(v,mode,True)
        await a.start()
        if mode=='lost':await until(lambda:a.state['state']=='auto_reacquire' and hw.state['joints']['p1']>1300)
        else:await until(lambda:a.state['state']=='auto_hold',timeout=12)
        n=len(hw.commands);pulse=hw.state['joints']['p1'];await asyncio.sleep(.15)
        assert a.running and b.auto_owner==a.token and 'S 3 2300' not in hw.commands
        assert hw.state['joints']['p1']==pulse<=1332
        assert not any(x.startswith(('P ','S ','C ','G ')) for x in hw.commands[n:])
        await a.stop()
    asyncio.run(run())


def test_old_firmware_is_refused_before_auto_pose_or_motor_commands(robot,tmp_path):
    async def run():
        b,hw,a,v,_,_=setup_follow(robot,tmp_path)
        hw.state.pop('smooth_protocol');b.last_hw.pop('smooth_protocol',None);n=len(hw.commands)
        await a.start()
        assert not a.running and a.state['state']=='auto_not_ready' and 'V3.9' in a.state['message']
        assert not any(x.startswith(('P ','S ','C ','G ')) for x in hw.commands[n:])
    asyncio.run(run())


def test_stop_during_ramp_never_restarts_motor_or_servo(robot,tmp_path):
    async def run():
        b,hw,a,v,_,_=setup_follow(robot,tmp_path)
        await a.start();await until(lambda:any(x.startswith('G ') for x in hw.commands) and any(hw.request('STATUS')['motors']))
        await a.stop(emergency=True);n=len(hw.commands);await asyncio.sleep(.3)
        assert b.stopped and not b.auto_owner and not a.running and hw.state['motors']==[0,0,0,0]
        assert not any(x.startswith(('G ','C ','P ','S ')) for x in hw.commands[n:])
    asyncio.run(run())


def test_front_ir_aborts_finite_profile_immediately(robot,tmp_path):
    async def run():
        b,hw,a,v,_,_=setup_follow(robot,tmp_path)
        await a.start();await until(lambda:any(x.startswith('G ') for x in hw.commands))
        hw.state['sensors']['D19']=0;b.last_hw['sensors']['D19']=0
        await until(lambda:a.state['state']=='auto_hold')
        assert hw.request('STATUS')['motors']==[0,0,0,0] and hw.profile is None
        assert 'P 1 1308' not in hw.commands
        await a.stop()
    asyncio.run(run())


@pytest.mark.parametrize('sensor',['D18','D19'])
def test_ir_activation_exactly_at_profile_start_is_not_a_fatal_auto_error(robot,tmp_path,sensor):
    async def run():
        b,hw,a,v,_,_=setup_follow(robot,tmp_path)
        request=hw.request;injected=False
        def edge(command):
            nonlocal injected
            if command.startswith('G ') and not injected:
                injected=True;hw.state['sensors'][sensor]=0
            return request(command)
        hw.request=edge
        await a.start()
        await until(lambda:a.flow=='rear' if sensor=='D18' else a.state['state']=='auto_hold')
        assert injected and a.running and b.auto_owner==a.token and not b.stopped
        assert hw.profile is None and hw.state['motors']==[0,0,0,0]
        await a.stop()
    asyncio.run(run())


@pytest.mark.parametrize('values,defaults',[
    ({'ramp_ms':0},MOTION_DEFAULTS),({'arm_step_us':101},MOTION_DEFAULTS),
    ({'blur_bottom_norm':.55,'trigger_bottom_norm':.54},VISIBILITY_DEFAULTS),
    ({'trigger_bottom_norm':float('nan')},VISIBILITY_DEFAULTS),
])
def test_bad_motion_parameters(values,defaults):
    with pytest.raises(ValueError):checked_settings(values,defaults)


def test_predictive_step_shrinks_before_losing_top_of_object():
    v=dict(VISIBILITY_DEFAULTS);m=dict(MOTION_DEFAULTS)
    obj={'bbox_norm':[.4,.06,.5,.30]};old={'bbox_norm':[.4,.085,.5,.325]}
    step=visible_step(obj,1308,1360,(1300,old),m,v)
    assert 5<=step<8 and .06-.025/8*step>.04


def test_prediction_holds_when_no_nonzero_step_keeps_bottom_margin():
    obj={'bbox_norm':[.4,.045,.5,.16]};old={'bbox_norm':[.4,.07,.5,.185]}
    with pytest.raises(ValueError,match='dự đoán'):
        visible_step(obj,1308,1360,(1300,old),MOTION_DEFAULTS,VISIBILITY_DEFAULTS)


def test_raw_preview_no_snapshot_and_motion_config_guard(robot,tmp_path):
    from fastapi.testclient import TestClient
    from calibration_ui import calibration_app
    b,hw,a,_,_,c=setup_follow(robot,tmp_path)
    v=LiveKitModal('https://app','https://modal');v.jpeg=b'unchanged-camera-jpeg';v.frame_at=time.monotonic()
    client=TestClient(calibration_app(b,v,a));page=client.get('/').text
    assert 'id="detectionshot"' not in page
    assert client.get('/api/preview').content==v.jpeg
    assert client.get('/api/detection-snapshot').status_code==404
    token=re.search("const token='([^']+)'",page).group(1)
    body={'motion':c['motion'],'visibility':c['visibility']};n=len(hw.commands)
    assert client.post('/api/motion',json=body).status_code==403
    headers={'X-Cal-Token':token}
    assert client.post('/api/motion',headers=headers,json=body).status_code==200
    assert not any(x.startswith(('G ','C ','P ','S ')) for x in hw.commands[n:])
    b.claim_auto('test');assert client.post('/api/motion',headers=headers,json=body).status_code==409;b.release_auto('test')
