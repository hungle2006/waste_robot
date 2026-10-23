"""Exercise D18 -> camera -> model -> real Brain command order and cancellation."""
import asyncio
import copy
import json
import time
from unittest.mock import AsyncMock

import pytest

from test_control import robot, ready, wait_for
from test_auto import calibrated, Vision, feed_sensors, finish
from auto_controller import AutoController, load_rear_config, AutoFault
from realtime_bridge import RealtimeService
from livekit_modal import parse_detection


def rear_settings(b, path):
    cfg = calibrated(b,path)
    cfg['rear'].update(settle_ms=100,retry_ms=100,min_open_ms=200,
                       clear_confirm_ms=200,max_open_ms=1000,max_cycle_s=30)
    path.write_text(json.dumps(cfg))
    return cfg


class RearVision:
    def __init__(self,b,hw,bin_number=3):
        self.b,self.hw,self.bin = b,hw,bin_number
        self.jpeg = b'new-image'; self.frame_at = time.monotonic()
        self.calls = []
        self.response = None
        self.delay = None
        self.cancelled = False

    async def detect(self,fresh_after=None):
        # Record the arm, lid and bin positions at CAPTURE, not at HTTP completion.
        self.calls.append((fresh_after,copy.deepcopy(self.hw.state['joints'])))
        try:
            if self.delay: await self.delay.wait()
        except asyncio.CancelledError:
            self.cancelled = True
            raise
        self.frame_at = time.monotonic()
        # No bbox/center: rear only needs classification, never image-based driving.
        obj = {'label':'battery','category':'hazardous','bin':self.bin}
        result = self.response or {'found':True,'object':obj}
        return dict(result,_frame_at=self.frame_at,_frame_size=[640,480])


async def clear_after_open(b,hw):
    # The test harness must outlast the configured 30s rear cycle; sensor
    # max_open_ms/clear_confirm_ms remain unchanged and are asserted below.
    async with asyncio.timeout(30):
        while hw.state['joints'].get('p4') != b.config['lid']['open']:
            await asyncio.sleep(.01)
        hw.state['sensors']['D18'] = 1


def queue_rear(b,hw):
    b.enable_rear(True)
    hw.state['sensors']['D18'] = 0
    wait_for(b.rear_requested)


@pytest.mark.parametrize('bin_number,expected',[(1,2350),(2,1000),(3,1450),(4,1900)])
def test_rear_complete_order_and_all_bin_mappings(robot,tmp_path,bin_number,expected):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';rear_settings(b,p)
        v=RearVision(b,hw,bin_number);a=AutoController(b,v,p,AsyncMock())
        queue_rear(b,hw);n=len(hw.commands)
        clear=asyncio.create_task(clear_after_open(b,hw))
        await a.service_rear();await finish(a,timeout=40);await clear
        assert a.state['state']=='rear_idle',a.state
        assert a.state['rear_cycles']==1 and not b.auto_owner and b.auto_rear
        assert b.snapshot()['gift']['eligible'], 'D18 + phân loại + nắp đóng phải mở quyền đổi quà'
        sent=hw.commands[n:]
        assert sent.index('S 2 500') < sent.index(f'P 9 {expected}') < sent.index('P 4 1600')
        assert sent.index('P 4 500',sent.index('P 4 1600')) < sent.index('S 2 2100')
        assert not any(x.startswith(('C ','S 3 ')) for x in sent)
        assert len(v.calls)==2 and v.calls[1][0] > v.calls[0][0]
        for fresh_after,joints in v.calls:
            assert fresh_after>0 and joints['p4']==500
            for j,value in b.config['poses']['REAR'].items(): assert joints[j]==value
        assert not b.rear_pending and not b.rear_handling
        # Level staying low cannot start a duplicate; it must clear after completion.
        hw.state['sensors']['D18']=0
        await asyncio.sleep(.25)
        await a.service_rear()
        assert a.state['rear_cycles']==1
    asyncio.run(run())


@pytest.mark.parametrize('response',[
    {'found':False,'object':None},
    {'found':True,'object':{'label':'battery','bin':9}},
    {'found':True,'object':{'label':'','bin':3}},
])
def test_failed_rear_model_never_rotates_or_opens(robot,tmp_path,response):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';rear_settings(b,p)
        v=RearVision(b,hw);v.response=response;a=AutoController(b,v,p,AsyncMock())
        queue_rear(b,hw);n=len(hw.commands)
        await a.service_rear();await finish(a,timeout=40)
        assert b.stopped and a.state['state']=='auto_error'
        assert 'P 4 1600' not in hw.commands[n:]
        assert not any(x.startswith('P 9 ') for x in hw.commands[n:])
    asyncio.run(run())


def test_rear_stop_cancels_pending_model_without_late_motion(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';rear_settings(b,p)
        v=RearVision(b,hw);v.delay=asyncio.Event();a=AutoController(b,v,p,AsyncMock())
        queue_rear(b,hw);await a.service_rear()
        end=time.monotonic()+5
        while not v.calls and time.monotonic()<end:await asyncio.sleep(.01)
        assert v.calls
        await a.stop(emergency=True);n=len(hw.commands)
        v.delay.set();await asyncio.sleep(.1)
        assert v.cancelled and not b.auto_rear and not b.rear_pending
        assert not any(x.startswith(('P ','S ','C ')) for x in hw.commands[n:])
    asyncio.run(run())


def test_rear_timeout_does_not_command_lid_close_over_detected_object(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';rear_settings(b,p)
        a=AutoController(b,RearVision(b,hw),p,AsyncMock());queue_rear(b,hw);n=len(hw.commands)
        await a.service_rear();await finish(a,timeout=40)
        assert 'D18 chưa hết' in a.state['message'] and b.stopped
        sent=hw.commands[n:];opened=sent.index('P 4 1600')
        assert not any(cmd.startswith('P 4 ') for cmd in sent[opened+1:])
    asyncio.run(run())


def test_active_d18_when_enabled_is_served_without_new_edge(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';rear_settings(b,p)
        hw.state['sensors']['D18']=0
        wait_for(lambda:b.last_hw['sensors']['D18']==0)
        b.command('AUTO REAR ON',b.epoch)
        assert b.rear_requested()
        v=RearVision(b,hw);a=AutoController(b,v,p,AsyncMock())
        clear=asyncio.create_task(clear_after_open(b,hw))
        await a.service_rear();await finish(a,timeout=40);await clear
        assert a.state['rear_cycles']==1
    asyncio.run(run())


def test_manual_arm_finishes_before_queued_rear_takes_ownership(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';rear_settings(b,p)
        a=AutoController(b,RearVision(b,hw),p,AsyncMock());b.enable_rear(True)
        b.command(None,b.epoch,'p1',1600)
        hw.state['sensors']['D18']=0
        b._observe_rear(0)
        await a.service_rear()
        assert not a.running and not b.stopped and b.busy
        # The service normally polls repeatedly. Wait for the post-motion hardware
        # heartbeat before expecting one service call to take ownership.
        async with asyncio.timeout(3):
            while b.busy or not b.snapshot()['fresh'] or b.snapshot()['hardware']['joints'].get('p1')!=1600:
                await asyncio.sleep(.01)
        clear=asyncio.create_task(clear_after_open(b,hw))
        await a.service_rear();assert a.running
        await finish(a,timeout=40);await clear
        assert a.state['rear_cycles']==1
    asyncio.run(run())


def test_pending_rear_interrupts_front_inference_then_reacquires_front_target(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';rear_settings(b,p)
        front=Vision(b,hw);rear=RearVision(b,hw)
        entered=asyncio.Event();cancelled=asyncio.Event();first=True
        a=AutoController(b,None,p,AsyncMock())
        class MixedVision:
            jpeg=b'image';frame_at=time.monotonic()
            async def detect(self,fresh_after=None):
                nonlocal first
                if a.flow=='rear':result=await rear.detect(fresh_after)
                else:
                    if first:
                        first=False;entered.set()
                        try:await asyncio.Event().wait()
                        except asyncio.CancelledError:cancelled.set();raise
                    result=await front.detect(fresh_after)
                self.frame_at=result['_frame_at'];return result
        a.vision=MixedVision()
        sensors=asyncio.create_task(feed_sensors(b,hw));clear=asyncio.create_task(clear_after_open(b,hw))
        await a.start();await asyncio.wait_for(entered.wait(),3)
        hw.state['sensors']['D18']=0
        await finish(a,timeout=40);await clear
        sensors.cancel();await asyncio.gather(sensors,return_exceptions=True)
        assert cancelled.is_set() and a.state['cycles']==1 and a.state['rear_cycles']==1,a.state
        assert a.state['state']=='auto_done'
        assert hw.commands.index('S 2 500') < hw.commands.index('S 3 2300')
    asyncio.run(run())


def test_rear_during_grasp_waits_until_front_release(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';rear_settings(b,p)
        front=Vision(b,hw);rear=RearVision(b,hw);a=AutoController(b,None,p,AsyncMock())
        class MixedVision:
            jpeg=b'image';frame_at=time.monotonic()
            async def detect(self,fresh_after=None):
                result=await (rear.detect(fresh_after) if a.flow=='rear' else front.detect(fresh_after))
                self.frame_at=result['_frame_at'];return result
        a.vision=MixedVision()
        async def trigger_after_grip():
            while hw.state['joints'].get('d3')!=b.config['grip']['close']:await asyncio.sleep(.01)
            hw.state['sensors']['D18']=0
        async def clear_rear_lid():
            while a.state['state']!='rear_wait_drop':await asyncio.sleep(.01)
            hw.state['sensors']['D18']=1
        sensors=asyncio.create_task(feed_sensors(b,hw));trigger=asyncio.create_task(trigger_after_grip())
        clear=asyncio.create_task(clear_rear_lid());n=len(hw.commands)
        await a.start();await finish(a,timeout=40);await trigger;await clear
        sensors.cancel();await asyncio.gather(sensors,return_exceptions=True)
        sent=hw.commands[n:];grip=sent.index('S 3 2300');release=sent.index('S 3 600',grip)
        assert release < sent.index('S 2 500')
        assert sent.index('P 4 1600') < release
        assert a.state['cycles']==1 and a.state['rear_cycles']==1,a.state
    asyncio.run(run())


def test_service_loop_processes_d18_without_manual_detect_command(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';rear_settings(b,p)
        channel=type('Channel',(),{'send_broadcast':AsyncMock()})()
        v=RearVision(b,hw);svc=RealtimeService(b,'robot_01',channel,v);svc.online=True
        svc.auto=AutoController(b,v,p,svc.auto_report)
        loop=asyncio.create_task(svc.run());queue_rear(b,hw)
        clear=asyncio.create_task(clear_after_open(b,hw))
        end=time.monotonic()+8
        while svc.auto.state['rear_cycles']!=1 and time.monotonic()<end:await asyncio.sleep(.02)
        assert svc.auto.state['rear_cycles']==1
        await clear
        loop.cancel();await asyncio.gather(loop,return_exceptions=True)
        await svc.auto.stop()
        statuses=[call.args[1]['state'] for call in channel.send_broadcast.call_args_list if call.args[0]=='status']
        assert 'rear_capture' in statuses and 'rear_open_lid' in statuses and 'rear_done' in statuses
    asyncio.run(run())


def test_rear_does_not_depend_on_incomplete_front_calibration(robot,tmp_path):
    b,hw=robot;ready(b);p=tmp_path/'auto.json';cfg=rear_settings(b,p)
    cfg['calibration']['stages']=[];cfg['calibration']['positive_error_x_move']=None
    p.write_text(json.dumps(cfg))
    assert load_rear_config(p)['rear']['sensor']=='D18'
    assert parse_detection({'found':True,'object':{'label':'battery','bin':3}})['found']


def test_bad_rear_timing_rejected_before_motion(robot,tmp_path):
    b,hw=robot;ready(b);p=tmp_path/'auto.json';cfg=rear_settings(b,p)
    cfg['rear']['max_open_ms']=cfg['rear']['min_open_ms']
    p.write_text(json.dumps(cfg))
    with pytest.raises(AutoFault):load_rear_config(p)


def test_object_removed_during_inference_does_not_open_lid(robot,tmp_path):
    async def run():
        b,hw=robot;ready(b);b.start_monitor();p=tmp_path/'auto.json';rear_settings(b,p)
        class RemovedVision(RearVision):
            async def detect(self,fresh_after=None):
                result=await super().detect(fresh_after)
                if len(self.calls)==2:
                    hw.state['sensors']['D18']=1;b.last_hw['sensors']['D18']=1
                return result
        a=AutoController(b,RemovedVision(b,hw),p,AsyncMock());queue_rear(b,hw);n=len(hw.commands)
        await a.service_rear();await finish(a,timeout=40)
        assert 'rời D18' in a.state['message'] and b.stopped
        assert 'P 4 1600' not in hw.commands[n:]
    asyncio.run(run())


def test_local_d18_switch_arms_only_with_token_camera_and_valid_config(robot,tmp_path):
    import re
    from fastapi.testclient import TestClient
    from calibration_ui import calibration_app
    b,hw=robot;ready(b);p=tmp_path/'auto.json';rear_settings(b,p)
    v=RearVision(b,hw);a=AutoController(b,v,p,AsyncMock());client=TestClient(calibration_app(b,v,a))
    token=re.search("const token='([^']+)'",client.get('/').text).group(1)
    headers={'X-Cal-Token':token};n=len(hw.commands)
    assert client.post('/api/rear',json={'enabled':True}).status_code==403
    assert client.post('/api/rear',json={'enabled':True},headers=headers).status_code==200
    assert b.auto_rear and len(hw.commands)==n
    assert client.post('/api/rear',json={'enabled':False},headers=headers).status_code==200
    assert not b.auto_rear
    v.jpeg=None
    assert client.post('/api/rear',json={'enabled':True},headers=headers).status_code==409
    assert not b.auto_rear and len(hw.commands)==n
