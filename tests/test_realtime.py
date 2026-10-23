import asyncio
import copy
import time
from unittest.mock import AsyncMock, patch

import pytest

from test_control import robot, ready, wait_for
from realtime_bridge import MechanicsDispatcher, RealtimeService
from livekit_modal import LiveKitModal, parse_detection


def packet(d, seq=1, action='command', body=None):
    return {'robot_id': d.robot_id,'type':'mechanics','action':action,'client_id':'test-client',
            'request_id':f'request-{seq}','session_id':d.session_id,'seq':seq,
            'issued_at':int(time.time()*1000),'ttl_ms':500 if body and str(body.get('command','')).startswith('DRIVE ') else 8000,
            'payload':body or {}}


def test_broker_gateway_connect_status_and_joint_pose_commands(robot):
    b,hw=robot;epoch=ready(b);d=MechanicsDispatcher(b,'robot_01')
    assert d.handle(packet(d,1,'status'))['ok']
    assert d.handle(packet(d,2,'connect'))['ok']
    p=packet(d,3,body={'joint':'p1','pulse_us':500,'epoch':epoch})
    assert d.handle(p)['ok'];wait_for(lambda:not b.busy)
    assert hw.state['joints']['p1']==500
    assert d.handle(packet(d,4,'poses',{'name':'SEARCH','values':{'p1':900},'epoch':epoch}))['ok']
    assert b.config['poses']['SEARCH']['p1']==900


def test_broker_gateway_drive_xm_out_of_order_and_deduplication(robot):
    b,hw=robot;epoch=ready(b);d=MechanicsDispatcher(b,'robot_01')
    p=packet(d,1,body={'command':'DRIVE RIGHT','speed':20,'epoch':epoch})
    assert d.handle(p)['ok']
    n=len(hw.commands)
    assert d.handle(p)['duplicate'] and len(hw.commands)==n
    assert d.handle(packet(d,3,body={'command':'XM','epoch':epoch}))['ok']
    assert hw.state['motors']==[0,0,0,0]
    with pytest.raises(ValueError,match='cũ'):d.handle(packet(d,2,body={'command':'DRIVE RIGHT','epoch':epoch}))
    assert hw.state['motors']==[0,0,0,0]

@pytest.mark.parametrize('change',[{'robot_id':'wrong'},{'session_id':'old'},{'seq':True},{'ttl_ms':501}, {'issued_at':0}])
def test_broker_gateway_rejects_invalid_or_expired_drive(robot,change):
    b,hw=robot;epoch=ready(b);d=MechanicsDispatcher(b,'robot_01')
    p=packet(d,1,body={'command':'DRIVE FORWARD','epoch':epoch});p.update(change)
    n=len(hw.commands)
    with pytest.raises(ValueError):d.handle(p)
    assert len(hw.commands)==n


def test_broker_gateway_failed_command_is_not_replayed(robot):
    b,hw=robot;epoch=ready(b);d=MechanicsDispatcher(b,'robot_01')
    p=packet(d,1,body={'joint':'p1','pulse_us':499,'epoch':epoch})
    assert not d.handle(p)['ok']
    assert d.handle(p)['duplicate']


def test_broker_gateway_stop_works_after_later_sequence(robot):
    b,hw=robot;epoch=ready(b);d=MechanicsDispatcher(b,'robot_01')
    d.handle(packet(d,10,body={'command':'DRIVE FORWARD','epoch':epoch}))
    assert d.handle(packet(d,9,body={'command':'STOP','epoch':epoch}))['ok']
    assert b.stopped and hw.state['motors']==[0,0,0,0]


def test_service_broadcasts_real_robot_ack_and_current_state(robot):
    b,hw=robot;epoch=ready(b);channel=type('Channel',(),{'send_broadcast':AsyncMock()})()
    service=RealtimeService(b,'robot_01',channel);service.online=True
    p=packet(service.dispatcher,1,body={'command':'DRIVE FORWARD_LEFT','epoch':epoch})
    asyncio.run(service.command(p))
    event,data=channel.send_broadcast.call_args.args
    assert event=='mechanical_ack' and data['ok']
    assert data['request_id']==p['request_id'] and data['client_id']==p['client_id']
    assert data['state']['hardware']['motors']==[-20,-20,0,0]
    asyncio.run(service.disconnected())
    assert b.stopped and hw.state['motors']==[0,0,0,0]


def test_service_without_auto_reports_not_ready(robot):
    b,hw=robot;ready(b);channel=type('Channel',(),{'send_broadcast':AsyncMock()})()
    service=RealtimeService(b,'robot_01',channel)
    service.online=True
    asyncio.run(service.mode({'robot_id':'robot_01','type':'mode','action':'auto_start'}))
    assert channel.send_broadcast.call_args.args[1]['state']=='auto_not_ready'


@pytest.mark.parametrize('obj',[{}, {'found':True,'object':{'bin':5}}, {'found':'yes'}, {'found':True,'object':None}])
def test_modal_invalid_result_cannot_be_used_as_target(obj):
    with pytest.raises(ValueError):parse_detection(obj)


def test_modal_receives_current_frame_and_existing_multipart_contract():
    async def run():
        import httpx
        obj={'found':True,'object':{'label':'battery','category':'hazardous','bin':3,'center':[50,80]}}
        def handle(request):
            assert request.url.path=='/detect'
            content=request.read()
            assert b'name="image"' in content and b'fresh-test-frame' in content
            return httpx.Response(200,json=obj)
        real=httpx.AsyncClient
        vision=LiveKitModal('https://app.test','https://modal.test')
        vision.jpeg=b'fresh-test-frame';vision.frame_at=time.monotonic()
        with patch('httpx.AsyncClient',side_effect=lambda **kw:real(transport=httpx.MockTransport(handle),**kw)):
            assert (await vision.detect())['object']['bin']==3
        vision.frame_at=0
        with pytest.raises(ValueError,match='frame'):await vision.detect()
    asyncio.run(run())
