import copy
import json
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"python"))
from brain import Brain, validate_config
from transport import SimTransport


def wait_for(fn, timeout=3):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if fn(): return
        time.sleep(.02)
    assert fn(), "Operation timed out"


@pytest.fixture
def robot(tmp_path):
    original=Path(__file__).resolve().parents[1]/"python"/"robot_config.json"
    c=json.loads(original.read_text())
    c.update(step_ms=20,step_us=100,settle_ms=0)
    path=tmp_path/"config.json"
    path.write_text(json.dumps(c))
    hw=SimTransport()
    b=Brain(hw,path,True)
    b.connect()
    yield b,hw
    b.close()


def ready(b):
    b.resume(b.epoch)
    wait_for(lambda:not b.busy)
    return b.epoch


def test_connect_initializes_front_search_and_all_servos(robot):
    b,hw=robot
    wait_for(lambda:not b.busy)
    expected=copy.deepcopy(b.config['poses']['SETUP'])
    expected.update(b.config['poses']['SEARCH'])
    assert not b.stopped
    assert hw.state['joints']==dict(expected,d4=2500)
    assert hw.state['motors']==[0,0,0,0]


def test_reconnect_to_stopped_session_does_not_override_stop(robot):
    b,hw=robot;ready(b);b.stop()
    count=len(hw.commands)
    b.connect();time.sleep(.06)
    assert b.stopped and hw.state['joints']=={}
    assert len(hw.commands)==count


def test_resume_uses_saved_search_preset_and_initializes_bin_lid(robot):
    b,hw=robot;epoch=ready(b)
    b.save_pose('SEARCH',{'p1':1200,'p10':1000,'p11':1300},epoch)
    b.stop();ready(b)
    assert hw.state['joints']['p1']==1200
    assert hw.state['joints']['p10']==1000
    assert hw.state['joints']['p11']==1300
    assert hw.state['joints']['p9']==1900
    assert hw.state['joints']['p4']==500


@pytest.mark.parametrize("pulse",[499,3501,True,600.1])
def test_invalid_pulses_never_reach_hardware(robot,pulse):
    b,hw=robot; epoch=ready(b); n=len(hw.commands)
    with pytest.raises(ValueError): b.command(None,epoch,"p1",pulse)
    assert len(hw.commands)==n


def test_pulse_500_and_hidden_joint_blocked(robot):
    b,hw=robot;epoch=ready(b)
    b.command(None,epoch,"p1",500)
    wait_for(lambda:not b.busy)
    assert hw.state["joints"]["p1"]==500
    b.command(None,epoch,'d2',500)
    wait_for(lambda:not b.busy)
    assert hw.state['joints']['d2']==500
    for joint in ('d3','p9','p4'):
        with pytest.raises(ValueError):b.command(None,epoch,joint,500)


def test_stop_cancels_ramp_and_rejects_delayed_commands(robot):
    b,hw=robot;epoch=ready(b)
    b.command(None,epoch,"p1",500);wait_for(lambda:not b.busy)
    b.command(None,epoch,"p1",2500)
    wait_for(lambda:any(x=="P 1 600" for x in hw.commands))
    b.stop();wait_for(lambda:not b.busy)
    after=hw.commands[hw.commands.index("STOP",1)+1:]
    assert not any(x.startswith(("P ","S ")) for x in after)
    ready(b)
    with pytest.raises(ValueError):b.command("GRIP OPEN",epoch)


def test_place_preserves_closed_gripper(robot):
    b,hw=robot;epoch=ready(b)
    b.command("GRIP CLOSE",epoch);wait_for(lambda:not b.busy)
    b.command("POSE PLACE",epoch);wait_for(lambda:not b.busy)
    assert hw.state["joints"]["d3"]==2300


def test_save_pose_persists_and_preserves_hidden_values(robot):
    b,hw=robot;epoch=ready(b)
    count=len(hw.commands)
    b.save_pose("REAR",{"p1":900,"p10":1700},epoch)
    c=json.loads(b.path.read_text())
    assert c["poses"]["REAR"]["p1"]==900
    assert c["poses"]["REAR"]["d2"]==500
    assert len(hw.commands)==count
    with pytest.raises(ValueError):b.save_pose("REAR",{"d2":600},epoch)


def test_bad_config_rejected_before_motion(robot):
    b,_=robot;c=copy.deepcopy(b.config)
    c["poses"]["REAR"]["d2"]=200
    with pytest.raises(ValueError):validate_config(c)


def test_auto_rear_triggers_only_once_per_edge(robot):
    b,hw=robot;epoch=ready(b);b.start_monitor()
    b.command("AUTO REAR ON",epoch)
    count=hw.commands.count("S 2 500")
    hw.state["sensors"]["D18"]=0
    wait_for(b.rear_requested)
    token='test-rear-owner';b.claim_auto(token)
    assert b.take_rear_request(token)
    time.sleep(.45)
    assert not b.rear_requested()  # Monitor only queues; no incomplete REAR pose.
    assert hw.commands.count("S 2 500")==count
    b.finish_rear_request(token)
    b.release_auto(token)
    hw.state["sensors"]["D18"]=1
    wait_for(lambda:b.rear_armed)
    hw.state["sensors"]["D18"]=0
    wait_for(b.rear_requested)
    assert hw.commands.count("S 2 500")==count


def test_disconnect_invalidates_sensor_data_and_auto(robot):
    b,hw=robot;epoch=ready(b);b.start_monitor()
    b.command("AUTO REAR ON",epoch);hw.close()
    wait_for(lambda:not b.connected)
    s=b.snapshot()
    assert s["stopped"] and not s["auto_rear"] and s["hardware"]=={}


def test_api_contract_auth_and_stop(tmp_path,monkeypatch):
    import server
    from fastapi.testclient import TestClient
    src=Path(__file__).resolve().parents[1]/"python"/"robot_config.json"
    path=tmp_path/"api_config.json";path.write_bytes(src.read_bytes())
    b=Brain(SimTransport(),path,True)
    token="test-control-key-1234567890123456"
    monkeypatch.setattr(server,"TOKEN",token)
    monkeypatch.setattr(server,"brain",b)
    h={"Authorization":"Bearer "+token}
    with TestClient(server.app) as client:
        assert client.get('/status').status_code==401
        assert client.get('/ui/robot-control.html').status_code==200
        r=client.post('/connect',headers=h,json={});assert r.status_code==200
        epoch=r.json()['epoch']
        assert client.post('/command',headers=h,json={'command':'GRIP OPEN','epoch':epoch}).status_code==409
        r=client.post('/command',headers=h,json={'command':'RESUME','epoch':epoch});epoch=r.json()['epoch']
        wait_for(lambda:not b.busy)
        assert client.post('/command',headers=h,json={'joint':'p1','pulse_us':500.5,'epoch':epoch}).status_code==422
        assert client.post('/command',headers=h,json={'joint':'p1','pulse_us':499,'epoch':epoch}).status_code==409
        assert client.post('/command',headers=h,json={'joint':'d2','pulse_us':2300,'epoch':epoch}).status_code==200
        wait_for(lambda:not b.busy)
        assert client.post('/poses',headers=h,json={'name':'REAR','values':{'p1':900},'epoch':epoch}).status_code==200
        assert client.post('/command',headers=h,json={'command':'GRIP OPEN','epoch':epoch}).status_code==200
        wait_for(lambda:not b.busy)
        r=client.get('/status',headers=h)
        assert r.json()['hardware']['joints']['d3']==600
        assert client.post('/command',headers=h,json={'command':'STOP','epoch':epoch}).json()['stopped']


def test_serial_reply_correlation_ignores_stale_reply():
    from transport import SerialTransport
    class Port:
        def __init__(self):self.lines=iter([b'12 OK\n',b'EVENT WATCHDOG\n',b'1 OK\n'])
        def write(self,data):assert data==b'1 P 1 1300\n'
        def readline(self):return next(self.lines)
    s=SerialTransport('test');s.port=Port()
    assert s.request('P 1 1300')=={'ok':True}


def test_s2_3500_and_other_joint_limits(robot):
    b,hw=robot;epoch=ready(b)
    b.command(None,epoch,"p1",3500)
    wait_for(lambda:not b.busy)
    assert hw.state["joints"]["p1"]==3500
    b.save_pose("SEARCH",{"p1":3500},epoch)
    assert validate_config(json.loads(b.path.read_text()))["poses"]["SEARCH"]["p1"]==3500
    for joint in ("p10","p11"):
        with pytest.raises(ValueError): b.command(None,epoch,joint,2501)
    c=copy.deepcopy(b.config);c["limits"]["d2"]=[500,3500]
    with pytest.raises(ValueError): validate_config(c)
