import json
import time
from pathlib import Path

import pytest

from test_control import robot, ready, wait_for
from brain import Brain
from transport import SimTransport
from mecanum import DRIVE_AXES, mix_mecanum

# Expected raw electrical signs supplied by the owner, for X rollers.
EXPECTED = {
    'FORWARD': [-20,-20,-20,20], 'BACK': [20,20,20,-20],
    'LEFT': [-20,-20,20,-20], 'RIGHT': [20,20,-20,20],
    'FORWARD_LEFT': [-20,-20,0,0], 'FORWARD_RIGHT': [0,0,-20,20],
    'BACK_LEFT': [0,0,20,-20], 'BACK_RIGHT': [20,20,0,0],
    'TURN_LEFT': [-20,20,-20,-20], 'TURN_RIGHT': [20,-20,20,20],
    'ARC_FORWARD_LEFT': [-20,-7,-20,7], 'ARC_FORWARD_RIGHT': [-7,-20,-7,20],
    'ARC_BACK_LEFT': [20,7,20,-7], 'ARC_BACK_RIGHT': [7,20,7,-20],
}

@pytest.mark.parametrize('mode,expected', EXPECTED.items())
def test_all_directions_reach_hardware_in_m1_m2_m3_m4_order(robot, mode, expected):
    b,hw=robot; epoch=ready(b)
    result=b.command('DRIVE '+mode,epoch)
    assert hw.state['motors']==expected
    assert result['hardware']['motors']==expected
    assert hw.commands[-2].startswith('C ')
    assert hw.commands[-2].endswith('20 500')

@pytest.mark.parametrize('speed',[0,256,-20,True,20.5])
def test_invalid_pwm_never_writes_motor_command(robot,speed):
    b,hw=robot;epoch=ready(b);count=len(hw.commands)
    with pytest.raises(ValueError):b.command('DRIVE FORWARD',epoch,speed=speed)
    assert len(hw.commands)==count


def test_hold_refresh_and_release_invalidate_delayed_drive(robot):
    b,hw=robot;epoch=ready(b)
    b.command('DRIVE RIGHT',epoch,speed=40)
    first=b.drive_until
    time.sleep(.03)
    b.command('DRIVE RIGHT',epoch,speed=40)
    assert b.drive_until>first and hw.commands.count('C 0 100 0 40 500')==2
    assert hw.state['motors']==[40,40,-40,40]
    result=b.command('XM',epoch)
    assert result['epoch']>epoch and hw.state['motors']==[0,0,0,0]
    assert not b.stopped
    count=len(hw.commands)
    with pytest.raises(ValueError):b.command('DRIVE RIGHT',epoch,speed=40)
    assert len(hw.commands)==count
    b.command('DRIVE BACK',b.epoch)
    assert hw.state['motors']==[20,20,20,-20]


def test_direction_change_brakes_before_reversing(robot):
    b,hw=robot;epoch=ready(b)
    b.command('DRIVE FORWARD',epoch)
    count=len(hw.commands)
    b.command('DRIVE BACK',epoch)
    assert hw.commands[count:count+2]==['XM','C -100 0 0 20 500']


def test_missing_browser_heartbeat_expires_even_while_status_polling(robot):
    b,hw=robot;epoch=ready(b);b.start_monitor()
    b.command('DRIVE FORWARD',epoch)
    wait_for(lambda:b.drive_until==0, timeout=1.2)
    assert hw.state['motors']==[0,0,0,0] and b.epoch>epoch
    with pytest.raises(ValueError):b.command('DRIVE FORWARD',epoch)


def test_stop_while_holding_cannot_restart(robot):
    b,hw=robot;epoch=ready(b)
    b.command('DRIVE FORWARD_LEFT',epoch)
    b.stop()
    assert hw.state['motors']==[0,0,0,0]
    with pytest.raises(ValueError):b.command('DRIVE FORWARD_LEFT',epoch)


def test_arm_and_auto_rear_interlocks(robot):
    b,hw=robot;epoch=ready(b)
    b.command('AUTO REAR ON',epoch)
    with pytest.raises(ValueError):b.command('DRIVE FORWARD',epoch)
    b.command('AUTO REAR OFF',epoch)
    b.command('DRIVE FORWARD',epoch)
    with pytest.raises(ValueError):b.command('GRIP OPEN',epoch)
    b.command('XM',epoch)
    b.command('GRIP OPEN',b.epoch)
    with pytest.raises(ValueError):b.command('DRIVE FORWARD',b.epoch)


def test_old_firmware_is_rejected_before_servo_initialization(tmp_path):
    hw=SimTransport();del hw.state['drive_protocol']
    path=tmp_path/'config.json';path.write_bytes(Path(__file__).resolve().parents[1].joinpath('python/robot_config.json').read_bytes())
    b=Brain(hw,path,True)
    with pytest.raises(ValueError,match='firmware'):b.connect()
    assert not b.connected and b.stopped
    assert not any(x.startswith(('C ','P ','S ')) for x in hw.commands)


def test_api_drive_speed_and_stale_release(tmp_path,monkeypatch):
    import server
    from fastapi.testclient import TestClient
    path=tmp_path/'config.json';path.write_bytes(Path(__file__).resolve().parents[1].joinpath('python/robot_config.json').read_bytes())
    b=Brain(SimTransport(),path,True)
    token='mecanum-api-test-key-1234567890'
    monkeypatch.setattr(server,'brain',b);monkeypatch.setattr(server,'TOKEN',token)
    headers={'Authorization':'Bearer '+token}
    with TestClient(server.app) as client:
        epoch=client.post('/connect',headers=headers,json={}).json()['epoch']
        wait_for(lambda:not b.busy)
        assert client.post('/command',headers=headers,json={'command':'DRIVE FORWARD','speed':True,'epoch':epoch}).status_code==422
        result=client.post('/command',headers=headers,json={'command':'DRIVE BACK_RIGHT','speed':60,'epoch':epoch})
        assert result.status_code==200
        assert result.json()['hardware']['motors']==[60,60,0,0]
        stop=client.post('/command',headers=headers,json={'command':'XM','epoch':epoch})
        assert stop.json()['hardware']['motors']==[0,0,0,0]
        assert client.post('/command',headers=headers,json={'command':'DRIVE BACK_RIGHT','speed':60,'epoch':epoch}).status_code==409
