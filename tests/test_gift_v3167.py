import time
import threading
import pytest
from test_control import robot, ready, wait_for
from gift_control import load_gift_params
from realtime_bridge import MechanicsDispatcher
from test_realtime import packet


def params(b,tmp_path,opened=1500,closed=500,hold=100,settle=20):
    p=tmp_path/'gift_params.py'
    p.write_text(f'GIFT_OPEN_US = {opened!r}\nGIFT_CLOSE_US = {closed!r}\nGIFT_OPEN_HOLD_MS = {hold!r}\nGIFT_CLOSE_SETTLE_MS = {settle!r}\n')
    b.gift.params_path=p
    return p


def writes(hw,start=0):return [s for s in hw.commands[start:] if s.startswith('S 4 ')]


@pytest.mark.parametrize('opened,closed',[(500,2500),(2500,500),(600,2500),(1250,750)])
def test_cycle_sends_configured_open_then_close_only(robot,tmp_path,opened,closed):
    b,hw=robot;ready(b);params(b,tmp_path,opened,closed);n=len(hw.commands)
    b.gift.grant_after_rear('RECYCLE')
    result=b.command('GIFT DISPENSE',b.epoch)
    assert result['gift']['busy']
    wait_for(lambda:not b.gift.state['busy'])
    assert writes(hw,n)==[f'S 4 {opened}',f'S 4 {closed}']
    assert b.snapshot()['gift']['phase']=='done' and b.gift.state['cycles']==1
    assert not any(s.startswith(('P ','S 2 ','S 3 ','G ')) for s in hw.commands[n:])
    assert b.command_trace[0]['source']=='gift'


@pytest.mark.parametrize('opened,closed,hold,settle',[
    (499,1500,100,20),(2501,500,100,20),(1500,499,100,20),
    (True,500,100,20),(1500.5,500,100,20),(500,500,100,20),
    (1500,500,0,20),(1500,500,10001,20),(1500,500,100,0)])
def test_invalid_parameters_never_move_hardware(robot,tmp_path,opened,closed,hold,settle):
    b,hw=robot;ready(b);params(b,tmp_path,opened,closed,hold,settle);n=len(hw.commands)
    with pytest.raises(ValueError):b.command('GIFT DISPENSE',b.epoch)
    assert not writes(hw,n) and not b.gift.state['busy']


def test_stop_mid_open_cancels_delayed_close_and_resume_does_not_replay(robot,tmp_path):
    b,hw=robot;ready(b);params(b,tmp_path,hold=1000);n=len(hw.commands)
    b.gift.grant_after_rear('RECYCLE')
    b.command('GIFT DISPENSE',b.epoch);wait_for(lambda:len(writes(hw,n))==1)
    b.stop();b.gift.worker.join(1);after=len(hw.commands);b.resume(b.epoch)
    time.sleep(.12)
    assert writes(hw,n)==['S 4 1500','S 4 500']
    assert writes(hw,after)==['S 4 500']  # Explicit RESUME homes closed, never dispenses.
    assert b.gift.state['phase']=='idle' and hw.state['joints'].get('d4')==500


def test_usb_fault_cancels_gift_and_reconnect_does_not_replay(robot,tmp_path):
    b,hw=robot;ready(b);params(b,tmp_path,hold=1000);n=len(hw.commands)
    b.gift.grant_after_rear('RECYCLE')
    b.command('GIFT DISPENSE',b.epoch);wait_for(lambda:len(writes(hw,n))==1)
    with b.lock:b._fault('INJECTED_USB_LOSS')
    b.gift.worker.join(1);b.connect();time.sleep(.1)
    assert writes(hw,n)==['S 4 1500','S 4 500'] and not b.gift.state['busy']


def test_new_gift_is_independent_of_auto_owner_busy_and_wheel_epoch(robot,tmp_path):
    b,hw=robot;ready(b);params(b,tmp_path,hold=200)
    b.gift.grant_after_rear('RECYCLE')
    b.claim_auto('existing-auto');b.busy=True
    try:
        n=len(hw.commands);b.command('GIFT DISPENSE',b.epoch)
        wait_for(lambda:len(writes(hw,n))==1)
        b.command('XM',b.epoch,_owner='existing-auto')
        wait_for(lambda:not b.gift.state['busy'])
        assert writes(hw,n)==['S 4 1500','S 4 500']
        assert b.auto_owner=='existing-auto' and b.busy
    finally:b.busy=False;b.release_auto('existing-auto')


def test_repeated_requests_are_deduplicated_and_concurrent_jobs_rejected(robot,tmp_path):
    b,hw=robot;ready(b);params(b,tmp_path,hold=200)
    b.gift.grant_after_rear('RECYCLE')
    dispatcher=MechanicsDispatcher(b,'robot_01');n=len(hw.commands)
    request=packet(dispatcher,body={'command':'GIFT DISPENSE','epoch':b.epoch})
    assert dispatcher.handle(request)['ok']
    assert dispatcher.handle(request)['duplicate']
    with pytest.raises(ValueError,match='Đang phát quà'):b.command('GIFT DISPENSE',b.epoch)
    wait_for(lambda:not b.gift.state['busy'])
    assert dispatcher.handle(request)['duplicate']
    assert writes(hw,n)==['S 4 1500','S 4 500']


def test_saved_changes_apply_next_cycle_not_current_close(robot,tmp_path):
    b,hw=robot;ready(b);p=params(b,tmp_path,hold=200);n=len(hw.commands)
    b.gift.grant_after_rear('RECYCLE')
    b.command('GIFT DISPENSE',b.epoch);wait_for(lambda:len(writes(hw,n))==1)
    params(b,tmp_path,opened=1600,closed=700,hold=20)
    wait_for(lambda:not b.gift.state['busy'])
    b.gift.grant_after_rear('OTHER')  # A second confirmed rear deposit.
    b.command('GIFT DISPENSE',b.epoch);wait_for(lambda:not b.gift.state['busy'])
    assert writes(hw,n)==['S 4 1500','S 4 500','S 4 1600','S 4 700']


def test_long_open_keeps_status_heartbeat_running(robot,tmp_path):
    b,hw=robot;ready(b);params(b,tmp_path,hold=2200);n=len(hw.commands)
    b.gift.grant_after_rear('RECYCLE')
    b.command('GIFT DISPENSE',b.epoch);wait_for(lambda:not b.gift.state['busy'],timeout=4)
    assert b.gift.state['phase']=='done'
    assert hw.commands[n:].count('STATUS')>=8


def test_old_firmware_stale_requests_and_unknown_joint_are_rejected(robot,tmp_path):
    b,hw=robot;ready(b);params(b,tmp_path);n=len(hw.commands)
    with pytest.raises(ValueError):b.command('GIFT DISPENSE',b.epoch-1)
    with pytest.raises(ValueError):b.command(None,b.epoch,joint='d4',pulse_us=1500)
    hw.state.pop('gift_protocol');b.last_hw=hw.request('STATUS')
    assert not b.snapshot()['gift']['available']
    with pytest.raises(ValueError,match='V3.16.11'):b.command('GIFT DISPENSE',b.epoch)
    assert not writes(hw,n)


@pytest.mark.parametrize('error_type',[ValueError,RuntimeError])
def test_failed_close_stops_without_retrying_or_claiming_success(robot,tmp_path,error_type):
    b,hw=robot;ready(b);params(b,tmp_path);request=hw.request;n=len(hw.commands)
    b.gift.grant_after_rear('RECYCLE')
    def fail(command):
        if command=='S 4 500':raise error_type('INJECTED_CLOSE_ERROR')
        return request(command)
    hw.request=fail
    b.command('GIFT DISPENSE',b.epoch);wait_for(lambda:not b.gift.state['busy'])
    assert b.stopped and b.gift.state['phase']=='error'
    assert b.gift.state['cycles']==0 and writes(hw,n)==['S 4 1500']


def test_bad_python_parameter_file_has_readable_error_before_writes(robot,tmp_path):
    b,hw=robot;ready(b);p=params(b,tmp_path);p.write_text('GIFT_OPEN_US = (')
    n=len(hw.commands)
    with pytest.raises(ValueError,match='Không đọc được gift_params.py'):b.command('GIFT DISPENSE',b.epoch)
    assert not writes(hw,n)


def test_hardware_reset_during_open_interrupts_without_close_replay(robot,tmp_path):
    b,hw=robot;ready(b);params(b,tmp_path,hold=1000);n=len(hw.commands)
    b.gift.grant_after_rear('RECYCLE')
    b.command('GIFT DISPENSE',b.epoch);wait_for(lambda:len(writes(hw,n))==1)
    with b.lock:hw.request('STOP')  # Board resets to STOP; Python learns at STATUS.
    b.gift.worker.join(2)
    assert not b.gift.worker.is_alive() and b.stopped
    assert b.gift.state['phase']=='interrupted' and writes(hw,n)==['S 4 1500']


def test_local_gift_button_command_uses_existing_auth_and_ack(robot,tmp_path):
    import re
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from operator_ui import mount_operator
    from test_box_tof_flow import prepared
    b,hw,a,_=prepared(robot,tmp_path);params(b,tmp_path,hold=20)
    b.gift.grant_after_rear('RECYCLE')
    app=FastAPI();mount_operator(app,b,a)
    with TestClient(app) as client:
        html=client.get('/control').text
        token=re.search(r"const token='([^']+)'",html).group(1)
        n=len(hw.commands);body={'action':'command','command':'GIFT DISPENSE','epoch':b.epoch}
        assert client.post('/operator/action',json=body).status_code==403
        assert not writes(hw,n)
        response=client.post('/operator/action',headers={'x-operator-token':token},json=body)
        assert response.status_code==200 and response.json()['gift']['busy']
        wait_for(lambda:not b.gift.state['busy'])
        assert writes(hw,n)==['S 4 1500','S 4 500']


def test_full_auto_cycle_and_independent_gift_both_complete(tmp_path):
    import asyncio
    from test_box_tof_flow import full_demo
    from test_auto import finish
    async def run():
        b,hw,a,_=full_demo(tmp_path);params(b,tmp_path,hold=300)
        b.gift.grant_after_rear('RECYCLE')
        try:
            await a.start()
            for _ in range(200):
                if any(c.startswith('G ') for c in hw.commands):break
                await asyncio.sleep(.02)
            assert a.running
            b.command('GIFT DISPENSE',b.epoch)
            await finish(a,timeout=45)
            assert a.state['state']=='auto_done' and hw.released,a.state
            assert b.gift.state['phase']=='done'
            assert writes(hw)==['S 4 2500','S 4 1500','S 4 500']
        finally:
            if a.running:await a.stop()
            b.close()
    asyncio.run(run())


def test_unavailable_or_stopped_gift_status_explains_why_button_is_disabled(robot):
    b,hw=robot;ready(b);b.stop()
    assert 'STOP' in b.snapshot()['gift']['message']
    b.resume(b.epoch);hw.state.pop('gift_protocol');b.last_hw=hw.request('STATUS')
    assert 'V3.16.11' in b.snapshot()['gift']['message']
