import asyncio
import re
import time
from pathlib import Path
from types import SimpleNamespace
import pytest
from test_control import robot, ready, wait_for
from test_box_tof_flow import prepared
from local_dashboard import dashboard_settings, serve_dashboard


def test_desktop_enabled_even_when_old_calibration_page_disabled():
    assert dashboard_settings({'AUTO_CALIBRATION_UI':False},{}) == {'enabled':True,'open_browser':True,'port':8766}
    assert dashboard_settings({}, {'LOCAL_OPERATOR_UI':'0','LOCAL_OPERATOR_OPEN_BROWSER':'0','AUTO_CALIBRATION_PORT':'8877'}) == {'enabled':False,'open_browser':False,'port':8877}


@pytest.mark.parametrize('port',['80','70000','abc'])
def test_bad_local_port_rejected(port):
    with pytest.raises(ValueError):dashboard_settings({}, {'AUTO_CALIBRATION_PORT':port})


@pytest.mark.parametrize('open_browser',[True,False])
def test_browser_opens_only_once_after_loopback_server_ready(monkeypatch,open_browser):
    import uvicorn
    opened=[];servers=[]
    class Server:
        def __init__(self,config):
            self.started=False;self.should_exit=False;self.config=config;servers.append(self)
        async def serve(self):
            await asyncio.sleep(.02);self.started=True
            while not self.should_exit:await asyncio.sleep(.01)
    monkeypatch.setattr(uvicorn,'Server',Server)
    def open_url(url,new):
        assert servers[0].started and servers[0].config.host=='127.0.0.1'
        opened.append((url,new));return True
    monkeypatch.setattr('local_dashboard.webbrowser.open',open_url)
    async def run():
        task=asyncio.create_task(serve_dashboard(object(),port=8877,open_browser=open_browser))
        try:
            await asyncio.sleep(.15)
            assert opened==([('http://127.0.0.1:8877/control',2)] if open_browser else [])
        finally:
            task.cancel()
            with pytest.raises(asyncio.CancelledError):await task
        assert servers[0].should_exit
    asyncio.run(run())


def test_busy_port_never_opens_other_robot_session(monkeypatch):
    import uvicorn
    class Server:
        started=False
        def __init__(self,config):pass
        async def serve(self):raise SystemExit(1)
    monkeypatch.setattr(uvicorn,'Server',Server)
    monkeypatch.setattr('local_dashboard.webbrowser.open',lambda *a,**k:pytest.fail('Must not open failed server'))
    async def run():
        with pytest.raises(RuntimeError,match='cổng8877'):
            await serve_dashboard(object(),port=8877)
    asyncio.run(run())


def test_shared_preview_and_authenticated_close_2300(robot,tmp_path):
    import cv2
    import numpy as np
    from fastapi.testclient import TestClient
    from calibration_ui import calibration_app
    b,hw,a,_=prepared(robot,tmp_path);b.release_auto(a.token)
    ok,data=cv2.imencode('.jpg',np.zeros((360,640,3),np.uint8));assert ok
    vision=SimpleNamespace(jpeg=data.tobytes(),frame_at=time.monotonic())
    app=calibration_app(b,vision,a)
    with TestClient(app) as client:
        n=len(hw.commands)
        html=client.get('/control').text
        assert all(f'id="{i}"' in html for i in ['cameraViewport','gift','cameraFullscreen','showGuides','robotConnection','gripClosePulse'])
        assert 'cameraColumn' in html and 'controlColumn' in html
        preview=client.get('/api/preview?guides=false')
        assert preview.status_code==200 and preview.content==vision.jpeg
        assert not any(c.startswith(('S ','P ','G ')) for c in hw.commands[n:])
        token=re.search(r"const token='([^']+)'",html).group(1)
        body={'action':'command','command':'GRIP CLOSE','epoch':b.epoch}
        assert client.post('/operator/action',json=body).status_code==403
        assert client.post('/operator/action',headers={'x-operator-token':token},json=body).status_code==200
        wait_for(lambda:not b.busy)
        assert hw.state['joints']['d3']==2300 and 'S 3 2300' in hw.commands
        assert hw.state['joints'].get('d4')==2500
        vision.frame_at=time.monotonic()-3
        assert client.get('/api/preview').status_code==409


def test_actual_loopback_server_serves_camera_without_moving_robot(robot,tmp_path,monkeypatch):
    import socket
    import httpx
    from calibration_ui import calibration_app
    b,hw,a,_=prepared(robot,tmp_path)
    vision=SimpleNamespace(jpeg=b'qa-jpeg-shared-frame',frame_at=time.monotonic())
    app=calibration_app(b,vision,a)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    opened=[]
    monkeypatch.setattr('local_dashboard.webbrowser.open',lambda url,new:opened.append(url) or True)
    async def run():
        task=asyncio.create_task(serve_dashboard(app,port=port,open_browser=True));n=len(hw.commands)
        try:
            async with httpx.AsyncClient(trust_env=False) as client:
                for _ in range(40):
                    try:
                        response=await client.get(f'http://127.0.0.1:{port}/control')
                        break
                    except httpx.ConnectError:await asyncio.sleep(.05)
                else:pytest.fail('Local server did not start')
                assert response.status_code==200 and 'gripClosePulse' in response.text
                assert (await client.get(f'http://127.0.0.1:{port}/api/preview?guides=false')).content==vision.jpeg
                await asyncio.sleep(.1)
                assert opened==[f'http://127.0.0.1:{port}/control']
                assert not any(c.startswith(('P ','S ','G ')) for c in hw.commands[n:])
        finally:
            task.cancel()
            with pytest.raises(asyncio.CancelledError):await task
        with socket.socket() as sock:
            sock.bind(('127.0.0.1',port))  # Shutdown released its listening socket.
    asyncio.run(run())
