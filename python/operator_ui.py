"""Loopback-only operator panel mounted on the existing calibration server."""
import asyncio
import secrets
from pathlib import Path
from fastapi import HTTPException, Request
from fastapi.responses import HTMLResponse


def mount_operator(app, brain, controller):
    token=secrets.token_urlsafe(32)
    page=Path(__file__).resolve().parent.parent/'web/operator.html'

    def state():
        value=brain.snapshot();value['auto']=controller.operator.snapshot();return value

    @app.get('/control',response_class=HTMLResponse)
    def index():
        return HTMLResponse(page.read_text(encoding='utf-8').replace('__OPERATOR_TOKEN__',token),
                            headers={'Cache-Control':'no-store','X-Frame-Options':'DENY'})

    @app.get('/operator/state')
    def snapshot():return state()

    @app.post('/operator/action')
    async def action(request:Request):
        if not secrets.compare_digest(request.headers.get('x-operator-token',''),token):
            raise HTTPException(403,'Tải lại trang điều khiển')
        body=await request.json()
        if not isinstance(body,dict) or set(body)-{'action','command','epoch','joint','pulse_us','speed'}:
            raise HTTPException(422,'Sai yêu cầu')
        kind=body.pop('action',None)
        try:
            if kind=='connect':
                if body:raise ValueError('connect không có payload')
                await asyncio.to_thread(brain.connect)
            elif kind=='operator':await controller.operator.action(body)
            elif kind=='command':
                if type(body.get('epoch')) is not int:raise ValueError('epoch phải số nguyên')
                if body.get('command')=='STOP':await controller.stop()
                else:
                    if body.get('command')=='XM' and controller.operator.mechanics_enabled and controller.running:
                        await controller.stop()
                    await asyncio.to_thread(brain.command,body.get('command'),body['epoch'],body.get('joint'),body.get('pulse_us'),body.get('speed'))
                    if body.get('command')=='RESUME':controller.emergency_latched=False
            else:raise ValueError('Lệnh không được hỗ trợ')
        except (ValueError,ConnectionError) as exc:raise HTTPException(409,str(exc)) from exc
        return state()
