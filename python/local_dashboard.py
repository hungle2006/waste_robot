"""Serve the existing robot session on loopback; never open another camera/serial port."""
import asyncio
import os
import time
import webbrowser


def dashboard_settings(parameters, environ=None):
    env = os.environ if environ is None else environ
    enabled = env.get('LOCAL_OPERATOR_UI', '1' if parameters.get('LOCAL_OPERATOR_UI', True) else '0') == '1'
    open_browser = env.get('LOCAL_OPERATOR_OPEN_BROWSER', '1' if parameters.get('LOCAL_OPERATOR_OPEN_BROWSER', True) else '0') == '1'
    port = int(env.get('AUTO_CALIBRATION_PORT', parameters.get('LOCAL_OPERATOR_PORT', 8766)))
    if not 1024 <= port <= 65535:
        raise ValueError('Cổng giao diện camera cần nằm trong1024..65535')
    return {'enabled': enabled, 'open_browser': open_browser, 'port': port}


async def serve_dashboard(application, *, port=8766, open_browser=True):
    import uvicorn
    url = f'http://127.0.0.1:{port}/control'
    server = uvicorn.Server(uvicorn.Config(application, host='127.0.0.1', port=port, log_level='warning'))

    async def serve():
        try:
            await server.serve()
        except SystemExit as exc:
            raise RuntimeError(f'Không mở được giao diện tại cổng{port}; có thể Python khác đang dùng cổng này') from exc

    async def ready():
        deadline = time.monotonic()+10
        while not server.started:
            if time.monotonic() >= deadline:
                raise RuntimeError('Giao diện camera chưa khởi động sau10giây')
            await asyncio.sleep(.05)
        print('Camera và bảng điều khiển máy tính: '+url)
        if open_browser:
            try:
                opened = await asyncio.to_thread(webbrowser.open, url, new=2)
                if not opened:
                    print('Chưa mở được trình duyệt tự động; mở địa chỉ camera phía trên.')
            except Exception:
                print('Chưa mở được trình duyệt tự động; mở địa chỉ camera phía trên.')

    serving = asyncio.create_task(serve())
    opening = asyncio.create_task(ready())
    combined = asyncio.gather(serving, opening)
    try:
        await asyncio.shield(combined)
    finally:
        server.should_exit = True
        if not opening.done():
            opening.cancel()
        if not serving.done():
            try:
                await asyncio.wait_for(asyncio.shield(serving), 3)
            except Exception:
                serving.cancel()
        await asyncio.gather(opening, serving, return_exceptions=True)
        await asyncio.gather(combined, return_exceptions=True)
