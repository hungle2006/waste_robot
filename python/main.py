"""Main entry point: auto-setup on Jetson Nano / VS Code then start the robot."""
# This entrypoint is compatible with system Python 3.6 until it relaunches in 3.10.
if __name__ == '__main__':
    import sys as _sys
    # Checks execute in the *current* Python environment without installing
    # anything or opening the camera, serial port, or motors.
    if '--firmware-check' in _sys.argv:
        from firmware_check import main as check_firmware
        raise SystemExit(check_firmware())
    if '--stream-check' in _sys.argv:
        from livekit_stream_check import run_check
        raise SystemExit(run_check())
    if '--diagnose' in _sys.argv or '--livekit-check' in _sys.argv:
        from jetson_diagnostics import diagnose, diagnose_livekit
        if '--livekit-check' in _sys.argv:
            raise SystemExit(diagnose_livekit())
        diagnose()
        raise SystemExit(0)
    from jetson_bootstrap import ensure_and_relaunch
    ensure_and_relaunch()
    if '--try-livekit' in _sys.argv:
        from livekit_install_test import test_install
        raise SystemExit(test_install())

import asyncio
import os
import signal
from pathlib import Path

import httpx
from dotenv import load_dotenv
from supabase import acreate_client
from realtime import RealtimeSubscribeStates

from brain import Brain
from transport import SerialTransport, SimTransport
from realtime_bridge import RealtimeService
from auto_controller import AutoController

ROOT = Path(__file__).resolve().parent


async def load_app_config(app_url):
    async with httpx.AsyncClient(timeout=15) as client:
        response = await client.get(app_url+"/api/robot/configuration")
        response.raise_for_status(); data = response.json()
    if data.get("transport") != "supabase" or not data.get("configured"):
        raise ValueError("App chưa dùng cấu hình Supabase mới. Kiểm tra URL app và bản triển khai.")
    cfg = data.get("supabase", {})
    if not cfg.get("url", "").startswith("https://") or not cfg.get("publishableKey"):
        raise ValueError("App thiếu NEXT_PUBLIC_SUPABASE_URL/PUBLISHABLE_KEY")
    return data


async def main():
    load_dotenv(ROOT/".env")
    params_path=Path(os.environ.get("ROBOT_PARAMETERS", str(ROOT/"robot_params.py")))
    os.environ['ROBOT_PARAMETERS']=str(params_path.resolve())
    from config_io import read_namespace
    parameters=read_namespace(params_path)
    app_url = os.environ.get("APP_URL", "https://waste-app-control.vercel.app").strip().rstrip("/")
    if not app_url.startswith("https://"): raise ValueError("APP_URL phải là địa chỉ HTTPS của app")
    config = await load_app_config(app_url)
    robot_id = config["robotId"]
    simulation = os.environ.get("ROBOT_SIMULATION") == "1"
    transport = SimTransport() if simulation else SerialTransport(os.environ.get("ARDUINO_PORT", "COM3" if os.name == "nt" else "/dev/ttyUSB0"))
    brain = Brain(transport, os.environ.get("ROBOT_CONFIG", str(params_path)), simulation)
    brain.start_monitor()
    supabase = await acreate_client(config["supabase"]["url"], config["supabase"]["publishableKey"])
    channel = supabase.channel("robot:"+robot_id, {"config": {"broadcast": {"ack": True}}})
    vision = None
    jetson_camera = os.environ.get('JETSON_LOCAL_CAMERA', '0') == '1'
    if jetson_camera:
        from livekit_modal import LiveKitModal
        vision = LiveKitModal(app_url, config.get('modalAiUrl', ''), config.get('livekit', {}).get('trackName', 'front_camera'))
        vision.prefer_local = True
        if not config.get('modalAiUrl'):
            print('CHƯA CÓ URL MODAL: camera hiển thị được nhưng AUTO chưa thể phân loại ảnh.')
        print('JETSON: camera OpenCV trực tiếp -> Modal AI; không bắt buộc SDK LiveKit.')
    elif os.environ.get("LIVEKIT_VIEWER", "1") == "1" and config.get("livekit", {}).get("configured"):
        try:
            from livekit import rtc
            from PIL import Image
            from livekit_modal import LiveKitModal
            vision = LiveKitModal(app_url, config.get("modalAiUrl", ""), config["livekit"]["trackName"])
        except ImportError:
            print("Chưa cài requirements-camera.txt; cơ khí vẫn chạy, DETECT cần camera SDK.")
    service = RealtimeService(brain, robot_id, channel, vision)
    service.auto = AutoController(brain,vision,os.environ.get('AUTO_CONFIG',str(params_path)),service.auto_report)
    from camera_guides import CameraGuides
    camera_guides = CameraGuides(service.auto.path, service.auto, brain)
    async def subscribed(status, error):
        if status == RealtimeSubscribeStates.SUBSCRIBED:
            service.online = True
            print("Supabase online · "+robot_id+" · mở app rồi bấm Kết nối Python")
            await service.publish_state()
        else: await service.disconnected()
    def on_subscribe(status, error): asyncio.create_task(subscribed(status, error))
    await channel.on_broadcast(event="command", callback=service.on_command).subscribe(on_subscribe)
    print("Waste Robot V3.16.37 · Jetson Nano/VS Code · D18 ưu tiên AUTO, chỉ giám sát trong MANUAL · D19 thu tay · đổi quà D18 120s · VL53 lỗi/chưa có → tiếp tục tiến; hợp lệ 20–25cm → gắp · box + D19 · "+("MÔ PHỎNG" if simulation else "USB "+transport.port_name))
    print('Tham số: '+str(params_path))
    if not service.auto.path.suffix == '.py':
        print('Đang dùng AUTO_CONFIG JSON cũ do biến môi trường; bỏ AUTO_CONFIG để dùng robot_params.py')
    elif parameters['AUTO_CONFIG'].get('box_tof_flow',{}).get('enabled'):
        print('AUTO: đáy trên dải→tiến; trong/dưới→hạ tay; nghi chạm mép trên/dưới→tiến. Đúng lock1 frame; mất/sai3 frame thì giữ chờ (require_visible_target=True).')
        print('VL53 200–250 mm xác nhận vùng gắp; D19 chỉ ở bước gắp/mang vật. Các nhánh tiến/lùi theo cấu hình AUTO.')
        print('Hai mép trái/phải: giữ tay, tiến160ms theo VL53. Đang tiến/mất box: đủ VL53 vẫn chuyển gắp; mọi nhịp tiến có phanh VL53.')
        print('Kết nối lại COM: khôi phục checkpoint SEARCH khi an toàn; có thể đang giữ vật thì yêu cầu kiểm tra trước.')
    elif not parameters['AUTO_CONFIG']['distance']['enabled']:
        print('7 ảnh USB: khoảng ống kính 60/75/90/45/24/12/5 cm; mốc S2 27 cm lưu riêng. AUTO theo mốc ảnh + D19.')
    tasks = [asyncio.create_task(service.run())]
    from local_dashboard import dashboard_settings, serve_dashboard
    dashboard = dashboard_settings(parameters)
    if dashboard['enabled']:
        from calibration_ui import calibration_app
        async def operator_task():
            try:
                await serve_dashboard(calibration_app(brain,vision,service.auto,camera_guides),
                                      port=dashboard['port'], open_browser=dashboard['open_browser'])
            except asyncio.CancelledError: raise
            except Exception:
                print(f"Không mở được giao diện camera cổng{dashboard['port']}; kiểm tra Python khác đang chạy. Điều khiển qua app vẫn hoạt động.")
        tasks.append(asyncio.create_task(operator_task()))
    if jetson_camera and vision is not None:
        from camera_publisher import run_local_capture, CAMERA_INDEX
        from jetson_livekit_camera import LocalCameraHub, livekit_import_check, publish_settings, run_livekit_from_hub
        hub = LocalCameraHub(vision.accept_local_frame)
        print('JETSON: camera USB index {} -> AUTO/OpenCV + localhost (không phụ thuộc LiveKit)'.format(CAMERA_INDEX))
        tasks.append(asyncio.create_task(run_local_capture(hub.accept, camera_guides)))
        settings, reason = publish_settings(robot_id, config)
        if settings is None:
            print('[LiveKit] {}. AUTO camera nội bộ vẫn chạy.'.format(reason))
        else:
            ready, reason = livekit_import_check()
            if ready:
                print('[LiveKit] Sẵn sàng phát camera LiveKit: phòng {} / track {}. OpenCV/AUTO chạy song song.'.format(settings['room'], settings['track']), flush=True)
                tasks.append(asyncio.create_task(run_livekit_from_hub(hub, robot_id, config)))
            else:
                print('[LiveKit] {}. Kiểm tra bằng: python3 python/main.py --livekit-check'.format(reason))
                print('[LiveKit] Tiếp tục camera OpenCV, không làm dừng AUTO.')
    elif os.environ.get("LIVEKIT_PUBLISHER", "0") == "1":
        if vision: vision.prefer_local = True
        from camera_publisher import CAMERA_INDEX, run_publisher
        print(f'Camera USB đã chọn: index {CAMERA_INDEX} · phát LiveKit và cấp ảnh cho AUTO')
        async def publisher_task():
            try: await run_publisher(robot_id, config["livekit"]["trackName"],vision.accept_local_frame if vision else None,camera_guides)
            except asyncio.CancelledError: raise
            except Exception:
                print("Không khởi tạo được camera publisher; kiểm tra requirements-camera.txt và .env. Cơ khí vẫn chạy.")
        tasks.append(asyncio.create_task(publisher_task()))
    if vision and not vision.prefer_local:
        async def camera_task():
            try: await vision.run()
            except asyncio.CancelledError: raise
            except Exception:
                print("LiveKit viewer chưa khởi tạo được. Cơ khí vẫn chạy; kiểm tra SDK/camera hoặc đặt LIVEKIT_VIEWER=0.")
        tasks.append(asyncio.create_task(camera_task()))
    try: await asyncio.gather(*tasks)
    finally:
        await service.auto.stop()
        for task in tasks: task.cancel()
        for task in service.tasks: task.cancel()
        await asyncio.gather(*tasks, *service.tasks, return_exceptions=True)
        await asyncio.to_thread(brain.close)
        await supabase.remove_channel(channel)
        await supabase.realtime.close()


if __name__ == "__main__":
    try: asyncio.run(main())
    except KeyboardInterrupt: print("Đã dừng Python và gửi STOP tới Arduino")
