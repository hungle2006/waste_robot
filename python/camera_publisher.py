"""Publish the local USB camera; credentials stay in .env, never in logs."""
import asyncio
import os
import time
from datetime import timedelta

# Chọn trực tiếp camera USB trong mã. Theo lần thử mới nhất của bạn: camera index 0.
# Biến CAMERA_INDEX cũ trong .env không ghi đè lựa chọn này.
from pathlib import Path
from config_io import read_namespace
_PARAMETERS = read_namespace(os.environ.get('ROBOT_PARAMETERS',str(Path(__file__).with_name('robot_params.py'))))
CAMERA_INDEX = _PARAMETERS['CAMERA_INDEX']


def make_token(room_name):
    from livekit import api
    key = os.environ.get('LIVEKIT_API_KEY', '').strip()
    secret = os.environ.get('LIVEKIT_API_SECRET', '').strip()
    if not key or not secret:
        raise ValueError('Thiếu LIVEKIT_API_KEY hoặc LIVEKIT_API_SECRET trong python/.env')
    return (api.AccessToken(key, secret)
        .with_identity('robot_camera_' + room_name)
        .with_ttl(timedelta(hours=1))
        .with_grants(api.VideoGrants(room_join=True, room=room_name,
            can_publish=True, can_subscribe=False, can_publish_data=False))
        .to_jwt())


def open_camera(width, height, fps):
    import cv2
    index = CAMERA_INDEX
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW if os.name == 'nt' else cv2.CAP_ANY)
    try:
        if os.name == 'nt' and not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(index, cv2.CAP_ANY)
        if not cap.isOpened(): raise RuntimeError(f'Không mở được camera USB index {index}')
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        aspect_retry=False
        for _ in range(30):
            ok, frame = cap.read()
            if ok and frame is not None:
                if abs((frame.shape[1]/frame.shape[0])/(width/height)-1)>.01:
                    if aspect_retry:
                        raise RuntimeError('Camera không trả đúng tỷ lệ ảnh giấy; không kéo giãn 4:3 thành 16:9')
                    # Many UVC cameras ignore 640x360 but support 1280x720.
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,round(1280*height/width))
                    aspect_retry=True
                    continue
                return cap
            time.sleep(.1)
        raise RuntimeError('Camera mở được nhưng chưa đọc được ảnh')
    except BaseException:
        cap.release()
        raise


async def run_publisher(robot_id='robot_01', track_name='front_camera', on_frame=None, guides=None):
    from livekit import rtc
    import cv2
    room_name = os.environ.get('LIVEKIT_ROOM', robot_id).strip()
    track_name = os.environ.get('LIVEKIT_TRACK_NAME', track_name).strip()
    if room_name != robot_id: raise ValueError('LIVEKIT_ROOM phải trùng robot ID của app')
    url = os.environ.get('LIVEKIT_URL', '').strip()
    if not url.startswith('wss://'): raise ValueError('Thiếu LIVEKIT_URL dạng wss:// trong .env')
    width = _PARAMETERS['CAMERA_WIDTH']
    height = _PARAMETERS['CAMERA_HEIGHT']
    fps = _PARAMETERS['CAMERA_FPS']
    if not (1 <= fps <= 60 and 16 <= width <= 3840 and 16 <= height <= 2160):
        raise ValueError('Cấu hình kích thước/FPS camera không hợp lệ')
    while True:
        cap = None
        room = rtc.Room()
        source = None
        ended = asyncio.Event()
        @room.on('disconnected')
        def disconnected(reason): ended.set()
        try:
            token = make_token(room_name)
            cap = await asyncio.to_thread(open_camera, width, height, fps)
            await room.connect(url, token)
            source = rtc.VideoSource(width, height)
            track = rtc.LocalVideoTrack.create_video_track(track_name, source)
            await room.local_participant.publish_track(track,
                rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_CAMERA))
            print(f'Camera USB index {CAMERA_INDEX} → LiveKit · ' + room_name + ' · ' + track_name)
            failures = 0
            while not ended.is_set():
                started = time.monotonic()
                ok, frame = await asyncio.to_thread(cap.read)
                if not ok or frame is None:
                    failures += 1
                    if failures >= 50: raise RuntimeError('Mất camera USB')
                    await asyncio.sleep(.05)
                    continue
                failures = 0
                if abs((frame.shape[1]/frame.shape[0])/(width/height)-1)>.01:
                    raise RuntimeError('Camera đã đổi tỷ lệ ảnh; dừng phát thay vì kéo giãn sai ma trận')
                if frame.shape[1] != width or frame.shape[0] != height:
                    frame = cv2.resize(frame, (width, height))
                from camera_guides import display_frame
                shown = display_frame(frame, on_frame, guides)
                rgba = cv2.cvtColor(shown, cv2.COLOR_BGR2RGBA)
                source.capture_frame(rtc.VideoFrame(width, height,
                    rtc.VideoBufferType.RGBA, rgba.tobytes()))
                await asyncio.sleep(max(0, 1/fps - (time.monotonic() - started)))
        except asyncio.CancelledError: raise
        except Exception:
            print(f'Chưa phát được camera USB index {CAMERA_INDEX}; kiểm tra USB/khóa LiveKit. Thử lại sau 5 giây; điều khiển robot vẫn chạy.')
        finally:
            if cap is not None: cap.release()
            await room.disconnect()
            if source is not None: await source.aclose()
        await asyncio.sleep(5)


async def run_local_capture(on_frame, guides=None):
    """Camera USB -> cached raw JPEG -> Modal detector, with no LiveKit SDK.

    Jetson Nano's standard JetPack4 glibc cannot import the current LiveKit RTC
    wheel. This capture task still supplies AUTO and the local /control page.
    """
    from camera_guides import display_frame
    import cv2
    width = _PARAMETERS['CAMERA_WIDTH']
    height = _PARAMETERS['CAMERA_HEIGHT']
    fps = max(1, min(int(_PARAMETERS['CAMERA_FPS']), 30))
    while True:
        cap = None
        try:
            cap = await asyncio.to_thread(open_camera, width, height, fps)
            print('JETSON: camera index {} hoạt động, kích thước {}x{}'.format(CAMERA_INDEX, width, height), flush=True)
            bad_frames = 0
            while True:
                start = time.monotonic()
                ok, frame = await asyncio.to_thread(cap.read)
                if not ok or frame is None:
                    bad_frames += 1
                    if bad_frames >= 40:
                        raise RuntimeError('Camera USB mất tín hiệu')
                    await asyncio.sleep(.05)
                    continue
                bad_frames = 0
                if abs((frame.shape[1]/frame.shape[0])/(width/height)-1) > .01:
                    raise RuntimeError('Sai tỷ lệ khung hình camera: không được kéo giãn ảnh hiệu chuẩn')
                if frame.shape[1] != width or frame.shape[0] != height:
                    frame = cv2.resize(frame, (width, height))
                # Render display overlay separately. Never send annotated frames to AI.
                on_frame(frame)
                await asyncio.sleep(max(0., 1./fps-(time.monotonic()-start)))
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print('JETSON: camera chưa sẵn sàng ({}); tự thử lại sau 5 giây.'.format(exc), flush=True)
        finally:
            if cap is not None:
                cap.release()
        await asyncio.sleep(5)
