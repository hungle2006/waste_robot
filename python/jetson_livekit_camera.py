"""Jetson single-camera capture, independently fed to AUTO and optional LiveKit.

The UVC camera is opened only by run_local_capture in camera_publisher.
Room/token/network errors never interrupt the local detector. No robot motion here.
"""
import asyncio
import os
import time


class LocalCameraHub:
    """Keep the latest *raw* BGR frame; annotated preview is never sent to AI."""

    def __init__(self, on_local_frame):
        self._on_local_frame = on_local_frame
        self._frame = None
        self._at = 0.0
        self._seq = 0

    def accept(self, bgr):
        self._on_local_frame(bgr)
        # Preserve frame memory: publisher runs asynchronously from camera read.
        self._frame = bgr.copy()
        self._at = time.monotonic()
        self._seq += 1

    def snapshot(self, after=0, freshness=3.0):
        if self._seq <= after or not self._at:
            return None
        if time.monotonic() - self._at > freshness:
            return None
        return self._seq, self._frame


def livekit_import_check():
    """Pure import check; never install packages or change the robot environment."""
    try:
        from livekit import rtc, api
        if not all((hasattr(rtc, 'Room'), hasattr(rtc, 'VideoSource'),
                    hasattr(rtc, 'LocalVideoTrack'), hasattr(api, 'AccessToken'))):
            return False, 'LiveKit RTC/API thiếu thành phần cần thiết'
    except (ImportError, OSError, RuntimeError, AttributeError) as exc:
        return False, '{}: {}'.format(type(exc).__name__, str(exc)[:220])
    return True, 'LiveKit RTC và API import thành công'


def publish_settings(robot_id, config):
    """Return validated runtime options; never return or print credentials."""
    mode = os.environ.get('LIVEKIT_PUBLISHER', 'auto').strip().lower()
    if mode in ('0', 'false', 'no', 'off'):
        return None, 'LIVEKIT_PUBLISHER=0 (chỉ dùng camera nội bộ)'
    if mode not in ('1', 'true', 'yes', 'on', 'auto'):
        return None, 'LIVEKIT_PUBLISHER phải là 0, 1 hoặc auto'
    livekit_config = (config or {}).get('livekit') or {}
    if not livekit_config.get('configured'):
        return None, 'Ứng dụng Vercel chưa bật cấu hình LiveKit'
    if not os.environ.get('LIVEKIT_URL', '').startswith('wss://'):
        return None, 'Chưa có LIVEKIT_URL dạng wss:// trong python/.env'
    if not all(os.environ.get(k, '').strip() for k in ('LIVEKIT_API_KEY', 'LIVEKIT_API_SECRET')):
        return None, 'Chưa có LIVEKIT_API_KEY và LIVEKIT_API_SECRET trong python/.env'
    if os.environ.get('LIVEKIT_ROOM', robot_id).strip() != robot_id:
        return None, 'LIVEKIT_ROOM phải trùng robotId trên ứng dụng'
    try:
        fps = min(15, max(1, int(os.environ.get('LIVEKIT_FPS', '8'))))
    except ValueError:
        return None, 'LIVEKIT_FPS phải là số nguyên 1–15'
    return {'room': robot_id,
            'track': os.environ.get('LIVEKIT_TRACK_NAME', livekit_config.get('trackName') or 'front_camera').strip(),
            'url': os.environ['LIVEKIT_URL'].strip(),
            'fps': fps}, ''


async def run_livekit_from_hub(hub, robot_id, config):
    """Publish raw frames without ever acquiring the camera device.

    Retries after disconnection; waits for new frames before sending. Import and
    token failures do not affect the always-running OpenCV capture task.
    """
    from livekit import rtc
    import cv2
    from camera_publisher import make_token

    settings, problem = publish_settings(robot_id, config)
    if problem:
        print('[LiveKit] ' + problem + ' – OpenCV/AUTO vẫn hoạt động.', flush=True)
        return
    room_name = settings['room']
    fps = settings['fps']
    while True:
        room = rtc.Room()
        source = None
        disconnected = asyncio.Event()

        @room.on('disconnected')
        def on_disconnect(reason):
            disconnected.set()

        try:
            # Wait for actual live camera frames. Never publish a blank image.
            data = None
            while data is None:
                data = hub.snapshot()
                if data is None:
                    await asyncio.sleep(.25)
            height, width = data[1].shape[:2]
            await room.connect(settings['url'], make_token(room_name))
            source = rtc.VideoSource(width, height)
            track = rtc.LocalVideoTrack.create_video_track(settings['track'], source)
            options = rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_CAMERA)
            await room.local_participant.publish_track(track, options)
            print('[LiveKit] Đang phát camera lên phòng {} / track {} ({}x{}, tối đa {} FPS)'.format(
                room_name, settings['track'], width, height, fps), flush=True)
            seq = 0
            next_frame_at = 0.0
            while not disconnected.is_set():
                if time.monotonic() < next_frame_at:
                    await asyncio.sleep(.02)
                    continue
                latest = hub.snapshot(after=seq, freshness=3.0)
                if latest is None:
                    await asyncio.sleep(.05)
                    continue
                seq, bgr = latest
                h, w = bgr.shape[:2]
                if (w, h) != (width, height):
                    print('[LiveKit] Camera đổi kích thước; sẽ mở lại track.', flush=True)
                    break
                rgba = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGBA)
                frame = rtc.VideoFrame(w, h, rtc.VideoBufferType.RGBA, rgba.tobytes())
                source.capture_frame(frame)
                next_frame_at = time.monotonic() + 1.0 / fps
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            # Do not print URL/token/keys or HTTP responses with secrets.
            print('[LiveKit] Mất kết nối hoặc lỗi phát camera ({}); thử lại sau 5 giây. AUTO vẫn dùng OpenCV.'.format(
                type(exc).__name__), flush=True)
        finally:
            try:
                await room.disconnect()
            except Exception:
                pass
            if source is not None and hasattr(source, 'aclose'):
                try:
                    await source.aclose()
                except Exception:
                    pass
        await asyncio.sleep(5)
