"""Read the existing single LiveKit camera; send a fresh image to the existing Modal API."""
import asyncio
import io
import time
import math


def parse_detection(data):
    """Accept legacy object or unordered objects[]; pixel xyxy is the only box format."""
    if not isinstance(data, dict) or type(data.get("found")) is not bool:
        raise ValueError("Modal trả sai định dạng found/object")
    objects = data.get('objects')
    if objects is None:
        objects = [data['object']] if data.get('object') is not None else []
    if not isinstance(objects,list) or len(objects)>100:
        raise ValueError('Modal objects phải là danh sách tối đa 100 vật')
    if data['found'] != bool(objects):
        raise ValueError('Modal found không khớp danh sách vật')
    for obj in objects:
        if not isinstance(obj,dict) or type(obj.get('bin')) is not int or obj['bin'] not in (1,2,3,4):
            raise ValueError('Modal trả ngăn rác không hợp lệ')
        if not isinstance(obj.get('label'),str) or not obj['label'].strip():
            raise ValueError('Modal trả mục tiêu không hợp lệ')
        if 'center' in obj:
            if not isinstance(obj['center'],list) or len(obj['center'])!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in obj['center']):
                raise ValueError('Modal trả tọa độ không hợp lệ')
    return dict(data, objects=objects, object=objects[0] if objects else None)


class LiveKitModal:
    def __init__(self, app_url, modal_url, track_name="front_camera"):
        self.app_url, self.modal_url, self.track_name = app_url, modal_url, track_name
        self.jpeg = None
        self.frame_at = 0.
        self.status = "Chờ LiveKit"
        self.readers = set()
        self.detect_lock = asyncio.Lock()
        self.frame_size = None
        self.prefer_local = False
        self.overlay = None

    def set_overlay(self, objects, selected, captured, frame_size, state, target_id=None, locked_bin=None, max_age=2.):
        self.overlay = {'objects':objects, 'selected':selected, 'captured':captured,
                        'frame_size':frame_size, 'state':state, 'target_id':target_id,
                        'locked_bin':locked_bin, 'max_age':max_age}

    def set_overlay_state(self,state):
        if self.overlay:self.overlay['state']=state

    def clear_overlay(self):
        self.overlay = None

    def accept_local_frame(self, bgr):
        import cv2
        if time.monotonic()-self.frame_at < .2: return
        ok, image = cv2.imencode('.jpg',bgr,[cv2.IMWRITE_JPEG_QUALITY,85])
        if ok:
            self.jpeg = image.tobytes()
            self.frame_size = [bgr.shape[1],bgr.shape[0]]
            self.frame_at = time.monotonic()
            self.status = 'Camera USB tại máy Python'

    async def _consume(self, track):
        from livekit import rtc
        from PIL import Image
        stream = rtc.VideoStream(track, capacity=1)
        try:
            async for event in stream:
                if self.prefer_local: continue
                if time.monotonic()-self.frame_at < .2: continue
                frame = event.frame.convert(rtc.VideoBufferType.RGB24)
                image = Image.frombytes("RGB", (frame.width, frame.height), bytes(frame.data))
                output = io.BytesIO(); image.save(output, format="JPEG", quality=85)
                self.jpeg = output.getvalue(); self.frame_at = time.monotonic()
                self.frame_size = [frame.width,frame.height]
                self.status = "Đã nhận camera LiveKit"
        finally: await stream.aclose()

    async def run(self):
        import httpx
        from livekit import rtc
        while True:
            room = rtc.Room()
            ended = asyncio.Event()
            @room.on("track_subscribed")
            def subscribed(track, publication, participant):
                if track.kind == rtc.TrackKind.KIND_VIDEO and publication.name == self.track_name:
                    task = asyncio.create_task(self._consume(track)); self.readers.add(task)
                    task.add_done_callback(self.readers.discard)
            @room.on("disconnected")
            def disconnected(reason): ended.set()
            try:
                async with httpx.AsyncClient(timeout=15) as client:
                    response = await client.get(self.app_url+"/api/livekit-token")
                    response.raise_for_status(); config = response.json()
                await room.connect(config["url"], config["token"])
                self.status = "LiveKit đã kết nối; chờ track "+self.track_name
                await ended.wait()
            except asyncio.CancelledError: raise
            except Exception:
                self.status = "Chưa kết nối được LiveKit; kiểm tra camera và biến trên Vercel"
            finally:
                self.jpeg = None; self.frame_at = 0.
                for task in list(self.readers): task.cancel()
                await asyncio.gather(*self.readers, return_exceptions=True)
                await room.disconnect()
            await asyncio.sleep(5)

    async def detect(self, fresh_after=None):
        import httpx
        if self.detect_lock.locked(): raise ValueError("Modal đang xử lý ảnh trước")
        async with self.detect_lock:
            if not self.modal_url: raise ValueError("Thiếu NEXT_PUBLIC_MODAL_AI_URL trên Vercel")
            if fresh_after is not None:
                deadline = time.monotonic()+3
                while self.frame_at <= fresh_after and time.monotonic() < deadline:
                    await asyncio.sleep(.04)
                if self.frame_at <= fresh_after: raise ValueError('Camera chưa có ảnh mới sau khi robot đứng yên')
            if not self.jpeg or time.monotonic()-self.frame_at > 2:
                raise ValueError("Chưa có frame camera mới từ LiveKit. Kiểm tra nguồn camera Jetson.")
            image = self.jpeg
            captured, frame_size = self.frame_at, self.frame_size
            async with httpx.AsyncClient(timeout=180) as client:
                response = await client.post(self.modal_url.rstrip("/")+"/detect",
                    files={"image": ("robot.jpg", image, "image/jpeg")})
                response.raise_for_status()
                result = parse_detection(response.json())
                from target_lock import attach_appearance
                attach_appearance(image,result['objects'],frame_size)
                result['_frame_at'] = captured
                result['_frame_size'] = frame_size
                # Preserve the exact JPEG associated with this detection result.
                result['_frame_jpeg'] = image
                return result
