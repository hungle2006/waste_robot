"""Supabase command gateway. Robot ACK is distinct from the broker's receipt."""
import copy
import threading
import time
import uuid
from collections import OrderedDict


class MechanicsDispatcher:
    def __init__(self, brain, robot_id):
        self.brain, self.robot_id = brain, robot_id
        self.session_id = str(uuid.uuid4())
        self.lock = threading.RLock()
        self.seen = OrderedDict()
        self.sequences = {}
        self.operator_handler = None
        self.before_command = None
        self.resume_precheck = None

    def handle(self, packet):
        with self.lock:
            if not isinstance(packet, dict) or packet.get("robot_id") != self.robot_id or packet.get("type") != "mechanics":
                raise ValueError("Sai robot hoặc loại lệnh")
            request_id, client_id = packet.get("request_id"), packet.get("client_id")
            if any(not isinstance(value, str) or not 1 <= len(value) <= 80 for value in (request_id, client_id)):
                raise ValueError("Thiếu mã request/client")
            seq = packet.get("seq")
            if type(seq) is not int or seq < 1: raise ValueError("Sai thứ tự lệnh")
            action = packet.get("action")
            body = packet.get("payload", {})
            if not isinstance(body, dict): raise ValueError("Payload phải là object")
            command = body.get("command")
            urgent = action == "command" and command in ("STOP", "XM")
            key = (client_id, request_id)
            if key in self.seen:
                saved = copy.deepcopy(self.seen[key]); saved["duplicate"] = True
                return saved  # Never execute a repeated servo/motor command.
            if action != "status" and packet.get("session_id") != self.session_id:
                raise ValueError("Phiên Python đã thay đổi. Đọc trạng thái và kết nối lại.")
            ttl, issued = packet.get("ttl_ms"), packet.get("issued_at")
            max_ttl = 500 if isinstance(command, str) and command.startswith("DRIVE ") else 8000
            if type(ttl) is not int or not 1 <= ttl <= max_ttl or type(issued) is not int:
                raise ValueError("Sai hạn dùng lệnh")
            age = time.time()*1000-issued
            if age > ttl or age < -5000: raise ValueError("Lệnh đã hết hạn; kiểm tra giờ máy và nhấn lại")
            if seq <= self.sequences.get(client_id, 0) and not urgent:
                raise ValueError("Lệnh cũ tới trễ đã bị hủy")
            self.sequences[client_id] = max(seq, self.sequences.get(client_id, 0))
            if len(self.sequences) > 200: self.sequences.pop(next(iter(self.sequences)))
            try:
                self._execute(action, body)
                result = {"ok": True}
            except (ConnectionError, ValueError) as exc:
                if isinstance(exc, ConnectionError): self.brain.stop()
                result = {"ok": False, "error": str(exc)}
            self.seen[key] = result
            while len(self.seen) > 400: self.seen.popitem(last=False)
            return copy.deepcopy(result)

    def _execute(self, action, body):
        if action in ("status", "connect"):
            if body: raise ValueError("status/connect không có payload")
            if action == "connect":
                pending = bool(self.resume_precheck and self.resume_precheck())
                self.brain.connect(skip_setup=pending)
            return
        epoch = body.get("epoch")
        if type(epoch) is not int: raise ValueError("epoch phải là số nguyên")
        if action == 'operator':
            if set(body) != {'epoch','command'} or not isinstance(body['command'],str):
                raise ValueError('Sai trường điều khiển AUTO')
            if epoch != self.brain.epoch: raise ValueError('Lệnh AUTO cũ')
            if not self.operator_handler: raise ValueError('Cần Python V3.15 main.py')
            self.operator_handler(body)
            return
        if action == "poses":
            if set(body) != {"epoch", "name", "values"}: raise ValueError("Sai trường tư thế")
            self.brain.save_pose(body["name"], body["values"], epoch)
            return
        if action != "command" or set(body)-{"epoch", "command", "joint", "pulse_us", "speed"}:
            raise ValueError("Lệnh không được hỗ trợ")
        command, joint, pulse = body.get("command"), body.get("joint"), body.get("pulse_us")
        if (command is None) == (joint is None): raise ValueError("Chọn một lệnh hoặc một khớp")
        if command is not None and not isinstance(command, str): raise ValueError("command phải là chuỗi")
        if joint is not None and (not isinstance(joint, str) or type(pulse) is not int): raise ValueError("Sai khớp/xung")
        if joint is None and pulse is not None: raise ValueError("Xung phải có khớp")
        if self.before_command: self.before_command(command)
        self.brain.command(command, epoch, joint, pulse, body.get("speed"))


class RealtimeService:
    def __init__(self, brain, robot_id, channel, vision=None):
        self.brain, self.robot_id, self.channel, self.vision = brain, robot_id, channel, vision
        self.dispatcher = MechanicsDispatcher(brain, robot_id)
        self.state_seq = 0
        self.online = False
        self.tasks = set()
        self.auto = None
        self.mode_barrier = 0

    def on_command(self, message):
        import asyncio
        p = message.get("payload", message) if isinstance(message, dict) else {}
        if not isinstance(p, dict) or p.get("robot_id") != self.robot_id: return
        if p.get("type") == "mechanics": coroutine = self.command(p)
        elif p.get("type") == "system" and p.get("action") == "emergency_stop": coroutine = self.emergency_stop()
        elif p.get("type") == "ai" and p.get("action") == "detect": coroutine = self.detect()
        elif p.get('type') == 'mode' and p.get('action') in ('auto_start','auto_stop','manual'):
            coroutine = self.mode(p)
        else: return  # Existing AUTO controller remains responsible for legacy mode/arm/bin events.
        task = asyncio.create_task(coroutine); self.tasks.add(task)
        def done(task):
            self.tasks.discard(task)
            if not task.cancelled():
                error = task.exception()
                if error: print('Lỗi xử lý lệnh:',type(error).__name__)
        task.add_done_callback(done)

    async def packet(self):
        import asyncio
        state = await asyncio.to_thread(self.brain.snapshot)
        if self.auto: state['auto'] = self.auto.operator.snapshot()
        # Stamp immediately after snapshot. Readers reject older state packets.
        self.state_seq += 1
        return {"robot_id": self.robot_id, "session_id": self.dispatcher.session_id,
                "state_seq": self.state_seq, "state": state}

    async def command(self, p):
        import asyncio
        try:
            body = p.get('payload',{})
            if p.get('action') == 'command' and body.get('command') == 'AUTO REAR ON':
                if not self.auto or not self.vision:
                    raise ValueError('Cần chạy main.py với camera và Modal để bật tự phân loại phía sau')
                from auto_controller import load_rear_config
                load_rear_config(self.auto.path)
            loop = asyncio.get_running_loop()
            def run(coro):
                future = asyncio.run_coroutine_threadsafe(coro,loop)
                try: return future.result(timeout=15)
                except TimeoutError:
                    future.cancel()
                    raise ValueError('Thao tác AUTO quá thời gian')
            async def before(command):
                if not self.auto: return
                if command=='STOP' or (command=='XM' and self.auto.operator.mechanics_enabled):
                    self.mode_barrier = max(self.mode_barrier,time.time()*1000)
                    if self.auto.running: await self.auto.stop()
            self.dispatcher.before_command = lambda cmd: run(before(cmd))
            self.dispatcher.operator_handler = (lambda body: run(self.auto.operator.action(body))) if self.auto else None
            self.dispatcher.resume_precheck = (
                lambda: bool(self.auto and self.auto.checkpoint.load()))
            result = await asyncio.to_thread(self.dispatcher.handle, p)
            if self.auto and p.get('action') == 'connect' and result.get('ok'):
                if self.auto.checkpoint.load():
                    try:
                        await self.auto.resume_after_connect()
                    except (ValueError, ConnectionError) as resume_error:
                        self.auto.state.update(
                            state='auto_reconnect_inspection', auto_running=False,
                            resume_available=True, resume_requires_inspection=True,
                            message='Không thể tự xác minh để tiếp tục: ' + str(resume_error))
                        await self.auto_report(copy.deepcopy(self.auto.state))
            if self.auto and body.get('command') == 'RESUME' and result.get('ok'):
                self.auto.emergency_latched = False
            if self.auto and p.get('action') == 'connect' and result.get('ok'):
                self.auto.emergency_latched = False
        except (ValueError, ConnectionError) as exc:
            result = {"ok": False, "error": str(exc)}
        # Read current state after execution, including any urgent STOP in between.
        result.update(await self.packet())
        result.update(client_id=p.get("client_id"), request_id=p.get("request_id"))
        if self.online:
            try: await self.channel.send_broadcast("mechanical_ack", result)
            except Exception: await self.disconnected()

    async def emergency_stop(self):
        import asyncio
        self.mode_barrier = max(self.mode_barrier,time.time()*1000)
        if self.auto: await self.auto.stop(emergency=True)
        else: await asyncio.to_thread(self.brain.stop)
        await self.publish_state()

    async def publish_state(self):
        if self.online:
            try: await self.channel.send_broadcast("mechanical_state", await self.packet())
            except Exception: await self.disconnected()

    async def disconnected(self):
        import asyncio
        self.online = False
        if self.auto: await self.auto.stop(emergency=True)
        elif self.brain.connected: await asyncio.to_thread(self.brain.stop)

    async def auto_report(self, payload):
        if self.online:
            try: await self.channel.send_broadcast('status',payload)
            except Exception:
                import asyncio
                self.online = False
                # Do not cancel/await the AUTO task from inside its own report callback.
                asyncio.create_task(self.disconnected())

    async def mode(self, p):
        if not self.auto:
            await self.auto_report({'state':'auto_not_ready','message':'Cần chạy Python V3.3 AUTO','auto_running':False})
            return
        issued = p.get('ts')
        if p['action'] == 'auto_start':
            if type(issued) is not int or not -5000 <= time.time()*1000-issued <= 8000:
                await self.auto_report({'state':'auto_not_ready','message':'Lệnh START đã cũ; nhấn lại','auto_running':False})
                return
            if issued <= self.mode_barrier: return
            self.mode_barrier = issued
            if self.online: await self.auto.start()
        else:
            self.mode_barrier = max(self.mode_barrier,time.time()*1000)
            if p['action'] == 'manual':
                # A MANUAL handover is NOT a physical STOP. Brake AUTO motors,
                # detach its writer, clear D18's pending preemption and retain
                # wheel controls. The actual STOP remains explicitly latched.
                await self.auto.operator.set_mechanics(False)
            else:
                await self.auto.stop()

    async def detect(self):
        if not self.online: return
        if self.auto and self.auto.running:
            await self.auto_report({'state':'auto_busy','message':'AUTO đang dùng nhận diện; dừng AUTO trước khi DETECT riêng','auto_running':True})
            return
        if self.brain.rear_pending or self.brain.rear_handling:
            await self.auto_report({'state':'rear_busy','message':'D18 đang ưu tiên phía sau; đợi xử lý xong mới DETECT riêng','auto_running':False})
            return
        try:
            if not self.vision: raise ValueError("Chưa bật LiveKit viewer hoặc chưa cài requirements-camera.txt trên máy Python")
            result = await self.vision.detect()
            obj = result.get("object") or {}
            payload = {"state": "detected" if result["found"] else "no_target",
                       "message": "Đã nhận kết quả Modal" if result["found"] else "Chưa tìm thấy rác",
                       "label": obj.get("label"), "category": obj.get("category"),
                       "bin": obj.get("bin"), "center": obj.get("center")}
        except Exception as exc:
            payload = {"state": "ai_error", "message": str(exc)}
        if self.online:
            try: await self.channel.send_broadcast("status", payload)
            except Exception: await self.disconnected()

    async def run(self):
        import asyncio
        next_publish = 0.
        while True:
            if self.auto:self.auto.operator.poll_sensors()
            # D18 is a local hardware priority: service it even without
            # a cloud UI connection (MANUAL/local desktop included).
            if self.auto: await self.auto.service_rear()
            if time.monotonic() >= next_publish:
                await self.publish_state()
                next_publish = time.monotonic()+.5
            await asyncio.sleep(.1)
