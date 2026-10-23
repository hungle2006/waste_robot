"""Python owns presets, ramping, cancellation and sensor-driven decisions."""
import copy
import json
import os
import threading
import time
from collections import deque
from pathlib import Path
from config_io import read_config
from gift_control import GiftController
from mecanum import DEFAULT_SPEED, DRIVE_AXES, DRIVE_LEASE_MS, DRIVE_PROTOCOL, mix_mecanum, SMOOTH_PROTOCOL, smooth_timing

VISIBLE = ("p1", "p10", "p11")
JOINTS = VISIBLE + ("p9", "p4", "d2", "d3")


def validate_config(c):
    if not isinstance(c, dict):
        raise ValueError("Cấu hình phải là object")
    if set(c) != {"limits","step_us","step_ms","settle_ms","grip","lid","bins","poses"}:
        raise ValueError("Thiếu hoặc thừa trường cấu hình")
    if not isinstance(c["limits"], dict) or set(c["limits"]) != set(JOINTS):
        raise ValueError("Giới hạn phải có đủ 7 servo")
    for j, bounds in c["limits"].items():
        if not isinstance(bounds,list) or len(bounds)!=2 or any(type(x)!=int for x in bounds):
            raise ValueError("Giới hạn phải là hai số nguyên")
        if not 500 <= bounds[0] < bounds[1] <= (3500 if j == "p1" else 2500):
            raise ValueError("Giới hạn S2/P1: 500–3500 µs; các khớp khác: 500–2500 µs")
    for key, lo, hi in (("step_us",1,100),("step_ms",20,200),("settle_ms",0,3000)):
        if type(c[key]) != int or not lo <= c[key] <= hi:
            raise ValueError("Tham số tốc độ không hợp lệ: " + key)
    def pulse(j,v):
        if type(v)!=int or not c["limits"][j][0] <= v <= c["limits"][j][1]:
            raise ValueError("Xung ngoài giới hạn: " + j)
    for key, joint, names in (("grip","d3",{"open","close"}),
                              ("lid","p4",{"open","close"}),
                              ("bins","p9",{"ORGANIC","HAZARD","OTHER","RECYCLE"})):
        if not isinstance(c[key],dict) or set(c[key])!=names:
            raise ValueError("Cấu hình sai: " + key)
        for v in c[key].values(): pulse(joint,v)
    if not isinstance(c["poses"],dict) or set(c["poses"])!={"SETUP","SEARCH","PLACE","REAR"}:
        raise ValueError("Cần đủ 4 tư thế")
    for pose, values in c["poses"].items():
        if not isinstance(values,dict) or not values or set(values)-set(JOINTS):
            raise ValueError("Tư thế sai: " + pose)
        for j,v in values.items(): pulse(j,v)
    if set(c["poses"]["SETUP"]) != set(JOINTS):
        raise ValueError("SETUP phải thiết lập cả 7 servo")
    return c


class Brain:
    def __init__(self, transport, config_path, simulation=False):
        self.hw, self.path = transport, Path(config_path)
        self.config = validate_config(read_config(self.path, "robot"))
        self.lock = threading.RLock()
        self.shutdown = threading.Event()
        self.cancel = threading.Event()
        self.worker = None
        self.monitor = None
        self.epoch = 0
        self.connected = False
        self.stopped = True
        self.auto_rear = False
        self.rear_manual_opt_out = False
        # Red MANUAL-mechanics mode: D18 is telemetry only, never an arm/wheel preemption.
        self.manual_mechanics_mode = False
        self.rear_armed = False
        self.rear_pending = False
        self.rear_handling = False
        self.rear_clear_since = None
        self.busy = False
        self.job = "Chờ kết nối"
        self.progress = 0
        self.updated = 0.
        self.last_hw = {}
        self.log = deque(maxlen=30)
        self.simulation = simulation
        self.drive_until = 0.
        self.drive_vector = None
        self.auto_owner = None
        self.motion_finished_at = 0.
        self.command_trace = deque(maxlen=80)
        self.gift = GiftController(self)

    def event(self, text):
        self.log.appendleft({"time": time.strftime("%H:%M:%S"), "text": text})

    def start_monitor(self):
        self.monitor = threading.Thread(target=self._monitor, daemon=True)
        self.monitor.start()

    def connect(self, skip_setup=False):
        with self.lock:
            if not self.connected:
                self.gift.cancel("Kết nối mới; không tự phát lại quà")
                self.hw.open()  # Opening Mega resets it. Starts latched STOP.
                self.connected = True
                self.stopped = True
                self.epoch += 1
                self.auto_rear = False
                self.last_hw = self.hw.request("STATUS")
                if self.last_hw.get("drive_protocol") != DRIVE_PROTOCOL:
                    self._fault("Cần nạp firmware Arduino Mecanum mới trước khi kết nối")
                    raise ValueError(self.job)
                self.updated = time.monotonic()
                self._send("RESUME")
                self.stopped = False
                self.epoch += 1
                self.gift.initialize_closed()
                self.event("Kết nối USB · khởi tạo tìm rác phía trước")
                if skip_setup:
                    self.event("Đã khôi phục USB; hoãn chuyển động để AUTO xác minh checkpoint")
                else:
                    self._start_job(self._front_setup(), "SEARCH · khởi tạo mặc định")
            return self.snapshot()

    def check(self, epoch, allow_drive=False):
        if not self.connected: raise ConnectionError("Arduino chưa kết nối")
        if epoch != self.epoch: raise ValueError("Lệnh cũ bị hủy. Đọc lại trạng thái.")
        if self.stopped: raise ValueError("Đang dừng. Nhấn Cho phép điều khiển.")
        if self.busy or (self.drive_until and not allow_drive):
            raise ValueError("Đang chạy thao tác. Chờ hoặc nhấn Dừng.")

    def _send(self, cmd):
        result = self.hw.request(cmd)
        if cmd != 'STATUS':
            self.command_trace.appendleft({'time': time.strftime('%H:%M:%S'),
                'command': cmd, 'source': 'auto' if self.auto_owner else 'manual',
                'delivery': 'acknowledged'})
        return result

    def refresh_status(self, epoch, owner=None):
        """Read a new USB sample at a movement boundary instead of reusing a poll."""
        with self.lock:
            if not self.connected:raise ConnectionError('Arduino chưa kết nối')
            if epoch!=self.epoch or owner is not None and owner!=self.auto_owner:
                raise ValueError('Chu trình đã bị hủy trước khi đọc cảm biến')
            self.last_hw=self._send('STATUS')
            self.updated=time.monotonic()
            if self.last_hw.get('stopped') and not self.stopped:self.stop()
            return self.snapshot()

    def detach_auto(self):
        """Cancel the current writer atomically; hold servo pulses, brake wheels."""
        with self.lock:
            self.cancel.set()
            self.auto_owner = None
            self.epoch += 1
            self.enable_rear(False)
            self.rear_pending = self.rear_handling = False
            self.drive_until = 0.
            self.drive_vector = None
            if self.connected:
                self._send('XM')
                if not self.stopped and self.last_hw.get('ir_drive_guard') is not None:
                    self._send('IG 1')
                if not self.stopped and self.last_hw.get('tof_protocol') == 1:
                    self._send('TF OFF')
                self.last_hw = self._send('STATUS')
                self.updated = time.monotonic()
            worker = self.worker
        # Never hold Brain.lock while joining a worker which needs that lock.
        if worker and worker is not threading.current_thread():
            worker.join(timeout=2.)
            if worker.is_alive():
                self.stop()
                raise ValueError('Không hủy được thao tác cơ khí; đã STOP')
        return self.snapshot()

    def stop(self):
        with self.lock:
            self.gift.cancel("Đã STOP; ngắt servo quà")
            self.auto_owner = None
            self.cancel.set()
            self.epoch += 1
            self.stopped = True
            self.auto_rear = False
            self.rear_pending = False
            self.rear_handling = False
            self.rear_armed = False
            self.rear_clear_since = None
            self.drive_until = 0.
            self.drive_vector = None
            self.job = "Đã dừng · chuỗi lệnh bị hủy"
            self.event("Dừng motor, tắt xung servo, khóa lệnh mới")
            if self.connected:
                try:
                    self._send("STOP")
                    self.last_hw = self._send("STATUS")
                    self.updated = time.monotonic()
                except (ConnectionError, ValueError):
                    self._fault("Không xác nhận được STOP qua USB")
            return self.snapshot()

    def resume(self, epoch):
        with self.lock:
            if not self.connected: raise ConnectionError("Arduino chưa kết nối")
            if epoch != self.epoch: raise ValueError("Lệnh cho phép đã cũ")
            if not self.stopped: return self.snapshot()
            if self.busy: raise ValueError("Đang hủy tư thế; chờ một chút")
            self._send("RESUME")
            self.stopped = False
            self.epoch += 1
            self.gift.initialize_closed()
            self.event("Cho phép điều khiển · thiết lập tìm rác phía trước")
            self._start_job(self._front_setup(), "SEARCH · khởi tạo mặc định")
            return self.snapshot()

    def _front_setup(self):
        # Initialize bin/lid as configured, arm always follows the SEARCH preset.
        targets = copy.deepcopy(self.config["poses"]["SETUP"])
        targets.update(self.config["poses"]["SEARCH"])
        return targets

    def _fault(self, text):
        self.gift.cancel(text)
        self.auto_owner = None
        self.cancel.set()
        self.connected = False
        self.stopped = True
        self.auto_rear = False
        self.rear_pending = False
        self.rear_handling = False
        self.rear_armed = False
        self.rear_clear_since = None
        self.drive_until = 0.
        self.drive_vector = None
        self.epoch += 1
        self.hw.close()
        self.job = text
        self.event(text)

    def snapshot(self):
        with self.lock:
            age = time.monotonic()-self.updated if self.updated else None
            fresh = self.connected and age is not None and age < 1.5
            return {"connected": self.connected, "fresh": fresh, "simulation": self.simulation,
                    "stopped": self.stopped, "epoch": self.epoch, "busy": self.busy,
                    "auto_rear": self.auto_rear, "job": self.job, "progress": self.progress,
                    "rear_pending": self.rear_pending, "rear_handling": self.rear_handling,
                    "manual_mechanics_mode": self.manual_mechanics_mode,
                    "auto_running": self.auto_owner is not None,
                    "drive": {"protocol": DRIVE_PROTOCOL, "default_speed": DEFAULT_SPEED,
                              "lease_ms": DRIVE_LEASE_MS, "active": bool(self.drive_until)},
                    "age_ms": round(age*1000) if age is not None else None,
                    "hardware": copy.deepcopy(self.last_hw) if fresh else {},
                    "gift": self.gift.snapshot(), "command_trace": list(self.command_trace), "log": list(self.log), "config": copy.deepcopy(self.config)}

    def save_pose(self, name, values, epoch):
        with self.lock:
            if self.auto_owner: raise ValueError("Dừng AUTO trước khi lưu tư thế")
            self.check(epoch)
            if name not in self.config["poses"] or not values or set(values)-set(VISIBLE):
                raise ValueError("Chỉ lưu 3 khớp cánh tay có thanh kéo vào tư thế đã định")
            if self.path.suffix == ".py":
                raise ValueError("Sửa tư thế trong robot_params.py rồi chạy lại Python")
            c = copy.deepcopy(self.config)
            # Preserve original D2/D3 actions and insertion order.
            c["poses"][name].update(values)
            validate_config(c)
            temp = self.path.with_suffix(".tmp")
            with temp.open("w",encoding="utf-8") as f:
                json.dump(c,f,ensure_ascii=False,indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(temp,self.path)
            self.config = c
            self.event("Đã lưu tư thế " + name + " vào Python")
            return self.snapshot()

    def command(self, command, epoch, joint=None, pulse_us=None, speed=None, _owner=None):
        if command == "STOP": return self.stop()
        if command == "RESUME": return self.resume(epoch)
        if command == "GIFT DISPENSE":
            if joint is not None or pulse_us is not None or speed is not None:
                raise ValueError("Đổi quà không nhận khớp/xung từ giao diện; sửa gift_params.py")
            return self.gift.start(epoch)
        with self.lock:
            if (_owner is None and not self.manual_mechanics_mode
                    and (self.rear_pending or self.rear_handling)
                    and command not in ('XM', 'AUTO REAR ON', 'AUTO REAR OFF')):
                raise ValueError('D18 đang ưu tiên phía sau; tạm dừng thao tác MANUAL. STOP luôn khả dụng')
            if self.auto_owner and _owner != self.auto_owner and command != "XM":
                raise ValueError("AUTO đang điều khiển; dừng AUTO trước khi điều khiển tay/bánh")
            if _owner is not None and self.auto_owner != _owner:
                raise ValueError("Chu trình AUTO đã bị hủy")
            if command == "XM":
                if not self.connected: raise ConnectionError("Arduino chưa kết nối")
                # Invalidates every earlier drive request, even one still in HTTPS transit.
                self.epoch += 1
                self.drive_until = 0.
                self.drive_vector = None
                self._send("XM")
                self.last_hw = self._send("STATUS")
                self.updated = time.monotonic()
                return self.snapshot()
            mode = command[6:] if isinstance(command, str) and command.startswith("DRIVE ") else None
            is_drive = mode in DRIVE_AXES
            self.check(epoch, allow_drive=is_drive)
            if speed is not None and not is_drive:
                raise ValueError("speed chỉ dùng cho lệnh DRIVE")
            if is_drive:
                if (self.rear_pending or self.rear_handling) and not self.manual_mechanics_mode and (_owner is None or _owner != self.auto_owner):
                    raise ValueError("D18 đang ưu tiên phía sau, tạm dừng bánh")
                power = DEFAULT_SPEED if speed is None else speed
                axes = DRIVE_AXES[mode]
                next_motors = mix_mecanum(*axes, power)
                vector = (*axes, power)
                if self.drive_vector != vector:
                    self._send("XM")
                    time.sleep(.02)  # De-energize the bridge before reversing a wheel.
                self._send(f"C {axes[0]} {axes[1]} {axes[2]} {power} {DRIVE_LEASE_MS}")
                if self.drive_vector != vector:
                    self.event(f"{command} · PWM {power} · M1..M4 {next_motors}")
                self.drive_vector = vector
                self.drive_until = time.monotonic()+DRIVE_LEASE_MS/1000
                self.last_hw = self._send("STATUS")
                self.updated = time.monotonic()
                return self.snapshot()
            c = self.config
            if joint is not None:
                if joint not in VISIBLE + ("d2",) or type(pulse_us)!=int or not c["limits"][joint][0]<=pulse_us<=c["limits"][joint][1]:
                    raise ValueError("Khớp/xung không hợp lệ")
                targets = {joint:pulse_us}
                name = joint.upper()+" → "+str(pulse_us)+" µs"
            elif command == "SETUP ALL": targets, name = c["poses"]["SETUP"], "SETUP"
            elif command in {"POSE SEARCH","POSE PLACE","POSE REAR"}:
                name=command.split()[1]; targets=c["poses"][name]
            elif command in {"GRIP OPEN","GRIP CLOSE"}:
                targets={"d3":c["grip"][command.split()[1].lower()]}; name=command
            elif command in {"LID OPEN","LID CLOSE"}:
                targets={"p4":c["lid"][command.split()[1].lower()]}; name=command
            elif command and command.startswith("SORT ") and command[5:] in c["bins"]:
                targets={"p9":c["bins"][command[5:]]}; name=command
            elif command in {"AUTO REAR ON","AUTO REAR OFF"}:
                if self.manual_mechanics_mode and command == "AUTO REAR ON":
                    raise ValueError('Đang MANUAL cơ khí: D18 chỉ hiển thị, không tự giành quyền điều khiển. Chuyển AUTO để bật phân loại D18')
                self.rear_manual_opt_out = command.endswith("OFF")
                self.enable_rear(command.endswith("ON"))
                self.event(command)
                return self.snapshot()
            else: raise ValueError("Lệnh không được hỗ trợ")
            self._start_job(copy.deepcopy(targets), name)
            return self.snapshot()

    def smooth_auto_drive(self, mode, epoch, power, equivalent_ms, ramp_cap_ms, token):
        with self.lock:
            if not token or self.auto_owner!=token:raise ValueError('AUTO không có quyền chạy bánh')
            self.check(epoch)
            if self.last_hw.get('smooth_protocol')!=SMOOTH_PROTOCOL:
                raise ValueError('Cần nạp firmware V3.9 hỗ trợ nhịp mềm trước khi START AUTO')
            if mode not in DRIVE_AXES:raise ValueError('Sai hướng bánh')
            axes=DRIVE_AXES[mode];mix_mecanum(*axes,power)
            ramp,hold,total=smooth_timing(equivalent_ms,ramp_cap_ms)
            self._send('XM')
            try:self._send(f'G {axes[0]} {axes[1]} {axes[2]} {power} {ramp} {hold}')
            except ValueError:
                # Sensor may activate between Python's last observation and G.
                self.last_hw=self._send('STATUS');self.updated=time.monotonic()
                self._observe_rear(self.last_hw.get('sensors',{}).get('D18'))
                raise
            self.drive_until=time.monotonic()+total/1000
            self.drive_vector=(*axes,power,'smooth')
            self.last_hw=self._send('STATUS');self.updated=time.monotonic()
            self.event(f'AUTO {mode} · PWM {power} · ramp {ramp}/{hold}/{ramp} ms')
            return total

    def tof_guard(self, enabled, token, minimum=200, maximum=250, *, ignore_invalid=False):
        """Arm a firmware wheel interlock, while retaining the current AUTO owner."""
        with self.lock:
            if not token or self.auto_owner != token:
                raise ValueError('AUTO không có quyền cấu hình VL53')
            self.check(self.epoch)
            if self.last_hw.get('tof_protocol') != 1:
                raise ValueError('Cần firmware V3.14 hỗ trợ VL53')
            if type(enabled) is not bool:
                raise ValueError('enabled phải boolean')
            if enabled and (type(minimum) is not int or type(maximum) is not int
                            or not 100 <= minimum < maximum <= 1000):
                raise ValueError('Sai cửa sổ VL53')
            if type(ignore_invalid) is not bool:
                raise ValueError('ignore_invalid phải boolean')
            if enabled and ignore_invalid and self.last_hw.get('tof_auto_continue') is not True:
                raise ValueError('Cần nạp firmware V3.16.4 để AUTO tiếp tục khi VL53 chưa có số đo')
            if enabled and ignore_invalid and self.last_hw.get('tof', {}).get('roi_size_configured') != 10:
                raise ValueError('Cần nạp firmware V3.16.11 có ROI 10x10 trước khi chạy AUTO')
            command = f'TF AUTO {minimum} {maximum}' if ignore_invalid else f'TF {minimum} {maximum}'
            self._send(command if enabled else 'TF OFF')
            self.drive_until = 0.
            self.drive_vector = None
            self.last_hw = self._send('STATUS')
            self.updated = time.monotonic()

    def claim_auto(self, token):
        with self.lock:
            self.check(self.epoch)
            if self.auto_owner: raise ValueError("AUTO đã chạy")
            self.auto_owner = token

    def set_manual_mechanics_mode(self, enabled):
        """Allow manual wheel/servo commands while D18 is LOW, without D18 preemption.

        Call only after the AUTO writer has been detached. The physical STOP,
        USB watchdog and firmware VL53 interlock are NOT affected.
        """
        with self.lock:
            if type(enabled) is not bool:
                raise ValueError('MANUAL mode phải là boolean')
            if enabled and self.auto_owner is not None:
                raise ValueError('Dừng AUTO trước khi cho phép MANUAL bỏ qua D18')
            self.manual_mechanics_mode = enabled
            if enabled:
                self.enable_rear(False)  # Clear queued D18 work before manual ownership.
                self.rear_pending = False
                self.rear_armed = False
                self.rear_clear_since = None
            self.event('MANUAL cơ khí: D18 chỉ báo cảm biến, không phanh bánh' if enabled
                       else 'AUTO cơ khí: cho phép D18 ưu tiên xử lý phía sau')

    def enable_rear(self, enabled):
        """Arm the D18 event queue. Only AutoController executes the full sequence."""
        with self.lock:
            if enabled == self.auto_rear: return
            self.auto_rear = enabled
            self.rear_pending = False
            self.rear_armed = enabled  # An object already present also gets one turn.
            self.rear_clear_since = None
            if enabled: self._observe_rear(self.last_hw.get("sensors",{}).get("D18"))

    def _observe_rear(self, level):
        # A manual operator retains control of wheel and servos, even with D18 LOW.
        # Reading and displaying D18 still works via STATUS. AUTO uses the normal guard.
        if self.manual_mechanics_mode: return
        if not self.auto_rear or self.stopped or self.rear_handling: return
        if level == 1:
            if self.rear_clear_since is None: self.rear_clear_since = time.monotonic()
            if time.monotonic()-self.rear_clear_since >= .5: self.rear_armed = True
        elif level == 0:
            self.rear_clear_since = None
            if self.rear_armed:
                self.rear_armed = False
                self.rear_pending = True
                # Wheel safety takes priority, including during MANUAL drive.
                # Do not cancel a servo job mid-motion: service rear after it stops.
                if self.drive_until or self.drive_vector is not None:
                    try: self._send('XM')
                    finally:
                        self.drive_until = 0.
                        self.drive_vector = None
                self.event("D18 phát hiện vật · phanh bánh, ưu tiên xử lý phía sau")
        else:
            self.rear_clear_since = None

    def rear_requested(self):
        with self.lock:
            return self.auto_rear and self.rear_pending and not self.stopped

    def take_rear_request(self, token):
        with self.lock:
            if self.auto_owner != token: raise ValueError("Chu trình AUTO đã bị hủy")
            if not self.rear_requested(): return False
            self.rear_pending = False
            self.rear_handling = True
            return True

    def finish_rear_request(self, token):
        with self.lock:
            if self.auto_owner != token: return
            self.rear_handling = False
            self.rear_armed = False
            self.rear_clear_since = None

    def auto_targets(self, targets, name, token, step_us=None, step_ms=None):
        with self.lock:
            if token != self.auto_owner: raise ValueError("Chu trình AUTO đã bị hủy")
            self.check(self.epoch)
            if not targets or set(targets)-set(JOINTS): raise ValueError("Sai khớp AUTO")
            for j,v in targets.items():
                if type(v) != int or not self.config["limits"][j][0] <= v <= self.config["limits"][j][1]:
                    raise ValueError("Xung AUTO ngoài giới hạn: "+j)
            if step_us is not None and (type(step_us) is not int or not 1<=step_us<=100):raise ValueError('Sai bước AUTO mềm')
            if step_ms is not None and (type(step_ms) is not int or not 20<=step_ms<=200):raise ValueError('Sai nhịp servo AUTO')
            self._start_job(copy.deepcopy(targets), name,step_us,step_ms)

    def ensure_d19_guard_for_auto(self):
        """Reset a previously disabled IG mode before AUTO without bypassing checks.

        Only a firmware that reports ir_drive_guard and acknowledges IG 1 may
        proceed. Brakes first; STOP, D18, ToF and watchdog are unchanged.
        """
        with self.lock:
            if not self.connected or self.stopped or self.auto_owner or self.busy:
                raise ValueError('Không thể khôi phục IG khi robot đang STOP/AUTO hoặc tay đang chạy')
            if self.last_hw.get('ir_drive_guard') is None:
                raise ValueError('Firmware Arduino không báo IG trong STATUS; cần nạp robot_control_v3.ino trong ZIP')
            if self.drive_until and time.monotonic() < self.drive_until:
                raise ValueError('Phải dừng bánh trước khi khôi phục IG')
            self._send('XM')
            self.drive_until=0.
            self.drive_vector=None
            self._send('IG 1')
            self.last_hw=self._send('STATUS')
            self.updated=time.monotonic()
            if self.last_hw.get('ir_drive_guard') is not True:
                raise ValueError('Arduino không xác nhận IG 1; cần nạp firmware mới')
            self.event('Đã khôi phục chặn D19 ở firmware; AUTO có thể chuyển sang IG 0 khi tiếp cận')

    def set_auto_ir_drive_guard(self, enabled, token):
        """Gate D19 only during the final grasp; firmware must explicitly support IG.

        D18, VL53, STOP and watchdog are independent of this setting.
        Mode switches brake wheels in the firmware; never switch mid-drive.
        """
        with self.lock:
            if not token or token != self.auto_owner or self.stopped:
                raise ValueError('AUTO không có quyền cấu hình D19')
            if self.busy or self.drive_until and time.monotonic() < self.drive_until:
                raise ValueError('Chỉ đổi chế độ IR khi bánh/tay đã dừng')
            if type(enabled) is not bool:
                raise ValueError('Chế độ IR phải là boolean')
            self._send('IG 1' if enabled else 'IG 0')
            self.last_hw=self._send('STATUS');self.updated=time.monotonic()
            if self.last_hw.get('ir_drive_guard') is not enabled:
                raise ValueError('Firmware chưa xác nhận chế độ IG; hãy nạp bản V3.16.23')
            self.event('D19 chặn bánh ở giai đoạn gắp' if enabled else 'D19 bỏ qua lúc tìm/căn/tiến, vẫn giữ D18/VL53/STOP')

    def pause_auto_arm_for_drop(self, token):
        """Soft-cancel a running servo job while retaining AUTO and joint PWM pulses.

        Unlike STOP this does not detach the servos or invalidate AUTO ownership.
        Join outside Brain.lock so a worker waiting for it cannot deadlock.
        """
        with self.lock:
            if token != self.auto_owner or self.stopped or not self.connected:
                raise ValueError('Không có quyền dừng tay khi nghi rơi vật')
            self.cancel.set()
            worker=self.worker
            self._send('XM')
            self.drive_until=0.
            self.drive_vector=None
        if worker and worker is not threading.current_thread():
            worker.join(timeout=3.0)
            if worker.is_alive():
                self.stop()
                raise ValueError('Không dừng được tay khi D19 mất vật; STOP khẩn cấp')
        with self.lock:
            if self.busy:
                self.stop()
                raise ValueError('Tay chưa dừng sau khi hủy lệnh; STOP khẩn cấp')
            self.last_hw=self._send('STATUS');self.updated=time.monotonic()
            self.event('AUTO tạm giữ xung servo để xác minh D19')
            return self.snapshot()

    def release_auto(self, token):
        with self.lock:
            if self.auto_owner == token:
                if self.connected and not self.stopped:
                    self._send('XM')
                    self._send('IG 1')
                self.auto_owner = None

    def hold_auto(self, token, reason):
        """Stop AUTO at its current pulses; never resume or move on a stale token."""
        with self.lock:
            if self.auto_owner != token or self.stopped or not self.connected:
                raise ValueError('Chu trình đã bị dừng; không khôi phục giữ tay')
            if self.busy: raise ValueError('Tay chưa đứng yên để giữ vị trí')
            self._send('XM')
            if self.last_hw.get('ir_drive_guard') is not None:
                self._send('IG 1')
            self.drive_until = 0.
            self.drive_vector = None
            self.enable_rear(False)
            self.auto_owner = None
            self.epoch += 1
            self.job = 'AUTO giữ vị trí · '+reason
            self.event(self.job)
            self.last_hw = self._send('STATUS')
            self.updated = time.monotonic()
            return self.snapshot()

    def _start_job(self, targets, name, step_us=None, step_ms=None):
        if self.busy: raise ValueError("Tư thế đang chạy")
        self._send("XM")
        self.cancel = threading.Event()
        cancel = self.cancel
        self.busy = True
        self.job = name
        self.progress = 0
        self.event("Bắt đầu " + name)
        self.worker=threading.Thread(target=self._run,args=(targets,name,cancel,step_us,step_ms),daemon=True)
        self.worker.start()

    def _run(self, targets, name, cancel, step_us=None, step_ms=None):
        try:
            # One joint at a time, using reported commanded pulses, not physical feedback.
            for index,(joint,target) in enumerate(targets.items()):
                with self.lock:
                    current=self.last_hw.get("joints",{}).get(joint)
                    step=self.config["step_us"] if step_us is None else step_us
                    interval=(self.config["step_ms"] if step_ms is None else step_ms)/1000
                    settle=self.config["settle_ms"]/1000
                pulses=[]
                if current is not None:
                    delta=step if target>=current else -step
                    pulses=list(range(current+delta,target,delta))
                pulses.append(target)
                for value in pulses:
                    with self.lock:
                        if cancel.is_set() or self.stopped or not self.connected: return
                        wire=("P " if joint[0]=="p" else "S ")+joint[1:]+" "+str(value)
                        self._send(wire)
                        self.last_hw.setdefault("joints",{})[joint]=value
                    if cancel.wait(interval): return
                with self.lock: self.progress=round((index+1)/len(targets)*100)
                if cancel.wait(settle): return
            with self.lock:
                if not cancel.is_set():
                    self.job=name+" · hoàn tất"
                    self.event(self.job)
        except (ValueError,ConnectionError) as exc:
            with self.lock:
                self.stop()
                self.job="Lỗi tư thế: "+str(exc)
                self.event(self.job)
        finally:
            with self.lock:
                self.busy=False
                self.motion_finished_at=time.monotonic()

    def _monitor(self):
        while not self.shutdown.wait(.2):
            with self.lock:
                if not self.connected: continue
                try:
                    if self.drive_until and time.monotonic()>=self.drive_until:
                        self._send("XM"); self.drive_until=0.; self.drive_vector=None
                        self.epoch += 1
                    self.last_hw=self._send("STATUS")
                    self.updated=time.monotonic()
                    if self.last_hw.get("stopped") and not self.stopped:
                        self.stop(); self.event("Arduino tự khóa: mất heartbeat hoặc reset")
                        continue
                    # Queue only. Never race a second arm writer against AUTO/manual.
                    self._observe_rear(self.last_hw.get("sensors",{}).get("D18"))
                except (ValueError,ConnectionError) as exc:
                    self._fault("Mất USB: "+str(exc))

    def close(self):
        self.shutdown.set()
        self.stop()
        if self.worker: self.worker.join(timeout=2)
        if self.gift.worker: self.gift.worker.join(timeout=2)
        if self.monitor: self.monitor.join(timeout=2)
        self.hw.close()
