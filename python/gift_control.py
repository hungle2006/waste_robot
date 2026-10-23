"""D4 homes closed on Python connect/resume, then explicit dispense jobs."""
import copy
import math
from pathlib import Path
import threading
import time

from config_io import read_namespace

GIFT_PIN = 4  # Digital D4 on Arduino Mega, independent of PCA9685 P4.
GIFT_PROTOCOL = 3  # Firmware V3.16.11 uses digital D4 and supports 2500 us.
from robot_params import GIFT_CLAIM_SECONDS as GIFT_WINDOW_SECONDS


class GiftCancelled(Exception):
    pass


def load_gift_params(path):
    try:
        values = read_namespace(path)
    except (OSError, SyntaxError, NameError, TypeError) as exc:
        raise ValueError('Không đọc được gift_params.py: ' + str(exc)) from exc
    cfg = {name: values.get(name) for name in
           ('GIFT_OPEN_US', 'GIFT_CLOSE_US', 'GIFT_OPEN_HOLD_MS')}
    for key in ('GIFT_OPEN_US', 'GIFT_CLOSE_US'):
        if type(cfg[key]) is not int or not 500 <= cfg[key] <= 2500:
            raise ValueError(key + ' phải là số nguyên 500–2500 us trong gift_params.py')
    if cfg['GIFT_OPEN_US'] == cfg['GIFT_CLOSE_US']:
        raise ValueError('Xung mở và đóng quà phải khác nhau')
    if type(cfg['GIFT_OPEN_HOLD_MS']) is not int or not 20 <= cfg['GIFT_OPEN_HOLD_MS'] <= 10000:
        raise ValueError('GIFT_OPEN_HOLD_MS phải là số nguyên 20–10000 ms')
    cfg['GIFT_CLOSE_SETTLE_MS'] = values.get('GIFT_CLOSE_SETTLE_MS', 500)
    if type(cfg['GIFT_CLOSE_SETTLE_MS']) is not int or not 20 <= cfg['GIFT_CLOSE_SETTLE_MS'] <= 5000:
        raise ValueError('GIFT_CLOSE_SETTLE_MS phải là số nguyên 20–5000 ms')
    return cfg


class GiftController:
    def __init__(self, brain):
        self.brain = brain
        self.params_path = Path(__file__).with_name('gift_params.py')
        self.cancel_event = threading.Event()
        self.worker = None
        self.claim_until = 0.0
        self.claim_seq = 0
        self.state = dict(protocol=GIFT_PROTOCOL, pin=GIFT_PIN, min_us=500, max_us=2500,
                          busy=False, phase='idle', message='Sẵn sàng nhận lệnh Đổi quà',
                          cycles=0, last_pulse_us=None, eligible=False, remaining_seconds=0,
                          reward_bin=None, reward_sequence=0)
        try:
            self._configuration(load_gift_params(self.params_path))
        except (OSError, ValueError, SyntaxError) as exc:
            self.state.update(phase='error', message=str(exc))

    def _configuration(self, cfg):
        self.state.update(open_us=cfg['GIFT_OPEN_US'], close_us=cfg['GIFT_CLOSE_US'],
                          hold_ms=cfg['GIFT_OPEN_HOLD_MS'])

    def snapshot(self):
        result=copy.deepcopy(self.state)
        result['available']=self.brain.connected and self.brain.last_hw.get('gift_protocol')==GIFT_PROTOCOL
        seconds=max(0., self.claim_until-time.monotonic())
        result['remaining_seconds']=int(math.ceil(seconds))
        result['eligible']=bool(seconds > 0 and not self.state['busy'] and result['available']
                                and not self.brain.stopped)
        if result['phase'] != 'error':
            if not self.brain.connected:
                result['message']='Chưa kết nối Arduino để Đổi quà'
            elif self.brain.last_hw.get('gift_protocol') != GIFT_PROTOCOL:
                result['message']='Cần nạp firmware V3.16.11 hỗ trợ D4 tới 2500 us'
            elif self.brain.stopped:
                result['message']='Robot đang STOP; cho phép điều khiển trước khi Đổi quà'
            elif result['eligible']:
                result['message']=f'Có thể đổi quà · còn {result["remaining_seconds"]} giây'
            elif not self.state['busy']:
                result['message']='Chưa có lượt bỏ rác D18 thành công hoặc quyền đổi quà đã hết hạn'
        return result

    def grant_after_rear(self, bin_number):
        """Called only after D18 classification, drop clear, and lid close succeeded."""
        with self.brain.lock:
            if not self.brain.connected or self.brain.stopped:
                return
            self.claim_seq += 1
            self.claim_until = time.monotonic() + GIFT_WINDOW_SECONDS
            self.state.update(reward_bin=bin_number, reward_sequence=self.claim_seq,
                              message='Đổi quà đã mở trong 120 giây')
            self.brain.event('QUÀ · D18 đã phân loại/nhận rác, cấp 120 giây đổi quà')

    def initialize_closed(self):
        """Called with Brain.lock after explicit connect/RESUME; never move arm joints."""
        b = self.brain
        if b.last_hw.get('gift_protocol') != GIFT_PROTOCOL:
            return  # Older firmware may still control the arm; gift remains unavailable.
        try:
            cfg = load_gift_params(self.params_path)
            value = cfg['GIFT_CLOSE_US']
            b._send(f'S {GIFT_PIN} {value}')
            if b.command_trace:
                b.command_trace[0]['source'] = 'gift'
            b.last_hw = b._send('STATUS')
            b.updated = time.monotonic()
            if b.last_hw.get('stopped') or b.last_hw.get('joints', {}).get('d4') != value:
                raise ValueError('Chưa xác nhận được xung đóng D4')
            self.cancel_event = threading.Event()
            self._configuration(cfg)
            self.state.update(busy=False, phase='idle', last_pulse_us=value,
                              message=f'D4 ở xung đóng {value} us; sẵn sàng Đổi quà')
            b.event(f'QUÀ · khởi tạo D4 → {value} us')
        except Exception as exc:
            b.stop()
            self.state.update(busy=False, phase='error', message='Lỗi khởi tạo quà: ' + str(exc))
            raise ValueError(self.state['message']) from exc

    def cancel(self, reason):
        # Caller holds Brain.lock. STOP/fault disables actual pulses separately.
        self.claim_until = 0.0
        self.cancel_event.set()
        if self.state['busy']:
            self.state.update(busy=False, phase='interrupted', message=reason)

    def start(self, epoch):
        cfg = load_gift_params(self.params_path)  # fresh compile, no stale .pyc
        b = self.brain
        with b.lock:
            if type(epoch) is not int or epoch != b.epoch:
                raise ValueError('Lệnh Đổi quà đã cũ; cập nhật trạng thái rồi nhấn lại')
            if not b.connected or b.stopped or b.shutdown.is_set():
                raise ValueError('Kết nối và cho phép điều khiển trước khi Đổi quà')
            if not b.snapshot()['fresh']:
                raise ValueError('Chưa có trạng thái Arduino mới để Đổi quà')
            if b.last_hw.get('gift_protocol') != GIFT_PROTOCOL:
                raise ValueError('Cần nạp firmware V3.16.11 hỗ trợ D4 tới 2500 us')
            if self.state['busy']:
                raise ValueError('Đang phát quà; chờ chu trình hiện tại hoàn tất')
            if time.monotonic() >= self.claim_until:
                raise ValueError('Chưa nhận rác D18 thành công hoặc đã quá 2 phút; chưa thể Đổi quà')
            self.claim_until = 0.0  # Atomic one-time redemption, even on USB failure.
            self._configuration(cfg)
            cancel = self.cancel_event = threading.Event()
            self.state.update(busy=True, phase='opening', message='Đang mở cửa phát quà')
            self.worker = threading.Thread(target=self._run, args=(cfg, cancel), daemon=True)
            self.worker.start()
            return b.snapshot()

    def _check(self, cancel):
        b = self.brain
        if (cancel.is_set() or cancel is not self.cancel_event or b.stopped
                or not b.connected or b.shutdown.is_set()):
            raise GiftCancelled()

    def _pulse(self, value, cancel):
        b = self.brain
        with b.lock:
            self._check(cancel)
            b._send(f'S {GIFT_PIN} {value}')
            if b.command_trace:
                b.command_trace[0]['source'] = 'gift'
            b.last_hw.setdefault('joints', {})['d4'] = value
            self.state['last_pulse_us'] = value
            b.event(f'QUÀ · D4 → {value} us')

    def _refresh(self, cancel):
        b = self.brain
        with b.lock:
            self._check(cancel)
            b.last_hw = b._send('STATUS')
            b.updated = time.monotonic()
            if b.last_hw.get('stopped'):
                b.stop()
                raise GiftCancelled()
            if b.last_hw.get('gift_protocol') != GIFT_PROTOCOL:
                raise ValueError('Firmware D4 không còn sẵn sàng')

    def _wait(self, milliseconds, cancel):
        deadline = time.monotonic() + milliseconds / 1000
        next_status = 0.
        while time.monotonic() < deadline:
            if cancel.wait(min(.05, max(0., deadline-time.monotonic()))):
                raise GiftCancelled()
            if time.monotonic() >= next_status:
                self._refresh(cancel)
                next_status = time.monotonic() + .2

    def _run(self, cfg, cancel):
        b = self.brain
        try:
            self._pulse(cfg['GIFT_OPEN_US'], cancel)
            self._wait(cfg['GIFT_OPEN_HOLD_MS'], cancel)
            with b.lock:
                self._check(cancel)
                self.state.update(phase='closing', message='Đang trả servo về vị trí đóng')
            self._pulse(cfg['GIFT_CLOSE_US'], cancel)
            self._wait(cfg['GIFT_CLOSE_SETTLE_MS'], cancel)
            self._refresh(cancel)
            with b.lock:
                self._check(cancel)
                self.state.update(phase='done', message='Đã hoàn tất chu trình phát quà',
                                  cycles=self.state['cycles'] + 1)
        except GiftCancelled:
            with b.lock:
                if cancel is self.cancel_event:
                    self.state.update(phase='interrupted', message='Đã ngắt chu trình quà; không tự phát lại')
        except Exception as exc:
            with b.lock:
                # No automatic close/replay after uncertain delivery or a lost connection.
                if cancel is self.cancel_event and not cancel.is_set():
                    if isinstance(exc, ConnectionError):
                        b._fault('Mất USB khi phát quà: ' + str(exc))
                    else:
                        b.stop()
                    self.state.update(phase='error', message='Lỗi phát quà: ' + str(exc))
        finally:
            with b.lock:
                if cancel is self.cancel_event:
                    self.state['busy'] = False
