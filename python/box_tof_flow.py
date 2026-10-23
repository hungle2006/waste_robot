"""V3.16.25: camera-locked ToF approach with angle-based split S2/S1 lowering.

Angle >=10deg uses S2-first at SEARCH, then ToF-conditioned D2/S1 grasp.
Angle within deadband keeps historical paired-arm path. Short ToF-verified
reverse pulses recover from distance overshoot; bounded safety holds remain.
"""
import asyncio
import copy
import math
import time

from arm_path import targets_at, pulse_steps
from mecanum import smooth_timing
from pose_profile import reference_at


DEFAULTS = {
    'enabled': False, 'require_visible_target': False, 'handoff_p1_us': 2050, 'handoff_p10_us': 727,
    'step_us': 50, 'missing_frames': 3, 'retry_ms': 200,
    'edge_margin_norm': 8 / 360, 'max_recovery_actions': 6,
    'max_forward_steps': None, 'max_lateral_steps': 25, 'max_arm_steps': 20,
    'track_min_iou': .08, 'track_max_shift_norm': .35,
    'track_min_area_ratio': .12, 'track_max_area_ratio': 8.,
    'tof_min_mm': 200, 'tof_max_mm': 250, 'tof_confirm_samples': 3,
    'tof_max_age_ms': 250, 'tof_fault_timeout_ms': 6000,
    'tof_approach_timeout_ms': 45000, 'max_tof_pulses': None,
    'slow_pulse_ms': 160, 'slow_settle_ms': 250,
    'final_pulse_ms': 160, 'max_final_pulses': None,
    'final_timeout_ms': 15000, 'final_settle_ms': 250, 'ir_confirm_ms': 300,
    # Reverse is strictly distance-controlled: 200ms ACTUAL including ramps.
    'overshoot_back_actual_ms': 200, 'overshoot_back_max_steps': 10,
    'overshoot_min_progress_mm': 3,
    'split_s2_first_on_rotation': True,
}


def nominal_for_actual(actual_ms, ramp_cap_ms):
    for nominal in range(50, 401):
        if smooth_timing(nominal, ramp_cap_ms)[2] == actual_ms:
            return nominal
    raise ValueError('Không tạo được nhịp thực thi ' + str(actual_ms) + ' ms')


def checked_tof_flow(values, config):
    if values is None:
        values = {}
    if not isinstance(values, dict) or set(values) - set(DEFAULTS):
        raise ValueError('Sai box_tof_flow')
    c = dict(DEFAULTS, **values)
    if type(c['enabled']) is not bool:
        raise ValueError('box_tof_flow.enabled phải boolean')
    if type(c['require_visible_target']) is not bool:
        raise ValueError('require_visible_target phải boolean')
    if type(c['split_s2_first_on_rotation']) is not bool:
        raise ValueError('split_s2_first_on_rotation phải boolean')
    if not c['enabled']:
        return c
    if (not config.get('arm_path') or not config['pose_profile']['enabled']
            or not config['motion']['enabled'] or not config['target_lock']['enabled']):
        raise ValueError('box_tof_flow cần arm_path + pose_profile + motion + target_lock')
    if config.get('distance', {}).get('enabled'):
        raise ValueError('Luồng VL53 phải tắt distance sàn')
    if (config['calibration']['active_level'] != 0
            or config['calibration']['contact_sensor'] != 'D19'
            or config['calibration']['hold_sensor'] != 'D19'):
        raise ValueError('Luồng VL53 cần D19 active LOW trước/sau nâng')
    if config['calibration']['positive_error_x_move'] not in ('LEFT', 'RIGHT'):
        raise ValueError('Luồng VL53 chỉ căn ngang LEFT/RIGHT')
    ints = {
        'handoff_p1_us': (1350, 2250), 'handoff_p10_us': (500, 2500),
        'step_us': (5, 50), 'missing_frames': (3, 3), 'retry_ms': (100, 2000),
        'max_recovery_actions': (1, 10), 'max_forward_steps': (1, 100),
        'max_lateral_steps': (1, 50), 'max_arm_steps': (1, 50),
        'tof_min_mm': (100, 800), 'tof_max_mm': (150, 1000),
        'tof_confirm_samples': (3, 10), 'tof_max_age_ms': (100, 250),
        'tof_fault_timeout_ms': (1000, 15000), 'tof_approach_timeout_ms': (5000, 90000),
        'max_tof_pulses': (1, 60), 'slow_pulse_ms': (100, 400),
        'slow_settle_ms': (200, 2000), 'final_pulse_ms': (100, 400),
        'max_final_pulses': (1, 20), 'final_timeout_ms': (1000, 30000),
        'final_settle_ms': (200, 2000), 'ir_confirm_ms': (200, 1500),
        'overshoot_back_actual_ms': (100, 400),
        'overshoot_back_max_steps': (1, 20),
        'overshoot_min_progress_mm': (1, 30),
    }
    for key, (lo, hi) in ints.items():
        if key in ('max_forward_steps', 'max_tof_pulses', 'max_final_pulses') and c[key] is None:
            continue  # Unlimited forward count; sensor and timeout gates remain active.
        if type(c[key]) is not int or not lo <= c[key] <= hi:
            raise ValueError('Sai box_tof_flow.' + key)
    for key, lo, hi in (
        ('edge_margin_norm', .005, .05), ('track_min_iou', .05, .3),
        ('track_max_shift_norm', .1, .4), ('track_min_area_ratio', .1, .5),
        ('track_max_area_ratio', 3., 10.),
    ):
        v = c[key]
        if type(v) not in (int, float) or not math.isfinite(v) or not lo <= v <= hi:
            raise ValueError('Sai box_tof_flow.' + key)
    path = config['arm_path']
    if not path['start']['p1'] < c['handoff_p1_us'] < path['end']['p1']:
        raise ValueError('Mốc tắt camera phải nằm trong quỹ đạo')
    if targets_at(path, c['handoff_p1_us'])['p10'] != c['handoff_p10_us']:
        raise ValueError('Mốc 2050/727 không khớp quỹ đạo P1/P10')
    if c['handoff_p1_us'] >= config['pose_profile']['max_wheel_p1_us']:
        raise ValueError('Mốc VL53 phải nằm trước giới hạn bánh thường')
    if c['tof_min_mm'] >= c['tof_max_mm']:
        raise ValueError('tof_min_mm phải nhỏ hơn tof_max_mm')
    for key in ('slow_pulse_ms', 'final_pulse_ms', 'overshoot_back_actual_ms'):
        nominal_for_actual(c[key], config['motion']['ramp_ms'])
    return c


def vertical_edge(box, margin):
    """Suspected frame truncation only. Never claims physical occlusion."""
    top, bottom = box[1] <= margin, box[3] >= 1 - margin
    return 'top+bottom' if top and bottom else 'top' if top else 'bottom' if bottom else None


TOF_STATUS_NAMES = {
    0: 'RangeValid', 1: 'SigmaFail', 2: 'SignalFail',
    3: 'RangeValidMinRangeClipped', 4: 'OutOfBoundsFail', 5: 'HardwareFail',
    6: 'RangeValidNoWrapCheckFail', 7: 'WrapTargetFail', 9: 'XtalkSignalFail',
    10: 'SynchronizationInt', 13: 'MinRangeFail', 255: 'None',
}


def tof_diagnostic(snapshot):
    t = snapshot.get('hardware', {}).get('tof', {})
    return {key: t.get(key) for key in ('present', 'valid', 'status', 'mm', 'seq', 'age_ms')}


def tof_error(snapshot):
    t = tof_diagnostic(snapshot)
    status = t['status']
    name = TOF_STATUS_NAMES.get(status, 'Unknown') if type(status) is int else 'Unknown'
    return (f"valid={t['valid']}, status={status} ({name}), mm={t['mm']}, "
            f"seq={t['seq']}, age_ms={t['age_ms']}, present={t['present']}")


def tof_sample(snapshot, cfg, *, check_age=True):
    hw = snapshot.get('hardware', {})
    t = hw.get('tof', {})
    if hw.get('tof_protocol') != 1 or t.get('present') is not True:
        raise ValueError('Chưa có firmware/VL53L1X: ' + tof_error(snapshot))
    age, status_age = t.get('age_ms'), snapshot.get('age_ms')
    if check_age and (type(age) is not int or age < 0 or type(status_age) not in (int, float)
            or status_age < 0 or age + status_age > cfg['tof_max_age_ms']):
        raise ValueError('Mẫu VL53 quá cũ/chưa có mẫu mới')
    mm, seq = t.get('mm'), t.get('seq')
    if (t.get('valid') is not True or type(t.get('status')) is not int or t['status'] != 0
            or type(mm) is not int or not 1 <= mm <= 4000
            or type(seq) is not int or not 1 <= seq <= 0xFFFFFFFF):
        raise ValueError('Mẫu VL53 không hợp lệ: ' + tof_error(snapshot))
    return seq, mm


class EarlyIRContact(Exception):
    """Brake wheels, finish descent, then verify the current IR signal."""


class BoxToFFlow:
    def __init__(self, owner):
        self.a = owner
        self.c = owner.cfg['box_tof_flow']
        self.path = owner.cfg['arm_path']
        self.last_action = None
        self.last_obj = None
        self.counts = {'FORWARD': 0, 'lateral': 0, 'LOWER': 0}
        self.recoveries = 0
        self.stage = 'box_follow'
        self.tof_guard_armed = False
        self.tof_confirmed = False
        self.tof_confirmed_at = None
        self.tof_pose = None
        self.early_ir_at = None
        self.rotated_wrist_us = None
        # Only for detected |angle| >= 10°: S2 reaches 2310 while S1 waits
        # at its SEARCH pulse. Never infer a camera pose using this split pose.
        self.pre_s2_only = False

    def range_sample(self, snapshot):
        """AUTO uses valid distance, not the age of the last ToF conversion."""
        return tof_sample(snapshot, self.c, check_age=False)

    def stationary_pose(self, p1=None):
        from auto_controller import AutoFault
        s = self.a.guard()
        hw = s['hardware']
        current = hw['joints'].get('p1')
        if s['busy'] or s['drive']['active'] or any(hw.get('motors', [])):
            raise AutoFault('Tay/bánh chưa dừng trước bước AUTO')
        if type(current) is not int or not self.path['start']['p1'] <= current <= self.path['end']['p1']:
            raise AutoFault('P1 ngoài quỹ đạo đã lưu')
        expected = targets_at(self.path, current)
        if p1 is not None and current != p1:
            raise AutoFault('Tay không đúng tư thế của giai đoạn')
        if self.pre_s2_only:
            if current != self.path['end']['p1']:
                raise AutoFault('S2 chưa ở xung gắp của nhánh xoay')
            expected['p10'] = self.path['start']['p10']
        if self.rotated_wrist_us is not None:
            # Once the wrist turns at VL53 handoff, preserve its angle through every
            # following P1/P10 step and D19 forward pulse; do not reset to 2100.
            expected['d2'] = self.rotated_wrist_us
        if any(hw['joints'].get(j) != v for j, v in expected.items()):
            raise AutoFault('P1/P10/P11/D2 không khớp quỹ đạo/góc D2 đã khóa')
        return s, current

    async def lower(self):
        from auto_controller import NearHold
        a = self.a
        a.check_rear()
        _, p1 = self.stationary_pose()
        target = min(p1 + self.c['step_us'], self.c['handoff_p1_us'])
        if target <= p1:
            raise NearHold('Tay đã tới mốc tắt nhận diện; không hạ theo box nữa')
        self.counts['LOWER'] += 1
        if self.counts['LOWER'] > self.c['max_arm_steps']:
            raise NearHold('Hết số bước hạ tay trước VL53')
        self.last_action = 'LOWER'
        await a.emit('auto_tof_arm', f'Hạ tay P1 {p1}→{target}, P10→{targets_at(self.path, target)["p10"]}; dừng rồi kiểm tra ảnh',
                     phase=self.stage, s2_us=target, p10_us=targets_at(self.path, target)['p10'])
        await a.move_targets(targets_at(self.path, target), 'AUTO hạ tay theo box', gentle=True)
        a.check_rear()

    async def box_drive(self, move):
        from auto_controller import NearHold, AutoFault
        a = self.a
        a.check_rear()
        _, p1 = self.stationary_pose()
        if move not in ('FORWARD', 'LEFT', 'RIGHT'):
            raise AutoFault('Luồng box/VL53 chỉ cho tiến hoặc căn trái/phải')
        if p1 >= self.c['handoff_p1_us']:
            raise AutoFault('Đã tới mốc VL53; không dùng bánh theo box')
        key = 'FORWARD' if move == 'FORWARD' else 'lateral'
        self.counts[key] += 1
        limit = self.c['max_forward_steps'] if key == 'FORWARD' else self.c['max_lateral_steps']
        if limit is not None and self.counts[key] > limit:
            raise NearHold('Hết số nhịp ' + key + '; giữ chờ')
        ref = reference_at(a.cfg['pose_profile'], p1)
        ms = ref['forward_ms'] if key == 'FORWARD' else ref['lateral_ms']
        if key == 'FORWARD' and self.recoveries:
            ms = nominal_for_actual(self.c['slow_pulse_ms'], a.cfg['motion']['ramp_ms'])
        self.last_action = move
        # An arm-only prediction is no longer current after wheel motion.
        a.lock_prediction = None
        await a.emit('auto_tof_box_drive', f'{move} {ms} ms danh định; dừng rồi lấy ảnh mới',
                     phase=self.stage, s2_us=p1, move=move, nominal_move_ms=ms)
        if key == 'FORWARD':
            await self.camera_forward(p1, ms, ref['settle_ms'])
        else:
            await a.motor_step(move, ms, settle_ms=ref['settle_ms'], finish_descent_on_ir=True)

    async def set_tof_guard(self, enabled):
        if self.tof_guard_armed == enabled:
            return
        await asyncio.to_thread(self.a.brain.tof_guard, enabled, self.a.token,
                                self.c['tof_min_mm'], self.c['tof_max_mm'], ignore_invalid=True)
        self.tof_guard_armed = enabled

    async def fresh_forward_sample(self, p1):
        """Missing/invalid ToF does not delay the next finite AUTO pulse."""
        from auto_controller import NearHold
        a = self.a
        a.check_rear()
        await a.fresh_hardware()
        s, _ = self.stationary_pose(p1)
        try:
            return self.range_sample(s)
        except ValueError:
            return None, None

    async def camera_forward(self, p1, nominal_ms, settle_ms):
        """Keep the camera decision, but brake by ToF throughout its pulse."""
        a = self.a
        await self.set_tof_guard(True)
        try:
            _, mm = await self.fresh_forward_sample(p1)
            if mm is not None and mm <= self.c['tof_max_mm']:
                return await self.sensor_approach('forward_range')
            actual = smooth_timing(nominal_ms, a.cfg['motion']['ramp_ms'])[2]
            await self.paused_pulse(p1, actual, tof_gate=True, camera=True)
            await a.wait(settle_ms/1000)
            a.frame_after = time.monotonic()
            _, mm = await self.fresh_forward_sample(p1)
            if mm is not None and mm <= self.c['tof_max_mm']:
                return await self.sensor_approach('forward_range')
        finally:
            if self.stage == 'box_follow' and a.brain.auto_owner == a.token:
                await self.set_tof_guard(False)

    async def forward_range_handoff(self):
        """Valid ToF can interrupt lost/pending vision after FORWARD."""
        if self.last_action != 'FORWARD':
            return
        try:
            _, mm = self.range_sample(self.a.guard())
        except ValueError:
            return
        if mm <= self.c['tof_max_mm']:
            await self.sensor_approach('forward_range')

    async def detect_with_forward_range(self):
        """Cancel a late vision reply before starting the sensor-only phase."""
        await self.forward_range_handoff()
        request = asyncio.create_task(self.a._detect_once())
        try:
            while not request.done():
                self.a.guard()
                self.a.check_rear()
                # Check without entering the arm phase until HTTP is cancelled.
                if self.last_action == 'FORWARD':
                    try:
                        _, mm = self.range_sample(self.a.guard())
                    except ValueError:
                        mm = None
                    if mm is not None and mm <= self.c['tof_max_mm']:
                        request.cancel()
                        await asyncio.gather(request, return_exceptions=True)
                        return await self.sensor_approach('forward_range')
                await asyncio.wait({request}, timeout=.03)
            await self.forward_range_handoff()
            return request.result()
        finally:
            if not request.done():
                request.cancel()
            await asyncio.gather(request, return_exceptions=True)

    async def observe(self):
        """One real matching frame succeeds. Three fresh negatives permit one retry.

        Network/stale-result errors are NOT negative frames and never authorize
        a blind motion. The old box remains a reference, not a current detection.
        """
        from auto_controller import NearHold, StaleDetection, VisionUnavailable
        a = self.a
        missing = failures = 0
        while missing < self.c['missing_frames']:
            self.stationary_pose()
            try:
                obj = await self.detect_with_forward_range()
            except (StaleDetection, VisionUnavailable) as exc:
                failures += 1
                await a.emit('auto_tof_vision_retry', str(exc) + ' · chưa tính là frame mất vật',
                             phase=self.stage, missing_frames=missing)
                if failures >= 3:
                    raise NearHold('Không có ảnh/kết quả mới hợp lệ; không lặp chuyển động') from exc
                await a.wait(self.c['retry_ms'] / 1000)
                continue
            failures = 0
            if obj is not None:
                obj['observed_bin'] = obj['bin']
                obj['bin'] = a.locked_bin
                obj['label'] = a.locked_label
                a.target = copy.deepcopy(obj)
                a.lock_prediction = None
                a.lock_last_seen = time.monotonic()
                self.last_obj = copy.deepcopy(obj)
                self.recoveries = 0
                a.overlay(a.last_candidates, a.last_selected, a.frame_after, a.observed_frame_size, 'LOCK')
                await a.emit('auto_target_observe', f'LOCK #{a.lock_sequence}: cập nhật box từ 1 frame đúng vật',
                             phase=self.stage, target_id=a.lock_sequence, bin=a.locked_bin,
                             target_bbox=obj['bbox'], target_bbox_norm=obj['bbox_norm'],
                             missing_frames=0, vision_mode='locked')
                return obj
            missing += 1
            a.clear_overlay()
            await a.emit('auto_tof_missing', f'{getattr(a, "lock_reason", "Không thấy vật")} · kiểm tra {missing}/{self.c["missing_frames"]} frame; tay/bánh dừng',
                         phase=self.stage, missing_frames=missing, target_bbox=None,
                         target_bbox_norm=None, vision_mode='reacquiring')
            if missing < self.c['missing_frames']:
                await a.wait(self.c['retry_ms'] / 1000)
        return None

    async def run(self):
        from auto_controller import AutoFault, NearHold
        a = self.a
        # V3.16.22: one fresh unambiguous initial detection locks BIN immediately.
        # The first mechanical actions are: closed lid -> rotate BIN -> open lid,
        # while the arm/gripper remain stationary at SEARCH. Orientation is then
        # measured at SEARCH, before any wheel or arm approach. The existing
        # 3-angle agreement (up to 9 retries) is independent of initial class vote.
        obj = await a.detect()
        if obj is None:
            raise NearHold('Chưa khóa vật ban đầu')
        self.last_obj = copy.deepcopy(obj)
        await a.prepare_bin(obj['bin'], open_for_pickup=True)
        # Do NOT proceed without a reliable orientation; D2 only moves after
        # VL53 confirms 20–25 cm, then lower and require D19 before D3 closes.
        try:
            await a.operator.remember_search_orientation(obj)
        except Exception:
            # Orientation failed while safely stationary at SEARCH: do not
            # leave the pre-opened lid exposed during indefinite AUTO HOLD.
            # STOP cancellation remains authoritative (CancelledError is not
            # caught here; it must never send more servo commands).
            if a.front_lid_preopened and a.brain.auto_owner == a.token:
                await a.move_targets({'p4':a.brain.config['lid']['close']},
                                     'AUTO đóng nắp vì chưa xác định được góc SEARCH')
                a.front_lid_preopened = False
            raise
        try:
            if (self.c['split_s2_first_on_rotation']
                    and a.cfg.get('grasp_orientation', {}).get('enabled')
                    and a.search_orientation and not a.search_orientation.get('deadband_applied')
                    and a.search_orientation.get('pulse_us') != self.path['start']['d2']):
                return await self.rotated_s2_first()
            while True:
                a.check_rear()
                await a.fresh_hardware()
                s, p1 = self.stationary_pose()
                if p1 == self.c['handoff_p1_us']:
                    return await self.sensor_approach()
                if p1 > self.c['handoff_p1_us']:
                    raise AutoFault('Tay vượt mốc tắt nhận diện')
                try:
                    _, mm = self.range_sample(s)
                except ValueError:
                    mm = None
                if mm is not None and self.c['tof_min_mm'] <= mm <= self.c['tof_max_mm']:
                    return await self.sensor_approach('distance_ready')
                obj = await self.observe()
                if obj is None:
                    if self.c['require_visible_target']:
                        raise NearHold('Ba ảnh mới chưa thấy đúng vật; giữ tay/bánh, không lặp chuyển động theo box cũ')
                    self.recoveries += 1
                    if self.last_action is None or self.recoveries > self.c['max_recovery_actions']:
                        raise NearHold('Hết số lần thử tiếp khi mất vật; giữ khóa và dừng chờ')
                    await a.emit('auto_tof_repeat', f'Đã kiểm tra 3 frame mất/sai vật; lặp một bước {self.last_action}',
                                 phase=self.stage, recovery_actions=self.recoveries)
                    if self.last_action == 'LOWER':
                        await self.lower()
                    else:
                        await self.box_drive(self.last_action)
                    continue
                ref = reference_at(a.cfg['pose_profile'], p1)
                self.last_obj = copy.deepcopy(obj)
                x1, y1, x2, y2 = obj['bbox_norm']
                ex = (x1 + x2) / 2 - ref['target_x_norm']
                left = x1 <= a.cfg['pose_profile']['side_margin_norm']
                right = x2 >= 1 - a.cfg['pose_profile']['side_margin_norm']
                if left and right:
                    return await self.sensor_approach('both_sides')
                positive = a.cfg['calibration']['positive_error_x_move']
                if left or right:
                    negative = 'LEFT' if positive == 'RIGHT' else 'RIGHT'
                    await self.box_drive(negative if left else positive)
                    continue
                small = (x2-x1 < a.cfg['pose_profile']['min_visible_rw']
                         or y2-y1 < a.cfg['pose_profile']['min_visible_rh'])
                edge = vertical_edge(obj['bbox_norm'], self.c['edge_margin_norm'])
                await a.emit('auto_tof_box_observe', f'Đáy={y2:.1%}; vùng cho phép {ref["target_bottom_norm"]-ref["bottom_tolerance_norm"]:.1%}–{ref["target_bottom_norm"]+ref["bottom_tolerance_norm"]:.1%}',
                             phase=self.stage, s2_us=p1, target_bottom_norm=ref['target_bottom_norm'],
                             bottom_tolerance_norm=ref['bottom_tolerance_norm'],
                             target_x_norm=ref['target_x_norm'], x_error_norm=ex,
                             edge_suspected=edge, occlusion_status='unknown')
                if abs(ex) > ref['x_tolerance_norm']:
                    await self.box_drive(positive if ex > 0 else ('LEFT' if positive == 'RIGHT' else 'RIGHT'))
                elif small or edge:
                    if edge:
                        await a.emit('auto_tof_edge', 'Nghi chạm mép ' + edge + '; tiến một nhịp theo yêu cầu, không kết luận bị che',
                                     phase=self.stage, edge_suspected=edge)
                    else:
                        await a.emit('auto_tof_small', 'Box nhỏ nhưng đúng khóa và đã căn ngang; tiến một nhịp có chặn VL53',
                                     phase=self.stage)
                    await self.box_drive('FORWARD')
                elif y2 < ref['target_bottom_norm'] - ref['bottom_tolerance_norm']:
                    await self.box_drive('FORWARD')
                else:
                    await self.lower()
        except EarlyIRContact:
            return await self.early_ir_handoff()
        finally:
            if self.tof_guard_armed and a.brain.auto_owner == a.token:
                await asyncio.to_thread(a.brain.tof_guard, False, a.token)
                self.tof_guard_armed = False

    async def overshoot_back_to_window(self, *, p1):
        """Back off a confirmed sub-200mm reading; never reverse without valid ToF.

        The firmware ToF brake guards *both* directions, so it is temporarily
        disabled for each verified BACK pulse, then rearmed by the caller's loop.
        D18, STOP, ownership and max reverse count remain mandatory. No rear
        obstacle sensor exists: a clear supervised test area is required.
        """
        from auto_controller import NearHold
        a, c = self.a, self.c
        previous_mm = None
        no_progress = 0
        for step in range(c['overshoot_back_max_steps'] + 1):
            a.check_rear()
            await a.fresh_hardware()
            s, _ = self.stationary_pose(p1)
            try:
                _, mm = self.range_sample(s)
            except ValueError as exc:
                raise NearHold('VL53 không hợp lệ khi chỉnh lùi; giữ bánh dừng') from exc
            if c['tof_min_mm'] <= mm <= c['tof_max_mm']:
                await a.emit('auto_tof_back_ready', f'Lùi chỉnh xong: VL53={mm}mm trong vùng gắp',
                             phase='tof_approach', tof_mm=mm, distance_ready=True)
                return mm
            if mm > c['tof_max_mm']:
                # A 200ms reverse may go slightly beyond 25cm. Return to the
                # normal ToF forward approach instead of HOLD/oscillating BACK.
                await a.emit('auto_tof_back_beyond',
                             f'Lùi tới {mm}mm (>250): dừng BACK, chuyển sang tiến VL53 lại',
                             phase='tof_approach', tof_mm=mm)
                return mm
            if previous_mm is not None:
                if mm - previous_mm < c['overshoot_min_progress_mm']:
                    no_progress += 1
                else:
                    no_progress = 0
                if no_progress >= 2:
                    raise NearHold('Lùi 2 nhịp nhưng VL53 không tăng khoảng cách; giữ tay/bánh')
            if step == c['overshoot_back_max_steps']:
                raise NearHold('Đã lùi đủ giới hạn mà vẫn dưới 20cm; giữ tay/bánh')
            previous_mm = mm
            await a.emit('auto_tof_overshoot_back',
                         f'VL53={mm}mm (<{c["tof_min_mm"]}): BACK thực chạy {c["overshoot_back_actual_ms"]}ms, nhịp {step+1}/{c["overshoot_back_max_steps"]}',
                         phase='tof_approach', tof_mm=mm, reverse_steps=step+1)
            if self.tof_guard_armed:
                await self.set_tof_guard(False)
            nominal = nominal_for_actual(c['overshoot_back_actual_ms'], a.cfg['motion']['ramp_ms'])
            a.base_steps += 1
            try:
                duration = await asyncio.to_thread(a.brain.smooth_auto_drive,
                                'BACK', s['epoch'], a.cfg['control']['speed'],
                                nominal, a.cfg['motion']['ramp_ms'], a.token)
                end = time.monotonic() + duration/1000
                while time.monotonic() < end:
                    a.check_rear()
                    a.guard()
                    await a.wait(min(.02, max(0, end-time.monotonic())))
            finally:
                if a.brain.auto_owner == a.token:
                    await asyncio.to_thread(a.brain.command,'XM',a.brain.epoch,_owner=a.token)
            await a.wait(c['slow_settle_ms']/1000)
            # Firmware guard must not be rearmed until the robot is >250mm or
            # we have finished a short reverse; sensor sample comes next.

    async def rotated_s2_first(self):
        """Angle branch: center in SEARCH, move S2 only, then VL53 drive.

        S1 remains at SEARCH=950us during S2 descent and wheel approach.
        Never use pose-conditioned camera boxes after this split-arm pose.
        """
        from auto_controller import NearHold, AutoFault
        a, c = self.a, self.c
        memory = a.search_orientation
        if not memory or memory.get('deadband_applied') or memory.get('pulse_us') == self.path['start']['d2']:
            raise AutoFault('Không có góc xoay hợp lệ cho nhánh hạ S2 trước')
        if memory.get('target_id') != a.lock_sequence or memory.get('bin') != a.locked_bin:
            raise NearHold('Vật/góc đã khóa không còn khớp')
        # Only make lateral corrections while the camera is still at SEARCH.
        for _ in range(c['max_lateral_steps'] + 1):
            a.check_rear()
            s, p1 = self.stationary_pose(self.path['start']['p1'])
            if p1 != self.path['start']['p1']:
                raise AutoFault('Căn ngang phải ở SEARCH trước khi đưa S2 xuống')
            obj = await self.observe()
            if obj is None:
                raise NearHold('Mất vật khi căn ngang SEARCH; không hạ S2 mù')
            x1, _, x2, _ = obj['bbox_norm']
            ref = reference_at(a.cfg['pose_profile'], p1)
            if x1 <= a.cfg['pose_profile']['side_margin_norm'] and x2 >= 1-a.cfg['pose_profile']['side_margin_norm']:
                raise NearHold('Chai che cả hai mép camera; không thể căn ngang an toàn')
            error = (x1+x2)/2 - ref['target_x_norm']
            if abs(error) <= ref['x_tolerance_norm']:
                break
            positive = a.cfg['calibration']['positive_error_x_move']
            move = positive if error > 0 else ('LEFT' if positive=='RIGHT' else 'RIGHT')
            await self.box_drive(move)
        else:
            raise NearHold('Hết nhịp căn ngang SEARCH trước khi hạ S2')
        # Require working VL53 before pre-positioning a low arm near the floor.
        await a.fresh_hardware()
        s, _ = self.stationary_pose(self.path['start']['p1'])
        try:
            _, mm = self.range_sample(s)
        except ValueError as exc:
            raise NearHold('Không có VL53 hợp lệ: không hạ S2 trước khi chạy bánh') from exc
        if mm < c['tof_min_mm']:
            # Correct the overshoot with arm still up at SEARCH.
            await self.overshoot_back_to_window(p1=self.path['start']['p1'])
        await a.emit('auto_s2_preposition',
                     'Góc >=10°: đưa RIÊNG S2 đến 2310µs; S1 vẫn ở 950µs, D2 chưa xoay',
                     phase='s2_preposition', s2_us=self.path['end']['p1'],
                     p10_us=self.path['start']['p10'])
        for pulse in pulse_steps(self.path['start']['p1'], self.path['end']['p1'], c['step_us']):
            a.check_rear()
            await a.fresh_hardware()
            hw = a.guard()['hardware']['joints']
            if hw.get('p10') != self.path['start']['p10'] or hw.get('d2') != self.path['start']['d2']:
                raise AutoFault('S1/D2 đã thay đổi ngoài ý muốn khi hạ S2 riêng')
            await a.move_targets({'p1':pulse}, 'AUTO góc >=10: chỉ hạ S2, giữ S1 SEARCH', gentle=True)
            # After the first pulse the tuple no longer matches the normal arm path.
            # Enable split-pose validation for the rest of the preposition travel.
            if pulse != self.path['end']['p1']:
                # Verify explicitly without claiming an interpolated S1 command.
                joints = a.guard()['hardware']['joints']
                if joints.get('p1') != pulse or joints.get('p10') != self.path['start']['p10']:
                    raise AutoFault('S2/S1 không đúng lệnh hạ riêng')
        self.pre_s2_only = True
        s, _ = self.stationary_pose(self.path['end']['p1'])
        self.tof_pose = self.path['end']['p1']
        a.vision_paused = True
        a.observed_jpeg = None
        a.clear_overlay()
        self.stage = 'tof_approach'
        await a.emit('auto_s2_wait_distance',
                     'S2=2310µs, S1=950µs; dùng VL53 tiến/lùi tới 20–25cm rồi mới hạ S1',
                     phase='tof_approach', s2_us=self.tof_pose,
                     p10_us=self.path['start']['p10'], vision_mode='tof_only')
        return await self.sensor_distance_loop(self.tof_pose)

    async def sensor_distance_loop(self, p1):
        """One bounded VL53 loop shared by split-S2 and original arm path."""
        from auto_controller import NearHold
        a, c = self.a, self.c
        await self.set_tof_guard(True)
        deadline = time.monotonic() + c['tof_approach_timeout_ms']/1000
        pulses = 0
        while time.monotonic() < deadline:
            a.check_rear()
            await a.fresh_hardware()
            s, _ = self.stationary_pose(p1)
            try:
                _, mm = self.range_sample(s)
            except ValueError:
                mm = None
            if mm is not None and mm < c['tof_min_mm']:
                await self.overshoot_back_to_window(p1=p1)
                await self.set_tof_guard(False)
                continue
            ready = mm is not None and c['tof_min_mm'] <= mm <= c['tof_max_mm']
            await a.emit('auto_tof_observe',
                         f'VL53={mm}mm; '+('đủ khoảng cách gắp' if ready else 'đang tiếp cận') if mm is not None else 'VL53 chưa có số đo hợp lệ; chờ đọc lại',
                         phase='tof_approach', tof_valid=mm is not None,
                         tof_mm=mm, tof_votes=int(ready), distance_cm=mm/10 if mm else None)
            if ready:
                self.tof_confirmed = True
                self.tof_confirmed_at = time.monotonic()
                await a.emit('auto_distance_ready', 'VL53 20–25cm: cho phép xoay D2/hạ S1',
                             phase='tof_approach', distance_ready=True, distance_cm=mm/10, tof_mm=mm)
                await self.set_tof_guard(False)
                return await self.final_approach()
            # Do not move the low split-arm posture without a valid range.
            if mm is None and self.pre_s2_only:
                raise NearHold('S2 đã hạ thấp nhưng VL53 không hợp lệ; không chạy bánh mù')
            if c['max_tof_pulses'] is not None and pulses >= c['max_tof_pulses']:
                raise NearHold('Hết nhịp VL53 trước cửa sổ gắp')
            pulses += 1
            await a.emit('auto_tof_slow_forward',
                         f'FORWARD {c["slow_pulse_ms"]}ms thực thi, nhịp {pulses}',
                         phase='tof_approach', actual_move_ms=c['slow_pulse_ms'])
            if not self.tof_guard_armed:
                await self.set_tof_guard(True)
            await self.paused_pulse(p1, c['slow_pulse_ms'], tof_gate=True)
            await a.wait(c['slow_settle_ms']/1000)
        raise NearHold('Hết thời gian VL53; không gắp')

    async def sensor_approach(self, reason='handoff'):
        from auto_controller import NearHold, AutoFault
        a, c = self.a, self.c
        s, p1 = self.stationary_pose()
        if (p1 > c['handoff_p1_us'] or not self.last_obj or a.selected_bin != a.locked_bin
                or reason not in ('handoff', 'both_sides', 'forward_range', 'distance_ready')):
            raise AutoFault('Chưa đủ khóa vật/tư thế để bàn giao VL53')
        if reason == 'handoff' and p1 != c['handoff_p1_us']:
            raise AutoFault('Chưa tới mốc bàn giao tay2050/727')
        if reason == 'forward_range' and self.last_action != 'FORWARD':
            raise AutoFault('Chưa có hành động tiến để bàn giao VL53 sớm')
        if reason == 'distance_ready':
            try:
                _, mm = self.range_sample(s)
            except ValueError as exc:
                raise AutoFault('Chưa có số đo VL53 hợp lệ để gắp') from exc
            if not c['tof_min_mm'] <= mm <= c['tof_max_mm']:
                raise AutoFault('VL53 chưa nằm trong khoảng cách gắp')
        margin = a.cfg['pose_profile']['side_margin_norm']
        box = self.last_obj['bbox_norm']
        if reason == 'both_sides' and not (box[0] <= margin and box[2] >= 1-margin):
            raise AutoFault('Box chưa chạm cả hai mép để bàn giao VL53 sớm')
        self.tof_pose = p1
        a.vision_paused = True
        a.observed_jpeg = None
        a.clear_overlay()
        self.stage = 'tof_approach'
        why = {'handoff': 'Đạt2050/727', 'both_sides': 'Box chạm cả hai mép trái/phải',
               'forward_range': 'VL53 đã tới ngưỡng khi đang tiến, kể cả mất box',
               'distance_ready': 'VL53 đã đủ khoảng cách tại tư thế hiện tại'}[reason]
        await a.emit('auto_tof_handoff', why + ': dừng nhận diện; giữ tay hiện tại, tiếp cận bằng VL53',
                     phase=self.stage, s2_us=p1, p10_us=targets_at(self.path, p1)['p10'],
                     tof_entry_reason=reason,
                     vision_mode='tof_only', target_bbox=None, target_bbox_norm=None)
        return await self.sensor_distance_loop(p1)

    async def early_ir_handoff(self):
        """Early IR permits descent, but cannot authorize closing the gripper."""
        from auto_controller import AutoFault
        a = self.a
        a.guard();a.check_rear()
        if not self.last_obj or a.locked_bin is None or a.selected_bin != a.locked_bin or a.final_authorized is not None:
            raise AutoFault('Chưa khóa vật/BIN để hạ tay khi IR nhận sớm')
        await asyncio.to_thread(a.brain.command, 'XM', a.brain.epoch, _owner=a.token)
        await a.fresh_hardware()
        _,p1=self.stationary_pose()
        await self.set_tof_guard(False)
        self.tof_pose=p1
        self.early_ir_at=time.monotonic()
        a.vision_paused=True
        a.observed_jpeg=None
        a.clear_overlay()
        self.stage='tof_approach'
        await a.emit('auto_early_ir_lower', 'IR nhận sớm: dừng bánh, hạ hết tới2310/650 rồi đọc lại IR mới',
                     phase=self.stage, grasp_trigger='early_ir', ir_check_enabled=False)
        return await self.final_approach()

    async def rotate_saved_grip(self):
        """Turn D2 while stationary at the confirmed VL53 20–25cm distance.

        The angle was sampled at SEARCH from the locked target. No fresh camera
        inference is made at close range, where the bottle can be cut off.
        """
        from auto_controller import NearHold
        a = self.a
        if not self.tof_confirmed or self.tof_confirmed_at is None:
            raise NearHold('Chưa có VL53 xác nhận khoảng cách; không xoay càng')
        s, p1 = self.stationary_pose(self.tof_pose)
        # Verify that the most recent latched sensor value STILL belongs to
        # the 20–25 cm window immediately before physically moving the wrist.
        try:
            _, mm = self.range_sample(s)
        except ValueError as exc:
            raise NearHold('VL53 không có số đo hợp lệ trước khi xoay D2') from exc
        if not self.c['tof_min_mm'] <= mm <= self.c['tof_max_mm']:
            raise NearHold('VL53 không còn trong khoảng 20–25 cm; không xoay D2')
        if s['hardware']['joints'].get('d3') != a.brain.config['grip']['open']:
            raise NearHold('D3 phải mở trước khi xoay D2')
        memory = a.search_orientation
        if (not isinstance(memory, dict) or memory.get('target_id') != a.lock_sequence
                or memory.get('bin') != a.locked_bin or memory.get('label') != a.locked_label
                or memory.get('source_pose') != self.path['start']
                or memory.get('applied') or a.selected_bin != a.locked_bin):
            raise NearHold('Thiếu góc SEARCH hợp lệ cho đúng vật đã khóa; không xoay D2')
        pulse = memory['pulse_us']
        action = (f'giữ D2 ở giữa, không xoay do |góc| < {memory.get("deadband_deg",10.0):g}°'
                  if memory.get('deadband_applied') else 'xoay D2 theo góc đã lưu')
        await a.emit('auto_rotate_grip',
                     f'VL53 đạt 20–25 cm: {action}; D2={pulse} µs, góc SEARCH {memory["angle_deg"]:.1f}° TRƯỚC khi hạ tay',
                     orientation=dict(memory), phase='tof_approach')
        if s['hardware']['joints'].get('d2') != pulse:
            await a.move_targets({'d2':pulse}, 'AUTO xoay D2 theo góc đã lưu ở SEARCH', gentle=True)
        await a.fresh_hardware()
        if a.guard()['hardware']['joints'].get('d2') != pulse:
            raise NearHold('Không xác nhận được D2 đã xoay đúng xung; không hạ tay')
        self.rotated_wrist_us = pulse
        memory['applied'] = True
        a.state['orientation'] = dict(memory, valid=True)
        self.stationary_pose(p1)

    async def final_approach(self):
        from auto_controller import AutoFault, NearHold, CloseRangeContact
        a, c = self.a, self.c
        if ((not self.tof_confirmed and self.early_ir_at is None) or self.tof_pose is None or self.stage != 'tof_approach' or not a.vision_paused
                or not self.last_obj or a.selected_bin != a.locked_bin or a.final_authorized is not None):
            raise AutoFault('Chưa đủ khóa vật/BIN để chuyển tư thế gắp')
        a.guard()
        if self.tof_confirmed_at is None and self.early_ir_at is None:
            raise AutoFault('Chưa có xác nhận khoảng cách hoặc IR sớm của lượt gắp này')
        # The wrist moves BEFORE final descent, ONLY on a valid VL53 reading.
        # D19 is ignored until the gripper has descended fully.
        if a.cfg.get('grasp_orientation',{}).get('enabled'):
            if not self.tof_confirmed:
                raise NearHold('D19 phát hiện sớm nhưng VL53 chưa xác nhận 20–25 cm; giữ tay, không xoay D2')
            await self.rotate_saved_grip()
        a.final_authorized = {'target_id': a.lock_sequence, 'bin': a.locked_bin, 'mode': 'box_tof'}
        self.stage = 'final_ir'
        await a.emit('auto_tof_final_pose', 'Đã xoay D2 theo góc lưu ở SEARCH; hạ hết tay tới2310/650 rồi kiểm tra D19',
                     phase=self.stage, vision_mode='ir_only', ir_check_enabled=False)
        _, p1 = self.stationary_pose(self.tof_pose)
        if self.pre_s2_only:
            # S2 is already 2310; do NOT move it again. The wrist has been
            # rotated at the confirmed distance, so lower only S1 / P10 now.
            await a.fresh_hardware()
            s, _ = self.stationary_pose(self.path['end']['p1'])
            try:
                _, mm = self.range_sample(s)
            except ValueError as exc:
                raise NearHold('VL53 lỗi trước khi hạ riêng S1') from exc
            if not c['tof_min_mm'] <= mm <= c['tof_max_mm']:
                raise NearHold('VL53 không còn 20–25cm; không hạ S1')
            await a.emit('auto_s1_final_lower',
                         'Đã xoay D2; vùng gắp hợp lệ, chỉ hạ S1 từ 950 về 650µs',
                         phase='final_ir', s2_us=self.path['end']['p1'],
                         p10_us=self.path['end']['p10'])
            await a.move_targets({'p10':self.path['end']['p10']},
                                 'AUTO đủ VL53: chỉ hạ S1 đến 650', gentle=True)
            self.pre_s2_only = False
        for pulse in pulse_steps(p1, self.path['end']['p1'], c['step_us']):
            await a.fresh_hardware()
            a.check_rear()
            # D19 is evaluated only after every final-pose step has completed.
            # STOP, cancellation, D18 and connection checks still apply per step.
            targets = targets_at(self.path, pulse)
            if self.rotated_wrist_us is not None:
                targets['d2'] = self.rotated_wrist_us
            await a.move_targets(targets, 'AUTO hạ tay giữ góc càng D2 đã xoay', gentle=True)
        await a.fresh_hardware()
        self.stationary_pose(self.path['end']['p1'])
        await asyncio.to_thread(a.brain.set_auto_ir_drive_guard, True, a.token)
        await a.emit('auto_tof_ir_wait', 'Đã xuống hết vị trí gắp; bật D19, chưa nhận thì nhích160ms',
                     phase=self.stage, ir_check_enabled=True)
        deadline = time.monotonic() + c['final_timeout_ms']/1000
        since = None
        pulses = 0
        while time.monotonic() < deadline:
            a.check_rear()
            await a.fresh_hardware()
            self.stationary_pose(self.path['end']['p1'])
            if a.sensor('D19'):
                if since is None:
                    since = time.monotonic()
                if time.monotonic()-since >= c['ir_confirm_ms']/1000:
                    await a.emit('auto_tof_ir_ready', 'D19 ổn định tại2310/650; cho phép đóng kẹp', phase=self.stage)
                    raise CloseRangeContact(copy.deepcopy(self.last_obj))
                await a.wait(.03)
                continue
            since = None
            if c['max_final_pulses'] is not None and pulses >= c['max_final_pulses']:
                raise NearHold('Hết nhịp160ms mà D19 chưa nhận; không đóng kẹp')
            pulses += 1
            await a.emit('auto_tof_ir_forward', f'Chờ D19: tiến {c["final_pulse_ms"]}ms, nhịp {pulses}/{c['max_final_pulses'] if c['max_final_pulses'] is not None else '∞'}',
                         phase=self.stage, actual_move_ms=c['final_pulse_ms'])
            await self.paused_pulse(self.path['end']['p1'], c['final_pulse_ms'], tof_gate=False)
            await a.wait(c['final_settle_ms']/1000)
        raise NearHold('Quá thời gian chờ D19; không đóng kẹp')

    async def paused_pulse(self, p1, actual_ms, tof_gate, camera=False):
        """Finite FORWARD only; ToF can brake camera and sensor pulses."""
        from auto_controller import AutoFault, NearHold
        a = self.a
        s, _ = self.stationary_pose(p1)
        a.check_rear()
        if a.selected_bin != a.locked_bin:
            raise AutoFault('Chưa được bàn giao bánh cho cảm biến')
        if camera:
            if (a.vision_paused or self.stage != 'box_follow' or not tof_gate
                    or self.last_action != 'FORWARD' or p1 >= self.c['handoff_p1_us']):
                raise AutoFault('Chưa được phép nhịp tiến theo camera')
        elif not a.vision_paused:
            raise AutoFault('Chưa dừng nhận diện trước nhịp cảm biến')
        if tof_gate:
            if (not camera and self.stage != 'tof_approach') or not self.tof_guard_armed:
                raise AutoFault('Chưa bật chặn bánh VL53')
            gate = s['hardware'].get('tof', {})
            if (gate.get('guard') is not True or gate.get('min_mm') != self.c['tof_min_mm']
                    or gate.get('max_mm') != self.c['tof_max_mm']
                    or gate.get('invalid_policy') != 'continue'):
                raise AutoFault('Firmware không xác nhận đúng chặn bánh VL53')
            try:
                _, mm = self.range_sample(s)
            except ValueError:
                mm = None
            if mm is not None and mm <= self.c['tof_max_mm']:
                return
        elif (self.stage != 'final_ir' or a.final_authorized != {
                'target_id': a.lock_sequence, 'bin': a.locked_bin, 'mode': 'box_tof'}):
            raise AutoFault('Chưa được phép nhịp D19 cuối')
        if not tof_gate and a.sensor('D19'):
            return
        nominal = nominal_for_actual(actual_ms, a.cfg['motion']['ramp_ms'])
        a.base_steps += 1
        a.aligned_anchor = None
        try:
            try:
                duration = await asyncio.to_thread(a.brain.smooth_auto_drive, 'FORWARD', s['epoch'],
                                                  a.cfg['control']['speed'], nominal, a.cfg['motion']['ramp_ms'], a.token)
            except ValueError as exc:
                a.guard()
                a.check_rear()
                if tof_gate and str(exc) == 'TOF_BLOCKS_AUTO_DRIVE':
                    return
                if str(exc) == 'IR_BLOCKS_AUTO_DRIVE' and not tof_gate and a.sensor('D19'):
                    return
                raise
            end = time.monotonic() + duration/1000
            while time.monotonic() < end:
                a.check_rear()
                state = a.guard()
                if not tof_gate and a.sensor('D19'):
                    break
                if tof_gate:
                    try:
                        _, mm = self.range_sample(state)
                    except ValueError:
                        mm = None
                    if mm is not None and mm <= self.c['tof_max_mm']:
                        break
                await a.wait(min(.02, max(0, end-time.monotonic())))
        finally:
            if a.brain.auto_owner == a.token:
                await asyncio.to_thread(a.brain.command, 'XM', a.brain.epoch, _owner=a.token)
