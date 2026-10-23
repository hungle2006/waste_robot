"""Step-and-observe AUTO: camera boxes, VL53L1X, then final D19 gate.
All hardware writes go through Brain.
"""
import asyncio
import copy
import json
import math
import time
import uuid
from pathlib import Path
from auto_checkpoint import AutoCheckpoint
from httpx import HTTPError
from target_lock import lock_config, associate, associate_rolling, pose_prediction
from box_tof_flow import checked_tof_flow, BoxToFFlow
from approach_progress import ForwardProgress
from auto_motion import MOTION_DEFAULTS,VISIBILITY_DEFAULTS,checked_settings,visible_step
from floor_distance import distance_config, measure_floor
from config_io import read_config
from confirmed_grasp import checked_reference, matches_close_view
from arm_path import validate_path, targets_at, pulse_steps
from pose_profile import (checked_profile,reference_at,view_error,view_problem,
                          wheel_correction,correction_error,search_lens_estimate)


class AutoFault(ValueError):
    pass


class StaleDetection(AutoFault):
    pass


class VisionUnavailable(AutoFault):
    pass


class RearRequested(Exception):
    """Leave the front approach at a completed motion; the gripper is still open."""


class SearchExhausted(Exception):
    """Retained for legacy compatibility; SEARCH is now continuous."""


class LostGrip(Exception):
    """D19 no longer confirms the object; cancel drop-off and recover safely."""


class CloseRangeContact(Exception):
    """Exit every approach loop without another lowering or image alignment step."""
    def __init__(self, obj):
        super().__init__('Đã xác nhận IR ở giai đoạn tiếp cận gần')
        self.obj = obj


class NearHold(AutoFault):
    """Expected close-view limit: keep servo pulses, stop wheels and require user action."""


NEAR_DEFAULTS = {
    'enabled': True, 'stop_rw': .60, 'edge_margin_norm': .01,
    'max_alignment_age_s': 30, 'wait_contact_ms': 2000, 'contact_confirm_ms': 300,
}

APPROACH_DEFAULTS = {
    'enabled':False, 'target_x_norm':.5, 'x_tolerance_norm':.03,
    'stop_bottom_norm':.78, 'stop_rw':.40, 'stop_rh':.70,
    'forward_ms':120, 'lateral_ms':100, 'max_forward_steps':None,
    'max_lateral_steps':25, 'max_no_progress':5, 'min_progress_norm':.003,
    'arm_x_tolerance_norm':.06, 'handoff_min_rw':0., 'handoff_min_rh':0.,
}
TRACKING_DEFAULTS = {'allow_label_variation':False,'min_iou':.30,'max_label_shift_norm':.08}


def approach_config(values,near):
    if not isinstance(values,dict) or set(values)-set(APPROACH_DEFAULTS):raise AutoFault('Sai cấu hình tiến gần')
    c=dict(APPROACH_DEFAULTS,**values)
    if type(c['enabled']) is not bool:raise AutoFault('approach.enabled phải là boolean')
    for name,lo,hi in [('target_x_norm',.1,.9),('x_tolerance_norm',.01,.15),
                       ('stop_bottom_norm',.5,.95),('stop_rw',.1,.55),('stop_rh',.2,.9),
                       ('min_progress_norm',.001,.05),('arm_x_tolerance_norm',.01,.15),
                       ('handoff_min_rw',0,.4),('handoff_min_rh',0,.7)]:
        v=c[name]
        if type(v) not in (int,float) or not math.isfinite(v) or not lo<=v<=hi:raise AutoFault('Sai tham số tiến gần: '+name)
    for name,lo,hi in [('forward_ms',50,400),('lateral_ms',50,400),('max_forward_steps',1,100),('max_lateral_steps',1,100),('max_no_progress',2,10)]:
        if name == 'max_forward_steps' and c[name] is None: continue
        if type(c[name]) is not int or not lo<=c[name]<=hi:raise AutoFault('Sai tham số tiến gần: '+name)
    if c['enabled'] and near['enabled'] and c['stop_rw']>=near['stop_rw']:
        raise AutoFault('Ngưỡng dừng tiến phải nhỏ hơn ngưỡng ngừng căn ảnh rw')
    return c


def tracking_config(values):
    if not isinstance(values,dict) or set(values)-set(TRACKING_DEFAULTS):raise AutoFault('Sai cấu hình theo dõi vật')
    c=dict(TRACKING_DEFAULTS,**values)
    if type(c['allow_label_variation']) is not bool:raise AutoFault('Sai chế độ đổi tên vật')
    for key,lo,hi in [('min_iou',.1,.9),('max_label_shift_norm',.01,.2)]:
        v=c[key]
        if type(v) not in (int,float) or not math.isfinite(v) or not lo<=v<=hi:raise AutoFault('Sai tham số theo dõi vật: '+key)
    return c


def normalized_iou(a,b):
    overlap=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-overlap
    return overlap/union if union>0 else 0.


def near_config(values):
    if not isinstance(values,dict) or set(values)-set(NEAR_DEFAULTS):
        raise AutoFault('Sai cấu hình tiếp cận gần')
    near = dict(NEAR_DEFAULTS,**values)
    if type(near['enabled']) is not bool: raise AutoFault('near.enabled phải là boolean')
    for name,lo,hi in [('stop_rw',.2,.95),('edge_margin_norm',0,.05),('max_alignment_age_s',1,60)]:
        value = near[name]
        if type(value) not in (int,float) or not math.isfinite(value) or not lo <= value <= hi:
            raise AutoFault('Sai tham số tiếp cận gần: '+name)
    for name,lo,hi in [('wait_contact_ms',300,10000),('contact_confirm_ms',200,2000)]:
        value = near[name]
        if type(value) is not int or not lo <= value <= hi:
            raise AutoFault('Sai tham số tiếp cận gần: '+name)
    if near['wait_contact_ms'] <= near['contact_confirm_ms']:
        raise AutoFault('Thời gian chờ IR phải lớn hơn thời gian xác nhận IR')
    return near


def image_extent(bbox,frame,near):
    """Normalized geometry; the cutoff is not a metric distance estimate."""
    x1,y1,x2,y2 = bbox
    w,h = frame
    rw,rh = (x2-x1)/w,(y2-y1)/h
    edge = min(x1/w,y1/h,(w-x2)/w,(h-y2)/h) <= near['edge_margin_norm']
    reasons = []
    if near['enabled']:
        if rw >= near['stop_rw']: reasons.append('rw')
        if edge: reasons.append('edge')
    return {'rw':rw,'rh':rh,'edge':edge,'limit_reasons':reasons}


BIN_KEYS = {1:'RECYCLE', 2:'ORGANIC', 3:'HAZARD', 4:'OTHER'}
ARM_JOINTS = ('p1','p10','p11','d2')
REAR_DEFAULTS = {
    'enabled_in_auto': True, 'sensor': 'D18', 'active_level': 0,
    'settle_ms': 700, 'stable_samples': 2, 'max_detection_attempts': 4,
    'retry_ms': 1000, 'min_open_ms': 1500, 'clear_confirm_ms': 500,
    'max_open_ms': 12000, 'max_cycle_s': 240,
    'post_rear_back_steps': 10, 'post_rear_back_ms': 400,
    'post_rear_settle_ms': 2000,
}


def rear_config(values):
    if not isinstance(values,dict) or set(values)-set(REAR_DEFAULTS):
        raise AutoFault('Sai cấu hình chu trình phía sau')
    rear = dict(REAR_DEFAULTS,**values)
    if type(rear['enabled_in_auto']) is not bool or rear['sensor'] != 'D18' or type(rear['active_level']) is not int or rear['active_level'] != 0:
        raise AutoFault('Cảm biến phía sau phải là D18, LOW phát hiện')
    for name,lo,hi in [('settle_ms',100,3000),('stable_samples',2,5),
                       ('max_detection_attempts',2,10),('retry_ms',100,5000),
                       ('min_open_ms',200,10000),('clear_confirm_ms',200,3000),
                       ('max_open_ms',1000,60000),('max_cycle_s',30,1200),
                       ('post_rear_back_steps',1,10),('post_rear_back_ms',100,400),
                       ('post_rear_settle_ms',500,5000)]:
        value = rear[name]
        if type(value) is not int or not lo <= value <= hi:
            raise AutoFault('Sai tham số phía sau: '+name)
    if rear['max_detection_attempts'] < rear['stable_samples']:
        raise AutoFault('Số lần chụp phía sau phải đủ xác nhận nhãn')
    if rear['max_open_ms'] <= rear['min_open_ms']+rear['clear_confirm_ms']:
        raise AutoFault('Thời gian mở tối đa phải đủ chờ bỏ rác và xác nhận D18')
    return rear


DROP_RECOVERY_DEFAULTS = {
    'max_retries': 1, 'retry_wait_ms': 800,
    'reverse_max_steps': 10, 'reverse_settle_ms': 2000,
    'loss_check_seconds': 15, 'loss_restore_ms': 300,
}


def drop_recovery_config(values):
    """Bound backward search after a confirmed loss; no silent unbounded driving."""
    if not isinstance(values, dict) or set(values) - set(DROP_RECOVERY_DEFAULTS):
        raise AutoFault('Sai cấu hình xử lý chai rơi')
    c = dict(DROP_RECOVERY_DEFAULTS, **values)
    for key, low, high in (
        ('max_retries', 0, 2), ('retry_wait_ms', 0, 5000),
        ('reverse_max_steps', 1, 10), ('reverse_settle_ms', 100, 5000),
        ('loss_check_seconds', 1, 30), ('loss_restore_ms', 200, 1500),
    ):
        if type(c[key]) is not int or not low <= c[key] <= high:
            raise AutoFault(f'Sai drop_recovery.{key}, cần {low}–{high}')
    return c


def load_rear_config(path):
    """Rear classification does not require calibrated grasp coordinates."""
    c = read_config(path)
    if c.get('version') not in (1,2,3): raise AutoFault('Sai phiên bản auto_config.json')
    c['drop_recovery'] = drop_recovery_config(c.get('drop_recovery', {}))
    c['rear'] = rear_config(c.get('rear',{}))
    control = c['control']
    for name,lo,hi in [('settle_ms',100,3000),('inference_timeout_s',1,180),('max_result_age_s',.1,60)]:
        value = control.get(name)
        if type(value) not in (int,float) or not math.isfinite(value) or not lo <= value <= hi:
            raise AutoFault('Sai tham số '+name)
    return c


def sensor_pin(name):
    """Only actual Mega digital inputs; PCA channel names are not sensor pins."""
    return name if isinstance(name,str) and name in ('D19','D15','D16') else None


def relative_stage(stage, frame):
    """Expand fractional image coordinates to pixels; old pixel files still work."""
    result = copy.deepcopy(stage)
    if 'target_norm' not in stage and 'tolerance_norm' not in stage: return result
    for field, pixel, high in [('target_norm','target_px',1),('tolerance_norm','tolerance_px',.25)]:
        values = stage.get(field)
        if not isinstance(values,list) or len(values) != 2 or any(
                type(v) not in (int,float) or not math.isfinite(v) or not 0 < v < high for v in values):
            raise AutoFault('Sai điểm/vùng sai số tương đối')
        result[pixel] = [v*size for v,size in zip(values,frame)]
    return result


def infer_image_direction(before, after, command, frame):
    """A manual trial establishes which action reduces a positive image error."""
    if command not in ('RIGHT','FORWARD'): raise AutoFault('Chỉ thử RIGHT hoặc FORWARD')
    for point in (before,after):
        if not isinstance(point,list) or len(point) != 2 or any(
                type(v) not in (int,float) or not math.isfinite(v) or not 0 < v < size for v,size in zip(point,frame)):
            raise AutoFault('Sai tâm vật của nhịp thử')
    delta = [b-a for a,b in zip(before,after)]
    axis = 0 if command == 'RIGHT' else 1
    if abs(delta[axis])/frame[axis] < .01:
        raise AutoFault('Vật dịch dưới 1% ảnh; thử lại nhịp ngắn hoặc chọn tâm rõ hơn')
    if abs(delta[1-axis]) > 2*abs(delta[axis]):
        raise AutoFault('Ảnh dịch chủ yếu sang trục khác; kiểm tra hướng xoay camera')
    opposite = {'RIGHT':'LEFT','FORWARD':'BACK'}[command]
    return command if delta[axis] < 0 else opposite, delta


def load_auto_config(path, robot_config):
    c = read_config(path)
    if c.get('version') not in (1,2,3): raise AutoFault('Sai phiên bản auto_config.json')
    c['drop_recovery'] = drop_recovery_config(c.get('drop_recovery', {}))
    cal, control = c['calibration'], c['control']
    frame = cal['frame_size']
    if len(frame) != 2 or any(type(v) is not int or v < 16 for v in frame):
        raise AutoFault('Sai kích thước ảnh hiệu chuẩn')
    for name, lo, hi in [('speed',1,255),('move_ms',50,400),('s2_step_us',1,50),
                         ('settle_ms',100,3000),('stable_samples',2,10),
                         ('max_align_moves_per_stage',1,100),('max_cycles',1,100),
                         ('max_no_target_attempts',1,100),('no_target_retry_ms',100,10000)]:
        value = control[name]
        if type(value) is not int or not lo <= value <= hi: raise AutoFault('Sai tham số '+name)
    for name, lo, hi in [('max_target_jump_normalized',.01,.5),('inference_timeout_s',1,180),
                         ('max_result_age_s',.1,60),('max_cycle_s',10,3600)]:
        value = control[name]
        if type(value) not in (int,float) or not math.isfinite(value) or not lo <= value <= hi:
            raise AutoFault('Sai tham số '+name)
    if type(control['repeat']) is not bool: raise AutoFault('repeat phải là boolean')
    mode = cal.get('mode','calibrated')
    if mode not in ('relative','calibrated'): raise AutoFault('Sai chế độ căn ảnh')
    cal['mode'] = mode
    if mode == 'relative':
        search = robot_config['poses']['SEARCH']
        final = cal['grasp_final_s2_us']
        if type(final) is not int or not search['p1'] < final <= robot_config['limits']['p1'][1]:
            raise AutoFault('Giới hạn hạ S2 phải lớn hơn SEARCH và trong giới hạn P1')
        goal = {'target_norm':cal['relative_target_norm'],'tolerance_norm':cal['relative_tolerance_norm']}
        relative_stage(goal,frame)  # Validate the approximate reference, not a measured contact point.
        response = control.get('relative_min_response_norm',.01)
        retries = control.get('relative_max_no_response',3)
        if type(response) not in (int,float) or not math.isfinite(response) or not .002 <= response <= .05:
            raise AutoFault('Sai ngưỡng phản hồi ảnh tương đối')
        if type(retries) is not int or not 1 <= retries <= 5: raise AutoFault('Sai giới hạn thử phản hồi ảnh')
        control.update(relative_min_response_norm=response,relative_max_no_response=retries)
        cal['frozen_joints_us'] = {j:search[j] for j in ('p10','p11','d2')}
        cal['positive_error_x_move'] = cal.get('positive_error_x_move') or 'RIGHT'
        cal['positive_error_y_move'] = cal.get('positive_error_y_move') or 'BACK'
        # Runtime references only. Do not write these as physically calibrated stages.
        cal['stages'] = [relative_stage(dict(goal,s2_us=pulse),frame) for pulse in (search['p1'],final)]
    missing = []
    frozen = cal['frozen_joints_us']
    if set(frozen) != {'p10','p11','d2'}: missing.append('xung cố định S1/P10, trụ/P11, nối/D2')
    for j,v in frozen.items():
        if j not in ('p10','p11','d2') or type(v) is not int or not robot_config['limits'][j][0] <= v <= robot_config['limits'][j][1]:
            raise AutoFault('Xung cố định không hợp lệ: '+j)
    search = robot_config['poses']['SEARCH']
    for j,v in frozen.items():
        if search.get(j) != v: raise AutoFault('SEARCH đã đổi '+j+'; hiệu chuẩn lại AUTO theo tư thế mới')
    if cal['positive_error_x_move'] not in ('LEFT','RIGHT'): missing.append('chiều căn ngang theo ảnh')
    if cal['positive_error_y_move'] not in ('FORWARD','BACK'): missing.append('chiều căn dọc theo ảnh')
    stages = [relative_stage(stage,frame) for stage in cal['stages']]
    cal['stages'] = stages
    if not stages: missing.append('các mốc S2 và điểm pixel mục tiêu')
    previous = None
    direction = None
    for i,stage in enumerate(stages):
        pulse = stage['s2_us']
        if type(pulse) is not int or not robot_config['limits']['p1'][0] <= pulse <= robot_config['limits']['p1'][1]:
            raise AutoFault('Xung S2 ngoài giới hạn')
        if i == 0 and pulse != search['p1']: raise AutoFault('Mốc đầu phải trùng S2 trong SEARCH')
        if previous is not None:
            delta = pulse-previous
            if not delta or (direction is not None and delta*direction <= 0):
                raise AutoFault('Các mốc S2 phải theo một chiều, không trùng')
            direction = delta
        previous = pulse
        for field, high in [('target_px',frame),('tolerance_px',[frame[0]/4,frame[1]/4])]:
            vals = stage[field]
            if len(vals) != 2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in vals):
                raise AutoFault('Sai điểm mục tiêu/vùng sai số')
            for v,limit in zip(vals,high):
                if not (0 < v < limit): raise AutoFault('Điểm/vùng sai số nằm ngoài ảnh')
    final = cal['grasp_final_s2_us']
    if type(final) is not int or not robot_config['limits']['p1'][0] <= final <= robot_config['limits']['p1'][1]:
        raise AutoFault('Xung gắp cuối ngoài giới hạn')
    if stages and stages[-1]['s2_us'] != final:
        raise AutoFault('Mốc cuối phải là tư thế gắp thực tế đã đo, không chỉ tư thế 90 độ')
    for name in ('contact_sensor','hold_sensor'):
        if sensor_pin(cal[name]) is None: missing.append(name+' (IR hiện tại nối D19 Mega; P15 PCA không đọc cảm biến)')
    if type(cal['active_level']) is not int or cal['active_level'] not in (0,1):
        raise AutoFault('Sai mức kích hoạt IR')
    hold_ms = cal['grasp_hold_confirm_ms']
    if type(hold_ms) is not int or not 200 <= hold_ms <= 3000: raise AutoFault('Sai thời gian xác nhận giữ vật')
    if missing: raise AutoFault('Cần hiệu chuẩn: '+ '; '.join(missing)+'. Chạy python calibrate_auto.py --help')
    try: c['arm_path'] = validate_path(c.get('arm_path'),robot_config,final)
    except ValueError as exc: raise AutoFault(str(exc)) from exc
    try: c['confirmed_grasp'] = checked_reference(c.get('confirmed_grasp'),c['arm_path'])
    except ValueError as exc: raise AutoFault(str(exc)) from exc
    c['rear'] = rear_config(c.get('rear',{}))
    c['near'] = near_config(c.get('near',{}))
    c['approach'] = approach_config(c.get('approach',{}),c['near'])
    c['tracking'] = tracking_config(c.get('tracking',{}))
    try: c['target_lock'] = lock_config(c.get('target_lock',{}))
    except ValueError as exc: raise AutoFault(str(exc)) from exc
    try:
        c['motion']=checked_settings(c.get('motion',{}),MOTION_DEFAULTS)
        c['visibility']=checked_settings(c.get('visibility',{}),VISIBILITY_DEFAULTS)
    except ValueError as exc:raise AutoFault(str(exc)) from exc
    if c['visibility']['enabled'] and (not c['approach']['enabled'] or cal['mode']!='relative' or not c['motion']['enabled']):
        raise AutoFault('Giữ ảnh cần approach relative và motion.enabled=true')
    if c['visibility']['min_step_us']>c['motion']['arm_step_us']:raise AutoFault('Bước tối thiểu lớn hơn bước tay')
    if c['approach']['enabled'] and cal['mode']!='relative':
        raise AutoFault('Tiến gần tách giai đoạn cần mode relative; tắt approach.enabled khi dùng mốc calibrated')
    try:
        from camera_publisher import CAMERA_INDEX
        c['distance'] = distance_config(c.get('distance', {}), robot_config, CAMERA_INDEX)
        if c['distance']['enabled'] and (not c['approach']['enabled'] or cal['mode'] != 'relative'):
            raise ValueError('Khoảng cách cm cần approach.enabled và mode relative')
    except (ValueError, KeyError, TypeError) as exc: raise AutoFault(str(exc)) from exc
    try: c['pose_profile']=checked_profile(c.get('pose_profile'),c['arm_path'],c)
    except (ValueError,KeyError,TypeError) as exc: raise AutoFault(str(exc)) from exc
    from grasp_orientation import checked_config
    try: c['grasp_orientation'] = checked_config(c.get('grasp_orientation'),c['arm_path'])
    except ValueError as exc: raise AutoFault(str(exc)) from exc
    try: c['box_tof_flow']=checked_tof_flow(c.get('box_tof_flow'),c)
    except (ValueError,KeyError,TypeError) as exc: raise AutoFault(str(exc)) from exc
    return c


def choose_direction(point, stage, cal):
    ex,ey = point[0]-stage['target_px'][0], point[1]-stage['target_px'][1]
    tx,ty = stage['tolerance_px']
    if abs(ex) <= tx and abs(ey) <= ty: return None, [ex,ey]
    # Correct one axis at a time to avoid coupling the image axes prematurely.
    axis = 0 if abs(ex)/tx >= abs(ey)/ty else 1
    move = cal['positive_error_x_move'] if axis == 0 else cal['positive_error_y_move']
    error = ex if axis == 0 else ey
    if error < 0: move = {'LEFT':'RIGHT','RIGHT':'LEFT','FORWARD':'BACK','BACK':'FORWARD'}[move]
    return move, [ex,ey]


class AutoController:
    def __init__(self, brain, vision, path, report):
        self.brain, self.vision, self.path, self.report = brain, vision, Path(path), report
        self.task = None
        self.token = None
        self.state = {'state':'auto_idle','message':'AUTO chưa chạy','auto_running':False,'cycles':0}
        self.lock = asyncio.Lock()
        self.cancelled = False
        self.frame_after = 0.
        self.target = None
        self.emergency_latched = False
        self.generation = 0
        self.selected_bin = None
        self.flow = None
        self.state['rear_cycles'] = 0
        self.aligned_anchor = None
        self.base_steps = 0
        self.vision_paused = False
        self.final_authorized = None
        self.lock_prediction = None
        self.observed_jpeg = None
        self.state.update(phase='search', ir_check_enabled=False, grasp_trigger=None)
        self.tracking_waiting = False
        self.lock_last_seen = 0.
        self.lock_sequence = 0
        self.locked_bin = None
        self.locked_label = None
        self.classification_vote = None
        self.classification_count = 0
        self.search_rotations = 0
        # Orientation belongs to ONE front target and ONE AUTO cycle, never a previous pickup.
        self.search_orientation = None
        # Failed pickups in the current AUTO session; no endless retry loop.
        self.drop_failures = 0
        # The next SEARCH cycle begins with a clockwise scan only if all ten
        # post-drop BACK observations were empty. Reset on each AUTO START.
        self.drop_search_pending = False
        self._resume_observing_after_rear = False
        self.holding_object = False
        self.front_lid_preopened = False
        self.checkpoint = AutoCheckpoint(self.path)
        self.resume_context = None
        self.state['resume_available'] = bool(self.checkpoint.load())
        self.state['resume_requires_inspection'] = False
        from operator_control import OperatorControl
        self.operator = OperatorControl(self)

    def clear_overlay(self):
        if self.vision and hasattr(self.vision,'clear_overlay'): self.vision.clear_overlay()

    def overlay(self, objects, selected, captured, frame, state):
        if hasattr(self.vision,'set_overlay'):
            self.vision.set_overlay(objects,selected,captured,frame,state,self.lock_sequence,
                                    self.locked_bin,self.cfg['target_lock']['overlay_max_age_s'])

    @property
    def running(self): return self.task is not None and not self.task.done()

    async def emit(self, state, message, **fields):
        if self.flow=='front' and self.locked_bin is not None and self.locked_label is not None:
            fields.setdefault('label',self.locked_label)
            fields.setdefault('bin',self.locked_bin)
        if state not in ('auto_base_observe','auto_tof_observe'):
            self.state.update(distance_cm=None,distance_uncertainty_cm=None)
        if state not in ('auto_pose_observe',):
            self.state.update(camera_range_estimate_cm=None,camera_range_quality=None)
        self.state.update(state=state,message=message,auto_running=self.running and (not self.operator.mechanics_enabled or self.brain.auto_owner == self.token),**fields)
        self.state['mechanics_enabled'] = self.operator.mechanics_enabled
        identity=(f' · Vật khóa: {self.locked_label} · BIN {self.locked_bin}'
                  if self.flow=='front' and self.locked_bin is not None else '')
        # Write a durable stage intent at boundaries. This is NOT a servo
        # encoder or permission to replay commands after USB reset.
        if (self.running and self.brain.connected and self.brain.auto_owner == self.token
                and state not in ('auto_done','auto_stopped','auto_error')):
            self.save_checkpoint(state)
        print('AUTO · '+state+' · '+message+identity)
        await self.report(copy.deepcopy(self.state))

    def save_checkpoint(self, stage=None, status='running'):
        """Persist progress on safe logical boundaries, never raw motor commands."""
        hw = self.brain.snapshot().get('hardware', {})
        self.checkpoint.write(
            state=stage or self.state.get('state', 'auto_search'),
            phase=self.state.get('phase', 'search'), flow=self.flow or 'front',
            # Closing D3 is a non-idempotent stage. Treat as potentially carrying
            # even when a USB fault happens before holding_object is assigned.
            holding_object=(self.holding_object or self.state.get('state') in (
                'auto_grip','auto_grip_check','auto_lift','auto_place',
                'auto_release')),
            selected_bin=self.selected_bin, label=self.locked_label,
            cycles=self.state.get('cycles', 0),
            rear_cycles=self.state.get('rear_cycles', 0),
            drop_failures=self.drop_failures,
            status=status, last_joints=hw.get('joints', {}))
        self.state['resume_available'] = True

    def requires_reconnect_inspection(self, checkpoint):
        """Unsafe phases require human verification, not automatic servo replay."""
        if not checkpoint or checkpoint.get('flow') != 'front':
            return True
        if checkpoint.get('holding_object'):
            return True
        stage = checkpoint.get('state', '')
        # Commands can be in flight when USB disappears. Only a SEARCH
        # observation boundary is resumable without proof of servo pose.
        if stage not in ('auto_search', 'auto_acquire', 'auto_reacquire',
                         'auto_camera_retry', 'auto_search_rotate',
                         'auto_search_pause', 'auto_cycle_done'):
            return True
        # Hardware STATUS only echoes *commanded* servo pulses, not real
        # mechanical positions. Restrict unattended resume to the last
        # checkpoint at SEARCH, with the empty gripper and closed lid. If the
        # arm was down or the lid open, request a physical inspection.
        joints = checkpoint.get('last_commanded_joints') or {}
        search = self.brain.config['poses']['SEARCH']
        for joint in ('p1','p10','p11','d2'):
            if joints.get(joint) != search.get(joint):
                return True
        if (joints.get('d3') != self.brain.config['grip']['open']
                or joints.get('p4') != self.brain.config['lid']['close']):
            return True
        return False

    async def resume_after_connect(self):
        """Called ONLY after a user-issued USB Connect, never on broker reconnect.

        Reconnect resets Mega and loses trustworthy servo pose feedback. Empty
        gripper/SEARCH may safely start a *new observation* of the interrupted
        target; carrying/release is withheld for a human to inspect first.
        """
        checkpoint = self.checkpoint.load()
        if not checkpoint:
            return False
        if self.task and not self.task.done():
            self.cancelled = True
            self.generation += 1
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
        await asyncio.to_thread(self.brain.refresh_status, self.brain.epoch)
        snap = self.brain.snapshot()
        sensors = snap['hardware'].get('sensors', {})
        if (self.requires_reconnect_inspection(checkpoint)
                or sensors.get('D19') != 1 or sensors.get('D18') != 1
                or snap['hardware'].get('stopped') is not False):
            self.state.update(state='auto_reconnect_inspection',
                message='Đã lưu công đoạn, nhưng tư thế tay/vật không xác minh được sau reset Arduino; kiểm tra thủ công rồi START mới',
                auto_running=False, resume_available=True, resume_requires_inspection=True,
                resume_stage=checkpoint.get('state'), resume_bin=checkpoint.get('selected_bin'))
            await self.report(copy.deepcopy(self.state))
            return False
        self.resume_context = checkpoint
        self.cancelled = False
        self.state.update(resume_requires_inspection=False,
            message='Đang tiếp tục lượt AUTO chưa xong: nhận diện lại từ SEARCH để tránh chạy lệnh cũ')
        # Target lock, angle and cached ToF are invalid across a USB reset.
        # Re-observe the same work area rather than replay an interrupted pulse.
        await self.start(resuming=True)
        return bool(self.running)

    def guard(self):
        s = self.brain.snapshot()
        if self.cancelled or self.brain.auto_owner != self.token: raise AutoFault('Chu trình đã bị hủy')
        if not s['connected'] or not s['fresh'] or s['stopped']: raise AutoFault('Arduino mất kết nối/đã dừng')
        if not self.tracking_waiting and time.monotonic() > self.deadline: raise AutoFault('AUTO vượt thời gian chu trình')
        return s

    async def wait(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.guard()
            await asyncio.sleep(min(.04,max(0,end-time.monotonic())))
        self.guard()

    async def fresh_hardware(self):
        state=self.guard()
        await asyncio.to_thread(self.brain.refresh_status,state['epoch'],self.token)
        return self.guard()

    async def move_targets(self, targets, name, settle_ms=None, gentle=False, arm_step_ms=None):
        if self.holding_object:
            await self.check_grip_or_drop('trước chuyển động '+name)
        before = self.guard()['hardware'].get('joints',{}).get('p1')
        prediction = None
        if (self.target and not self.vision_paused and self.cfg.get('target_lock',{}).get('compensate_arm_pose')
                and type(before) is int and type(targets.get('p1')) is int
                and 0 < targets['p1']-before <= 50 and self.cfg.get('pose_profile',{}).get('enabled')):
            profile = self.cfg['pose_profile']
            prediction = pose_prediction(self.target,reference_at(profile,before),reference_at(profile,targets['p1']))
        self.clear_overlay()
        # S1 first, then S2; D2 is changed only in the grasp stage.
        if 'p10' in targets and 'p1' in targets:
            targets = dict([('p10', targets['p10']), ('p1', targets['p1'])] + [(j,v) for j,v in targets.items() if j not in ('p10','p1')])
        if gentle:
            await asyncio.to_thread(self.brain.auto_targets,targets,name,self.token,self.cfg['motion']['arm_step_us'],
                                   self.cfg['motion']['arm_step_ms'] if arm_step_ms is None else arm_step_ms)
        else:await asyncio.to_thread(self.brain.auto_targets, targets, name, self.token)
        while self.guard()['busy']:
            # Poll fresh D19 while the servo worker is active. On a dropout,
            # cancel only the servo job, leaving the last pulses energised.
            # Do not continue toward the bin until the 15s check has finished.
            if self.holding_object:
                await self.fresh_hardware()
                if not self.sensor(self.cfg['calibration']['hold_sensor']):
                    await asyncio.to_thread(self.brain.pause_auto_arm_for_drop, self.token)
                    await self.check_grip_or_drop('đang chuyển động '+name)
                    # Sensor returned and stayed stable: resume from the reported
                    # commanded pulses to the original destination.
                    await asyncio.to_thread(self.brain.auto_targets, targets, name+' · tiếp tục sau D19',
                                             self.token,
                                             self.cfg['motion']['arm_step_us'] if gentle else None,
                                             (self.cfg['motion']['arm_step_ms'] if arm_step_ms is None else arm_step_ms) if gentle else None)
                    continue
            await asyncio.sleep(.08)
        await self.wait((self.cfg['control']['settle_ms'] if settle_ms is None else settle_ms)/1000)
        self.frame_after = time.monotonic()
        if prediction is not None:self.lock_prediction=prediction
        # Check D19 after EVERY completed servo movement while carrying a bottle.
        # This includes the long trip to PLACE and opening the lid, not just P1 lift.
        # Recover only after the movement has stopped; never issue a new servo job
        # while an earlier worker is still busy.
        if self.holding_object:
            await self.check_grip_or_drop('sau chuyển động '+name)

    def arm_pose(self, name):
        # Pose edits cannot silently open the lid, turn a bin or release a held object.
        return {j:v for j,v in self.brain.config['poses'][name].items() if j in ARM_JOINTS}

    def check_rear(self):
        if self.flow == 'front' and self.brain.rear_requested():
            # Never open a gripping jaw in the air or interrupt the already
            # locked drop-off. Finish the known BIN deposit, then service D18.
            if self.holding_object:
                return
            raise RearRequested()

    async def prepare_bin(self, target_bin, rear=False, *, open_for_pickup=False):
        # Rotate BIN while closed. Front pickup may open lid while arm stays SEARCH.
        # D18/rear retains its own sensor-controlled lid sequence.
        if target_bin not in BIN_KEYS: raise AutoFault('Ngăn rác không hợp lệ')
        if rear and open_for_pickup: raise AutoFault('Không mở nắp gắp sớm trong chế độ D18')
        if open_for_pickup:
            self.check_rear()
            hw = self.guard()['hardware']
            pose = self.arm_pose('SEARCH')
            if (self.flow != 'front' or self.holding_object or hw.get('drive', {}).get('active')
                    or self.guard()['busy'] or any(hw['joints'].get(k) != v for k,v in pose.items())
                    or hw['joints'].get('d3') != self.brain.config['grip']['open']
):
                raise NearHold('Không cho mở nắp sớm: tay/bánh/kẹp chưa an toàn tại SEARCH')
        prefix = 'rear' if rear else 'auto'
        await self.emit(prefix+'_sort',f'Phân loại BIN {target_bin}; đóng nắp rồi xoay đúng ngăn',bin=target_bin)
        await self.move_targets({'p4':self.brain.config['lid']['close']},'Đóng nắp trước khi xoay thùng')
        await self.move_targets({'p9':self.brain.config['bins'][BIN_KEYS[target_bin]]},'Xoay BIN '+str(target_bin))
        self.selected_bin = target_bin
        if open_for_pickup:
            self.check_rear()
            if self.guard()['hardware']['joints'].get('p9') != self.brain.config['bins'][BIN_KEYS[target_bin]]:
                raise NearHold('Chưa xác nhận xung P9 của ngăn đã chọn; không mở nắp')
            await self.move_targets({'p4':self.brain.config['lid']['open']},'AUTO mở nắp trước khi gắp')
            if self.guard()['hardware']['joints'].get('p4') != self.brain.config['lid']['open']:
                raise NearHold('Chưa xác nhận xung mở nắp P4; không tiến đến gắp')
            self.front_lid_preopened = True
            await self.emit('auto_lid_ready',
                            f'BIN {target_bin} đã xoay và mở nắp; đo góc SEARCH, sau đó tiếp cận để gắp',
                            bin=target_bin, lid_ready=True)

    async def motor_step(self, move, duration_ms=None, settle_ms=None, *, finish_descent_on_ir=False):
        s = self.guard()
        self.clear_overlay()
        if self.vision_paused: raise AutoFault('Đã dừng căn ảnh ở giai đoạn gần; không chạy bánh tiếp')
        self.aligned_anchor = None
        self.base_steps += 1
        duration=self.cfg['control']['move_ms'] if duration_ms is None else duration_ms
        if self.cfg.get('motion',{}).get('enabled'):
            try:
                duration=await asyncio.to_thread(self.brain.smooth_auto_drive,move,s['epoch'],self.cfg['control']['speed'],duration,self.cfg['motion']['ramp_ms'],self.token)
            except ValueError as exc:
                if str(exc)!='IR_BLOCKS_AUTO_DRIVE':raise
                state=self.guard()
                if self.flow=='front' and state['hardware'].get('sensors',{}).get('D18')==0:
                    raise RearRequested() from exc
                self.check_rear()
                raise NearHold('Firmware chặn bánh bởi D18/D19 khi chế độ gắp đang bật; kiểm tra cảm biến') from exc
        else:await asyncio.to_thread(self.brain.command,'DRIVE '+move,s['epoch'],speed=self.cfg['control']['speed'],_owner=self.token)
        try:
            end=time.monotonic()+duration/1000
            while time.monotonic()<end:
                if self.cfg.get('motion',{}).get('enabled'):
                    self.check_rear()
                    # D19 is intentionally ignored during wheel search/centering.
                    # In the final grasp, BoxToFFlow checks D19 separately.
                await self.wait(min(.03,max(0,end-time.monotonic())))
        finally:
            # STOP removes ownership; a late cleanup must not issue a new command.
            if self.brain.auto_owner == self.token:
                await asyncio.to_thread(self.brain.command,'XM',self.brain.epoch,_owner=self.token)
        await self.wait((self.cfg['control']['settle_ms'] if settle_ms is None else settle_ms)/1000)
        self.frame_after = time.monotonic()

    async def search_turn_right(self):
        """Approx 5 s clockwise, then 1 s stationary before a new camera frame.

        Firmware smooth motion accepts <=400 ms nominal: use 9 separately
        guarded pulses (~560 ms each), STOP/XM between pulses. This is NOT
        calibrated to one 360-degree revolution (no heading encoder).
        """
        from robot_params import AUTO_SEARCH_TURN_SECONDS, AUTO_SEARCH_STOP_SECONDS, AUTO_SEARCH_MAX_TURNS
        self.check_rear()
        self.search_rotations += 1
        count = self.search_rotations
        group_turn = (count - 1) % AUTO_SEARCH_MAX_TURNS + 1
        group = (count - 1) // AUTO_SEARCH_MAX_TURNS + 1
        await self.emit('auto_search_rotate',
                        f'Chưa thấy rác: xoay phải {AUTO_SEARCH_TURN_SECONDS:g}s · lượt {group_turn}/{AUTO_SEARCH_MAX_TURNS} · vòng quét {group} (không giới hạn lượt)',
                        scan_turn=group_turn, scan_total=AUTO_SEARCH_MAX_TURNS,
                        scan_completed=count, scan_group=group, vision_mode='scanning')
        # This duration is composed from multiple firmware-limited movement leases.
        # Each pulse uses the existing per-command D19, STOP and USB guards.
        pulses = max(1, round(AUTO_SEARCH_TURN_SECONDS / .56))
        for _ in range(pulses):
            self.guard()
            self.check_rear()
            await self.motor_step('TURN_RIGHT', duration_ms=400, settle_ms=0)
        await self.emit('auto_search_pause', f'Đã dừng xoay; chờ {AUTO_SEARCH_STOP_SECONDS:g}s để nhận diện ảnh mới',
                        scan_turn=group_turn, scan_total=AUTO_SEARCH_MAX_TURNS,
                        scan_completed=count, scan_group=group)
        await self.wait(AUTO_SEARCH_STOP_SECONDS)
        self.frame_after = time.monotonic()  # never reuse the pre-turn frame

    async def detect(self, require_bbox=True, interrupt_for_rear=True):
        if not require_bbox or not self.cfg.get('target_lock',{}).get('enabled'):
            return await self._detect_once(require_bbox,interrupt_for_rear)
        waiting_since=None
        pending=None
        confirmations=0
        consecutive_camera_failures = 0
        cfg=self.cfg['target_lock']
        try:
            while True:
                self.guard()
                if interrupt_for_rear:self.check_rear()
                try:
                    obj=await self._detect_once(require_bbox,interrupt_for_rear)
                except (StaleDetection,VisionUnavailable) as exc:
                    obj=None
                    consecutive_camera_failures += 1
                    self.clear_overlay()
                    self.lock_reason=str(exc)
                    if consecutive_camera_failures >= 3 and self.locked_bin is None:
                        # No blind rotation when camera fails; stay in SEARCH,
                        # retry until fresh inference returns. Never HOLD just
                        # because the initial scene is empty or AI unavailable.
                        await self.emit('auto_camera_retry',
                            f'Camera/AI chưa có ảnh mới ({consecutive_camera_failures} lần) · thử lại, không xoay mù',
                            phase='search', vision_mode='reacquiring')
                    elif consecutive_camera_failures >= 3:
                        raise NearHold('Mất hình vật đã khóa 3 lần; cần kiểm tra trước khi gắp') from exc
                else:
                    consecutive_camera_failures = 0
                empty_initial_frame = obj is None and self.locked_bin is None and consecutive_camera_failures == 0
                if obj is not None:
                    # Initial classification requires consecutive matching bins BEFORE moving.
                    if self.locked_bin is None:
                        observed=obj['bin']
                        self.classification_count=self.classification_count+1 if observed==self.classification_vote else 1
                        self.classification_vote=observed
                        if self.classification_count>=cfg['classify_frames']:
                            self.locked_bin=observed
                            self.locked_label=obj.get('observed_label',obj['label'])
                            if cfg.get('restart_initial_candidate'):self.lock_sequence+=1
                        else:
                            obj=None
                            self.lock_reason='Đang xác nhận phân loại ban đầu'
                    if obj is not None and waiting_since is not None:
                        index,_=associate(pending,[obj],cfg) if pending else (None,'')
                        confirmations=confirmations+1 if index is not None else 1
                        pending=copy.deepcopy(obj)
                        if confirmations<cfg['confirm_frames']:
                            obj=None
                            self.lock_reason='Đã thấy vật; xác nhận thêm ảnh trước khi tiếp tục'
                    if obj is not None:
                        obj['label']=self.locked_label or obj['label']
                        obj['observed_bin']=obj['bin'];obj['bin']=self.locked_bin
                        self.target=copy.deepcopy(obj)
                        self.lock_prediction=None
                        self.lock_last_seen=time.monotonic()
                        self.overlay(self.last_candidates,self.last_selected,self.frame_after,self.observed_frame_size,'LOCK')
                        if waiting_since is not None:
                            await self.emit('auto_target_locked','Đã xác nhận vật đã khóa; tiếp tục AUTO',
                                            target_id=self.lock_sequence,bin=self.locked_bin)
                        await self.emit('auto_target_observe',f'LOCK #{self.lock_sequence} · {len(self.last_candidates)} box · BIN đã chốt {self.locked_bin}',
                                        target_id=self.lock_sequence,bin=self.locked_bin,observed_bin=obj['observed_bin'],
                                        detections=[{'bbox':o['bbox'],'label':o['label'],'bin':o['bin'],'selected':i==self.last_selected} for i,o in enumerate(self.last_candidates)],
                                        vision_mode='locked')
                        return obj
                else:
                    pending=None;confirmations=0
                    if self.locked_bin is None:
                        self.classification_vote=None;self.classification_count=0
                if waiting_since is None:
                    waiting_since=time.monotonic()
                    self.tracking_waiting=True
                    self.aligned_anchor=None
                    await asyncio.to_thread(self.brain.command,'XM',self.brain.epoch,_owner=self.token)
                if hasattr(self.vision,'set_overlay_state'):
                    self.vision.set_overlay_state('REACQUIRING')
                elif hasattr(self.vision,'overlay') and self.vision.overlay:
                    self.vision.overlay['state']='REACQUIRING'
                acquiring=self.locked_bin is None and cfg.get('restart_initial_candidate')
                await self.emit('auto_acquire' if acquiring else 'auto_reacquire',getattr(self,'lock_reason','Chưa thấy vật đã khóa')+' · AUTO vẫn bật, bánh/tay đứng yên',
                                target_id=self.lock_sequence,bin=self.locked_bin,vision_mode='reacquiring')
                if empty_initial_frame and self.flow == 'front':
                    # Search only when a genuinely fresh empty frame was returned.
                    # If an object is visible but classification is pending, stay still.
                    await self.search_turn_right()
                else:
                    await self.wait(cfg['retry_ms']/1000)
        finally:
            if waiting_since is not None:self.deadline+=time.monotonic()-waiting_since
            self.tracking_waiting=False

    async def _detect_once(self, require_bbox=True, interrupt_for_rear=True):
        self.guard()
        self.observed_jpeg=None
        if require_bbox and self.vision_paused:
            raise AutoFault('Đã chuyển sang chờ IR; không gửi thêm ảnh căn vật')
        if interrupt_for_rear: self.check_rear()
        # Keep checking D18/STOP while an HTTP inference is pending.
        request = asyncio.create_task(self.vision.detect(fresh_after=self.frame_after))
        deadline = time.monotonic()+self.cfg['control']['inference_timeout_s']
        try:
            while not request.done():
                self.guard()
                if interrupt_for_rear: self.check_rear()
                remaining = deadline-time.monotonic()
                if remaining <= 0: raise VisionUnavailable('Model quá thời gian nhận diện; chờ thử ảnh mới')
                await asyncio.wait({request},timeout=min(.04,remaining))
            try:result = request.result()
            except (HTTPError,OSError,TimeoutError) as exc:
                raise VisionUnavailable('Tạm mất kết nối Modal; đứng yên chờ thử lại') from exc
        finally:
            if not request.done(): request.cancel()
            await asyncio.gather(request,return_exceptions=True)
        self.guard()  # Modal result received after STOP cannot trigger motion.
        if interrupt_for_rear: self.check_rear()
        captured = result.get('_frame_at')
        if type(captured) not in (int,float) or not math.isfinite(captured) or captured <= self.frame_after or captured > time.monotonic()+.1:
            raise AutoFault('Kết quả Modal không gắn với ảnh mới sau khi chuyển động')
        if time.monotonic()-captured > self.cfg['control']['max_result_age_s']:
            raise StaleDetection('Modal trả ảnh quá cũ; chờ nhận diện ảnh mới')
        if not self.vision.jpeg or time.monotonic()-self.vision.frame_at > 2:
            raise AutoFault('Camera đã mất ảnh mới')
        self.frame_after = captured
        frame = result.get('_frame_size')
        if not isinstance(frame,list) or len(frame)!=2 or any(type(v) is not int or v<16 for v in frame):
            raise AutoFault('Camera không có kích thước ảnh hợp lệ')
        self.observed_frame_size = frame
        self.observed_jpeg = result.get('_frame_jpeg')
        self.observed_frame_at = captured
        if require_bbox and self.cfg['calibration']['mode'] != 'relative' and frame != self.cfg['calibration']['frame_size']:
            raise AutoFault('Kích thước ảnh đã đổi; hiệu chuẩn lại tại đúng chế độ camera')
        if require_bbox and self.cfg.get('target_lock',{}).get('enabled'):
            return self.select_locked_target(result)
        if not result['found']: return None
        obj = result['object']
        if not isinstance(obj,dict) or type(obj.get('bin')) is not int or obj['bin'] not in (1,2,3,4) or not isinstance(obj.get('label'),str) or not obj['label'].strip():
            raise AutoFault('Nhãn/ngăn rác Modal không hợp lệ')
        if not require_bbox: return copy.deepcopy(obj)
        bbox = obj.get('bbox')
        w,h = result['_frame_size']
        if not isinstance(bbox,list) or len(bbox) != 4 or any(type(v) not in (int,float) or not math.isfinite(v) for v in bbox):
            raise AutoFault('Modal cần trả bbox [x1,y1,x2,y2] để chạy AUTO')
        x1,y1,x2,y2 = bbox
        if not (0 <= x1 < x2 <= w and 0 <= y1 < y2 <= h): raise AutoFault('BBox ngoài ảnh')
        point = [(x1+x2)/2,(y1+y2)/2]
        obj = copy.deepcopy(obj); obj['center'] = point
        obj['center_norm'] = [point[0]/w,point[1]/h]
        obj['bbox_norm'] = [x1/w,y1/h,x2/w,y2/h]
        obj['extent'] = image_extent(bbox,frame,self.cfg['near'])
        if self.target:
            if obj['bin'] != self.target['bin']:
                raise AutoFault('Modal đã đổi mục tiêu; dừng để tránh gắp vật khác')
            old = self.target['center_norm']
            jump = math.hypot(obj['center_norm'][0]-old[0],obj['center_norm'][1]-old[1])
            if obj['label'].strip().lower() != self.target['label'].strip().lower():
                tracking=self.cfg['tracking']
                overlap=normalized_iou(obj['bbox_norm'],self.target['bbox_norm'])
                if not tracking['allow_label_variation'] or jump>tracking['max_label_shift_norm'] or overlap<tracking['min_iou']:
                    raise AutoFault(f'Modal đã đổi mục tiêu/nhãn {self.target["label"]} → {obj["label"]}; chưa đủ liên tục hình học để giữ mục tiêu')
                observed=obj['label'];obj['observed_label']=observed;obj['label']=self.target['label']
                await self.emit('auto_track_label',f'Giữ cùng vật: {observed} → {obj["label"]}; IoU={overlap:.2f}',observed_label=observed)
            # A clipped/large bbox has an unreliable center. It may only enter a
            # stationary IR gate below; never use that center to command a wheel.
            if jump > self.cfg['control']['max_target_jump_normalized'] and not obj['extent']['limit_reasons']:
                raise AutoFault('Mục tiêu nhảy quá xa giữa hai ảnh; cần tìm lại')
        self.target = obj
        return obj

    def select_locked_target(self, result):
        cfg=self.cfg['target_lock'];frame=result['_frame_size'];w,h=frame
        raw=result.get('objects')
        if raw is None:raw=[result['object']] if result.get('found') else []
        if not isinstance(raw,list) or len(raw)>100:raise AutoFault('Sai danh sách box Modal')
        candidates=[]
        for entry in raw:
            if not isinstance(entry,dict) or type(entry.get('bin')) is not int or entry['bin'] not in BIN_KEYS:
                raise AutoFault('Ngăn rác Modal không hợp lệ')
            if not isinstance(entry.get('label'),str) or not entry['label'].strip():raise AutoFault('Nhãn Modal không hợp lệ')
            box=entry.get('bbox')
            if not isinstance(box,list) or len(box)!=4 or any(type(v) not in (int,float) or not math.isfinite(v) for v in box):
                raise AutoFault('Mỗi vật Modal phải trả bbox pixel [x1,y1,x2,y2]')
            x1,y1,x2,y2=box
            if not (0<=x1<x2<=w and 0<=y1<y2<=h):raise AutoFault('BBox ngoài ảnh')
            obj=copy.deepcopy(entry)
            obj.update(center=[(x1+x2)/2,(y1+y2)/2],center_norm=[(x1+x2)/2/w,(y1+y2)/2/h],
                       bbox_norm=[x1/w,y1/h,x2/w,y2/h],extent=image_extent(box,frame,self.cfg['near']))
            candidates.append(obj)
        self.last_candidates=candidates;self.last_selected=None
        if self.target:
            if frame!=getattr(self,'lock_frame_size',frame):raise AutoFault('Kích thước camera thay đổi trong khi khóa vật')
            if time.monotonic()-self.lock_last_seen>cfg['reference_max_age_s']:
                selected=None;reason='Tham chiếu vật đã quá cũ; giữ chờ, STOP rồi START để chọn lại'
            else:
                flow_cfg=self.cfg.get('box_tof_flow',{})
                if flow_cfg.get('enabled'):
                    selected,reason=associate_rolling(self.target,candidates,cfg,flow_cfg)
                else:
                    selected,reason=associate(self.target,candidates,cfg)
                if selected is None and self.lock_prediction is not None and cfg.get('compensate_arm_pose'):
                    # A measured camera-pose change supplies a bounded prediction;
                    # never use a photo box as the object's detected box.
                    stricter=dict(cfg,min_iou=max(.25,cfg['min_iou']),max_shift_norm=min(.15,cfg['max_shift_norm']))
                    if flow_cfg.get('enabled'):
                        selected,reason=associate_rolling(self.lock_prediction,candidates,stricter,flow_cfg)
                    else:
                        selected,reason=associate(self.lock_prediction,candidates,stricter)
            if selected is None and self.locked_bin is None and cfg.get('restart_initial_candidate'):
                # No identity has been committed yet. Restart classification rather
                # than reporting a lost "locked" object before the initial vote.
                self.target=None
                self.classification_vote=None;self.classification_count=0
        else:
            selected=None
        if self.target is None:
            # Select the closest to the optical center; reject an equally plausible first choice.
            order=sorted((math.hypot(o['center_norm'][0]-.5,o['center_norm'][1]-.5),i) for i,o in enumerate(candidates))
            selected=order[0][1] if order else None
            reason='Chưa thấy vật'
            if len(order)>1 and order[1][0]-order[0][0]<.03:
                selected=None;reason='Hai vật gần tâm ngang nhau; cần chọn rõ một vật'
        self.lock_reason=reason
        self.last_selected=selected
        self.overlay(candidates,selected,self.frame_after,frame,'LOCK' if selected is not None else 'REACQUIRING')
        if selected is None:return None
        obj=copy.deepcopy(candidates[selected])
        if self.target:
            obj['observed_label']=obj['label']
            obj['label']=self.target['label']
        else:
            if not cfg.get('restart_initial_candidate'):self.lock_sequence+=1
            self.lock_frame_size=frame[:]
        # Reacquisition does not move the reference until consecutive confirmation succeeds.
        if not self.tracking_waiting or self.target is None or (self.locked_bin is None and cfg.get('restart_initial_candidate')):
            self.target=copy.deepcopy(obj)
            self.lock_last_seen=time.monotonic()
        return obj

    def sensor(self, name):
        s = self.guard()['hardware'].get('sensors',{})
        value = s.get(sensor_pin(name))
        if type(value) is not int or value not in (0,1): raise AutoFault('Không đọc được cảm biến '+str(name))
        return value == self.cfg['calibration']['active_level']

    def record_alignment(self, obj):
        hw = self.guard()['hardware']
        self.aligned_anchor = {
            'at':self.frame_after, 'base_steps':self.base_steps,
            's2_us':hw['joints'].get('p1'),
            'frozen':{j:hw['joints'].get(j) for j in ('p10','p11','d2')},
            'label':obj['label'].strip().lower(), 'bin':obj['bin'],
        }

    def check_near_anchor(self, obj):
        hw = self.guard()['hardware']
        anchor = self.aligned_anchor
        if not anchor or anchor['base_steps'] != self.base_steps:
            raise NearHold('Ảnh quá lớn/sát mép nhưng chưa có lần căn ổn định; không hạ thêm hoặc đóng kẹp')
        if time.monotonic()-anchor['at'] > self.cfg['near']['max_alignment_age_s']:
            raise NearHold('Lần căn trước đã quá cũ; giữ nguyên, không đóng kẹp theo ảnh cũ')
        if anchor['label'] != obj['label'].strip().lower() or anchor['bin'] != obj['bin'] or self.selected_bin != obj['bin']:
            raise NearHold('Vật/ngăn không khớp lần căn trước; không đóng kẹp')
        current = hw['joints'].get('p1')
        if type(current) is not int or type(anchor['s2_us']) is not int or abs(current-anchor['s2_us']) > self.cfg['control']['s2_step_us']:
            raise NearHold('Tay đã đi quá một bước S2 kể từ lần căn trước; không đóng kẹp')
        if any(hw['joints'].get(j) != value for j,value in anchor['frozen'].items()):
            raise NearHold('Tư thế tay đã đổi sau lần căn; không đóng kẹp')

    async def close_range_contact(self, obj, reason=None):
        self.guard()
        self.check_rear()
        self.vision_paused = True
        # XM stops wheel PWM while retaining servo pulses and the AUTO owner.
        await asyncio.to_thread(self.brain.command,'XM',self.brain.epoch,_owner=self.token)
        extent = obj['extent']
        reason = reason or ('vật chiếm nhiều ảnh' if 'rw' in extent['limit_reasons'] else 'khung vật sát/chạm mép ảnh')
        await self.emit('auto_near_wait',f'{reason} · rw={extent["rw"]:.2f}; giữ tay/bánh, chờ IR',
                        rw=extent['rw'],rh=extent['rh'],near_reasons=extent['limit_reasons'],vision_mode='ir_only')
        self.check_near_anchor(obj)
        near = self.cfg['near']
        deadline = time.monotonic()+near['wait_contact_ms']/1000
        active_since = None
        while True:
            self.guard()
            self.check_rear()
            self.check_near_anchor(obj)
            now = time.monotonic()
            if self.sensor(self.cfg['calibration']['contact_sensor']):
                if active_since is None: active_since = now
                if now-active_since >= near['contact_confirm_ms']/1000:
                    await self.emit('auto_near_contact','Đã căn trước đó và IR xác nhận ổn định; chuyển sang gắp')
                    raise CloseRangeContact(obj)
            else: active_since = None
            if now >= deadline:
                raise NearHold('Hết thời gian chờ IR khi ảnh quá gần; không hạ tiếp, không đóng kẹp')
            await self.wait(.04)

    async def base_approach(self):
        """Keep SEARCH fixed. First center X, then advance; a centered far target is not close."""
        c=self.cfg['approach'];stable=forwards=laterals=no_progress=attempts=0
        pending=None
        windowed=self.cfg.get('target_lock',{}).get('enabled',False)
        progress=ForwardProgress(c['max_no_progress'],c['min_progress_norm'])
        while True:
            self.guard();self.check_rear()
            actual=self.guard()['hardware']['joints']
            for joint,value in self.arm_pose('SEARCH').items():
                if actual.get(joint)!=value:raise AutoFault('Tay phải giữ SEARCH trong lúc bánh tiến: '+joint)
            obj=await self.detect()
            if not obj:
                if self.target:raise NearHold('Mất vật khi bánh đang tiếp cận; giữ vị trí, không hạ tay')
                attempts+=1
                if attempts>self.cfg['control']['max_no_target_attempts']:raise NearHold('Chưa thấy rác; bánh đứng yên, chưa hạ tay')
                await self.emit('auto_search','Chưa có vật; đứng yên chờ ảnh mới')
                await self.wait(self.cfg['control']['no_target_retry_ms']/1000);continue
            metric = self.cfg.get('distance', {}).get('enabled', False)
            measurement = None
            if metric:
                try: measurement = measure_floor(obj, self.cfg['distance'], actual, self.observed_frame_size)
                except ValueError as exc: raise NearHold(str(exc)) from exc
                if measurement['too_close']:
                    raise NearHold('Chai đã gần hơn vùng dừng đã đo; giữ bánh, kiểm tra tầm gắp')
            if obj['extent']['limit_reasons']:
                if metric: raise NearHold('Box sát mép/quá lớn; không còn đo được chân chai tin cậy, không hạ tay')
                await self.close_range_contact(obj)
            if self.sensor(self.cfg['calibration']['contact_sensor']):
                raise NearHold('D19 đã thấy vật khi bánh đang tiếp cận; dừng bánh, không tiến/hạ thêm')
            if self.selected_bin is None:
                await self.emit('auto_classified',f'{obj["label"]} · BIN {obj["bin"]}',label=obj['label'],bin=obj['bin'])
                await self.prepare_bin(obj['bin']);continue
            ex=obj['center_norm'][0]-c['target_x_norm']
            close=(obj['bbox_norm'][3]>=c['stop_bottom_norm'] or obj['extent']['rw']>=c['stop_rw'] or obj['extent']['rh']>=c['stop_rh'])
            visibility=self.cfg.get('visibility',{})
            if metric: close = measurement['close']
            elif visibility.get('enabled') and obj['bbox_norm'][3]>=visibility['trigger_bottom_norm']:close=True
            if metric and not close and visibility.get('enabled') and obj['bbox_norm'][3]>=visibility['blur_bottom_norm']:
                raise NearHold('Chai vào vùng kẹp che trước khi tới khoảng dừng; không tiến mù, chỉnh góc camera/vùng đo')
            visual_handoff = bool(self.cfg.get('confirmed_grasp',{}).get('enabled')) if self.cfg.get('confirmed_grasp') else False
            if not metric and not visual_handoff:
                close = close and obj['extent']['rw'] >= c['handoff_min_rw'] and obj['extent']['rh'] >= c['handoff_min_rh']
                if not close and visibility.get('enabled') and obj['bbox_norm'][3]>=visibility['blur_bottom_norm']:
                    raise NearHold('Vật vào vùng kẹp che nhưng chưa đủ điều kiện chuyển tay; giữ bánh, chưa xác nhận tầm gắp')
            at_goal=abs(ex)<=c['x_tolerance_norm'] and close
            if pending:
                move,old=pending
                if move in ('LEFT','RIGHT'):
                    progress.samples=[]
                    delta=obj['center_norm'][0]-old['center_norm'][0]
                    if abs(delta)>=c['min_progress_norm']:
                        learned=move if delta<0 else {'LEFT':'RIGHT','RIGHT':'LEFT'}[move]
                        self.cfg['calibration']['positive_error_x_move']=learned
                        no_progress=0
                    else:no_progress+=1
                elif windowed:
                    no_progress=0  # Forward evidence has its own bounded window.
                    evidence=progress.observe(old,obj)
                    if evidence:
                        await self.emit('auto_progress_check',f'Qua {evidence["steps"]} nhịp: Δrw={evidence["gains"][0]:+.4f}, Δrh={evidence["gains"][1]:+.4f}, Δđáy={evidence["gains"][2]:+.4f}',progress=evidence)
                        if not evidence['confirmed'] and not at_goal:
                            raise NearHold('Ảnh chưa xác nhận tiến triển trong cả nhóm nhịp; giữ tại chỗ, kiểm tra bánh/PWM và vùng càng che ảnh')
                else:
                    # Compare only at fixed SEARCH, so arm motion cannot fake forward progress.
                    growth=max(obj['extent']['rw']-old['extent']['rw'],obj['extent']['rh']-old['extent']['rh'])
                    down=obj['bbox_norm'][3]-old['bbox_norm'][3]
                    if max(growth,down)>=c['min_progress_norm']:no_progress=0
                    else:no_progress+=1
                if no_progress>=c['max_no_progress'] and not at_goal:raise NearHold('Bánh đã chạy nhưng vật chưa gần lên rõ; kiểm tra chiều motor/PWM, không hạ tay')
                pending=None
            origin = 'hình chiếu S2' if metric and measurement.get('origin')=='s2_floor_projection' else 'mốc trước xe'
            quality = 'ƯỚC LƯỢNG; sai số chưa kiểm chứng' if metric and measurement.get('quality')=='preliminary' else 'đã kiểm chứng'
            distance_text = f' · cách {origin} {measurement["forward_cm"]:.1f} cm ({quality}); dừng {self.cfg["distance"]["stop_forward_cm"]:g} cm' if metric else ''
            await self.emit('auto_base_observe',f'SEARCH · lệch ngang {ex*100:.1f}% · đáy {obj["bbox_norm"][3]*100:.1f}% · rw {obj["extent"]["rw"]:.2f}'+distance_text,
                            label=obj['label'],bin=obj['bin'],center=obj['center'],rw=obj['extent']['rw'],rh=obj['extent']['rh'],
                            bottom_norm=obj['bbox_norm'][3],horizontal_error_norm=ex,approach_close=close,
                            forward_steps=forwards,lateral_steps=laterals,phase='base_approach',distance_mode='floor_cm' if metric else 'relative_image',
                            distance_cm=measurement['forward_cm'] if metric else None,
                            distance_uncertainty_cm=measurement['uncertainty_cm'] if metric else None,
                            distance_quality=measurement.get('quality') if metric else None)
            if abs(ex)<=c['x_tolerance_norm'] and close:
                stable+=1
                if stable>=self.cfg['control']['stable_samples']:
                    await asyncio.to_thread(self.brain.command,'XM',self.brain.epoch,_owner=self.token)
                    message=f'Đã vào vùng dừng {self.cfg["distance"]["stop_forward_cm"]:g} cm và căn ngang ổn định; dừng bánh, hạ tay' if metric else ('Đáy vật tới vùng chuyển tay trước khi càng che; bánh dừng, tay theo ảnh' if visibility.get('enabled') else 'Đã tới vùng tiếp cận và căn ngang; dừng bánh, chuyển sang hạ tay')
                    if visual_handoff and not metric:
                        message='Dừng bánh để hạ tay giữ chai trong ảnh; chưa xác nhận khoảng cách 27 cm'
                    await self.emit('auto_base_done',message,phase='arm_lower')
                    return obj
                continue
            stable=0
            if abs(ex)>c['x_tolerance_norm']:
                if laterals>=c['max_lateral_steps']:raise NearHold('Đã hết số nhịp căn ngang; không hạ tay')
                move=self.cfg['calibration']['positive_error_x_move']
                if ex<0:move={'LEFT':'RIGHT','RIGHT':'LEFT'}[move]
                laterals+=1;duration=c['lateral_ms']
            else:
                if c['max_forward_steps'] is not None and forwards>=c['max_forward_steps']:
                    raise NearHold('Đã hết số nhịp tiến nhưng chưa tới vùng gần; không hạ tay')
                move='FORWARD';forwards+=1;duration=c['forward_ms']
                if visibility.get('enabled') and obj['bbox_norm'][3]>=visibility['trigger_bottom_norm']-visibility['near_margin_norm']:
                    duration=min(duration,visibility['near_forward_ms'])
                if metric and measurement['slow']: duration=min(duration,visibility.get('near_forward_ms',60))
            await self.emit('auto_base_move',f'{move} {duration} ms; tay giữ SEARCH',move=move)
            pending=(move,copy.deepcopy(obj))
            await self.motor_step(move,duration_ms=duration)

    async def arm_approach(self):
        """With the base stationary, observe after each bounded S2 step; never drive wheels here."""
        if self.cfg.get('arm_path'): return await self.path_arm_approach()
        if self.cfg.get('visibility',{}).get('enabled'):return await self.visible_arm_approach()
        cal=self.cfg['calibration'];current=self.guard()['hardware']['joints'].get('p1')
        final=cal['grasp_final_s2_us'];step=self.cfg['control']['s2_step_us']
        if type(current) is not int or final<=current:raise NearHold('Không còn hành trình hạ S2; giữ tay tại giới hạn')
        last=None
        for pulse in list(range(current+step,final,step))+[final]:
            self.check_rear()
            if self.sensor(cal['contact_sensor']):
                if last and self.aligned_anchor:await self.close_range_contact(last,'D19 phát hiện giữa các bước hạ')
                raise NearHold('D19 phát hiện trước khi xác nhận căn tay; không hạ thêm')
            await self.move_targets({'p1':pulse},'AUTO hạ S2, bánh đứng yên')
            stable=0
            while True:
                self.check_rear();obj=await self.detect()
                if not obj:raise NearHold('Mất vật khi hạ tay; giữ tay, bánh không chạy theo ảnh cũ')
                for j,v in cal['frozen_joints_us'].items():
                    if self.guard()['hardware']['joints'].get(j)!=v:raise AutoFault('Khớp cố định đã đổi: '+j)
                if obj['extent']['limit_reasons']:await self.close_range_contact(obj)
                stage=relative_stage({'target_norm':cal['relative_target_norm'],'tolerance_norm':cal['relative_tolerance_norm']},self.observed_frame_size)
                ex=(obj['center'][0]-stage['target_px'][0])/self.observed_frame_size[0]
                ey=(obj['center'][1]-stage['target_px'][1])/self.observed_frame_size[1]
                if abs(ex)>self.cfg['approach']['arm_x_tolerance_norm']:
                    self.aligned_anchor=None
                    raise NearHold('Vật lệch ngang khi tay đang hạ; giữ tay, chỉnh lại SEARCH trước khi tiến')
                centered=abs(ex)<=cal['relative_tolerance_norm'][0] and abs(ey)<=cal['relative_tolerance_norm'][1]
                await self.emit('auto_arm_observe',f'S2 {pulse} µs · lệch {ex*100:.1f}%, {ey*100:.1f}% · bánh đứng yên',
                                s2_us=pulse,error_norm=[ex,ey],rw=obj['extent']['rw'],rh=obj['extent']['rh'],phase='arm_lower')
                if centered:
                    stable+=1
                    if stable<self.cfg['control']['stable_samples']:continue
                    self.record_alignment(obj)
                    if self.sensor(cal['contact_sensor']):await self.close_range_contact(obj,'Ảnh đã căn và D19 phát hiện')
                else:
                    self.aligned_anchor=None
                    if self.sensor(cal['contact_sensor']):raise NearHold('D19 thấy vật nhưng ảnh chưa căn; không đóng kẹp')
                last=obj;break
        if last and self.aligned_anchor:
            await self.close_range_contact(last,'Đã tới giới hạn hạ S2; không hạ thêm, chờ D19')
        raise NearHold(f'Đã tới giới hạn hạ S2 {final} µs; dừng tay, không hạ thêm hoặc đóng kẹp')

    async def visible_arm_approach(self):
        cal=self.cfg['calibration'];v=self.cfg['visibility'];m=self.cfg['motion']
        previous=None;stable=no_response=wrong_response=0;checked_pulse=None
        final=cal['grasp_final_s2_us']
        while True:
            self.check_rear();s=self.guard();current=s['hardware']['joints'].get('p1')
            if type(current) is not int or not cal['stages'][0]['s2_us']<=current<=final:
                raise NearHold('S2 ngoài hành trình theo ảnh; không di chuyển')
            if s['drive']['active'] or any(s['hardware'].get('motors',[])):
                raise AutoFault('Bánh chưa đứng yên; không hạ tay')
            for j,pulse in cal['frozen_joints_us'].items():
                if s['hardware']['joints'].get(j)!=pulse:raise AutoFault('Khớp cố định đã đổi: '+j)
            obj=await self.detect()
            if not obj:raise NearHold('Chưa xác nhận vật; giữ nguyên tay')
            if obj['extent']['limit_reasons']:await self.close_range_contact(obj)
            ex=obj['center_norm'][0]-cal['relative_target_norm'][0]
            ey=obj['center_norm'][1]-cal['relative_target_norm'][1]
            contact=self.sensor(cal['contact_sensor'])
            if abs(ex)>self.cfg['approach']['arm_x_tolerance_norm']:
                self.aligned_anchor=None;raise NearHold('Vật lệch ngang khi tay hạ; không chạy bánh với tay thấp')
            centered=abs(ex)<=cal['relative_tolerance_norm'][0] and abs(ey)<=cal['relative_tolerance_norm'][1]
            if centered:
                stable+=1
                if stable<self.cfg['control']['stable_samples']:continue
                self.record_alignment(obj)
                if contact:await self.close_range_contact(obj,'Đã căn ảnh và D19 phát hiện; dừng tay')
            else:
                stable=0;self.aligned_anchor=None
                if contact:raise NearHold('D19 thấy vật nhưng chưa căn đúng; không hạ hoặc đóng kẹp')
            if previous and current>previous[0] and checked_pulse!=current:
                checked_pulse=current
                delta=obj['bbox_norm'][3]-previous[1]['bbox_norm'][3]
                no_response=no_response+1 if abs(delta)<v['min_response_norm'] else 0
                wrong_response=wrong_response+1 if delta>v['response_noise_norm'] else 0
                if wrong_response>=2:raise NearHold('Hạ S2 làm đáy vật xuống gần càng; kiểm tra chiều chuyển động camera')
                if no_response>=v['max_no_response']:raise NearHold('Không thấy phản hồi ảnh sau các bước S2; không hạ mù')
            await self.emit('auto_arm_follow',f'S2 {current} µs · đáy {obj["bbox_norm"][3]*100:.1f}% · tay theo ảnh, bánh đứng yên',
                            s2_us=current,bottom_norm=obj['bbox_norm'][3],error_norm=[ex,ey],phase='arm_follow')
            if current>=final:
                if self.aligned_anchor:await self.close_range_contact(obj,'Đã tới giới hạn S2; giữ tay, chờ IR')
                raise NearHold('Đã tới giới hạn S2; không hạ thêm khi chưa xác nhận gắp')
            try:step=visible_step(obj,current,final,previous,m,v)
            except ValueError as exc:
                # Return only to the immediately preceding observed pose, never scan blindly.
                if previous and previous[0]<current and not contact:
                    await self.move_targets({'p1':previous[0]},'Rút S2 một bước để giữ vật trong ảnh',gentle=True)
                    await self.emit('auto_visibility_recover','Đã rút về bước ảnh trước; giữ tại chỗ để kiểm tra')
                raise NearHold(str(exc)) from exc
            previous=(current,copy.deepcopy(obj));stable=0
            self.aligned_anchor=None
            if self.sensor(cal['contact_sensor']):continue
            await self.move_targets({'p1':min(final,current+step)},'Hạ S2 mềm theo ảnh mới',gentle=True)

    async def confirm_close_pose(self, obj):
        reference=self.cfg['confirmed_grasp']
        w,h=self.observed_frame_size
        if abs((w/h)/(1920/1080)-1)>.01:
            raise NearHold('Ảnh gắp 27 cm dùng tỷ lệ 16:9; không dùng hình học ảnh khác')
        await asyncio.to_thread(self.brain.command,'XM',self.brain.epoch,_owner=self.token)
        for _ in range(reference['stable_frames']-1):
            self.check_rear()
            current=await self.detect()
            joints=self.guard()['hardware']['joints']
            if not current or not matches_close_view(current,joints,reference):
                self.aligned_anchor=None
                raise NearHold('Ảnh gắp gần chưa ổn định; giữ tay và bánh, chưa đóng kẹp')
            obj=current
        self.record_alignment(obj)
        await self.emit('auto_grasp_pose_confirmed',
            'Đúng cặp xung gắp đã đo ở mốc 27 cm; ảnh gần ổn định, chờ D19. Không suy ra khoảng cách chai hiện tại',
            s2_us=reference['pose_us']['p1'],p10_us=reference['pose_us']['p10'],
            reference_forward_cm=reference['measured_forward_cm'],distance_cm=None,
            reference_lens_cm=reference.get('measured_lens_cm'),
            phase='grasp_contact')
        await self.close_range_contact(obj,'Ảnh gần tại tư thế gắp đã xác nhận; giữ nguyên, chờ IR D19')

    async def measured_pose_approach(self):
        """Alternate bounded wheel corrections and two-joint steps with fresh vision.

        A reference image is an image goal, not a distance sensor. Compare wheel
        response only at the SAME arm pose, never across a camera-pose change.
        """
        profile=self.cfg['pose_profile'];path=self.cfg['arm_path'];cal=self.cfg['calibration']
        counts={};pending=None;previous_arm=None;no_progress=0;arm_no_response=0;stable=0
        final=path['end']['p1']
        while True:
            self.check_rear();state=await self.fresh_hardware();hw=state['hardware'];p1=hw['joints'].get('p1')
            if state['drive']['active'] or any(hw.get('motors',[])):
                raise AutoFault('Bánh chưa đứng yên trước khi quan sát/hạ tay')
            expected=targets_at(path,p1)
            if any(hw['joints'].get(j)!=v for j,v in expected.items()):
                raise AutoFault('P1/P10/P11/D2 không khớp quỹ đạo đã khai báo')
            obj=await self.detect()
            if obj is None:
                raise NearHold('Chưa thấy chai; giữ tại chỗ, không hạ theo số cm tham chiếu')
            w,h=self.observed_frame_size
            if abs((w/h)/(1920/1080)-1)>.01:
                raise NearHold('Các mốc USB dùng ảnh 16:9; cần giữ đúng tỷ lệ khung hình')
            if self.selected_bin is None:
                await self.prepare_bin(obj['bin'])
                continue  # Bin movement invalidates the previous frame timestamp.
            if self.selected_bin!=obj['bin']:
                raise NearHold('Vật/ngăn không khớp sau khi đã chốt mục tiêu')
            if self.sensor(cal['contact_sensor']) and p1!=final:
                raise NearHold('D19 xuất hiện trước tư thế gắp đã đo; phanh và giữ tay, không hạ mù')
            problem=view_problem(obj,profile,p1)
            if problem:raise NearHold(problem)
            ref=reference_at(profile,p1)
            ex,ey=view_error(obj,ref)
            if pending:
                old_p1,move,old_error=pending
                if old_p1!=p1:raise AutoFault('Không so phản hồi bánh giữa hai tư thế camera')
                error=correction_error(obj,ref,move);gain=old_error-error
                if gain < -.015:
                    raise NearHold('Ảnh đi xa vùng mục tiêu sau nhịp '+move+'; kiểm tra chiều bánh/thời gian')
                no_progress=no_progress+1 if gain<profile['min_progress_norm'] else 0
                if no_progress>=profile['max_no_progress']:
                    raise NearHold('Bánh đã nhích nhưng ảnh không tiến triển; không chạy thêm theo thời gian mù')
                pending=None
            if previous_arm:
                before=previous_arm;previous_arm=None
                delta=max(abs(a-b) for a,b in zip(obj['bbox_norm'],before))
                if delta>profile['max_arm_feature_jump_norm']:
                    raise NearHold('Khung chai đổi quá mạnh sau một bước tay; giữ nguyên để chỉnh mốc')
                arm_no_response=arm_no_response+1 if delta<.002 else 0
                if arm_no_response>=4:
                    raise NearHold('Ảnh không đổi qua nhiều bước P1/P10; kiểm tra servo/camera trước khi hạ tiếp')
            estimate=search_lens_estimate(obj,profile.get('search_samples',[]),p1)
            await self.emit('auto_pose_observe',
                f'Mốc ảnh {ref["stage_index"]+1} · P1={p1}/P10={expected["p10"]} · tham chiếu ống kính {ref["lens_cm"]:.1f} cm',
                phase='measured_pose',s2_us=p1,p10_us=expected['p10'],
                pose_stage=ref['stage_index']+1,reference_lens_cm=ref['lens_cm'],
                camera_range_estimate_cm=estimate,
                camera_range_quality='empirical_same_bottle_search' if estimate is not None else None,
                target_x_norm=ref['target_x_norm'],target_bottom_norm=ref['target_bottom_norm'],
                x_error_norm=ex,bottom_error_norm=ey,distance_cm=None)
            move=wheel_correction(obj,ref,cal['positive_error_x_move'])
            if move:
                stable=0
                if p1>=profile['max_wheel_p1_us'] or not ref['wheels_enabled']:
                    raise NearHold('Đã vào vùng tắt bánh nhưng ảnh chưa khớp; giữ nguyên, chưa đóng kẹp')
                kind='lateral' if move in ('LEFT','RIGHT') else ('forward' if move=='FORWARD' else 'back')
                key=(ref['stage_index'],kind);counts[key]=counts.get(key,0)+1
                if ref['max_'+kind+'_steps'] is not None and counts[key]>ref['max_'+kind+'_steps']:
                    raise NearHold('Đã hết số nhịp '+kind+' trong đoạn; chỉnh STAGE_TIMING hoặc mốc ảnh')
                pending=(p1,move,correction_error(obj,ref,move))
                await self.emit('auto_pose_drive',f'{move} {ref[kind+"_ms"]} ms danh định; tay đứng yên, chờ ảnh mới',
                                move=move,nominal_move_ms=ref[kind+'_ms'])
                await self.motor_step(move,ref[kind+'_ms'],settle_ms=ref['settle_ms'])
                continue
            stable+=1
            if stable<profile['stable_frames']:continue
            stable=0
            if p1==final:
                if not matches_close_view(obj,hw['joints'],self.cfg.get('confirmed_grasp')):
                    raise NearHold('Ảnh ở mốc cuối chưa đủ vùng chai; giữ nguyên, không đóng kẹp')
                return await self.confirm_close_pose(obj)
            self.check_rear()
            await self.fresh_hardware()
            if self.sensor(cal['contact_sensor']):
                raise NearHold('D19 xuất hiện ngay trước bước hạ; giữ tay')
            # Stop exactly on every measured knot even if a user changes step size.
            next_knot=next(k['p1'] for k in path.get('knots',[path['end']]) if k['p1']>p1)
            next_p1=min(p1+self.cfg['control']['s2_step_us'],next_knot,final)
            previous_arm=list(obj['bbox_norm'])
            await self.emit('auto_pose_lower',f'Hạ P1/P10 tới {next_p1} µs; lấy ảnh mới sau khi tay dừng')
            await self.move_targets(targets_at(path,next_p1),'AUTO mốc ảnh P1/P10',gentle=True,
                                    settle_ms=ref['settle_ms'],arm_step_ms=ref['arm_step_ms'])

    async def path_arm_approach(self):
        """Observe at each point on the declared P1/P10 command path.

        The endpoints are measured; intermediate PWM commands are interpolated.
        They do not claim a measured Cartesian path or an angle calibration.
        """
        path=self.cfg['arm_path'];cal=self.cfg['calibration']
        v=self.cfg['visibility'];m=self.cfg['motion'];final=path['end']['p1']
        previous=None;last_response=None;no_response=0;stable=0
        wheel_pending=None;wheel_count=wheel_no_progress=0
        while True:
            self.check_rear();s=self.guard();hw=s['hardware'];current=hw['joints'].get('p1')
            if s['drive']['active'] or any(hw.get('motors',[])):
                raise AutoFault('Bánh phải đứng yên trước khi chạy quỹ đạo P1/P10')
            expected=targets_at(path,current)
            if any(hw['joints'].get(j)!=value for j,value in expected.items()):
                raise AutoFault('P1/P10/P11/D2 không khớp quỹ đạo; giữ tay')
            obj=await self.detect()
            if not obj: raise NearHold('Mất vật trên quỹ đạo P1/P10; giữ tay')
            if (self.cfg.get('confirmed_grasp') or {}).get('enabled'):
                if matches_close_view(obj,hw['joints'],self.cfg['confirmed_grasp']):
                    await self.confirm_close_pose(obj)
                if self.sensor(cal['contact_sensor']):
                    raise NearHold('D19 kích hoạt trước khi xác nhận đúng tư thế/ảnh gắp cuối; giữ tay, không đóng kẹp')
            if obj['extent']['limit_reasons']: await self.close_range_contact(obj)
            ex=obj['center_norm'][0]-cal['relative_target_norm'][0]
            ey=obj['center_norm'][1]-cal['relative_target_norm'][1]
            if abs(ex)>self.cfg['approach']['arm_x_tolerance_norm']:
                raise NearHold('Vật lệch ngang trên quỹ đạo P1/P10; không chạy bánh khi tay thấp')
            reference=self.cfg.get('confirmed_grasp') or {}
            follow=reference.get('base_follow',{}) if reference.get('enabled') else {}
            if wheel_pending:
                if current!=wheel_pending[0]: raise AutoFault('Tay đã đổi trong nhịp bánh trung gian')
                old=wheel_pending[1]
                gain=max(obj['bbox_norm'][3]-old['bbox_norm'][3],
                         obj['extent']['rw']-old['extent']['rw'],obj['extent']['rh']-old['extent']['rh'])
                wheel_no_progress=wheel_no_progress+1 if gain<self.cfg['approach']['min_progress_norm'] else 0
                wheel_pending=None
                if wheel_no_progress>=follow['max_no_progress']:
                    raise NearHold('Nhịp bánh trung gian chưa có tiến triển qua ảnh; giữ tay/bánh')
            if (follow.get('enabled') and current<=follow['max_p1_us']
                    and obj['bbox_norm'][3]<follow['trigger_bottom_norm']):
                if abs(ex)>self.cfg['approach']['x_tolerance_norm']:
                    raise NearHold('Chai lệch ngang khi tay đã hạ; chưa cho bánh tiến')
                if wheel_count>=follow['max_steps']:
                    raise NearHold('Hết số nhịp bánh trung gian; chưa xác nhận tầm gắp')
                wheel_count+=1;wheel_pending=(current,copy.deepcopy(obj))
                self.aligned_anchor=None
                await self.emit('auto_view_follow',
                    f'Tay đứng P1={current}/P10={expected["p10"]}; tiến {follow["pulse_ms"]} ms rồi chụp lại',
                    phase='view_follow',s2_us=current,p10_us=expected['p10'],view_forward_steps=wheel_count)
                await self.motor_step('FORWARD',duration_ms=follow['pulse_ms'])
                previous=None;last_response=None;stable=no_response=0
                continue
            centered=abs(ex)<=cal['relative_tolerance_norm'][0] and abs(ey)<=cal['relative_tolerance_norm'][1]
            if centered:
                stable+=1
                if stable<self.cfg['control']['stable_samples']: continue
                self.record_alignment(obj)
                if self.sensor(cal['contact_sensor']):
                    await self.close_range_contact(obj,'Ảnh đã căn và D19 xác nhận trên quỹ đạo P1/P10')
            else:
                stable=0;self.aligned_anchor=None
                if self.sensor(cal['contact_sensor']):
                    raise NearHold('D19 kích hoạt nhưng ảnh chưa căn; giữ tay, chưa đóng kẹp')
            if previous and current!=last_response:
                last_response=current
                old=previous[1]
                response=max(abs(a-b) for a,b in zip(obj['bbox_norm'],old['bbox_norm']))
                no_response=no_response+1 if response<v['min_response_norm'] else 0
                if no_response>=v['max_no_response']:
                    raise NearHold(f'Ảnh chưa phản hồi qua {no_response} bước P1/P10; giữ tại chỗ')
            await self.emit('auto_arm_follow',f'P1 {current} µs · P10 {expected["p10"]} µs · đích {final}/{path["end"]["p10"]} · bánh đứng yên',
                            s2_us=current,p10_us=expected['p10'],bottom_norm=obj['bbox_norm'][3],
                            error_norm=[ex,ey],phase='arm_follow')
            if current==final:
                if self.aligned_anchor: await self.close_range_contact(obj,'Đã tới cặp xung gắp đã đo; chờ D19')
                raise NearHold('Đã tới P1/P10 gắp; chưa xác nhận vật, không hạ thêm')
            try:
                step=visible_step(obj,current,final,previous,m,v)
            except ValueError as exc:
                if previous and not self.sensor(cal['contact_sensor']):
                    await self.move_targets(targets_at(path,previous[0]),'Rút cả P1/P10 về bước đã quan sát',gentle=True)
                raise NearHold(str(exc)) from exc
            previous=(current,copy.deepcopy(obj));stable=0;self.aligned_anchor=None
            if self.sensor(cal['contact_sensor']): continue
            await self.move_targets(targets_at(path,current+step),'Hạ P1/P10 theo ảnh mới',gentle=True)

    async def align(self, stage, index):
        stable = moves = attempts = 0
        pending = None
        no_response = 0
        relative = self.cfg['calibration']['mode'] == 'relative'
        limit = self.cfg['control']['max_align_moves_per_stage']
        while moves <= limit:
            self.guard()
            self.check_rear()
            obj = await self.detect()
            if not obj:
                self.aligned_anchor = None
                if self.target: raise AutoFault('Mất vật trong khi tiếp cận; dừng, không chạy theo tọa độ cũ')
                attempts += 1
                if attempts > self.cfg['control']['max_no_target_attempts']: raise AutoFault('Chưa tìm thấy rác; đã dừng')
                await self.emit('auto_search','Chưa có rác, chờ ảnh mới',label=None,category=None,bin=None,center=None)
                await self.wait(self.cfg['control']['no_target_retry_ms']/1000)
                continue
            if obj['extent']['limit_reasons']:
                await self.close_range_contact(obj)
            if self.selected_bin is None:
                # Classify and finish rotating BEFORE approaching/grasping.
                await self.emit('auto_classified',f'{obj["label"]} · BIN {obj["bin"]}',
                                label=obj['label'],category=obj.get('category'),bin=obj['bin'])
                await self.prepare_bin(obj['bin'])
                continue  # Re-observe after the bin motion; do not align from the old frame.
            active_stage = relative_stage(stage,self.observed_frame_size) if relative else stage
            if pending:
                old,axis,previous_move = pending
                delta = [b-a for a,b in zip(old,obj['center_norm'])]
                minimum = self.cfg['control']['relative_min_response_norm']
                if abs(delta[axis]) < minimum:
                    no_response += 1
                    if no_response >= self.cfg['control']['relative_max_no_response']:
                        raise AutoFault('Không thấy phản hồi ảnh rõ sau các nhịp bánh; đã dừng')
                else:
                    if abs(delta[1-axis]) > 2*abs(delta[axis]):
                        raise AutoFault('Camera dịch chủ yếu sang trục khác; kiểm tra hướng lắp')
                    no_response = 0
                    opposite = {'LEFT':'RIGHT','RIGHT':'LEFT','FORWARD':'BACK','BACK':'FORWARD'}
                    learned = previous_move if delta[axis] < 0 else opposite[previous_move]
                    key = 'positive_error_x_move' if axis==0 else 'positive_error_y_move'
                    if learned != self.cfg['calibration'][key]:
                        self.cfg['calibration'][key] = learned
                        await self.emit('auto_direction',f'Phản hồi ảnh: cập nhật chiều bù {learned}')
                pending = None
            move,error = choose_direction(obj['center'],active_stage,self.cfg['calibration'])
            error_norm = [v/size for v,size in zip(error,self.observed_frame_size)]
            message = f'S2 {stage["s2_us"]} µs · lệch {error_norm[0]*100:.1f}%, {error_norm[1]*100:.1f}%' if relative else f'Mốc S2 {stage["s2_us"]} µs · lệch {error[0]:.1f}, {error[1]:.1f} px'
            await self.emit('auto_align',message,
                            label=obj['label'],category=obj.get('category'),bin=obj['bin'],center=obj['center'],
                            stage_index=index,error_px=error,error_norm=error_norm,s2_us=stage['s2_us'],
                            rw=obj['extent']['rw'],rh=obj['extent']['rh'],vision_mode='align')
            if move is None:
                stable += 1
                if stable >= self.cfg['control']['stable_samples']:
                    self.record_alignment(obj)
                    return obj
            else:
                self.aligned_anchor = None
                stable = 0
                if self.sensor(self.cfg['calibration']['contact_sensor']):
                    raise AutoFault('IR đã phát hiện nhưng vật chưa căn đúng; không chạy bánh sát kẹp')
                if moves >= limit: break
                moves += 1
                if relative:
                    axis = 0 if move in ('LEFT','RIGHT') else 1
                    pending = (obj['center_norm'][:],axis,move)
                await self.motor_step(move)
        raise AutoFault('Không căn được vật trong số bước cho phép')

    async def lower_to(self, stage, previous_stage, index):
        pulse = stage['s2_us']
        current = self.guard()['hardware']['joints'].get('p1')
        if type(current) is not int: raise AutoFault('Chưa biết xung S2')
        step = self.cfg['control']['s2_step_us']*(1 if pulse >= current else -1)
        cal = self.cfg['calibration']
        for next_pulse in list(range(current+step,pulse,step))+[pulse]:
            if self.sensor(cal['contact_sensor']):
                raise AutoFault('IR kích hoạt trước mốc căn cuối; giữ tay, không hạ tiếp hoặc đóng kẹp')
            await self.move_targets({'p1':next_pulse},'AUTO hạ S2')
            fraction = (next_pulse-previous_stage['s2_us'])/(pulse-previous_stage['s2_us'])
            intermediate = copy.deepcopy(stage)
            intermediate['s2_us'] = next_pulse
            intermediate['target_px'] = [a+fraction*(b-a) for a,b in zip(previous_stage['target_px'],stage['target_px'])]
            # Re-observe after every S2 step; only interpolate inside measured endpoint poses.
            await self.align(intermediate,index)
        # Xung các khớp cố định phải giữ nguyên trong toàn bộ tiếp cận.
        actual = self.guard()['hardware']['joints']
        for j,v in cal['frozen_joints_us'].items():
            if actual.get(j) != v: raise AutoFault('Khớp cố định đã thay đổi: '+j)

    async def check_grip_or_drop(self, where):
        """D19 is meaningful only at grasp or while carrying, never while searching.

        Pause for up to 15 seconds on a suspected loss. Stable return for 300ms
        resumes the action; absence throughout the full window causes recovery.
        A missing sample cannot count as confirmation of a lost bottle.
        """
        pin = self.cfg['calibration']['hold_sensor']
        await self.fresh_hardware()
        if self.sensor(pin):
            return
        await asyncio.to_thread(self.brain.command, 'XM', self.brain.epoch, _owner=self.token)
        duration=self.cfg['drop_recovery']['loss_check_seconds']
        stable_s=self.cfg['drop_recovery']['loss_restore_ms']/1000
        await self.emit('auto_drop_verify',
                        f'D19 mất tín hiệu {where}; dừng, kiểm tra tối đa {duration} giây trước khi kết luận rơi',
                        phase='drop_verify', drop_check_seconds=duration)
        end=time.monotonic()+duration
        restored_since=None
        while time.monotonic()<end:
            self.guard()
            await self.fresh_hardware()
            if self.sensor(pin):
                if restored_since is None:
                    restored_since=time.monotonic()
                if time.monotonic()-restored_since>=stable_s:
                    await self.emit('auto_drop_signal_restored',
                                    'D19 trở lại ổn định; tiếp tục giữ chai, không lùi',
                                    phase='carrying')
                    return
            else:
                restored_since=None
            await self.wait(.08)
        raise LostGrip(f'D19 mất liên tục {duration}s {where}')

    async def recover_lost_grip(self, reason):
        """Cancel drop-off, keep lid closed and retract along the measured path.

        Do not mark sorting complete, nor open the lid/reward. An unsafe pose
        still raises AutoFault and invokes the existing emergency STOP route.
        """
        # A confirmed drop cancels the carrying state BEFORE making recovery
        # movements. Keep D3 closed until the arm reaches a safe SEARCH pose.
        self.holding_object = False
        await asyncio.to_thread(self.brain.command, 'XM', self.brain.epoch, _owner=self.token)
        await self.emit('auto_drop_detected', reason+'; hủy lượt gắp, không mở nắp/thả')
        hw = self.guard()['hardware']
        joints = hw.get('joints', {})
        path = self.cfg.get('arm_path')
        if path:
            current = joints.get('p1')
            place = self.arm_pose('PLACE')
            search_pose = self.arm_pose('SEARCH')
            # A D19 loss can cancel a multi-joint PLACE job halfway through.
            # Only allow automated retreat when every joint is inside the
            # known SEARCH->PLACE joint interval, never from an unknown pose.
            inside_place_path = (current == search_pose['p1'] and
                all(type(joints.get(j)) is int and
                    min(search_pose[j], place[j]) <= joints[j] <= max(search_pose[j], place[j])
                    for j in ('p10', 'p11')) and type(joints.get('d2')) is int)
            if all(joints.get(j) == v for j, v in place.items() if j != 'd2') or inside_place_path:
                await self.move_targets(search_pose, 'DROP rút tay từ PLACE hoặc chặng PLACE chưa xong')
            elif type(current) is int and path['start']['p1'] <= current <= path['end']['p1']:
                await self.retract_command_path('DROP thu tay theo quỹ đạo', require_hold=False, preserve_wrist=True)
                await self.move_targets(self.arm_pose('SEARCH'), 'DROP về SEARCH')
            else:
                raise AutoFault('DROP: tay ở tư thế không xác định; không tự thu, cần STOP')
        else:
            await self.move_targets({'p1':self.arm_pose('SEARCH')['p1']}, 'DROP thu S2')
            await self.move_targets(self.arm_pose('SEARCH'), 'DROP về SEARCH')
        # Check again: the bottle may be stuck in the jaws while D19 was briefly blind.
        # Never open the jaws or retry driving when the sensor still sees something.
        await self.fresh_hardware()
        if self.sensor(self.cfg['calibration']['hold_sensor']):
            raise NearHold('D19 phát hiện vật trở lại sau thu tay; giữ kẹp, cần kiểm tra thủ công')
        # Keep the lid as-is until the arm has cleared the drop-off area.
        await self.move_targets({'p4': self.brain.config['lid']['close']}, 'DROP đóng nắp sau thu tay')
        await self.move_targets({'d3':self.brain.config['grip']['open']}, 'DROP mở kẹp ở SEARCH')
        self.holding_object = False
        self.vision_paused = False
        self.final_authorized = None
        self.locked_bin = None
        self.locked_label = None
        self.target = None
        self.selected_bin = None
        self.search_orientation = None
        self.state['orientation'] = {'valid': False, 'reason': 'Đã hủy lượt gắp do mất vật'}
        self.clear_overlay()
        await self.emit('auto_drop_recovered','Đã thu tay; lượt thất bại không tính hoàn tất và không cấp quà',
                        label=None, bin=None, grasp_trigger=None)

    async def reverse_search_after_drop(self):
        """Backtrack at SEARCH up to ten *observed* steps, then hand off to scan.

        A fresh image is read before the first movement and after every BACK.
        Only a real camera result (not D19 and not a stale cached box) stops
        backtracking. The ordinary cycle will reacquire/classify in multiple
        frames and remeasure orientation before attempting another grasp.
        """
        recovery = self.cfg['drop_recovery']
        await asyncio.to_thread(self.brain.set_auto_ir_drive_guard, False, self.token)
        search = self.arm_pose('SEARCH')
        joints = self.guard()['hardware'].get('joints', {})
        if any(joints.get(j) != pulse for j, pulse in search.items()):
            raise NearHold('DROP: tay chưa ở đúng SEARCH, không cho lùi')
        if joints.get('d3') != self.brain.config['grip']['open']:
            raise NearHold('DROP: D3 chưa mở tại SEARCH, không cho lùi')
        if joints.get('p4') != self.brain.config['lid']['close']:
            raise NearHold('DROP: nắp chưa đóng, không cho lùi')
        if self.sensor(self.cfg['calibration']['hold_sensor']):
            raise NearHold('DROP: D19 vẫn thấy vật, không cho lùi')
        if self.brain.rear_requested():
            raise NearHold('DROP: D18 đang nhận rác, không cho robot lùi')
        # A motor pulse must use exactly the same nominal value as camera
        # FORWARD, not the short normal pose correction "back_ms" setting.
        step_ms = self.cfg['pose_profile']['stages'][0]['forward_ms']
        # Match the actual SEARCH FORWARD pulse (400ms nominal), not a legacy back_ms.

        async def fresh_object():
            for attempt in range(3):
                self.guard()
                if self.brain.rear_requested():
                    raise NearHold('DROP: D18 kích hoạt trong khi tìm chai; không tiếp tục lùi')
                # An unsuccessful look must not make the next frame associate
                # against a nonconfirmed candidate from a previous location.
                self.target = None
                self.lock_prediction = None
                try:
                    obj = await self._detect_once(require_bbox=True, interrupt_for_rear=False)
                except (StaleDetection, VisionUnavailable) as exc:
                    if attempt == 2:
                        raise NearHold('DROP: lỗi camera/AI 3 lần, không lùi mù') from exc
                    await self.emit('auto_drop_camera_wait',
                                    'DROP: ảnh chưa hợp lệ, đứng yên thử camera lại')
                    await self.wait(self.cfg['target_lock']['retry_ms'] / 1000)
                    continue
                finally:
                    # Never carry over box identity across a reversed step.
                    self.target = None
                    self.lock_prediction = None
                return obj
            return None

        maximum = recovery['reverse_max_steps']
        await self.emit('auto_drop_reverse_start',
                        f'Đã về SEARCH; kiểm tra ảnh rồi lùi tối đa {maximum} nhịp, '
                        f'mỗi nhịp như FORWARD {step_ms}ms danh định',
                        reverse_step=0, reverse_limit=maximum, phase='drop_backtrack')
        for step in range(maximum + 1):
            self.guard()
            if await fresh_object():
                await asyncio.to_thread(self.brain.command, 'XM', self.brain.epoch, _owner=self.token)
                await self.emit('auto_drop_target_found',
                                f'Đã thấy vật trong ảnh tại SEARCH sau {step} nhịp lùi; '
                                'dừng lùi, xác nhận phân loại và đo góc lại',
                                reverse_step=step, reverse_limit=maximum, phase='search')
                return True
            if step == maximum:
                break
            await self.emit('auto_drop_reverse',
                            f'Chưa thấy vật: BACK nhịp {step+1}/{maximum}',
                            reverse_step=step+1, reverse_limit=maximum, phase='drop_backtrack')
            await self.motor_step('BACK', duration_ms=step_ms,
                                  settle_ms=recovery['reverse_settle_ms'])
        await self.emit('auto_drop_reverse_exhausted',
                        f'Đã lùi {maximum} nhịp vẫn không thấy vật; '
                        'chuyển sang quay phải tìm kiếm',
                        reverse_step=maximum, reverse_limit=maximum, phase='search')
        return False

    async def handle_lost_grip(self, reason):
        """One controlled retry after a confirmed loss, then a latched HOLD.

        Reacquire and remeasure from SEARCH. Never reuse the old target's angle,
        bin assignment or D19 latch after an object has fallen.
        """
        self.drop_failures += 1
        await self.emit('auto_grip_lost',
                        f'D19 xác nhận mất chai (lần {self.drop_failures}); hủy lượt bỏ rác, thu tay',
                        drop_failures=self.drop_failures)
        await self.recover_lost_grip(reason)
        maximum = self.cfg.get('drop_recovery',{}).get('max_retries',1)
        if self.drop_failures > maximum:
            raise NearHold(f'Chai rơi {self.drop_failures} lần liên tiếp: AUTO HOLD; '
                           'không tự thử gắp thêm; STOP để kiểm tra càng, D19 và chai')
        await self.emit('auto_drop_retry',
                        f'Chai đã rơi; sẽ nhận diện/đo góc lại từ SEARCH (thử lại {self.drop_failures}/{maximum})',
                        drop_failures=self.drop_failures)
        await self.wait(self.cfg['drop_recovery']['retry_wait_ms']/1000)
        # The loss may have occurred near the old cycle deadline. The next
        # SEARCH/reverse phase has its own bounded movement budget.
        self.deadline = time.monotonic() + self.cfg['control']['max_cycle_s']
        if self.brain.rear_requested():
            self.drop_search_pending = False
            raise RearRequested()  # D18 takes priority over backtracking.
        found = await self.reverse_search_after_drop()
        self.drop_search_pending = not found

    async def confirm_hold(self):
        cal = self.cfg['calibration']
        end = time.monotonic()+cal['grasp_hold_confirm_ms']/1000
        while time.monotonic() < end:
            await self.check_grip_or_drop('sau khi nâng')
            await self.wait(.05)

    async def retract_command_path(self, name, require_hold=False, preserve_wrist=False):
        path=self.cfg['arm_path'];joints=self.guard()['hardware']['joints']
        current=joints.get('p1');expected=targets_at(path,current)
        wrist = joints.get('d2') if require_hold or preserve_wrist else expected['d2']
        expected['d2'] = wrist
        if any(joints.get(j)!=value for j,value in expected.items()):
            raise AutoFault('Không thể rút quỹ đạo: tư thế thực tế không khớp P1/P10')
        step=self.cfg['control']['s2_step_us']
        for pulse in pulse_steps(current,path['start']['p1'],step):
            if require_hold and not self.sensor(self.cfg['calibration']['hold_sensor']):
                await self.check_grip_or_drop('trước bước nâng')
            await self.move_targets(dict(targets_at(path,pulse), d2=wrist),name,gentle=True)
            if require_hold and not self.sensor(self.cfg['calibration']['hold_sensor']):
                await self.check_grip_or_drop('trong khi nâng')

    async def approach_relative(self):
        cal = self.cfg['calibration']
        start = self.guard()['hardware']['joints'].get('p1')
        if type(start) is not int: raise AutoFault('Chưa biết xung S2')
        final = cal['grasp_final_s2_us']
        step = self.cfg['control']['s2_step_us']
        if start >= final: raise AutoFault('S2 không còn hành trình hạ tương đối')
        for pulse in list(range(start+step,final,step))+[final]:
            if self.sensor(cal['contact_sensor']):
                raise AutoFault('IR xuất hiện giữa các bước; giữ tay, cần căn lại trước khi gắp')
            await self.emit('auto_lower',f'Hạ S2 tới {pulse} µs; dùng vùng tham chiếu tương đối')
            await self.move_targets({'p1':pulse},'AUTO hạ S2 tương đối')
            actual = self.guard()['hardware']['joints']
            for j,v in cal['frozen_joints_us'].items():
                if actual.get(j) != v: raise AutoFault('Khớp cố định đã thay đổi: '+j)
            stage = {'s2_us':pulse,'target_norm':cal['relative_target_norm'],
                     'tolerance_norm':cal['relative_tolerance_norm']}
            obj = await self.align(stage,0)
            if self.sensor(cal['contact_sensor']):
                await self.emit('auto_contact',f'IR {cal["contact_sensor"]} thấy vật tại S2 {pulse} µs; dừng hạ')
                return obj
        raise AutoFault('Đã tới giới hạn hạ tương đối nhưng IR chưa thấy vật; không hạ thêm/đóng kẹp')

    async def cycle(self):
        cal = self.cfg['calibration']
        self.check_rear()
        await asyncio.to_thread(self.brain.set_auto_ir_drive_guard, False, self.token)
        self.aligned_anchor = None
        self.vision_paused = False
        self.final_authorized = None
        self.lock_prediction = None
        self.observed_jpeg = None
        self.state.update(phase='search', ir_check_enabled=False, grasp_trigger=None)
        self.state.update(distance_ready=False)
        self.state.update(tof_mm=None,tof_seq=None,tof_valid=False,tof_votes=0,target_bbox=None,target_bbox_norm=None,missing_frames=0)
        self.target=None;self.locked_bin=None;self.locked_label=None
        self.classification_vote=None;self.classification_count=0
        self.search_rotations = 0
        self.search_orientation = None
        self.holding_object = False
        self.front_lid_preopened = False
        self.state.update(label=None,bin=None,orientation={'valid':False,'reason':'Chờ đo góc ở SEARCH'})
        await self.emit('auto_search','Đưa tay về SEARCH; chỉ căn bánh khi tay đứng yên',
                        rw=None,rh=None,near_reasons=[],vision_mode='align')
        search = self.arm_pose('SEARCH')
        search['d3'] = self.brain.config['grip']['open']
        await self.move_targets({'p4':self.brain.config['lid']['close']},'AUTO đóng nắp trước khi tìm')
        await self.move_targets(search,'AUTO SEARCH')
        if not self.cfg.get('box_tof_flow',{}).get('enabled') and (self.sensor(cal['contact_sensor']) or self.sensor(cal['hold_sensor'])):
            raise AutoFault('IR đang kích hoạt ở SEARCH; kiểm tra vật còn trong kẹp/cảm biến')
        self.clear_overlay()
        self.selected_bin = None
        if self.drop_search_pending:
            # After 10 empty BACK checks, start the EXISTING right-turn search
            # immediately (5s rotation + 1s stop); the scan has no global cap.
            self.drop_search_pending = False
            await self.emit('auto_drop_scan_start',
                            'Lùi 10 nhịp không thấy chai: chuyển sang quét phải 5s/dừng 1s',
                            scan_turn=0, scan_total=30, phase='search')
            await self.search_turn_right()
        try:
            if self.cfg.get('box_tof_flow',{}).get('enabled'):
                await BoxToFFlow(self).run()
                raise AutoFault('Luồng box/VL53 chưa xác nhận D19 gắp')
            elif self.cfg.get('pose_profile',{}).get('enabled'):
                await self.measured_pose_approach()
                raise AutoFault('Mốc ảnh chưa xác nhận gắp')
            elif self.cfg['approach']['enabled'] and cal['mode']=='relative':
                await self.base_approach()
                await self.arm_approach()
                raise AutoFault('Chu trình tiếp cận chưa xác nhận gắp')
            elif cal['mode'] == 'relative':
                await self.align(cal['stages'][0],0)
                obj = await self.approach_relative()
            else:
                for i,stage in enumerate(cal['stages']):
                    if i:
                        await self.emit('auto_lower',f'Hạ riêng S2 tới {stage["s2_us"]} µs; S1/trụ/nối giữ nguyên')
                        await self.lower_to(stage,cal['stages'][i-1],i)
                    obj = await self.align(stage,i)
        except CloseRangeContact as contact:
            obj = contact.obj
        if not self.sensor(cal['contact_sensor']):
            raise AutoFault('Đã tới mốc cuối nhưng IR chưa thấy vật; không hạ ngoài mốc đã đo')
        target_bin = obj['bin']
        self.check_rear()
        if self.selected_bin != target_bin: raise AutoFault('Ngăn đã xoay không trùng vật đang gắp')
        if self.cfg.get('box_tof_flow',{}).get('enabled'):
            if self.cfg.get('grasp_orientation',{}).get('enabled'):
                memory = self.search_orientation
                if (not memory or not memory.get('applied')
                        or memory['target_id'] != self.lock_sequence or memory['bin'] != target_bin
                        or self.guard()['hardware']['joints'].get('d2') != memory['pulse_us']):
                    raise NearHold('D2 chưa xoay theo góc lưu ở SEARCH; không cho đóng kẹp')
        else:
            # Legacy/manual non-ToF path keeps its own final-pose orientation behavior.
            await self.operator.rotate_for_grasp(obj)
        await self.check_grip_or_drop('trước khi đóng kẹp')
        await self.emit('auto_grip','Vật đã căn đúng và IR phát hiện; đóng kẹp')
        await self.move_targets({'d3':self.brain.config['grip']['close']},'AUTO đóng kẹp')
        self.holding_object = True
        # D19 contact before D3 closes does not prove the object stayed between the jaws.
        # Observe a full confirmation window after closing, BEFORE any lift.
        await self.emit('auto_grip_check','Đã đóng D3; xác nhận D19 còn giữ chai trước khi nâng')
        await self.confirm_hold()
        # Lift with S2 only, preserve S1/P11/D2; do not reopen the gripper.
        await self.emit('auto_lift','Nâng S2 về SEARCH, giữ kẹp đóng')
        if self.cfg.get('arm_path'):
            await self.retract_command_path('AUTO nâng P1/P10 theo quỹ đạo ngược',require_hold=True)
        else:
            await self.move_targets({'p1':cal['stages'][0]['s2_us']},'AUTO nâng S2')
        await self.confirm_hold()
        single_ir = sensor_pin(cal['contact_sensor']) == sensor_pin(cal['hold_sensor'])
        message = f'IR {cal["hold_sensor"]} vẫn thấy vật sau nâng' if single_ir else 'IR xác nhận giữ vật'
        await self.emit('auto_place',f'{message} · đưa vật tới BIN {target_bin} đã xoay')
        place = self.arm_pose('PLACE')
        place['d2'] = self.guard()['hardware']['joints']['d2']
        await self.move_targets(place,'AUTO PLACE')
        await self.check_grip_or_drop('trước khi thả')
        if self.front_lid_preopened:
            if self.guard()['hardware']['joints'].get('p4') != self.brain.config['lid']['open']:
                raise NearHold('Nắp đã bị đóng sau khi mở sớm; không thả chai khi nắp chưa mở')
            await self.emit('auto_open_lid','Nắp đã mở sẵn sau khi xoay BIN; không lặp lệnh mở khi tay ở PLACE')
        else:
            await self.emit('auto_open_lid','Đã gắp/nâng/đưa vật tới vị trí thả; mở nắp')
            await self.move_targets({'p4':self.brain.config['lid']['open']},'AUTO mở nắp')
        await self.check_grip_or_drop('sau mở nắp, trước khi thả')
        await self.emit('auto_release',f'Thả vật vào BIN {target_bin}')
        self.holding_object = False  # Only the deliberately authorized release ends monitoring.
        await self.move_targets({'d3':self.brain.config['grip']['open']},'AUTO mở kẹp')
        if self.cfg.get('box_tof_flow',{}).get('enabled'):
            # A short settle may finish before the 200ms STATUS poll sees the
            # opened gripper. Require a stable clear signal, not that old sample.
            end=time.monotonic()+2.;clear_since=None;cleared=False
            while time.monotonic()<end:
                if self.sensor(cal['hold_sensor']):clear_since=None
                else:
                    if clear_since is None:clear_since=time.monotonic()
                    if time.monotonic()-clear_since>=.2:
                        cleared=True;break
                await self.wait(.03)
            if not cleared:raise AutoFault('D19 chưa hết phát hiện ổn định sau mở kẹp; giữ tại vị trí thả')
        if self.sensor(cal['hold_sensor']): raise AutoFault('IR vẫn thấy vật sau mở kẹp; chưa xác nhận thả')
        # Retract before closing lid so the lid does not close over the arm.
        await self.move_targets(search,'AUTO rút tay về SEARCH')
        await self.move_targets({'p4':self.brain.config['lid']['close']},'AUTO đóng nắp')
        self.front_lid_preopened = False
        self.state['cycles'] += 1
        self.drop_failures = 0  # A complete deposit starts a fresh retry budget.
        await self.emit('auto_cycle_done','Hoàn tất một lượt theo phản hồi hiện diện IR',bin=target_bin)

    def rear_sensor(self):
        value = self.guard()['hardware'].get('sensors',{}).get('D18')
        if type(value) is not int or value not in (0,1): raise AutoFault('Không đọc được cảm biến D18')
        return value == self.cfg['rear']['active_level']

    async def deposit_manual_carry_before_rear(self, target_bin):
        """D18 priority: deposit a MANUAL-held bottle to its KNOWN BIN first.

        Requires an already-closed gripper, fresh D19 confirmation and a
        recognized arm configuration. Never release into open air, and never
        guess a category, joint angle, or carried-object status.
        """
        if target_bin not in BIN_KEYS:
            raise NearHold('D18: kẹp có vật nhưng chưa biết BIN, cần kiểm tra MANUAL')
        b = self.brain
        # Rear/manual carry always uses the physical gripper IR D19, even if a
        # legacy image-only calibration file points hold_sensor to D16.
        self.cfg['calibration']['hold_sensor'] = 'D19'
        await self.fresh_hardware()
        joints = self.guard()['hardware']['joints']
        if joints.get('d3') != b.config['grip']['close'] or not self.sensor('D19'):
            raise NearHold('D18: không xác nhận chắc vật đang giữ trong kẹp D19')
        search = self.arm_pose('SEARCH')
        place = self.arm_pose('PLACE')
        at_place = all(joints.get(k) == place[k] for k in ('p1','p10','p11'))
        at_search = all(joints.get(k) == search[k] for k in ('p1','p10','p11'))
        if not (at_search or at_place):
            try:
                from arm_path import targets_at
                expected = targets_at(self.cfg['arm_path'], joints.get('p1'))
                valid = all(joints.get(k) == expected.get(k) for k in ('p1','p10','p11'))
            except (ValueError, KeyError, TypeError):
                valid = False
            if not valid:
                raise NearHold('D18: chưa xác minh quỹ đạo tay đang giữ vật; không tự thả')
        self.holding_object = True
        self.selected_bin = target_bin
        await self.emit('rear_carry_priority',
                        f'D18 ưu tiên: đang giữ chai, đưa vào BIN {target_bin} trước khi nhìn sau',
                        phase='rear_carry', bin=target_bin)
        try:
            if not at_place:
                if not at_search:
                    await self.retract_command_path('D18 rút tay giữ chai về SEARCH', require_hold=True, preserve_wrist=True)
                await self.prepare_bin(target_bin, rear=True)
                position = self.arm_pose('PLACE')
                position['d2'] = self.guard()['hardware']['joints']['d2']
                await self.move_targets(position,'D18 đưa chai tới BIN')
            else:
                if joints.get('p9') != b.config['bins'][BIN_KEYS[target_bin]]:
                    raise NearHold('D18: tay ở PLACE nhưng thùng chưa đúng BIN; không thả')
            await self.check_grip_or_drop('trước khi D18 thả vật đang giữ')
            await self.move_targets({'p4':b.config['lid']['open']},'D18 mở nắp đúng BIN')
            await self.check_grip_or_drop('sau khi mở nắp, trước thả')
            # Only a commanded release at a verified PLACE pose ends D19 monitoring.
            self.holding_object = False
            await self.move_targets({'d3':b.config['grip']['open']},'D18 thả chai vào BIN')
            end = time.monotonic() + 2.0
            clear_since = None
            while time.monotonic() < end:
                await self.fresh_hardware()
                if self.sensor('D19'):
                    clear_since = None
                elif clear_since is None:
                    clear_since = time.monotonic()
                elif time.monotonic() - clear_since >= .2:
                    break
                await self.wait(.04)
            else:
                raise NearHold('D18: D19 vẫn thấy vật sau mở kẹp; giữ tư thế để kiểm tra')
            await self.move_targets(search,'D18 rút tay khỏi thùng')
            await self.move_targets({'p4':b.config['lid']['close']},'D18 đóng nắp')
            self.operator.held_bin = None
            await self.emit('rear_carry_done','Đã thả vật đang giữ an toàn trước khi xử lý D18',bin=target_bin)
        finally:
            # If a real error occurs, the Python AUTO error handler locks STOP.
            self.holding_object = False

    async def post_rear_front_search(self):
        """AUTO only: look, BACK up to ten observed steps, then turn right.

        This never runs for MANUAL rear service and never reverses while D18
        remains active or the arm/lid/gripper are not at safe SEARCH poses.
        """
        cfg = self.cfg['rear']
        self.flow = 'front'
        await asyncio.to_thread(self.brain.set_auto_ir_drive_guard,False,self.token)
        await self.fresh_hardware()
        snap = self.guard()['hardware']
        joints = snap.get('joints', {})
        if (any(joints.get(k) != v for k,v in self.arm_pose('SEARCH').items())
                or joints.get('d3') != self.brain.config['grip']['open']
                or joints.get('p4') != self.brain.config['lid']['close']):
            raise NearHold('D18: tay/nắp/kẹp chưa về SEARCH; không cho lùi')
        if self.rear_sensor() or self.brain.rear_requested():
            await self.emit('rear_front_wait_d18','D18 lại có vật; không di chuyển bánh khi phía sau chưa an toàn')
            return False
        self.target = None
        self.lock_prediction = None
        self.search_orientation = None
        maximum = cfg['post_rear_back_steps']
        await self.emit('rear_front_reacquire','D18 đã xong: tìm vật phía trước, nếu chưa thấy thì lùi từng nhịp',
                        phase='post_rear',reverse_step=0,reverse_limit=maximum)
        for idx in range(maximum+1):
            if self.brain.rear_requested() or self.rear_sensor():
                await self.emit('rear_front_wait_d18','D18 vừa phát hiện vật mới; dừng tìm trước')
                return False
            obj = None
            vision_ok = False
            for retry in range(3):
                try:
                    obj = await self._detect_once(require_bbox=True,interrupt_for_rear=True)
                    vision_ok = True
                    break
                except RearRequested:
                    return False
                except (VisionUnavailable, StaleDetection):
                    await self.emit('rear_front_camera_wait',
                                    'Camera chưa có ảnh mới; giữ bánh đứng yên chờ thử lại')
                    await self.wait(.5)
            if not vision_ok:
                # Do not count this as an empty camera frame, and never
                # start blind BACK/rotation when vision is unavailable.
                return False
            self.target = None
            self.lock_prediction = None
            if obj:
                await self.emit('rear_front_found',
                                f'Đã thấy rác phía trước sau {idx} nhịp; khóa vật mới ở SEARCH',
                                phase='search',reverse_step=idx,reverse_limit=maximum)
                return True
            if idx == maximum:
                break
            await self.emit('rear_front_back',f'Chưa thấy: BACK {idx+1}/{maximum} · {cfg["post_rear_back_ms"]} ms, dừng {cfg["post_rear_settle_ms"]} ms',
                            phase='post_rear',reverse_step=idx+1,reverse_limit=maximum)
            try:
                await self.motor_step('BACK', duration_ms=cfg['post_rear_back_ms'],
                                      settle_ms=cfg['post_rear_settle_ms'])
            except RearRequested:
                return False  # New rear event has priority; no blind BACK.
        self.drop_search_pending = True
        await self.emit('rear_front_scan','Đã lùi đủ số nhịp; chuyển quay phải tìm kiếm liên tục',phase='search')
        return False

    async def rear_cycle(self):
        if not self.brain.take_rear_request(self.token): return
        rear = self.cfg['rear']
        self.flow = 'rear'
        self.target = None
        self.selected_bin = None
        self.aligned_anchor = None
        try:
            hw = self.guard()['hardware']
            if hw.get('joints',{}).get('d3') == self.brain.config['grip']['close']:
                await self.deposit_manual_carry_before_rear(self.operator.held_bin)
                hw = self.guard()['hardware']
            await self.emit('rear_look','D18 phát hiện vật; dừng bánh và chuyển sang nhìn sau',flow='rear',label=None,bin=None,center=None)
            # Leave an interrupted front approach by retracting S2 before turning the arm.
            if self.cfg.get('arm_path'):
                # A rear-only service may start from REAR; only reverse an actual front path.
                joints=hw.get('joints',{});path=self.cfg['arm_path']
                current=joints.get('p1')
                search = self.brain.config['poses']['SEARCH']
                if (type(current) is int and path['start']['p1']<=current<=path['end']['p1']
                        and joints.get('p10') == search['p10'] and joints.get('p11') == search['p11']):
                    # V3.16.25: S2 can be extended ALONE while S1 stays at
                    # SEARCH. That is NOT on the coupled arm_path curve.
                    await self.move_targets({'p1':search['p1']},'REAR thu riêng S2 về SEARCH',gentle=True)
                elif type(current) is int and path['start']['p1']<=current<=path['end']['p1']:
                    from arm_path import targets_at
                    expected=targets_at(path,current)
                    if any(joints.get(k)!=expected[k] for k in ('p1','p10','p11')):
                        raise NearHold('D18: tư thế tay phía trước không khớp quỹ đạo đã biết; không quay REAR')
                    await self.retract_command_path('REAR rút P1/P10 theo quỹ đạo ngược',preserve_wrist=True)
                elif all(joints.get(k)==search[k] for k in ('p10','p11')):
                    await self.move_targets({'p1':search['p1']},'REAR rút S2 từ tư thế đã xác nhận')
                elif all(joints.get(k)==self.brain.config['poses']['REAR'].get(k) for k in ('p1','p10','p11')):
                    pass  # Already at REAR after a previous manual observation.
                else:
                    raise NearHold('D18: tay ở vị trí chưa hiệu chuẩn; không tự xoay ra sau')
            else:
                await self.move_targets({'p1':self.brain.config['poses']['SEARCH']['p1']},'REAR rút S2')
            await self.move_targets({'p4':self.brain.config['lid']['close']},'REAR đóng nắp trước khi phân loại')
            await self.move_targets(self.arm_pose('REAR'),'REAR nhìn sau',settle_ms=rear['settle_ms'])
            previous = None
            stable = 0
            obj = None
            for attempt in range(rear['max_detection_attempts']):
                await self.emit('rear_capture','Chụp ảnh mới phía sau và gửi model',attempt=attempt+1)
                current = await self.detect(require_bbox=False,interrupt_for_rear=False)
                if not current:
                    previous = None; stable = 0
                else:
                    key = (current['label'].strip().lower(),current['bin'])
                    stable = stable+1 if key == previous else 1
                    previous = key
                    obj = current
                    await self.emit('rear_classified',f'{current["label"]} · BIN {current["bin"]} · xác nhận {stable}/{rear["stable_samples"]}',
                                    label=current['label'],category=current.get('category'),bin=current['bin'])
                    if stable >= rear['stable_samples']: break
                await self.wait(rear['retry_ms']/1000)
            else: raise AutoFault('Chưa nhận diện ổn định vật phía sau; không mở nắp')
            if not self.rear_sensor(): raise AutoFault('Vật đã rời D18 trước khi xoay thùng; không mở nắp')
            await self.prepare_bin(obj['bin'],rear=True)
            if not self.rear_sensor(): raise AutoFault('Vật đã rời D18 trước khi mở nắp; đã dừng')
            await self.emit('rear_open_lid','Đã xoay đúng ngăn; mở nắp để nhận rác',bin=obj['bin'])
            await self.move_targets({'p4':self.brain.config['lid']['open']},'REAR mở nắp')
            opened = time.monotonic()
            clear_since = None
            await self.emit('rear_wait_drop','Chờ bỏ rác; D18 phải hết phát hiện ổn định trước khi đóng nắp')
            while True:
                now = time.monotonic()
                if self.rear_sensor(): clear_since = None
                elif clear_since is None: clear_since = now
                if now-opened >= rear['min_open_ms']/1000 and clear_since is not None and now-clear_since >= rear['clear_confirm_ms']/1000:
                    break
                if now-opened >= rear['max_open_ms']/1000:
                    raise AutoFault('D18 chưa hết phát hiện; dừng và không tự ra lệnh đóng nắp')
                await self.wait(.04)
            await self.emit('rear_close_lid','D18 đã hết phát hiện; đóng nắp')
            await self.move_targets({'p4':self.brain.config['lid']['close']},'REAR đóng nắp sau nhận rác')
            await self.move_targets(self.arm_pose('SEARCH'),'REAR trở về SEARCH')
            self.state['rear_cycles'] += 1
            self.brain.gift.grant_after_rear(obj['bin'])
            await self.emit('rear_done','Hoàn tất lượt nhận rác phía sau',bin=obj['bin'])
        finally:
            self.brain.finish_rear_request(self.token)
            self.target = None
            self.selected_bin = None

    async def start(self, rear_only=False, resuming=False):
        if self.operator.assist and not self.operator.assist.done():
            raise AutoFault('Chờ thao tác gắp/thả hoàn tất trước khi START AUTO')
        if not rear_only and not self.operator.mechanics_enabled:
            return await self.operator.start_observing()
        async with self.lock:
            if self.running:
                await self.report(copy.deepcopy(self.state)); return
            try:
                generation = self.generation
                self.cfg = load_rear_config(self.path) if rear_only else load_auto_config(self.path,self.brain.config)
                if not self.vision: raise AutoFault('Chưa có camera/Modal; cài requirements-camera.txt')
                if not rear_only and self.cfg.get('distance',{}).get('enabled') and not getattr(self.vision,'prefer_local',False):
                    raise AutoFault('Khoảng cách cm cần camera USB tại máy Python: LIVEKIT_PUBLISHER=1')
                if self.emergency_latched: raise AutoFault('Sau dừng khẩn cấp, bấm Cho phép điều khiển trong MANUAL trước')
                s = self.brain.snapshot()
                if not s['connected'] or not s['fresh']: raise AutoFault('Kết nối Python/Arduino trong MANUAL trước')
                if s['busy']: raise AutoFault('Chờ tay dừng trước khi START AUTO')
                if rear_only and (s['stopped'] or not self.brain.rear_requested()): return
                if not rear_only and self.cfg.get('motion',{}).get('enabled') and s['hardware'].get('smooth_protocol')!=1:
                    raise AutoFault('Cần nạp robot_control_v3.ino V3.9 trước khi START AUTO mềm')
                if not rear_only and self.cfg.get('box_tof_flow',{}).get('enabled'):
                    # A prior interrupted AUTO may leave IG 0: re-arm it safely,
                    # but NEVER treat a firmware that lacks the capability as ready.
                    if s['hardware'].get('ir_drive_guard') is False:
                        await asyncio.to_thread(self.brain.ensure_d19_guard_for_auto)
                        s = self.brain.snapshot()
                    if s['hardware'].get('ir_drive_guard') is not True:
                        raise AutoFault('Arduino chưa xác nhận IG trong STATUS. Nạp arduino/robot_control_v3/robot_control_v3.ino từ ZIP mới lên Mega 2560; chạy python3 python/main.py --firmware-check để kiểm tra. Không bỏ qua khóa an toàn.')
                    if (s['hardware'].get('tof_protocol') != 1
                            or s['hardware'].get('tof_auto_continue') is not True):
                        raise AutoFault('Cần nạp firmware V3.16.4 hỗ trợ AUTO tiếp tục khi VL53 chưa có số đo')
                if s['stopped']:
                    # START is a deliberate user command. Never resume on network reconnect.
                    await asyncio.to_thread(self.brain.resume,s['epoch'])
                    while self.brain.snapshot()['busy']: await asyncio.sleep(.04)
                if generation != self.generation: raise AutoFault('START đã bị hủy bởi lệnh dừng')
                token = str(uuid.uuid4())
                await asyncio.to_thread(self.brain.claim_auto,token)
                self.token = token
                if generation != self.generation:
                    await asyncio.to_thread(self.brain.stop)
                    raise AutoFault('START đã bị hủy bởi lệnh dừng')
                if not rear_only:
                    self.brain.enable_rear(self.cfg['rear']['enabled_in_auto'])
            except Exception as exc:
                if rear_only: await asyncio.to_thread(self.brain.stop)
                await self.emit('auto_not_ready',str(exc),label=None,bin=None,center=None)
                return
            self.cancelled = False
            if not resuming:
                self.checkpoint.clear()  # Explicit START is a fresh AUTO session.
                self.drop_failures = 0
                self.drop_search_pending = False
                if not rear_only:
                    self.state['cycles'] = 0
                    self.state['rear_cycles'] = 0
            else:
                previous = self.resume_context or {}
                self.drop_failures = previous.get('drop_failures', 0)
                self.drop_search_pending = False
                self.state['cycles'] = previous.get('cycles', 0)
                self.state['rear_cycles'] = previous.get('rear_cycles', 0)
                self.state['resume_from_stage'] = previous.get('state')
            self.holding_object = False
            self.flow = 'rear' if rear_only else 'front'
            self.state.update(flow=self.flow,label=None,category=None,bin=None,center=None)
            self.task = asyncio.create_task(self.run(rear_only=rear_only))
            self.resume_context = None

    async def run(self, rear_only=False):
        try:
            if rear_only:
                self.deadline = time.monotonic()+self.cfg['rear']['max_cycle_s']
                await self.rear_cycle()
                self.brain.release_auto(self.token)
                await self.emit('rear_idle','Chờ lượt D18 tiếp theo')
                return
            while True:
                self.flow = 'front'
                self.state['flow'] = 'front'
                self.deadline = time.monotonic()+self.cfg['control']['max_cycle_s']
                try:
                    await self.cycle()
                except LostGrip as exc:
                    try:
                        await self.handle_lost_grip(str(exc))
                    except RearRequested:
                        self.deadline = time.monotonic()+self.cfg['rear']['max_cycle_s']
                        await self.rear_cycle()
                        self.deadline = time.monotonic()+self.cfg['control']['max_cycle_s']
                        await self.post_rear_front_search()
                    continue
                except RearRequested:
                    self.deadline = time.monotonic()+self.cfg['rear']['max_cycle_s']
                    await self.rear_cycle()
                    self.deadline = time.monotonic()+self.cfg['control']['max_cycle_s']
                    await self.post_rear_front_search()
                    continue  # New target, no stale previous target or angle.
                if self.brain.rear_requested():
                    self.deadline = time.monotonic()+self.cfg['rear']['max_cycle_s']
                    await self.rear_cycle()
                    self.deadline = time.monotonic()+self.cfg['control']['max_cycle_s']
                    await self.post_rear_front_search()
                continuous=self.cfg.get('target_lock',{}).get('enabled') and self.cfg['target_lock']['continuous_auto']
                if not continuous and (not self.cfg['control']['repeat'] or self.state['cycles'] >= self.cfg['control']['max_cycles']): break
                await self.wait(1.)
            self.state['auto_running'] = False
            self.brain.release_auto(self.token)
            self.checkpoint.clear()
            await self.emit('auto_done','AUTO hoàn tất; bánh đã dừng')
        except asyncio.CancelledError:
            raise
        except NearHold as exc:
            if self.cfg.get('target_lock',{}).get('enabled'):
                try:
                    await asyncio.to_thread(self.brain.command,'XM',self.brain.epoch,_owner=self.token)
                    self.clear_overlay()
                    self.tracking_waiting=True
                    await self.emit('auto_hold',str(exc)+' · AUTO giữ chờ tại chỗ; STOP để chỉnh lại',vision_mode='held')
                    while True:await self.wait(.1)
                except asyncio.CancelledError:raise
                except Exception as hold_error:
                    if not self.brain.snapshot()['connected']:
                        self.checkpoint.suspend()
                        await self.emit('auto_reconnect_wait',
                                        'Mất COM3 trong HOLD; lưu checkpoint để kiểm tra sau kết nối lại',
                                        resume_available=True)
                    else:
                        await asyncio.to_thread(self.brain.stop)
                        self.checkpoint.clear()
                        await self.emit('auto_error',str(hold_error))
                finally:self.tracking_waiting=False
                return
            # This is a controlled hold, not STOP: maintain the commanded arm
            # pulses and disable D18 so it cannot move the arm after this hold.
            try:
                await asyncio.to_thread(self.brain.hold_auto,self.token,str(exc))
                await self.emit('auto_near_hold',str(exc)+' · D18 đã tắt; chỉnh MANUAL hoặc START lại',vision_mode='held')
            except (ValueError,ConnectionError) as hold_error:
                await asyncio.to_thread(self.brain.stop)
                await self.emit('auto_error',str(hold_error))
        except Exception as exc:
            # A lost serial port is not an ordinary user STOP. Preserve the
            # journal so pressing Connect can continue the *intent* safely.
            if not self.brain.snapshot()['connected']:
                self.checkpoint.suspend()
                self.state['auto_running'] = False
                await self.emit('auto_reconnect_wait',
                                f'Mất COM3; đã lưu bước đang dở ({self.state.get("state")}). Nhấn Kết nối lại để xác minh và phục hồi',
                                resume_available=True, resume_requires_inspection=False)
            else:
                await asyncio.to_thread(self.brain.stop)
                self.checkpoint.clear()
                self.state['auto_running'] = False
                await self.emit('auto_error',str(exc))
        finally:
            self.clear_overlay()
            self.brain.release_auto(self.token)
            self.state['auto_running'] = False
            await self.report(copy.deepcopy(self.state))

    async def service_rear(self):
        """D18 may take the arm only in AUTO; red MANUAL retains operator control."""
        if self.operator.transitioning:
            return  # Never start rear AUTO during a MANUAL/AUTO handover.
        if not self.operator.mechanics_enabled:
            # Do not create a pending rear task while operator controls the robot.
            # D18 remains visible on /control, and START AUTO re-arms it later.
            if self.brain.auto_rear or self.brain.rear_pending:
                self.brain.enable_rear(False)
            return
        s = self.brain.snapshot()
        if (s['connected'] and s['fresh'] and not s['stopped'] and self.vision
                and not self.brain.rear_manual_opt_out and not self.brain.auto_owner
                and (not self.running or not self.operator.mechanics_enabled)):
            self.brain.enable_rear(True)
            s = self.brain.snapshot()
        if self._resume_observing_after_rear and not self.running and not self.brain.rear_requested():
            self._resume_observing_after_rear = False
            if not self.operator.mechanics_enabled and not s['stopped']:
                await self.operator.start_observing()
                return
        if (self.running and not self.operator.mechanics_enabled and self.brain.rear_requested()
                and self.state.get('phase') == 'observe'):
            # Observation owns no actuators. D18 can preempt its camera task,
            # then the same manual observation mode resumes after rear service.
            self._resume_observing_after_rear = True
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
            self.task = None
            self.state['auto_running'] = False
        if self.running or s['busy'] or s['stopped'] or not s['connected'] or not s['fresh'] or self.brain.auto_owner:
            return
        if not self.brain.rear_requested():return
        if self.operator.assist is not None and not self.operator.assist.done():
            return  # Wait until current servo writer finishes; never race writers.
        joints=s['hardware'].get('joints',{})
        if joints.get('d3') == self.brain.config['grip']['close']:
            # A manually closed gripper without an observed held object/BIN
            # cannot be opened automatically. Keep D18 pending for inspection.
            if (self.operator.held_bin not in BIN_KEYS or
                    s['hardware'].get('sensors',{}).get('D19') != 0):
                if self.state.get('state') != 'rear_manual_inspection':
                    await self.emit('rear_manual_inspection',
                                    'D18: tay đang kẹp nhưng không xác minh được chai/BIN; giữ kẹp, kiểm tra MANUAL trước',
                                    flow='rear',resume_requires_inspection=True)
                return
        await self.start(rear_only=True)

    async def stop(self, emergency=False):
        # Immediately invalidate ownership before waiting for cancellation of HTTP/thread work.
        self.cancelled = True
        self.clear_overlay()
        self.generation += 1
        if emergency: self.emergency_latched = True
        await asyncio.to_thread(self.brain.stop)
        await self.operator.cancel_assist()
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.gather(self.task,return_exceptions=True)
        self.checkpoint.clear()
        self._resume_observing_after_rear = False
        self.state['resume_available'] = False
        self.state['resume_requires_inspection'] = False
        self.state['auto_running'] = False
        await self.emit('auto_stopped','Đã dừng AUTO; hủy kết quả Modal và các lệnh chờ')
