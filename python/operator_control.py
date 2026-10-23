"""AUTO observation/manual handover. Observation never owns/writes actuators."""
import asyncio
import copy
import math
import time
import threading
import uuid
from arm_path import targets_at
from box_tof_flow import tof_sample, vertical_edge
from pose_profile import reference_at
from grasp_orientation import estimate_axis, OrientationWindow, pulse_for_angle, auto_pulse_for_angle, angle_for_pulse


class OperatorControl:
    def __init__(self, auto):
        self.a=auto
        self.mechanics_enabled=True
        self.lock=asyncio.Lock()
        self.vision_lock=asyncio.Lock()
        self.assist=None
        self.orientation=OrientationWindow()
        self.last_frame=0.
        self.last_seq=None
        self.votes=0
        self.range_at=0.
        self.observation_at=0.
        self.held_bin=None
        self.transitioning=False
        self.range_lock=threading.RLock()
        self.range_context=None
        self.manual_cfg=None
        self.wrist_reference=None

    def control_config(self):
        cfg=getattr(self.a,'cfg',None)
        if cfg is not None:return cfg
        if self.manual_cfg is None:
            from auto_controller import load_auto_config
            self.manual_cfg=load_auto_config(self.a.path,self.a.brain.config)
        return self.manual_cfg

    def poll_sensors(self):
        """Read only; called at 100 ms even when camera/AUTO is off."""
        s=self.a.brain.snapshot()
        try:
            cfg=self.control_config()
            j=s['hardware'].get('joints',{})
            context=(s['epoch'],*(j.get(k) for k in ('p1','p10','p11','d2')))
            with self.range_lock:
                if (self.range_context is not None and context!=self.range_context or s['busy']
                        or s['stopped'] or s['drive']['active'] or any(s['hardware'].get('motors',[]))):
                    self.votes=0;self.range_at=0.;self.last_seq=None
                self.range_context=context
                if s['busy'] or s['stopped'] or s['drive']['active'] or any(s['hardware'].get('motors',[])):
                    return None,False
                return self.range_sample(s,cfg['box_tof_flow'])
        except (OSError,ValueError,KeyError,TypeError):
            with self.range_lock:self.votes=0;self.range_at=0.
            return None,False

    def manual_status(self, s, range_ready):
        b=self.a.brain;j=s['hardware'].get('joints',{})
        result={'sensor_grasp_ready':False,'release_ready':False,'lift_ready':False,
                'sensor_grasp_reason':'Chưa kết nối Arduino','held_bin':self.held_bin}
        if not s['connected'] or not s['fresh']:return result
        if s['stopped']:result['sensor_grasp_reason']='Nhấn Cho phép điều khiển sau STOP';return result
        if (s.get('rear_pending') or s.get('rear_handling')
                or s.get('auto_rear') and not s.get('manual_mechanics_mode')):
            result['sensor_grasp_reason']='D18/AUTO đang được ưu tiên; chuyển nút cơ khí sang Đỏ để tự điều khiển';return result
        if s['busy'] or s['drive']['active'] or any(s['hardware'].get('motors',[])) or b.auto_owner:
            result['sensor_grasp_reason']='Chờ cơ khí dừng; chuyển AUTO sang đỏ nếu đang chạy';return result
        if self.transitioning or self.assist and not self.assist.done():
            result['sensor_grasp_reason']='Đang chạy thao tác';return result
        cfg=self.control_config();path=cfg['arm_path']
        result['release_ready']=(self.held_bin is not None and j.get('d3')==b.config['grip']['close']
                                 and s['hardware'].get('sensors',{}).get('D19')==0)
        try:
            expected=targets_at(path,j.get('p1'))
            if any(j.get(k)!=expected[k] for k in ('p1','p10','p11')):raise ValueError()
        except (ValueError,TypeError,KeyError):
            result['sensor_grasp_reason']='Đưa tay về tư thế tìm rác/quỹ đạo đã đo';return result
        result['lift_ready']=j['p1']!=path['start']['p1']
        if j.get('d3')!=b.config['grip']['open']:
            result['sensor_grasp_reason']='Mở kẹp trước khi gắp vật mới';return result
        if not range_ready and not (j['p1']==path['end']['p1'] and s['hardware'].get('sensors',{}).get('D19')==0):
            result['sensor_grasp_reason']='Chờ 3 mẫu VL53 mới trong khoảng gắp';return result
        if self.current_bin(s) is None:
            result['sensor_grasp_reason']='Chọn ngăn thả vật trong Thùng & nắp';return result
        result.update(sensor_grasp_ready=True,sensor_grasp_reason='Sẵn sàng · bấm Gắp theo cảm biến')
        return result

    def current_bin(self, snapshot):
        from auto_controller import BIN_KEYS
        pulse=snapshot['hardware'].get('joints',{}).get('p9')
        return next((n for n,key in BIN_KEYS.items() if pulse==self.a.brain.config['bins'][key]),None)

    def wrist_step(self, command, epoch):
        b=self.a.brain
        with b.lock:
            b.check(epoch)
            s=b.snapshot()
            if not s['fresh']:raise ValueError('Chưa có trạng thái Arduino mới')
            current=s['hardware'].get('joints',{}).get('d2')
            angle=angle_for_pulse(current)
            if self.wrist_reference and current==self.wrist_reference[0]:angle=self.wrist_reference[1]
            target=0 if command=='WRIST CENTER' else max(-90,min(60,angle+(10 if command=='WRIST RIGHT' else -10)))
            if abs(target-angle)<.1:raise ValueError('Đã đạt giới hạn xoay kẹp')
            pulse=pulse_for_angle(target)
            value=b.command(None,epoch,'d2',pulse)
            self.wrist_reference=(pulse,target)
            return value

    def snapshot(self):
        mm,range_ready=self.poll_sensors()
        state=copy.deepcopy(self.a.state)
        orientation_cfg=getattr(self.a,'cfg',{}).get('grasp_orientation',{})
        state.update(mechanics_enabled=self.mechanics_enabled,
                     operator_protocol=2, assist_running=self.assist is not None and not self.assist.done(),
                     transitioning=self.transitioning,
                     orientation_enabled=bool(orientation_cfg.get('enabled')),
                     orientation_calibrated=bool(orientation_cfg.get('calibrated')))
        s=self.a.brain.snapshot()
        # Never confuse AUTO's camera-only observer with physical motor ownership.
        # In red mode the observer may run while MANUAL wheel commands are legal.
        state['manual_wheels_available']=(
            s['connected'] and s['fresh'] and not s['stopped']
            and not s['busy'] and not s['auto_running']
            and not self.transitioning and not (self.assist and not self.assist.done())
            and (s['manual_mechanics_mode'] or not (s['rear_pending'] or s['rear_handling']))
        )
        state['d18_detected']=s['hardware'].get('sensors',{}).get('D18') == 0
        try:state.update(self.manual_status(s,range_ready))
        except (OSError,ValueError,KeyError,TypeError) as exc:
            state.update(sensor_grasp_ready=False,release_ready=False,lift_ready=False,sensor_grasp_reason=str(exc))
        state.update(sensor_distance_ready=range_ready,sensor_tof_mm=mm)
        try:
            c=self.control_config()['box_tof_flow']
            state.update(sensor_min_mm=c['tof_min_mm'],sensor_max_mm=c['tof_max_mm'])
        except (OSError,ValueError,KeyError):pass
        pulse=s['hardware'].get('joints',{}).get('d2')
        try:angle=round(angle_for_pulse(pulse),1)
        except ValueError:angle=None
        state['wrist']={'pulse_us':pulse,'angle_deg':angle,'min_deg':-90,'max_deg':60,'step_deg':10}
        if not (self.mechanics_enabled and self.a.running):
            state.update(distance_ready=range_ready,tof_mm=mm)
        if not self.mechanics_enabled:
            state['auto_running']=self.a.running
            if time.monotonic()-self.observation_at>2:
                state['planned_commands']=[]
                state['target_fresh']=False
        return state

    async def action(self, body):
        from auto_controller import load_auto_config
        if set(body)!={'command','epoch'} or type(body['epoch']) is not int:
            raise ValueError('Sai yêu cầu điều khiển AUTO')
        command=body['command']
        if command=='AUTO STOP':
            return await self.a.stop()
        async with self.lock:
            if body['epoch']!=self.a.brain.epoch: raise ValueError('Lệnh cũ; cập nhật trạng thái')
            if command=='AUTO START':
                await self.a.start()
            elif command in ('MECHANICS OFF','MECHANICS ON'):
                await self.set_mechanics(command.endswith('ON'))
            elif command=='HOLD':
                await self.set_mechanics(False,force=True)
                self.a.state.update(state='manual_hold',message='Đã giữ vị trí tay/kẹp và dừng bánh')
            elif command in ('WRIST LEFT','WRIST RIGHT','WRIST CENTER'):
                await asyncio.to_thread(self.wrist_step,command,body['epoch'])
            elif command in ('GRASP','GRASP SENSOR','RELEASE','LIFT'):
                if self.mechanics_enabled and (self.a.running or command=='GRASP'):
                    raise ValueError('Chuyển nút sang đỏ trước khi gắp/thả theo yêu cầu')
                if self.assist and not self.assist.done(): raise ValueError('Đang gắp/thả; chờ hoàn tất')
                self.a.cfg=load_auto_config(self.a.path,self.a.brain.config)
                s=self.a.brain.snapshot()
                self.a.brain.check(s['epoch'])
                if self.a.brain.auto_owner:raise ValueError('AUTO đang điều khiển cơ khí')
                if s.get('rear_pending') or s.get('rear_handling'):
                    raise ValueError('D18 đang ưu tiên phía sau; chờ xử lý trước khi điều khiển tay')
                chosen_bin=self.held_bin
                if command=='GRASP SENSOR':
                    _,range_ready=self.poll_sensors()
                    status=self.manual_status(s,range_ready)
                    if not status['sensor_grasp_ready']:raise ValueError(status['sensor_grasp_reason'])
                    chosen_bin=self.current_bin(s)
                elif command=='GRASP':
                    joints=s['hardware'].get('joints',{})
                    if joints.get('d3')!=self.a.brain.config['grip']['open']:
                        raise ValueError('Mở kẹp trước khi gắp vật mới')
                    path=self.a.cfg['arm_path']
                    expected=targets_at(path,joints.get('p1'))
                    if any(joints.get(j)!=expected[j] for j in ('p1','p10','p11')):
                        raise ValueError('S1/S2 chưa nằm trên quỹ đạo đã đo')
                    if joints['p1']!=path['end']['p1'] and not self.snapshot().get('distance_ready'):
                        raise ValueError('Chưa xác nhận đủ khoảng cách 20–25 cm')
                    if not self.a.target or not self.a.state.get('target_fresh') or time.monotonic()-self.observation_at>2 or self.a.locked_bin is None:
                        raise ValueError('Chưa khóa được vật mới từ camera')
                    chosen_bin=self.a.locked_bin
                elif command=='RELEASE':
                    if self.held_bin is None: raise ValueError('Chưa có vật đã gắp và ngăn được xác nhận')
                    if s['hardware'].get('joints',{}).get('d3')!=self.a.brain.config['grip']['close']:
                        raise ValueError('Kẹp chưa đóng; không chạy chu trình thả')
                else:
                    status=self.manual_status(s,False)
                    if not status['lift_ready']:raise ValueError(status['sensor_grasp_reason'] if not s['fresh'] else 'Tay chưa ở quỹ đạo có thể nâng/rút')
                self.a.token=str(uuid.uuid4())
                # STOP/XM may arrive from another dispatcher while config/readiness
                # is being checked. Claim only the epoch of this exact request.
                with self.a.brain.lock:
                    self.a.brain.check(body['epoch'])
                    self.a.brain.claim_auto(self.a.token)
                    self.held_bin=chosen_bin
                self.a.cancelled=False
                self.a.flow='front'
                self.a.deadline=time.monotonic()+120
                self.assist=asyncio.create_task(self.run_assist(command,copy.deepcopy(self.a.target)))
            else: raise ValueError('Lệnh AUTO không được hỗ trợ')
        return self.snapshot()

    async def cancel_assist(self):
        if self.assist and self.assist is not asyncio.current_task() and not self.assist.done():
            self.assist.cancel()
            await asyncio.gather(self.assist,return_exceptions=True)

    async def set_mechanics(self, enabled, force=False):
        if enabled==self.mechanics_enabled and not force: return
        if self.assist and not self.assist.done():
            if enabled: raise ValueError('Chờ thao tác gắp/thả hoàn tất trước khi nối AUTO')
        a=self.a
        if enabled:
            s=a.brain.snapshot();j=s['hardware'].get('joints',{})
            if s['busy'] or s['drive']['active']: raise ValueError('Dừng thao tác tay/bánh trước khi nối AUTO')
            if j.get('d3')==a.brain.config['grip']['close'] or s['hardware'].get('sensors',{}).get('D19')==0:
                raise ValueError('Thả vật và đưa kẹp về trạng thái trống trước khi nối AUTO')
        was_running=a.running
        held=a.brain.snapshot()
        auto_owned=(a.brain.auto_owner==a.token and not (self.assist and not self.assist.done()))
        if (not enabled and (self.held_bin is None or auto_owned) and held['fresh'] and a.selected_bin in (1,2,3,4)
                and held['hardware'].get('joints',{}).get('d3')==a.brain.config['grip']['close']
                and held['hardware'].get('sensors',{}).get('D19')==0):
            self.held_bin=a.selected_bin
        self.transitioning=True
        a.cancelled=True
        a.generation+=1
        generation=a.generation
        # Cancel before yielding: handover must not be interpreted as a hardware fault.
        if a.task and not a.task.done(): a.task.cancel()
        # Brake and invalidate the physical writer before accepting manual commands.
        try:
            await asyncio.to_thread(a.brain.detach_auto)
            await self.cancel_assist()
            if a.task and not a.task.done():
                a.task.cancel();await asyncio.gather(a.task,return_exceptions=True)
            # Brain must know about the red MANUAL mode before polling D18 again.
            # detach_auto above has already canceled the previous physical writer.
            a.brain.set_manual_mechanics_mode(not enabled)
            self.mechanics_enabled=enabled
            a.cancelled=False
            a.clear_overlay()
            a.state.update(mechanics_enabled=enabled,auto_running=False,planned_commands=[],distance_ready=False)
            if generation!=a.generation:
                a.cancelled=True
                return
            if was_running and generation==a.generation: await a.start()
        finally:
            self.transitioning=False
        # State only: changing the toggle does not produce a toast/status notification.

    async def start_observing(self):
        from auto_controller import load_auto_config
        a=self.a
        if a.running: return
        if a.emergency_latched: raise ValueError('Cho phép điều khiển sau STOP trước khi START')
        if not a.vision: raise ValueError('Chưa có camera/model')
        a.cfg=load_auto_config(a.path,a.brain.config)
        a.cancelled=False;a.flow='front'
        a.target=None;a.locked_bin=None;a.locked_label=None
        a.classification_vote=None;a.classification_count=0
        a.lock_prediction=None;a.tracking_waiting=False
        self.last_frame=0.;self.votes=0;self.last_seq=None;self.range_at=0.
        a.state.update(auto_running=True,mechanics_enabled=False,phase='observe',planned_commands=[])
        a.task=asyncio.create_task(self.observe_loop())

    def range_sample(self, s, cfg=None):
        with self.range_lock:return self._range_sample(s,cfg)

    def _range_sample(self, s, cfg=None):
        c=cfg if cfg is not None else self.a.cfg['box_tof_flow']
        try:
            seq,mm=tof_sample(s,c)
            if not s['connected'] or not s['fresh']: raise ValueError('USB chưa có mẫu mới')
        except ValueError:
            self.votes=0;self.range_at=0.;return None,False
        if not c['tof_min_mm']<=mm<=c['tof_max_mm']:
            self.votes=0;self.range_at=0.
        elif seq!=self.last_seq:
            self.votes=self.votes+1 if time.monotonic()-self.range_at<.35 or self.votes<1 else 1
            self.range_at=time.monotonic()
        self.last_seq=seq
        ready=self.votes>=c['tof_confirm_samples'] and time.monotonic()-self.range_at<.35
        return mm,ready

    async def fresh_detection(self):
        a=self.a
        async with self.vision_lock:
            result=await asyncio.wait_for(a.vision.detect(fresh_after=self.last_frame),a.cfg['control']['inference_timeout_s'])
        now=time.monotonic();at=result.get('_frame_at');frame=result.get('_frame_size')
        if type(at) not in (int,float) or not math.isfinite(at) or not self.last_frame<at<=now+.1 or now-at>a.cfg['control']['max_result_age_s']:
            raise ValueError('Kết quả camera quá cũ')
        if not isinstance(frame,list) or len(frame)!=2 or any(type(n) is not int or n<16 for n in frame):
            raise ValueError('Kích thước ảnh không hợp lệ')
        self.last_frame=at;a.frame_after=at
        obj=a.select_locked_target(result)
        if obj is None:
            a.classification_count=0
            if time.monotonic()-a.lock_last_seen>3:
                a.target=None;a.locked_bin=None;a.locked_label=None
            return result,None
        vote=(obj['bin'],obj['label'].strip().lower())
        if vote==a.classification_vote:a.classification_count+=1
        else:a.classification_vote=vote;a.classification_count=1
        if a.classification_count>=3:
            a.locked_bin=obj['bin'];a.locked_label=obj['label']
        return result,obj

    def plan(self, obj, snapshot, ready):
        a=self.a;c=a.cfg['box_tof_flow'];j=snapshot['hardware'].get('joints',{})
        commands=[]
        def add(component,command): commands.append({'component':component,'command':command,'delivery':'planned'})
        if obj is None or a.locked_bin is None:
            add('Bánh','XM · chờ vật ổn định');return commands
        from auto_controller import BIN_KEYS
        add('Nắp',f"P4 {a.brain.config['lid']['close']}")
        add('Thùng',f"P9 {a.brain.config['bins'][BIN_KEYS[a.locked_bin]]}")
        try:
            p1=j['p1'];path=a.cfg['arm_path'];expected=targets_at(path,p1)
            if any(j.get(k)!=expected[k] for k in ('p1','p10','p11')):raise ValueError('pose')
            if ready or p1>=c['handoff_p1_us']:
                add('Bánh','XM · xác nhận VL53/D19')
                add('Tay',f"S1/P10 {path['end']['p10']} → S2/P1 {path['end']['p1']}")
                add('Kẹp','D2 theo góc đã xác nhận → D3 đóng → nâng')
            else:
                ref=reference_at(a.cfg['pose_profile'],p1);x1,y1,x2,y2=obj['bbox_norm']
                ex=(x1+x2)/2-ref['target_x_norm']
                if abs(ex)>ref['x_tolerance_norm']:
                    positive=a.cfg['calibration']['positive_error_x_move']
                    move=positive if ex>0 else ('LEFT' if positive=='RIGHT' else 'RIGHT')
                    add('Bánh','DRIVE '+move)
                elif y2<ref['target_bottom_norm']-ref['bottom_tolerance_norm'] or vertical_edge(obj['bbox_norm'],c['edge_margin_norm']):
                    add('Bánh','DRIVE FORWARD · có chặn VL53')
                else:
                    nxt=targets_at(path,min(c['handoff_p1_us'],p1+c['step_us']))
                    add('Tay',f"S1/P10 {nxt['p10']} → S2/P1 {nxt['p1']}")
        except (KeyError,ValueError,TypeError):add('Tay','Đưa tay về quỹ đạo SEARCH đã đo')
        return commands

    async def observe_loop(self):
        a=self.a
        async def sensors():
            while not a.cancelled:
                mm,ready=self.range_sample(a.brain.snapshot())
                a.state.update(distance_ready=ready,tof_mm=mm,tof_votes=self.votes)
                await asyncio.sleep(.05)
        sensor_task=asyncio.create_task(sensors())
        try:
            while not a.cancelled:
                if self.assist and not self.assist.done():
                    await asyncio.sleep(.1);continue
                try:
                    result,obj=await self.fresh_detection()
                    if self.assist and not self.assist.done(): continue
                    # A motion may finish while inference was in flight; do not use its old image.
                    if result['_frame_at']<a.brain.motion_finished_at or a.brain.snapshot()['busy']:
                        raise ValueError('Chờ ảnh mới sau thao tác cơ khí')
                    s=a.brain.snapshot();mm,ready=self.range_sample(s)
                    self.observation_at=time.monotonic()
                    a.state.update(state='auto_observing',message='AUTO camera đang chạy',auto_running=True,
                        mechanics_enabled=False,label=obj['label'] if obj else None,bin=a.locked_bin,
                        target_fresh=obj is not None,distance_ready=ready,tof_mm=mm,tof_votes=self.votes,
                        planned_commands=self.plan(obj,s,ready))
                    if obj and result.get('_frame_jpeg'):
                        try:
                            axis=await asyncio.to_thread(estimate_axis,result['_frame_jpeg'],obj['bbox'])
                            joints=s['hardware'].get('joints',{})
                            axis=self.orientation.add(axis,(a.locked_bin,a.locked_label,*(joints.get(j) for j in ('p1','p10','p11','d2'))))
                            ocfg=a.cfg.get('grasp_orientation',{})
                            mapped=axis['angle_deg']*ocfg.get('camera_sign',1)+ocfg.get('offset_deg',0)
                            a.state['orientation']=dict(axis,pulse_us=auto_pulse_for_angle(mapped,ocfg.get('deadband_deg',10.0)),calibrated=ocfg.get('calibrated',False))
                        except (ValueError,ImportError) as exc:a.state['orientation']={'valid':False,'reason':str(exc)}
                    else:a.state['orientation']={'valid':False,'reason':'Chưa có vật trong ảnh mới'}
                except asyncio.CancelledError:raise
                except Exception as exc:
                    a.state.update(state='auto_observing',message=str(exc),auto_running=True,
                        target_fresh=False,distance_ready=False,planned_commands=[])
                    self.range_at=0.;self.votes=0
                await asyncio.sleep(.2)
        finally:
            sensor_task.cancel()
            await asyncio.gather(sensor_task,return_exceptions=True)
            a.state['auto_running']=False

    async def remember_search_orientation(self, obj):
        """Measure/store 3 new SEARCH-view axes bound to the locked object.

        No wheel/arm motion is allowed until the three readings agree. The
        saved direction is not re-estimated near the gripper and is invalidated
        when the next cycle starts or the gripper loses the object.
        """
        from auto_controller import NearHold, normalized_iou
        a = self.a
        cfg = a.cfg.get('grasp_orientation', {})
        a.search_orientation = None
        if not cfg.get('enabled', False):
            return
        if not cfg.get('calibrated', False):
            raise NearHold('Chưa có hiệu chuẩn chiều xoay càng D2')
        if (not isinstance(obj, dict) or obj.get('bin') != a.locked_bin
                or obj.get('label') != a.locked_label or a.locked_bin is None):
            raise NearHold('Vật chưa khóa đúng nhãn/BIN tại SEARCH')
        initial_id, initial_bin, initial_label = a.lock_sequence, a.locked_bin, a.locked_label
        path = a.cfg['arm_path']
        pose = path['start']
        snap = a.guard()
        def search_stationary():
            current = a.guard()
            j = current['hardware']['joints']
            if (current['busy'] or current['drive']['active'] or any(current['hardware'].get('motors', []))
                    or any(j.get(k) != v for k, v in pose.items())
                    or j.get('d3') != a.brain.config['grip']['open']):
                raise NearHold('Chỉ được ghi nhớ góc khi tay/bánh đứng yên ở SEARCH, D3 mở')
            if (a.lock_sequence, a.locked_bin, a.locked_label) != (initial_id, initial_bin, initial_label):
                raise NearHold('Vật khóa đã thay đổi trong lúc đo góc SEARCH')
        search_stationary()
        angles = OrientationWindow()
        previous_box = obj.get('bbox_norm')
        current_label = initial_label
        axis = None
        # V3.16.20: A single blurred/background-contaminated segmentation must
        # not HOLD immediately. Allow up to 9 NEW stationary SEARCH frames but
        # require 3 CONSECUTIVE valid angles; never infer a neutral wrist angle.
        max_frames = 9
        invalid_frames = 0
        last_axis_error = None
        for frame_no in range(1, max_frames + 1):
            a.check_rear()
            search_stationary()
            # _detect_once enforces timestamp freshness, tracks the locked
            # object's appearance/box and refreshes observed_jpeg each time.
            try:
                sample = await a._detect_once()
            except Exception as exc:
                from auto_controller import StaleDetection, VisionUnavailable
                if isinstance(exc, (StaleDetection, VisionUnavailable)):
                    raise NearHold('Không lấy được 3 ảnh SEARCH mới: ' + str(exc)) from exc
                raise
            search_stationary()
            if (sample is None or sample.get('bin') != initial_bin
                    or sample.get('label') != current_label):
                raise NearHold('Mất/đổi vật khóa trong lúc ghi nhớ góc SEARCH')
            box = sample.get('bbox_norm')
            if (not isinstance(box, (list, tuple)) or len(box) != 4
                    or not isinstance(previous_box, (list, tuple)) or len(previous_box) != 4
                    or normalized_iou(box, previous_box) < .5):
                raise NearHold('Vật dịch chuyển/không khớp giữa 3 ảnh SEARCH')
            previous_box = box
            if not isinstance(a.observed_jpeg, (bytes, bytearray)) or not a.observed_jpeg:
                raise NearHold('Ảnh SEARCH thiếu JPEG gốc để xác định góc')
            if sum(1 for x in a.last_candidates if x.get('bin') == initial_bin) != 1:
                raise NearHold('Nhiều vật cùng loại ở SEARCH; góc gắp không rõ')
            try:
                item = await asyncio.to_thread(estimate_axis, a.observed_jpeg, sample['bbox'])
                axis = angles.add(item, (initial_id, initial_bin, initial_label))
                if axis is not None:
                    break
            except (ValueError, ImportError) as exc:
                if str(exc).startswith('Đang lấy trung bình'):
                    continue
                invalid_frames += 1
                last_axis_error = str(exc)
                # Three consecutive trustworthy samples, never combine angles
                # separated by a failed segmentation or large angular change.
                angles = OrientationWindow()
                axis = None
                a.state['orientation'] = {'valid': False, 'reason': last_axis_error,
                                          'attempt': frame_no, 'max_attempts': max_frames}
                await a.emit('auto_orientation_retry',
                             f'SEARCH: ảnh góc {frame_no}/{max_frames} không hợp lệ ({last_axis_error}); tay/bánh đứng yên, chụp lại',
                             angle_attempt=frame_no, angle_max_attempts=max_frames,
                             orientation=dict(a.state['orientation']))
                if frame_no < max_frames:
                    await a.wait(0.15)
        if axis is None:
            reason = last_axis_error or 'Chưa đủ 3 ảnh SEARCH có góc ổn định'
            raise NearHold(f'Không đo được góc vật tại SEARCH sau {max_frames} ảnh mới: {reason}. '
                           'Giữ bánh/tay, kiểm tra toàn thân chai trong camera và nền ảnh')
        mapped = axis['angle_deg'] * cfg['camera_sign'] + cfg['offset_deg']
        try:
            pulse = auto_pulse_for_angle(mapped, cfg.get('deadband_deg',10.0))
        except ValueError as exc:
            raise NearHold('Góc SEARCH nằm ngoài giới hạn càng: ' + str(exc)) from exc
        deadband_deg = cfg.get('deadband_deg',10.0)
        deadband_applied = abs(mapped) < deadband_deg
        # The servo can only turn up to +60°. Retain the measured/mapped
        # angles for diagnostics and record the reachable command explicitly.
        commanded_deg = angle_for_pulse(pulse)
        angle_limited = mapped > 60.0 and commanded_deg == 60.0
        search_stationary()
        a.search_orientation = dict(target_id=initial_id, bin=initial_bin,
                                    label=initial_label, source_pose=dict(pose),
                                    angle_deg=axis['angle_deg'], mapped_deg=mapped,
                                    commanded_deg=commanded_deg,
                                    angle_limited=angle_limited,
                                    deadband_deg=deadband_deg,
                                    deadband_applied=deadband_applied,
                                    pulse_us=pulse, samples=axis['samples'],
                                    captured_at=time.monotonic(), applied=False)
        a.state['orientation'] = dict(a.search_orientation, valid=True)
        limit_notice = ('; góc phải vượt +60°, GIỚI HẠN về +60°' if angle_limited else '')
        deadband_notice = (f'; |góc| < {deadband_deg:g}°, giữ D2 trung tâm'
                           if deadband_applied else '')
        await a.emit('auto_orientation_saved',
                     f'SEARCH: ghi nhớ góc {axis["angle_deg"]:.1f}° → D2 {pulse} µs'
                     f'{limit_notice}{deadband_notice}; CHƯA xoay, chờ VL53 20–25 cm',
                     orientation=dict(a.state['orientation']))

    async def rotate_for_grasp(self, obj):
        """D19 confirmed at final pose -> 3 NEW frames -> D2 -> caller closes D3.

        The bbox from camera tracking was measured before the arm descended.
        It must NOT be matched by IoU against an object in the final close-up;
        camera perspective/scale has changed dramatically. Instead require one
        uniquely identifiable same-bin target in each of three fresh close-up
        frames, consistent across those frames, before commanding the wrist.
        """
        from auto_controller import NearHold, normalized_iou
        a=self.a;cfg=a.cfg.get('grasp_orientation',{})
        if not cfg.get('enabled',False):return
        if not cfg.get('calibrated',False):
            raise NearHold('Chưa bật hiệu chuẩn góc camera → D2; không xoay kẹp')
        if not isinstance(obj,dict) or obj.get('bin')!=a.locked_bin or a.selected_bin!=a.locked_bin:
            raise NearHold('Không còn đúng vật/BIN đã khóa trước khi xoay kẹp')
        snap=a.guard()
        joints=snap['hardware']['joints']
        if snap['busy'] or snap['drive']['active'] or any(snap['hardware'].get('motors',[])):
            raise NearHold('Bánh/tay phải đứng yên trước khi xoay D2')
        if joints.get('d3')!=a.brain.config['grip']['open']:
            raise NearHold('Càng kẹp phải mở khi đo góc và xoay D2')
        if any(joints.get(j)!=v for j,v in cfg['calibration_pose'].items()):
            raise NearHold('Tư thế camera khác tư thế đo góc cuối; không xoay')
        if not a.sensor('D19'):
            await a.check_grip_or_drop('trước khi đo góc')
        if a.vision is None:
            raise NearHold('Không có camera để lấy góc gắp')

        angles=OrientationWindow()
        last=max(time.monotonic(),a.brain.motion_finished_at)
        axis=None;previous_box=None;previous_label=None
        for _ in range(3):
            a.guard();a.check_rear()
            async with self.vision_lock:
                try:
                    r=await asyncio.wait_for(a.vision.detect(fresh_after=last),
                                             a.cfg['control']['inference_timeout_s'])
                except (OSError, ValueError, TimeoutError, asyncio.TimeoutError) as exc:
                    raise NearHold('Không nhận được ảnh hướng chai mới: '+str(exc)) from exc
            a.guard();a.check_rear()
            if not a.sensor('D19'):
                await a.check_grip_or_drop('khi xác nhận góc')
            snap=a.guard()
            joints=snap['hardware']['joints']
            if (snap['busy'] or snap['drive']['active'] or any(snap['hardware'].get('motors',[]))
                    or any(joints.get(j)!=v for j,v in cfg['calibration_pose'].items())):
                raise NearHold('Tay/bánh đã đổi tư thế khi chụp ảnh góc; giữ chờ')
            at=r.get('_frame_at');now=time.monotonic()
            if type(at) not in (int,float) or not last<at<=now+.1 or now-at>a.cfg['control']['max_result_age_s']:
                raise NearHold('Ảnh góc quá cũ; không xoay kẹp')
            last=at
            candidates=r.get('objects') or ([r['object']] if r.get('object') else [])
            frame=r.get('_frame_size');jpeg=r.get('_frame_jpeg')
            if not isinstance(frame,(list,tuple)) or len(frame)!=2 or any(type(v) is not int or v<16 for v in frame):
                raise NearHold('Thiếu kích thước ảnh góc')
            if not isinstance(jpeg,(bytes,bytearray)) or not jpeg:
                raise NearHold('Không có JPEG gốc để tính hướng chai')
            matches=[]
            for o in candidates:
                box=o.get('bbox')
                if o.get('bin')!=obj['bin'] or not isinstance(box,(list,tuple)) or len(box)!=4:
                    continue
                if not all(type(v) in (int,float) and math.isfinite(v) for v in box):continue
                x1,y1,x2,y2=box
                if not (0<=x1<x2<=frame[0] and 0<=y1<y2<=frame[1]):continue
                norm=[x1/frame[0],y1/frame[1],x2/frame[0],y2/frame[1]]
                matches.append((o,norm))
            # A single current target, not an old handoff bbox. Do not guess in
            # a crowd; even identical bins could refer to distinct objects.
            if len(matches)!=1:raise NearHold('Ảnh cuối không có đúng 1 vật cùng ngăn để xoay kẹp')
            candidate,box=matches[0]
            label=candidate.get('label')
            if previous_box is not None and normalized_iou(box,previous_box)<.5:
                raise NearHold('Vật dịch chuyển giữa 3 ảnh đo góc; không xoay kẹp')
            if previous_label is not None and label!=previous_label:
                raise NearHold('Nhãn vật đổi giữa 3 ảnh đo góc; không xoay kẹp')
            previous_box=box;previous_label=label
            try:
                item=await asyncio.to_thread(estimate_axis,jpeg,candidate['bbox'])
                axis=angles.add(item,(a.lock_sequence,a.locked_bin))
            except (ValueError,ImportError) as exc:
                if str(exc).startswith('Đang lấy trung bình'):continue
                raise NearHold('Không đo được hướng vật trong ảnh gần: '+str(exc)) from exc
        if axis is None:raise NearHold('Chưa đủ 3 ảnh đo góc ổn định')
        angle=axis['angle_deg']*cfg['camera_sign']+cfg['offset_deg']
        try:pulse=auto_pulse_for_angle(angle,cfg.get('deadband_deg',10.0))
        except ValueError as exc:raise NearHold(str(exc)) from exc
        # Another final presence check immediately before the physical wrist turn.
        await a.fresh_hardware()
        if not a.sensor('D19'):
            await a.check_grip_or_drop('ngay trước khi xoay càng')
        await a.emit('auto_rotate_grip',f'Đã nhận 3 ảnh mới; xoay D2 {pulse} µs trước D3',
                     orientation=dict(axis,mapped_deg=angle,pulse_us=pulse))
        await a.move_targets({'d2':pulse},'AUTO xoay càng theo góc chai',gentle=True)

    async def run_assist(self, command, grasp_target=None):
        a=self.a;b=a.brain
        try:
            if command in ('GRASP','GRASP SENSOR'):
                path=a.cfg['arm_path'];j=a.guard()['hardware']['joints']
                from arm_path import pulse_steps
                for p in pulse_steps(j['p1'],path['end']['p1'],a.cfg['control']['s2_step_us']):
                    await a.fresh_hardware()
                    if a.sensor('D19'):raise ValueError('D19 nhận trước tư thế cuối; giữ tay')
                    targets=targets_at(path,p);targets['d2']=j['d2']
                    await a.move_targets(targets,'Gắp: S1 → S2',gentle=True)
                await a.fresh_hardware()
                await a.confirm_hold()  # stable D19 before rotating/closing; never drives blindly.
                if command=='GRASP':
                    await self.rotate_for_grasp(grasp_target if grasp_target is not None else a.target)
                await a.confirm_hold()
                await a.move_targets({'d3':b.config['grip']['close']},'Gắp: đóng kẹp')
                await a.retract_command_path('Gắp: nâng vật',require_hold=True)
                await a.confirm_hold()
                a.state.update(message='Đã gắp và nâng vật; bấm Thả vật khi sẵn sàng',state='manual_grasp_done')
            elif command=='LIFT':
                held=a.guard()['hardware']['joints'].get('d3')==b.config['grip']['close']
                if held:await a.confirm_hold()
                await a.retract_command_path('Nâng/rút tay, giữ góc D2',require_hold=held,preserve_wrist=True)
                a.state.update(state='manual_lift_done',message='Đã nâng/rút tay, giữ nguyên góc càng')
            else:
                await a.confirm_hold()
                j=a.guard()['hardware']['joints'];search=b.config['poses']['SEARCH']
                pose=a.arm_pose('PLACE');pose['d2']=a.guard()['hardware']['joints']['d2']
                at_place=all(j.get(k)==pose[k] for k in ('p1','p10','p11'))
                if not at_place:
                    if any(j.get(k)!=search[k] for k in ('p1','p10','p11')):
                        await a.retract_command_path('Thả: rút tay',require_hold=True)
                    await a.prepare_bin(self.held_bin)
                    await a.move_targets(pose,'Thả: đưa tới thùng')
                else:
                    # Never close the lid or rotate the bin underneath an arm at PLACE.
                    from auto_controller import BIN_KEYS
                    if j.get('p9')!=b.config['bins'][BIN_KEYS[self.held_bin]]:
                        raise ValueError('Tay ở PLACE nhưng sai ngăn; chỉnh tay trước khi thả')
                await a.confirm_hold()
                await a.move_targets({'p4':b.config['lid']['open']},'Thả: mở nắp')
                await a.move_targets({'d3':b.config['grip']['open']},'Thả: mở kẹp')
                end=time.monotonic()+2;clear=None
                while time.monotonic()<end:
                    if a.sensor('D19'):clear=None
                    elif clear is None:clear=time.monotonic()
                    elif time.monotonic()-clear>=.2:break
                    await a.wait(.03)
                else:raise ValueError('D19 còn thấy vật sau mở kẹp')
                await a.move_targets(a.arm_pose('SEARCH'),'Thả: rút tay')
                await a.move_targets({'p4':b.config['lid']['close']},'Thả: đóng nắp')
                self.held_bin=None
                a.state.update(state='manual_release_done',message='Đã thả vật và đóng nắp')
        except asyncio.CancelledError:raise
        except Exception as exc:
            a.state.update(state='manual_assist_error',message=str(exc))
            await asyncio.to_thread(b.detach_auto)
        finally:
            b.release_auto(a.token)
            a.target=None;a.locked_bin=None;a.locked_label=None
            a.classification_count=0;self.observation_at=0.
