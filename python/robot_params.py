"""THAM SỐ WASTE ROBOT V3.14.2 — sửa trực tiếp ở file này, STOP rồi chạy lại main.py.

Không cần mở trang hiệu chuẩn. AUTO_CONFIG/ROBOT_CONFIG ở dưới là nguồn chính;
các JSON cũ chỉ dành cho tương thích. Thông tin tài khoản giữ trong .env.
Mốc xung gắp đo được là P1=2310, P10=650; nội suy xung không phải góc cơ khí.
"""
from pathlib import Path
import runpy

# ===== CÁC THAM SỐ HAY CHỈNH =====
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 360  # cùng 16:9 với ma trận giấy 1920x1080
CAMERA_FPS = 15
AUTO_CALIBRATION_UI = False  # trang cũ không còn là bước bắt buộc
LOCAL_OPERATOR_UI = True  # Camera và bảng điều khiển trên máy tính
LOCAL_OPERATOR_OPEN_BROWSER = True  # Tự mở /control khi máy chủ đã sẵn sàng
LOCAL_OPERATOR_PORT = 8766

S2_STEP_US = 50
GRASP_P1_US = 2310
GRASP_P10_US = 650
GRASP_FORWARD_CM = 27.0  # từ hình chiếu trục S2 tới tâm chai, KHÔNG từ mép xe
BASE_SPEED = 20
SETTLE_MS = 500

FORWARD_MS = 400  # danh định; ramp160 -> tối đa560 ms thực thi

# ===== NĂM MỐC MỚI: P1/S2, P10/S1 (µs), ỐNG KÍNH -> CHAI (cm) =====
# SEARCH có 3 ảnh 60/75/90 cm. Chọn 75 cm làm mốc chuyển tay vì ảnh 60 cm
# bị gá kẹp che phần dưới chai. 5 cm cuối KHÔNG thay thế mốc S2 -> chai 27 cm.
ARM_MEASURED_POINTS = [
    (1300, 950, 75.0),
    (1550, 876, 45.0),
    (1800, 801, 24.0),
    (2050, 727, 12.0),
    (2310, 650, 5.0),
]

# Đơn vị THỜI GIAN là ms. Một hàng áp dụng từ mốc này tới trước mốc kế.
# forward/back/lateral: nhịp bánh danh định, KHÔNG là quãng đường cm.
# Có ramp: 60 ms danh định -> 120 ms thực thi; xem check_system.py.
# Tiến FORWARD không giới hạn số nhịp; vẫn kiểm tra ảnh, VL53, D19, timeout và STOP.
STAGE_TIMING = [
    dict(forward_ms=FORWARD_MS, back_ms=60, lateral_ms=160, settle_ms=500, arm_step_ms=60,
         max_forward_steps=None, max_back_steps=3, max_lateral_steps=12, wheels_enabled=True),
    dict(forward_ms=FORWARD_MS, back_ms=60, lateral_ms=120, settle_ms=500, arm_step_ms=70,
         max_forward_steps=None, max_back_steps=3, max_lateral_steps=10, wheels_enabled=True),
    dict(forward_ms=FORWARD_MS, back_ms=60, lateral_ms=120, settle_ms=600, arm_step_ms=80,
         max_forward_steps=None, max_back_steps=2, max_lateral_steps=8, wheels_enabled=True),
    dict(forward_ms=FORWARD_MS, back_ms=60, lateral_ms=120, settle_ms=700, arm_step_ms=90,
         max_forward_steps=None, max_back_steps=2, max_lateral_steps=6, wheels_enabled=True),
    dict(forward_ms=FORWARD_MS, back_ms=60, lateral_ms=60, settle_ms=700, arm_step_ms=90,
         max_forward_steps=None, max_back_steps=0, max_lateral_steps=0, wheels_enabled=False),
]
MAX_WHEEL_P1_US = 2260  # tắt bánh thường; IR cuối ở2310 có nhịp160ms riêng và giới hạn

# Đọc viền chai bằng mắt từ ảnh USB gốc 1920x1080; sai số tham chiếu khoảng
# ±25 px (ảnh gần ±40 px), CHƯA phải nhãn đo chính xác/hiệu chuẩn camera K.
# Thứ tự ảnh: 1,2,3 SEARCH; 4=1550; 7=1800; 5=2050; 6=2310.
USB_MEASUREMENTS = [
    dict(image_no=1, file='WIN_20261016_00_50_07_Pro.jpg', p1=1300, p10=950,
         lens_cm=60., bbox_px=[942,540,1052,751], quality='lower_occluded'),
    dict(image_no=2, file='WIN_20261016_00_50_48_Pro.jpg', p1=1300, p10=950,
         lens_cm=75., bbox_px=[947,402,1046,621], quality='full_manual'),
    dict(image_no=3, file='WIN_20261016_00_52_00_Pro.jpg', p1=1300, p10=950,
         lens_cm=90., bbox_px=[974,285,1055,448], quality='full_manual'),
    dict(image_no=4, file='WIN_20261016_00_55_48_Pro.jpg', p1=1550, p10=876,
         lens_cm=45., bbox_px=[918,28,1075,454], quality='full_manual_near_top'),
    dict(image_no=7, file='WIN_20261016_01_19_13_Pro.jpg', p1=1800, p10=801,
         lens_cm=24., bbox_px=[947,28,1202,790], quality='full_manual_near_top'),
    dict(image_no=5, file='WIN_20261016_01_03_16_Pro.jpg', p1=2050, p10=727,
         lens_cm=12., bbox_px=[780,0,1240,755], quality='top_clipped'),
    dict(image_no=6, file='WIN_20261016_01_05_13_Pro.jpg', p1=2310, p10=650,
         lens_cm=5., bbox_px=[260,0,1450,900], quality='top_clipped_lower_occluded'),
]

# ===== SỐ ĐO ĐÃ TRUY LẠI =====
PAPER_SIDE_CM = 17.0
SEARCH_CAMERA_HEIGHT_CM = 64.0
S2_HEIGHT_CM = 42.0
LENS_TO_PAPER_CM_RANGE = (92.0, 93.0)
CAMERA_ALONG_ARM_CM = 22.0
CAMERA_NORMAL_OFFSET_CM = 7.0
CAMERA_TO_GRIPPER_CM = 9.0
CAMERA_ANGLE_REPORTED_DEG = 60.0  # chưa rõ góc so với trục nào; không dùng suy ra pose
PAPER_FRAME_SIZE = (1920, 1080)
PAPER_CORNERS_PX = ((816,606),(1007,594),(1050,711),(838,737))
PAPER_H_PIXEL_TO_CM = (
    (0.12356038926153347, -0.020750599723310913, -88.25041420508491),
    (0.01403192133119075, 0.2233414145214528, -146.79494500625205),
    (-0.0005816989780184921, 0.0016644034809201015, 1.0),
)
# Ma trận trên là SƠ BỘ từ viền giấy vẽ tay. Không tự gán gốc giấy thành gốc S2.

# ===== TOÀN BỘ THAM SỐ CƠ KHÍ / TƯ THẾ =====
ROBOT_CONFIG = {'limits': {'p1': [500, 3500],
            'p10': [500, 2500],
            'p11': [500, 2500],
            'p9': [500, 2500],
            'p4': [500, 2500],
            'd2': [500, 2500],
            'd3': [500, 2500]},
 'step_us': 20,
 'step_ms': 40,
 'settle_ms': 300,
 'grip': {'open': 600, 'close': 2300},
 'lid': {'open': 1600, 'close': 500},
 'bins': {'ORGANIC': 1000, 'HAZARD': 1450, 'OTHER': 1900, 'RECYCLE': 2350},
 'poses': {'SETUP': {'p1': 1300, 'p10': 950, 'p11': 1250, 'd2': 2100, 'd3': 600, 'p9': 1900, 'p4': 500},
           'SEARCH': {'p1': 1300, 'p10': 950, 'p11': 1250, 'd2': 2100, 'd3': 600},
           'PLACE': {'p10': 1750, 'p11': 1100, 'p1': 1300, 'd2': 2100},
           'REAR': {'p11': 1300, 'p10': 1630, 'p1': 800, 'd2': 500}}}

# ===== TOÀN BỘ THAM SỐ AUTO =====
AUTO_CONFIG = {'version': 3,
 'calibration': {'mode': 'relative',
                 'relative_target_norm': [0.5, 0.5],
                 'relative_tolerance_norm': [0.03, 0.08],
                 'frame_size': [640, 480],
                 'frozen_joints_us': {'p10': 950, 'p11': 1250, 'd2': 2100},
                 'positive_error_x_move': 'RIGHT',
                 'positive_error_y_move': 'BACK',
                 'stages': [],
                 'grasp_final_s2_us': 2310,
                 'contact_sensor': 'D19',
                 'hold_sensor': 'D19',
                 'sensor_note': 'IR ở digital 19 Mega; cảm biến thùng rác ở D18, độc lập với IR gắp. Một IR '
                                'D19 kiểm tra hiện diện trước/sau nâng, không đo lực kẹp. PCA9685 không có '
                                'đầu vào cảm biến.',
                 'point_mode': 'relative',
                 'default_tolerance_norm': [0.025, 0.035],
                 'direction_trials': {},
                 'active_level': 0,
                 'grasp_hold_confirm_ms': 300},
 'control': {'speed': 20,
             'move_ms': 120,
             's2_step_us': 50,
             'settle_ms': 500,
             'stable_samples': 2,
             'max_align_moves_per_stage': 25,
             'max_target_jump_normalized': 0.2,
             'inference_timeout_s': 180,
             'max_result_age_s': 20,
             'max_cycle_s': 1200,
             'relative_min_response_norm': 0.01,
             'relative_max_no_response': 3,
             'no_target_retry_ms': 2000,
             'max_no_target_attempts': 20,
             'repeat': False,
             'max_cycles': 10},
 'measurements_reference_only': {'search_s1_s2_deg': 195,
                                 'search_pulses_us': {'p1': 1300,
                                                      'p10': 950,
                                                      'p11': 1250,
                                                      'd2': 2100,
                                                      'd3': 600},
                                 'approach_s1_s2_deg': 90,
                                 's2_pivot_height_cm': 42,
                                 'camera_along_arm_cm': 22,
                                 'camera_normal_offset_cm': 7,
                                 'camera_to_gripper_cm': 9,
                                 'search_camera_height_cm': 64,
                                 'search_lens_to_paper_cm_range': [92, 93],
                                 'camera_angle_reported_deg': 60,
                                 'camera_angle_reference': 'CHUA_XAC_DINH',
                                 's2_angle_mapping': 'SEARCH_1300_US_195_DEG_LOWER_REFERENCE_2900_US_90_DEG_CHUA_CO_NOI_SUY_GOC',
                                 'metric_distance_enabled': False},
 'rear': {'enabled_in_auto': True,
          'sensor': 'D18',
          'active_level': 0,
          'settle_ms': 700,
          'stable_samples': 2,
          'max_detection_attempts': 4,
          'retry_ms': 1000,
          'min_open_ms': 1500,
          'clear_confirm_ms': 500,
          'max_open_ms': 12000,
          'max_cycle_s': 240,
          'post_rear_back_steps': 10,
          'post_rear_back_ms': 400,
          'post_rear_settle_ms': 2000},
 'near': {'enabled': True,
          'stop_rw': 0.6,
          'edge_margin_norm': 0.01,
          'max_alignment_age_s': 30,
          'wait_contact_ms': 2000,
          'contact_confirm_ms': 300},
 'approach': {'enabled': True,
              'target_x_norm': 0.5,
              'x_tolerance_norm': 0.03,
              'stop_bottom_norm': 0.6,
              'stop_rw': 0.4,
              'stop_rh': 0.7,
              'forward_ms': 120,
              'lateral_ms': 100,
              'max_forward_steps': None,
              'max_lateral_steps': 25,
              'max_no_progress': 5,
              'min_progress_norm': 0.003,
              'arm_x_tolerance_norm': 0.06,
              'handoff_min_rw': 0.08,
              'handoff_min_rh': 0.12},
 'tracking': {'allow_label_variation': True, 'min_iou': 0.3, 'max_label_shift_norm': 0.08},
 'target_lock': {'enabled': True,
                 'min_iou': 0.15,
                 'max_shift_norm': 0.22,
                 'min_area_ratio': 0.35,
                 'max_area_ratio': 3.0,
                 'ambiguity_margin': 0.12,
                 'appearance_similarity': 0.7,
                 'confirm_frames': 2,
                 'classify_frames': 1,  # V3.16.22: khóa BIN ngay ảnh đầu tiên tại SEARCH; khớp lại vẫn 2 ảnh
                 'retry_ms': 500,
                 'reference_max_age_s': 30,
                 'overlay_max_age_s': 2.0,
                 'continuous_auto': True},
 'motion': {'enabled': True, 'ramp_ms': 160, 'arm_step_us': 50, 'arm_step_ms': 50},
 'visibility': {'enabled': True,
                'trigger_bottom_norm': 0.54,
                'blur_bottom_norm': 0.6,
                'min_bottom_norm': 0.15,
                'min_top_norm': 0.03,
                'near_forward_ms': 60,
                'near_margin_norm': 0.06,
                'min_step_us': 50,
                'prediction_margin_norm': 0.01,
                'response_noise_norm': 0.015,
                'min_response_norm': 0.002,
                'max_no_response': 4},
 'distance': {'enabled': False,
              'stop_forward_cm': 27.0,
              'tolerance_cm': 3.0,
              'slow_zone_cm': 10.0,
              'max_error_cm': 2.0,
              'calibration': None},
 'arm_path': {'start': {'p1': 1300, 'p10': 950, 'p11': 1250, 'd2': 2100},
              'end': {'p1': 2310, 'p10': 650, 'p11': 1250, 'd2': 2100}}}

# Các biến dễ chỉnh phía trên luôn có ưu tiên so với các giá trị mặc định trong dict.
AUTO_CONFIG['control'].update(speed=BASE_SPEED, settle_ms=SETTLE_MS, s2_step_us=S2_STEP_US)
AUTO_CONFIG['motion']['arm_step_us'] = S2_STEP_US
AUTO_CONFIG['visibility']['min_step_us'] = S2_STEP_US
AUTO_CONFIG['calibration']['grasp_final_s2_us'] = GRASP_P1_US
AUTO_CONFIG['calibration']['frame_size'] = [CAMERA_WIDTH, CAMERA_HEIGHT]
AUTO_CONFIG['arm_path']['start'] = {j: ROBOT_CONFIG['poses']['SEARCH'][j] for j in ('p1','p10','p11','d2')}
AUTO_CONFIG['arm_path']['end'].update(p1=GRASP_P1_US, p10=GRASP_P10_US,
    p11=ROBOT_CONFIG['poses']['SEARCH']['p11'], d2=ROBOT_CONFIG['poses']['SEARCH']['d2'])
AUTO_CONFIG['distance']['stop_forward_cm'] = GRASP_FORWARD_CM

# Ảnh WIN_20261016_00_22_10_Pro.jpg đã được người dùng xác nhận:
# chai ở mốc 27 cm khi P1=2310, P10=650. Đây là ảnh GẮP, không phải SEARCH.
# Các ngưỡng box sau là dung sai thử cho ảnh nhìn gần, không phải số cm/pixel.
AUTO_CONFIG['confirmed_grasp'] = {
    'enabled': True,
    'pose_us': {'p1':2310,'p10':650,'p11':1250,'d2':2100},
    'measured_forward_cm':27.0,
    'source_image':'WIN_20261016_00_22_10_Pro.jpg',
    'target_x_norm':.5, 'x_tolerance_norm':.06,
    'min_rw':.20, 'min_rh':.25, 'max_rw':.80,
    'side_margin_norm':.02, 'stable_frames':2,
    # Bánh chỉ nhích khi tay ĐỨNG YÊN ở tư thế trung gian và chai còn rõ.
    # Đây là giới hạn thử, chưa đo quãng đường cm của một nhịp bánh.
    'base_follow': {'enabled':True, 'trigger_bottom_norm':.48, 'pulse_ms':60,
                    'max_p1_us':GRASP_P1_US-S2_STEP_US, 'max_steps':40, 'max_no_progress':4},
}
# SEARCH không nhìn thấy chai ở 27 cm: không tự nạp mốc khoảng cách SEARCH cũ.
# Mốc 54% giờ chỉ chuyển BÁNH -> HẠ TAY GIỮ ẢNH; không kết luận đã tới 27 cm.
AUTO_CONFIG['distance']['enabled'] = False

# ===== NỘI SUY TỪNG ĐOẠN + MỤC TIÊU ẢNH THEO TƯ THẾ =====
# Tất cả mốc ở trên phải khớp SEARCH/GRASP; sửa cả bảng nếu đổi tư thế cơ khí.
AUTO_CONFIG['arm_path']['knots'] = [
    dict(p1=p1,p10=p10,p11=ROBOT_CONFIG['poses']['SEARCH']['p11'],
         d2=ROBOT_CONFIG['poses']['SEARCH']['d2'])
    for p1,p10,_ in ARM_MEASURED_POINTS
]
AUTO_CONFIG['pose_profile'] = {
    'enabled':True, 'frame_size':[1920,1080],
    'top_clip_from_p1':1800, 'max_wheel_p1_us':MAX_WHEEL_P1_US,
    'stable_frames':2, 'max_no_progress':4, 'min_progress_norm':.003,
    'side_margin_norm':.01, 'bottom_limit_norm':.94,
    'min_visible_rw':.02, 'min_visible_rh':.05,
    'max_arm_feature_jump_norm':.22,
    'search_samples':[dict(s) for s in USB_MEASUREMENTS if s['p1']==1300],
    'stages':[],
}
for i,(p1,p10,lens_cm) in enumerate(ARM_MEASURED_POINTS):
    sample=next(s for s in USB_MEASUREMENTS if s['p1']==p1 and s['lens_cm']==lens_cm)
    AUTO_CONFIG['pose_profile']['stages'].append(dict(sample,**STAGE_TIMING[i],
        x_tolerance_norm=.06 if i<4 else .09,
        bottom_tolerance_norm=.07 if i<4 else .10))
# Ảnh cuối bị cắt đầu nên chỉ dùng vùng ngang/đáy nhìn thấy + D19; không tính
# khoảng cách theo chiều cao bbox và không đẩy bánh thêm để "đủ 5 cm".
AUTO_CONFIG['confirmed_grasp'].update(
    source_image='WIN_20261016_01_05_13_Pro.jpg',
    measured_forward_cm=GRASP_FORWARD_CM, measured_lens_cm=5.0,
    target_x_norm=(260+1450)/2/1920, x_tolerance_norm=.09,
    min_rw=.20, min_rh=.35, max_rw=.85)
AUTO_CONFIG['confirmed_grasp']['base_follow']['enabled']=False  # thay bằng STAGE_TIMING

# ===== BÁM BOX + VL53L1X + D19 =====
AUTO_CONFIG['control']['move_ms'] = FORWARD_MS
AUTO_CONFIG['approach']['forward_ms'] = FORWARD_MS
AUTO_CONFIG['target_lock'].update(
    geometry_overrides_color=True, restart_initial_candidate=True, compensate_arm_pose=True)

TOF_HANDOFF_P1_US = 2050
TOF_HANDOFF_P10_US = 727
TOF_MIN_MM = 200
TOF_MAX_MM = 250
TOF_CONFIRM_SAMPLES = 3  # Manual/sensor-grasp confirmation; Auto uses one valid in-window reading.
TOF_SLOW_ACTUAL_MS = 160
TOF_SLOW_SETTLE_MS = 250
AUTO_CONFIG['distance']['enabled'] = False
AUTO_CONFIG['target_lock']['reference_max_age_s'] = 120
AUTO_CONFIG['box_tof_flow'] = {
    'enabled':True,
    'handoff_p1_us':TOF_HANDOFF_P1_US,'handoff_p10_us':TOF_HANDOFF_P10_US,
    'step_us':S2_STEP_US,'missing_frames':3,'retry_ms':200,
    'edge_margin_norm':8/360,'max_recovery_actions':6,
    'max_forward_steps':None,'max_lateral_steps':25,'max_arm_steps':20,
    'track_min_iou':.08,'track_max_shift_norm':.35,
    'track_min_area_ratio':.12,'track_max_area_ratio':8.,
    'tof_min_mm':TOF_MIN_MM,'tof_max_mm':TOF_MAX_MM,
    'tof_confirm_samples':TOF_CONFIRM_SAMPLES,'tof_max_age_ms':250,  # Freshness policy for Manual only.
    'tof_fault_timeout_ms':6000,'tof_approach_timeout_ms':45000,'max_tof_pulses':None,
    'slow_pulse_ms':TOF_SLOW_ACTUAL_MS,'slow_settle_ms':TOF_SLOW_SETTLE_MS,
    'final_pulse_ms':160,'max_final_pulses':None,'final_timeout_ms':15000,
    'final_settle_ms':250,'ir_confirm_ms':300,
    # Quá gần (<200mm) thì BACK 200ms THỰC, kiểm tra lại VL53; tối đa 10 nhịp.
    'overshoot_back_actual_ms':200,'overshoot_back_max_steps':10,
    'overshoot_min_progress_mm':3,
    'split_s2_first_on_rotation':True,
}


# V3.16.17: đo/ghi nhớ hướng của vật ở SEARCH (3 ảnh mới cùng vật),
# xoay D2 KHI VL53 đủ 20–25cm, TRƯỚC khi hạ P1/P10 tới vị trí gắp.
# D19 chỉ xác nhận vật sau khi hạ tay xong rồi mới đóng D3.
# LƯU Ý: calibrated=True là cờ PHẦN MỀM cho phép dùng bảng xung đang khai báo,
# KHÔNG chứng minh camera_sign/offset hay góc cơ khí đã đo đúng trên robot thật.
# Thử không tải, kiểm tra chiều xoay, hành trình, va chạm trước khi đưa rác thật.
AUTO_CONFIG['grasp_orientation'] = {
    'enabled': True, 'calibrated': True,
    'camera_sign': 1, 'offset_deg': 0.0,
    # |góc đã quy đổi| < 10°: không xoay D2, giữ 2100 µs. Đúng 10°: có xoay.
    'deadband_deg': 10.0,
    'calibration_pose': {'p1': 2310, 'p10': 650, 'p11': 1250, 'd2': 2100},
}

# Không lặp chuyển động khi đã mất vật qua 3 ảnh mới.
AUTO_CONFIG['box_tof_flow']['require_visible_target'] = True

# Quét liên tục: TURN_RIGHT ~5 giây, dừng 1 giây; thống kê từng nhóm 30 lượt, không tự tắt AUTO.
AUTO_SEARCH_TURN_SECONDS = 5.0
AUTO_SEARCH_STOP_SECONDS = 1.0
AUTO_SEARCH_MAX_TURNS = 30
# Khoảng quyền đổi quà sau khi nhận rác D18 và đóng nắp thành công.
GIFT_CLAIM_SECONDS = 120

# V3.16.18: phát hiện tuột chai bằng D19 sau đóng D3 và trong lúc nâng/đưa tới thùng.
# Tự nhận diện & gắp lại từ SEARCH tối đa một lần; rơi tiếp thì AUTO HOLD.
# D19 chỉ phát hiện hiện diện, không phải cảm biến lực kẹp, nên thử thực tế.
# V3.16.19: Sau khi D19 báo rơi và thu tay về SEARCH, kiểm tra ảnh rồi
# BACK tối đa 10 nhịp, dùng control.move_ms = FORWARD_MS (400ms danh định,
# ~560ms thực tế nhờ ramp). Dừng/kiểm tra camera sau MỖI nhịp.
# Nếu vẫn không thấy vật sau 10 lần -> quét phải 5s/dừng 1s liên tục cho tới khi có vật hoặc STOP.
# Không có cảm biến chống va chạm phía sau: chỉ dùng nơi đã bảo đảm thông thoáng.
AUTO_CONFIG['drop_recovery'] = {
    'max_retries': 1, 'retry_wait_ms': 800,
    'reverse_max_steps': 10, 'reverse_settle_ms': 2000,
    'loss_check_seconds': 15, 'loss_restore_ms': 300,
}
