"""Offline diagnostics: no serial commands, camera access, or web calls."""
import argparse
import math
from pathlib import Path
from arm_path import pulse_steps, targets_at
from config_io import read_namespace
from brain import validate_config
from gift_control import load_gift_params
from auto_controller import load_auto_config
from paper_distance import paper_position
from pose_profile import reference_at
from mecanum import smooth_timing


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parameters',default=str(Path(__file__).with_name('robot_params.py')))
    parser.add_argument('--point',type=float,nargs=2,metavar=('X_NORM','Y_NORM'))
    args=parser.parse_args();p=read_namespace(args.parameters)
    robot=validate_config(p['ROBOT_CONFIG']);cfg=load_auto_config(args.parameters,robot)
    gift=load_gift_params(Path(__file__).with_name('gift_params.py'))
    print('D4 Arduino · quà:',gift['GIFT_OPEN_US'],'→',gift['GIFT_CLOSE_US'],'us; kẹp đóng:',robot['grip']['close'],'us; ROI10x10')
    print('CẤU HÌNH PYTHON: HỢP LỆ')
    print('SEARCH:',robot['poses']['SEARCH'])
    print('Đích gắp:',cfg['arm_path']['end'])
    points=pulse_steps(cfg['arm_path']['start']['p1'],cfg['arm_path']['end']['p1'],p['S2_STEP_US'])
    print(f'{len(points)} bước P1; bước danh định {p["S2_STEP_US"]} µs, bước cuối {points[-1]-points[-2]} µs')
    print('Nội suy P10 từng đoạn qua TẤT CẢ mốc xung, không phải góc cơ khí:')
    knots=cfg['arm_path'].get('knots',[cfg['arm_path']['start'],cfg['arm_path']['end']])
    for a,b in zip(knots,knots[1:]):
        print(f'  [{a["p1"]},{b["p1"]}]: P10={a["p10"]}+({b["p10"]-a["p10"]})*(P1-{a["p1"]})/{b["p1"]-a["p1"]}')
    for pulse in [cfg['arm_path']['start']['p1'],*points]:
        target=targets_at(cfg['arm_path'],pulse)
        print(f'  P1={pulse:4d}, P10={target["p10"]:4d}')
    tof=cfg.get('box_tof_flow',{})
    if tof.get('enabled'):
        print('V3.16.14: BOX CẬP NHẬT + VL53L1X + D19.')
        print('  Pha tiếp cận trước gắp dùng FORWARD/LEFT/RIGHT; sau khi rơi có BACK tối đa 10 nhịp để tìm lại vật.')
        print(f'  DROP: BACK {cfg["control"]["move_ms"]}ms danh định, tối đa {cfg["drop_recovery"]["reverse_max_steps"]} nhịp; sau đó quét phải.')
        print('  Đúng lock: 1 frame; mất/sai 3 frame mới: '+('giữ chờ tại chỗ.' if tof.get('require_visible_target') else 'lặp có giới hạn theo cấu hình cũ.'))
        print('  Đáy trên dải: tiến. Trong/dưới dải: hạ tay. Chạm mép trên/dưới: tiến.')
        print('  Chạm một mép ngang: căn lại; box nhỏ: căn ngang rồi tiến; hai mép: tiếp cận VL53 ở tư thế hiện tại.')
        print('  AUTO: VL53 đủ khoảng cách thì chuyển gắp tại tư thế hiện tại, không bắt buộc chờ thêm ảnh.')
        print(f'  Tới {tof["handoff_p1_us"]}/{tof["handoff_p10_us"]}: ngừng nhận diện, tiến chậm {tof["slow_pulse_ms"]}ms thực thi/nhịp.')
        print(f'  AUTO: 1 số đo VL53 hợp lệ {tof["tof_min_mm"]}–{tof["tof_max_mm"]}mm ->2310/650; không kiểm tra tuổi mẫu ToF.')
        print('  Đã chốt đủ khoảng cách: không hết hạn vì chậm báo trạng thái hoặc mẫu VL53 già đi.')
        print('  IR D19 nhận sớm sau khóa vật/BIN: dừng bánh, hạ hết2310/650 rồi đọc IR mới để gắp; không giữ tay giữa đường.')
        print('  AUTO: số đo lỗi/chưa có -> vẫn tiến theo nhịp; 1 số đo hợp lệ đủ khoảng -> gắp. Manual giữ xác nhận riêng.')
        print('  Giới hạn thời gian/số nhịp, STOP, D18/D19 và USB vẫn áp dụng.')
        print(f'  Cuối: {tof["final_pulse_ms"]}ms/nhịp, D19 ổn định {tof["ir_confirm_ms"]}ms mới đóng kẹp.')
        print('  Firmware kèm theo V3.16.11 vẫn tương thích: VL53L1X Short / ROI10x10 / center199 / budget50ms / period60ms.')
        print('  SDA20/SCL21 dùng chung PCA0x40 và VL530x29. Điện áp theo đúng module đang dùng.')
        print('  FORWARD_MS=400 danh định ->560ms có ramp160; nhịp chậm/cuối160ms ->ramp80+80.')
        print('  Không bổ sung model phân tích che khuất; chỉ đánh dấu nghi chạm mép bằng box mới.')
        return
    ranges=[math.sqrt(d*d-p['SEARCH_CAMERA_HEIGHT_CM']**2) for d in p['LENS_TO_PAPER_CM_RANGE']]
    print(f'Nếu số đo cũ cùng điểm trên sàn: tâm giấy cách hình chiếu CAMERA {ranges[0]:.2f}–{ranges[1]:.2f} cm; không phải khoảng S2.')
    if cfg.get('pose_profile',{}).get('enabled'):
        print('Mốc ảnh USB: cm là ỐNG KÍNH -> CHAI; 27 cm là mốc S2 riêng.')
        for stage in cfg['pose_profile']['stages']:
            ref=reference_at(cfg['pose_profile'],stage['p1'])
            f=smooth_timing(stage['forward_ms'],cfg['motion']['ramp_ms'])[2]
            b=smooth_timing(stage['back_ms'],cfg['motion']['ramp_ms'])[2]
            print(f'  {stage["p1"]}/{stage["p10"]} -> {stage["lens_cm"]:g} cm tham chiếu; x={ref["target_x_norm"]:.3f}, đáy={ref["target_bottom_norm"]:.3f}; tiến/lùi={stage["forward_ms"]}/{stage["back_ms"]} ms danh định ({f}/{b} ms với ramp), nghỉ={stage["settle_ms"]} ms; bánh={stage["wheels_enabled"]}')
        print(f'Bánh bị khóa khi P1 >= {cfg["pose_profile"]["max_wheel_p1_us"]}; gắp vẫn cần ảnh mới ổn định + D19.')
    if cfg['distance']['enabled']:
        print(f'Khoảng cách sơ bộ từ mốc S2: bật, mục tiêu {cfg["distance"]["stop_forward_cm"]:g} cm.')
        print('Ma trận giấy vẽ tay chưa được kiểm chứng độc lập; số cm là ước lượng.')
    else:
        print('AUTO theo ảnh: sẵn sàng. Khoảng cách S2 liên tục: chưa được hiệu chuẩn.')
        if cfg.get('confirmed_grasp'):
            print('Ảnh gắp 27 cm đã xác nhận tại 2310/650; không cần ghi mốc trong SEARCH.')
            print('Chuyển tay để giữ ảnh; gắp cần ảnh gần nhất quán và D19, không tự đo cm.')
        else:
            print('Ghi tự động 3 ảnh USB tại mốc có sẵn: python record_grasp_reference.py')
    if args.point:
        xy=paper_position(args.point,p,[p['CAMERA_WIDTH'],p['CAMERA_HEIGHT']])
        print(f'Điểm ảnh trong hệ gốc GIẤY: X={xy[0]:.2f}, Y={xy[1]:.2f} cm (SEARCH, sơ bộ).')


if __name__=='__main__': main()
