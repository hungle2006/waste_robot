"""Inspect calibration only; live capture/record page is served by main.py."""
import argparse,json
from pathlib import Path
from auto_controller import load_auto_config

if __name__ == '__main__':
    root=Path(__file__).resolve().parent
    p=argparse.ArgumentParser(description='AUTO tương đối không cần ghi mốc. Chạy main.py rồi mở http://127.0.0.1:8766 nếu muốn chỉnh vùng tham chiếu. Công cụ này không gửi lệnh phần cứng.')
    p.add_argument('--config',type=Path,default=root/'auto_config.json')
    p.add_argument('--robot-config',type=Path,default=root/'robot_config.json')
    args=p.parse_args()
    try:
        cfg=load_auto_config(args.config,json.loads(args.robot_config.read_text()))
        print('AUTO tương đối sẵn cấu hình; không cần ghi các mốc pixel. Điểm tham chiếu là ước lượng, thử một lượt trước khi lặp.' if cfg['calibration']['mode']=='relative' else 'Cấu hình các mốc đủ trường; thử robot trước khi lặp AUTO.')
    except Exception as exc:
        print(str(exc));raise SystemExit(1)
