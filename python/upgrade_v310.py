"""One-time settings migration; no camera/serial/network access and no movement."""
import argparse
import copy
import json
import os
from pathlib import Path
from floor_distance import DEFAULTS


def upgrade(path):
    path=Path(path)
    c=json.loads(path.read_text(encoding='utf-8'))
    backup=path.with_suffix('.before_v310.json')
    if not backup.exists():
        with backup.open('x',encoding='utf-8') as f:json.dump(c,f,ensure_ascii=False,indent=2)
    c['control']['s2_step_us']=50
    c.setdefault('motion',{})['arm_step_us']=50
    c.setdefault('visibility',{})['min_step_us']=50
    c['distance']=dict(copy.deepcopy(DEFAULTS),**c.get('distance',{}))
    c['distance']['enabled']=True
    temp=path.with_suffix('.upgrade.tmp')
    with temp.open('w',encoding='utf-8') as f:
        json.dump(c,f,ensure_ascii=False,indent=2);f.flush();os.fsync(f.fileno())
    os.replace(temp,path)
    return backup


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('auto_config.json'))
    args=parser.parse_args()
    backup=upgrade(args.config)
    print('Đã đặt bước S2 = 50 µs, bật điều kiện khoảng cách cm. Sao lưu:',backup)
    print('Chạy main.py, mở trang hiệu chuẩn để đo/lưu 4 FIT + 2 CHECK và khoảng dừng thật trước khi START.')
