"""Ghi mốc USB 27 cm không nhập pixel và không điều khiển robot.

STOP AUTO, đặt tâm chai ở mốc 27 cm đã đo, giữ xe và tay đúng SEARCH.
Tắt main.py để nhường camera. Chạy: python record_grasp_reference.py
Chương trình chỉ mở camera, gọi Modal và ghi grasp_reference.py sau 3 ảnh khớp.
Không dùng ảnh chụp robot bằng điện thoại làm ảnh camera USB.
"""
import argparse
import asyncio
import math
import os
from pathlib import Path
import pprint
import time

from config_io import read_namespace
from livekit_modal import parse_detection
from paper_distance import create_reference

ROOT=Path(__file__).resolve().parent


def select_reference_point(result, frame_size):
    objects=parse_detection(result)['objects']
    if len(objects)!=1:
        raise ValueError('Cần đúng một chai trong ảnh; chưa tự chọn giữa nhiều vật')
    obj=objects[0];box=obj.get('bbox');w,h=frame_size
    if not isinstance(box,list) or len(box)!=4 or any(type(v) not in (int,float) or not math.isfinite(v) for v in box):
        raise ValueError('Modal cần trả bbox pixel [x1,y1,x2,y2]')
    x1,y1,x2,y2=box
    if not 0<x1<x2<w-1 or not 0<y1<y2<h-1:
        raise ValueError('Chai sát mép ảnh; chưa ghi mốc')
    # This is an approximate floor contact, not the center of a tall 3D box.
    return [((x1+x2)/2)/w,y2/h],obj['bin']


async def record(parameters, camera_factory, detect, count=3):
    cap=camera_factory(parameters['CAMERA_WIDTH'],parameters['CAMERA_HEIGHT'],parameters['CAMERA_FPS'])
    points=[];bins=[];frame_size=None
    try:
        for i in range(count):
            # Flush queued frames after slow inference. Camera never receives drawn guides.
            for _ in range(5):
                ok,frame=cap.read()
            if not ok or frame is None: raise ValueError('Không đọc được ảnh USB mới')
            import cv2
            frame_size=[frame.shape[1],frame.shape[0]]
            cw,ch=parameters['PAPER_FRAME_SIZE']
            if abs((frame_size[0]/frame_size[1])/(cw/ch)-1)>.01:
                raise ValueError('Camera phải trả ảnh 16:9; không kéo giãn ảnh 4:3 thành 16:9')
            ok,jpeg=cv2.imencode('.jpg',frame)
            if not ok: raise ValueError('Không tạo được ảnh camera')
            result=await detect(jpeg.tobytes())
            point,bin_id=select_reference_point(result,frame_size)
            points.append(point);bins.append(bin_id)
            print(f'Ảnh {i+1}/{count}: điểm chân box {point[0]:.4f}, {point[1]:.4f}')
    finally:
        cap.release()
    if len(set(bins))!=1 or max(math.dist(a,b) for a in points for b in points)>.015:
        raise ValueError('Ba ảnh chưa khớp ổn định; chưa ghi file')
    mean=[sum(p[k] for p in points)/count for k in (0,1)]
    reference=create_reference(parameters,mean,frame_size)
    reference['capture_count']=count
    reference['captured_at_local']=time.strftime('%Y-%m-%d %H:%M:%S')
    return reference


async def main(parameters_path):
    from dotenv import load_dotenv
    import httpx
    load_dotenv(ROOT/'.env')
    os.environ['ROBOT_PARAMETERS']=str(Path(parameters_path).resolve())
    from camera_publisher import open_camera
    p=read_namespace(parameters_path)
    if (p['AUTO_CONFIG'].get('confirmed_grasp') or {}).get('enabled'):
        raise ValueError('Đã có ảnh gắp 27 cm tại 2310/650; không dùng công cụ ghi mốc SEARCH cho bố trí này.')
    print(f'Giữ chai tại mốc {p["GRASP_FORWARD_CM"]:g} cm từ hình chiếu S2; tay SEARCH, xe đứng yên.')
    print('Cả main.py và ứng dụng khác phải nhường camera. Chương trình không phát lệnh cơ khí.')
    input('Nhấn ENTER khi bố trí trên đã đúng (Ctrl+C để hủy): ')
    modal_url=os.environ.get('MODAL_AI_URL','').strip()
    async with httpx.AsyncClient(timeout=180) as client:
        if not modal_url:
            app_url=os.environ.get('APP_URL','https://waste-app-control.vercel.app').rstrip('/')
            response=await client.get(app_url+'/api/robot/configuration');response.raise_for_status()
            modal_url=response.json().get('modalAiUrl','')
        if not modal_url.startswith('https://'): raise ValueError('Chưa có Modal URL HTTPS từ app/.env')
        async def detect(jpeg):
            response=await client.post(modal_url.rstrip('/')+'/detect',files={'image':('reference.jpg',jpeg,'image/jpeg')})
            response.raise_for_status();return response.json()
        reference=await record(p,open_camera,detect)
    output=Path(parameters_path).resolve().with_name('grasp_reference.py')
    if output.exists():
        backup=output.with_name('grasp_reference.previous.py')
        backup.write_bytes(output.read_bytes())
    temp=output.with_suffix('.tmp')
    temp.write_text('"""Mốc gắp camera USB; sơ bộ, chưa có kiểm chứng sai số độc lập."""\nREFERENCE = '+
                    pprint.pformat(reference,sort_dicts=False,width=100)+'\n',encoding='utf-8')
    os.replace(temp,output)
    print('Đã ghi '+str(output)+'; main.py sẽ tự dùng điều kiện khoảng cách thay đường 54%.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parameters',default=os.environ.get('ROBOT_PARAMETERS',str(ROOT/'robot_params.py')))
    args=parser.parse_args()
    try: asyncio.run(main(args.parameters))
    except KeyboardInterrupt: print('Đã hủy; không thay mốc.')
    except Exception as exc:
        print('Chưa ghi mốc: '+str(exc))
        raise SystemExit(1)
