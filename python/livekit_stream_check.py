"""Safe preflight for Jetson -> LiveKit -> Vercel video. No motors, no camera, no secrets printed."""
import asyncio
import os
from pathlib import Path


def run_check():
    from dotenv import load_dotenv
    load_dotenv(Path(__file__).resolve().parent / '.env')
    print('=== WASTE ROBOT: KIỂM TRA PHÁT LIVEKIT LÊN APP ===')
    try:
        from jetson_livekit_camera import livekit_import_check
        ok, detail = livekit_import_check()
    except Exception as exc:
        ok, detail = False, type(exc).__name__
    print('SDK RTC/API:', 'OK' if ok else 'CHƯA SẴN SÀNG', detail)
    values = {'JETSON_LOCAL_CAMERA': os.getenv('JETSON_LOCAL_CAMERA',''),
              'LIVEKIT_PUBLISHER': os.getenv('LIVEKIT_PUBLISHER','auto'),
              'LIVEKIT_URL': os.getenv('LIVEKIT_URL',''),
              'LIVEKIT_API_KEY': os.getenv('LIVEKIT_API_KEY',''),
              'LIVEKIT_API_SECRET': os.getenv('LIVEKIT_API_SECRET','')}
    for key,val in values.items():
        if key == 'LIVEKIT_URL':
            display = 'OK (wss://)' if val.startswith('wss://') else 'THIẾU hoặc không bắt đầu wss://'
        elif key in ('LIVEKIT_API_KEY','LIVEKIT_API_SECRET'):
            display = 'Đã khai báo' if val else 'THIẾU'
        else:
            display = val or 'THIẾU'
        print(key+':', display)
    app_url = os.getenv('APP_URL','https://waste-app-control.vercel.app').rstrip('/')
    if not app_url.startswith('https://'):
        print('APP_URL chưa hợp lệ (cần https://).')
        return 2
    print('App:', app_url)
    if ok and values['LIVEKIT_API_KEY'] and values['LIVEKIT_API_SECRET']:
        try:
            from camera_publisher import make_token
            room_candidate = os.getenv('LIVEKIT_ROOM', 'robot_01').strip() or 'robot_01'
            encoded_token = make_token(room_candidate)
            print('Tạo token publisher:', 'OK' if isinstance(encoded_token, str) and len(encoded_token.split('.'))==3 else 'LỖI')
        except Exception as exc:
            print('Tạo token publisher: LỖI', type(exc).__name__)

    try:
        import httpx
        response = httpx.get(app_url+'/api/robot/configuration', timeout=12, follow_redirects=False)
        response.raise_for_status()
        data = response.json()
        livekit = data.get('livekit') or {}
        print('App LiveKit:', 'ĐÃ BẬT' if livekit.get('configured') else 'CHƯA BẬT')
        print('Track app:', livekit.get('trackName') or '(chưa có)')
        rid = data.get('robotId')
        print('Robot ID:', rid or '(chưa có)')
        configured_room = os.getenv('LIVEKIT_ROOM','').strip()
        if configured_room and rid and configured_room != rid:
            print('LỖI: LIVEKIT_ROOM không trùng robotId của app.')
        actual_track = os.getenv('LIVEKIT_TRACK_NAME','front_camera').strip() or 'front_camera'
        if livekit.get('trackName') and actual_track != livekit['trackName']:
            print('LỖI: LIVEKIT_TRACK_NAME không trùng trackName trên app.')
        if not livekit.get('configured'):
            print('CẦN THIẾT LẬP LIVEKIT TRÊN VERCEL, nếu không app chưa thể hiển thị video.')
    except Exception as exc:
        print('Không đọc được cấu hình app:', type(exc).__name__)
        print('Kiểm tra APP_URL, Internet và endpoint /api/robot/configuration.')
        return 2
    valid = (ok and values['JETSON_LOCAL_CAMERA']=='1' and
             values['LIVEKIT_PUBLISHER'].lower() not in ('0','off','false') and
             values['LIVEKIT_URL'].startswith('wss://') and
             values['LIVEKIT_API_KEY'] and values['LIVEKIT_API_SECRET'] and
             livekit.get('configured') and
             (not os.getenv('LIVEKIT_ROOM','').strip() or os.getenv('LIVEKIT_ROOM','').strip()==rid) and
             (not livekit.get('trackName') or (os.getenv('LIVEKIT_TRACK_NAME','front_camera').strip()==livekit.get('trackName'))))
    print('SẴN SÀNG THỬ PHÁT' if valid else 'CHƯA ĐỦ ĐIỀU KIỆN PHÁT VIDEO')
    print('Lưu ý: bước này không kiểm tra kết nối phòng hoặc khung hình camera thật.')
    return 0 if valid else 2

if __name__ == '__main__':
    raise SystemExit(run_check())
