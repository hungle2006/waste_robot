"""STOP-only read-only/firmware capability check; opens USB, resetting Arduino.

Never sends RESUME or motor/servo commands. Run only after main.py is stopped.
"""
import os
from pathlib import Path


def main():
    try:
        from dotenv import load_dotenv
        load_dotenv(Path(__file__).with_name('.env'))
        from transport import SerialTransport
        port = os.environ.get('ARDUINO_PORT', '/dev/ttyUSB0')
        serial = SerialTransport(port)
        try:
            serial.open()  # USB open resets Mega, then STOP; motion not allowed.
            s = serial.request('STATUS')
            print('Arduino:',serial.port_name)
            print('Drive protocol:',s.get('drive_protocol'))
            print('Smooth protocol:',s.get('smooth_protocol'))
            print('IG D19 guard:',s.get('ir_drive_guard', 'KHÔNG CÓ TRONG STATUS'))
            print('VL53 protocol:',s.get('tof_protocol'))
            print('AUTO continue:',s.get('tof_auto_continue'))
            print('D18:',s.get('sensors',{}).get('D18'))
            print('STOP latched:',s.get('stopped'))
            if (s.get('drive_protocol')==4 and s.get('smooth_protocol')==1
                    and isinstance(s.get('ir_drive_guard'),bool)
                    and s.get('tof_protocol')==1 and s.get('tof_auto_continue') is True):
                print('OK: firmware báo đầy đủ tính năng cần thiết cho AUTO. Chạy lại main.py và bấm Kết nối.')
                return 0
            print('CẦN NẠP LẠI FIRMWARE: arduino/robot_control_v3/robot_control_v3.ino')
            print('Chọn Arduino Mega 2560 và cổng CH341 tương ứng trong Arduino IDE; Upload, rồi kiểm tra lại.')
            return 2
        finally:
            serial.close()
    except (ConnectionError, TimeoutError, OSError, ValueError, ImportError) as exc:
        print('Chưa kiểm tra được firmware:',type(exc).__name__,str(exc))
        print('Đóng Arduino Serial Monitor và main.py; kiểm tra /dev/ttyUSB0, quyền dialout, dây USB.')
        return 3

if __name__=='__main__':
    raise SystemExit(main())
