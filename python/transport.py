"""One owner for USB, numbered replies, no reconnect that replays a move."""
import json
import os
import threading
import time
from mecanum import DRIVE_PROTOCOL, mix_mecanum, SMOOTH_PROTOCOL, smooth_pwm


def resolve_arduino_port(requested):
    """Select a USB serial device; never fall back to built-in Jetson UARTs.

    Explicit non-USB names (Windows COM ports, custom paths) are preserved.
    An explicit USB path is preferred when present; when unplugged, reconnect
    can follow its new ttyUSB/ttyACM number. No probing commands are sent here.
    """
    if os.name == 'nt':
        return requested
    if requested and not (requested == 'auto' or requested.startswith(('/dev/ttyUSB', '/dev/ttyACM'))):
        return requested
    from serial.tools import list_ports
    candidates = [p for p in list_ports.comports()
                  if p.device.startswith(('/dev/ttyUSB', '/dev/ttyACM'))]
    if requested != 'auto' and requested and os.path.exists(requested):
        return requested
    # CH340/CH341 preferred for this robot, but recognize USB CDC Arduino too.
    def rank(port):
        hwid = ((port.hwid or '') + ' ' + (port.description or '')).lower()
        chip = 0 if ('1a86:7523' in hwid or 'ch340' in hwid or 'ch341' in hwid) else 1
        tty = 0 if port.device.startswith('/dev/ttyUSB') else 1
        return (chip, tty, port.device)
    if candidates:
        chosen = sorted(candidates, key=rank)[0].device
        print('[Arduino] USB serial được phát hiện: {}'.format(chosen))
        return chosen
    raise ConnectionError('Không thấy Arduino USB Serial (/dev/ttyUSB* hoặc /dev/ttyACM*). Kiểm tra dây, driver CH341 và quyền dialout.')


class SerialTransport:
    def __init__(self, port, baud=115200):
        self.port_name, self.baud = port, baud
        self.port = None
        self.lock = threading.Lock()
        self.seq = 0

    def open(self):
        import serial
        if not callable(getattr(serial,'Serial',None)):
            raise ConnectionError('Cài nhầm module serial. Chạy: python -m pip uninstall -y serial pyserial; python -m pip install pyserial==3.5. Kiểm tra file serial.py trùng tên.')
        with self.lock:
            if self.port:
                return
            selected_port = resolve_arduino_port(self.port_name)
            options = {"exclusive": True} if os.name == "posix" else {}
            try:
                self.port = serial.Serial(selected_port, self.baud, timeout=.08,
                                          write_timeout=.5, **options)
            except OSError as exc:
                raise ConnectionError("Không mở được cổng USB: " + str(exc)) from exc
            self.port_name = selected_port
            time.sleep(2.3)
            self.port.reset_input_buffer()
        self.request("STOP")

    def request(self, command):
        with self.lock:
            if self.port is None:
                raise ConnectionError("Chưa kết nối USB. Nhấn Kết nối để thử lại.")
            self.seq += 1
            number = str(self.seq)
            try:
                self.port.write((number + " " + command + "\n").encode("ascii"))
                deadline = time.monotonic() + 1.2
                while time.monotonic() < deadline:
                    line = self.port.readline().decode("ascii", errors="replace").strip()
                    if not line.startswith(number + " "):
                        continue  # Ignore startup, watchdog events and old replies.
                    reply = line[len(number) + 1:]
                    if reply.startswith("ERR "):
                        raise ValueError(reply[4:])
                    if reply.startswith("STATUS "):
                        return json.loads(reply[7:])
                    if reply == "OK":
                        return {"ok": True}
                raise TimeoutError("Arduino không trả lời đúng mã lệnh.")
            except (OSError, TimeoutError) as exc:
                self.port.close()
                self.port = None
                raise ConnectionError(str(exc)) from exc

    def close(self):
        with self.lock:
            if self.port:
                self.port.close()
                self.port = None


class SimTransport:
    """Explicit demo. Never opens USB. Status includes simulation=true."""
    def __init__(self):
        self.connected = False
        self.state = {"pca": True, "stopped": True, "joints": {},
                      "drive_protocol": DRIVE_PROTOCOL, "gift_protocol": 3, "smooth_protocol": SMOOTH_PROTOCOL,
                      "ir_drive_guard": True,
                      "tof_protocol": 1, "tof_auto_continue": True, "tof": {"present": False,"valid": False,"mm": None,
                          "status":255,"seq":0,"age_ms":None,"guard":False,
                          "window_count":0,"min_mm":200,"max_mm":250,"roi_size_configured":10},
                      "motors": [0,0,0,0], "sensors": {"D15": 1,"D16": 1,"D18": 1,"D19": 1}}
        self.commands = []
        self.profile = None
        self.tof_at = None

    def tof_sample(self, mm, valid=True):
        """Explicit synthetic sample injection. Never invent a real sensor value."""
        t=self.state['tof']
        t.update(present=True,valid=valid,mm=mm,status=0 if valid else 255,
                 seq=(t['seq'] % 0xFFFFFFFF)+1,age_ms=0)
        self.tof_at=time.monotonic()
        t['window_count']=(min(255,t['window_count']+1)
            if t['guard'] and valid and type(mm) is int and t['min_mm']<=mm<=t['max_mm'] else 0)

    def tof_blocks_drive(self):
        t=self.state['tof']
        if self.tof_at is not None:
            t['age_ms']=int((time.monotonic()-self.tof_at)*1000)
        if t['guard'] and t.get('invalid_policy') == 'continue':
            return (t['present'] and t['valid'] and t['status'] == 0 and t['seq'] > 0
                    and type(t['mm']) is int and 0 < t['mm'] <= t['max_mm'])
        return t['guard'] and (not t['present'] or not t['valid'] or t['age_ms'] is None
                               or t['age_ms']>250 or t['mm']<=t['max_mm'])

    def open(self):
        self.connected = True
        self.request("STOP")

    def request(self, command):
        if not self.connected:
            raise ConnectionError("Demo chưa kết nối")
        if self.profile:
            if (self.state['sensors']['D18']==0 or (self.state['ir_drive_guard'] and self.state['sensors']['D19']==0)
                    or self.tof_blocks_drive()):
                self.profile=None;self.state['motors']=[0,0,0,0]
        if self.profile:
            start,axes,power,ramp,hold=self.profile
            elapsed=int((time.monotonic()-start)*1000)
            pwm=smooth_pwm(power,elapsed,ramp,hold)
            self.state['motors']=mix_mecanum(*axes,pwm) if pwm else [0,0,0,0]
            if elapsed>=2*ramp+hold:self.profile=None
        self.commands.append(command)
        parts = command.split()
        if command == "STATUS":
            self.tof_blocks_drive()
            return json.loads(json.dumps(self.state))
        if command == "STOP":
            self.profile = None
            self.state.update(stopped=True, motors=[0,0,0,0], joints={}, ir_drive_guard=True)
            self.state['tof'].update(guard=False,window_count=0,invalid_policy='stop')
        elif command == "RESUME":
            self.state["stopped"] = False
            self.state["ir_drive_guard"] = True
        elif command == "XM":
            self.profile = None
            self.state["motors"] = [0,0,0,0]
        elif self.state["stopped"]:
            raise ValueError("STOP_LATCHED")
        elif parts[0]=='IG':
            if len(parts)!=2 or parts[1] not in ('0','1'):raise ValueError('IG_RANGE')
            self.profile=None;self.state['motors']=[0,0,0,0]
            self.state['ir_drive_guard'] = parts[1]=='1'
        elif parts[0]=='TF':
            if command=='TF OFF':
                self.profile=None;self.state['motors']=[0,0,0,0]
                self.state['tof'].update(guard=False,window_count=0,invalid_policy='stop')
            else:
                auto_continue = len(parts)==4 and parts[1]=='AUTO'
                if not auto_continue and len(parts)!=3:raise ValueError('TOF_RANGE')
                lo,hi=map(int,parts[2:] if auto_continue else parts[1:])
                if not 100<=lo<hi<=1000:raise ValueError('TOF_RANGE')
                if not auto_continue and not self.state['tof']['present']:raise ValueError('TOF_NOT_PRESENT')
                self.profile=None;self.state['motors']=[0,0,0,0]
                self.state['tof'].update(guard=True,window_count=0,min_mm=lo,max_mm=hi,
                                         invalid_policy='continue' if auto_continue else 'stop')
        elif parts[0] in ("P", "S"):
            if parts[0]=="S" and (parts[1] not in {"2","3","4"} or not 500<=int(parts[2])<=2500):
                raise ValueError("PULSE_RANGE")
            self.state["joints"][("p" if parts[0] == "P" else "d")+parts[1]] = int(parts[2])
        elif parts[0] == 'G':
            f,r,c,power,ramp,hold=map(int,parts[1:])
            if not 40<=ramp<=400 or not 0<=hold<=400 or 2*ramp+hold>1000:raise ValueError('SMOOTH_RANGE')
            mix_mecanum(f,r,c,power)
            if self.state['sensors']['D18']==0 or (self.state['ir_drive_guard'] and self.state['sensors']['D19']==0):raise ValueError('IR_BLOCKS_AUTO_DRIVE')
            if self.tof_blocks_drive():raise ValueError('TOF_BLOCKS_AUTO_DRIVE')
            if self.profile or any(self.state['motors']):raise ValueError('STOP_BEFORE_SMOOTH')
            self.profile=(time.monotonic(),(f,r,c),power,ramp,hold)
            self.state['motors']=[0,0,0,0]
        elif parts[0] == "C":
            if self.tof_blocks_drive():raise ValueError('TOF_BLOCKS_AUTO_DRIVE')
            self.state["motors"] = mix_mecanum(*(int(x) for x in parts[1:5]))
        elif parts[0] == "D":
            if self.tof_blocks_drive():raise ValueError('TOF_BLOCKS_AUTO_DRIVE')
            self.state["motors"] = [int(x) for x in parts[1:5]]
        return {"ok": True}

    def close(self):
        self.connected = False
