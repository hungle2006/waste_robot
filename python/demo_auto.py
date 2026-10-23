"""Run a complete OFFLINE simulated cycle; no USB, camera, .env or web access.

    python demo_auto.py

Synthetic boxes follow the reference trajectory and simulated D19 activates at
the last pose. This checks software sequencing, not real camera/robot accuracy.
"""
import asyncio
import copy
import json
import tempfile
import time
from pathlib import Path
from brain import Brain
from transport import SimTransport
from config_io import read_config
from auto_controller import AutoController
from pose_profile import reference_at


class ToFDemoTransport(SimTransport):
    """Explicit synthetic range decreases only on pulses at2050, IR at2310."""
    def __init__(self, final, grip):
        super().__init__()
        self.final,self.grip=final,grip
        self.held=self.released=False
        self.slow_pulses=self.ir_pulses=0
        self.tof_sample(900)

    def request(self, command):
        joints=self.state['joints']
        if command.startswith('G 100 0 0 '):
            if joints.get('p1')==2050:self.slow_pulses+=1
            elif joints.get('p1')==self.final:self.ir_pulses+=1
        if command=='STATUS':
            if joints.get('d3')==self.grip['close']:self.held=True
            if self.held and joints.get('d3')==self.grip['open']:self.released=True
            active=(self.held or (joints.get('p1')==self.final and self.ir_pulses>=2)) and not self.released
            self.state['sensors']['D19']=0 if active else 1
            if self.tof_at is None or time.monotonic()-self.tof_at>=.06:
                self.tof_sample(max(225,900-220*self.slow_pulses))
        return super().request(command)


class ToFDemoVision:
    def __init__(self,hw,profile):
        self.hw, self.profile = hw, profile
        self.jpeg = b'synthetic-demo-only'
        self.frame_at = 0
        self.calls=[]

    async def detect(self,fresh_after=None):
        p1=self.hw.state['joints']['p1']
        if p1>=2050:raise AssertionError('Không được gọi detector từ2050/727')
        self.calls.append(p1)
        self.frame_at=time.monotonic()
        ref=reference_at(self.profile,p1)
        forwards=sum(s.startswith('G 100 0 0 ') for s in self.hw.commands)
        bottom=ref['target_bottom_norm']
        if p1==1300:bottom-=max(.12-.10*forwards,0.)
        height=.20+.00025*(p1-1300)
        cx=ref['target_x_norm'];width=.14+.00010*(p1-1300)
        box=[cx-width/2,bottom-height,cx+width/2,bottom]
        return {'found':True,'objects':[{'label':'can','bin':4,'_appearance':[1.,0.],
                 'bbox':[x*s for x,s in zip(box,[640,360,640,360])]}],
                '_frame_at':self.frame_at,'_frame_size':[640,360],'_frame_jpeg':self.jpeg}


async def main():
    source=Path(__file__).with_name('robot_params.py')
    r=read_config(source,'robot');c=read_config(source)
    r.update(step_us=100,step_ms=20,settle_ms=0)
    c['control'].update(settle_ms=100,repeat=False,max_cycle_s=120)
    c['motion']['arm_step_ms']=20
    c['target_lock']['continuous_auto']=False
    c['near'].update(wait_contact_ms=1000,contact_confirm_ms=200)
    for stage in c['pose_profile']['stages']:stage.update(settle_ms=100,arm_step_ms=20)
    with tempfile.TemporaryDirectory(prefix='waste_robot_demo_') as folder:
        robot_path=Path(folder)/'robot.json';auto_path=Path(folder)/'auto.json'
        robot_path.write_text(json.dumps(r));auto_path.write_text(json.dumps(c))
        hw=ToFDemoTransport(c['arm_path']['end']['p1'],r['grip'])
        brain=Brain(hw,robot_path,True)
        events=[]
        async def report(state):events.append(copy.deepcopy(state))
        vision=ToFDemoVision(hw,c['pose_profile'])
        auto=AutoController(brain,vision,auto_path,report)
        try:
            brain.connect();brain.start_monitor()
            while brain.busy or not brain.snapshot()['fresh']:await asyncio.sleep(.02)
            print('MÔ PHỎNG OFFLINE: dùng mốc xung thật, ảnh và IR được tạo giả lập.')
            await auto.start()
            if auto.task:await asyncio.wait_for(auto.task,timeout=120)
            result=auto.state['state'];ok=result=='auto_done' and hw.released
            print(f'KẾT QUẢ: {result}; chu trình={auto.state["cycles"]}; thả mô phỏng={hw.released}')
            print('Chưa xác nhận độ chính xác cơ khí/camera của robot thật.')
            return 0 if ok else 1
        finally:
            if auto.running:await auto.stop()
            brain.close()


if __name__=='__main__':raise SystemExit(asyncio.run(main()))
