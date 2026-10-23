"""Fast logic tests: split S2/S1 branch and range-verified BACK."""
import asyncio
import copy
from pathlib import Path
from types import SimpleNamespace
import pytest
from robot_params import ROBOT_CONFIG
from auto_controller import load_auto_config, NearHold, CloseRangeContact
from box_tof_flow import BoxToFFlow

PARAMS = Path(__file__).resolve().parents[1]/'python'/'robot_params.py'

class FakeBrain:
    def __init__(self, owner):
        self.owner=owner
        self.auto_owner='test'
        self.epoch=1
        self.config={'grip':{'open': 500}}
        self.moves=[]
        self.autoguards=[]
    def smooth_auto_drive(self, mode, epoch, speed, nominal, ramp, token):
        self.moves.append((mode, nominal, ramp))
        if mode == 'BACK' and self.owner.back_progress:
            self.owner.mm += self.owner.back_progress
        return 0
    def command(self, cmd, epoch, _owner=None):
        self.moves.append((cmd,))
    def tof_guard(self, enabled, token, minimum, maximum, ignore_invalid=False):
        self.autoguards.append(enabled)
    def set_auto_ir_drive_guard(self, enabled, token):
        pass

class FakeOwner:
    def __init__(self, mm=400):
        self.cfg=load_auto_config(PARAMS, ROBOT_CONFIG)
        self.token='test'
        self.lock_sequence=1
        self.locked_bin=self.selected_bin=1
        self.locked_label='plastic bottle'
        self.search_orientation=dict(target_id=1,bin=1,label='plastic bottle',
            source_pose=copy.deepcopy(self.cfg['arm_path']['start']),
            deadband_applied=False,pulse_us=2300,angle_deg=30,applied=False)
        self.state={}
        self.mm=mm
        self.back_progress=0
        self.joints=dict(p1=1300,p10=950,p11=1250,d2=2100,d3=500)
        self.events=[]
        self.joint_moves=[]
        self.brain=FakeBrain(self)
        self.vision_paused=False
        self.final_authorized=None
        self.base_steps=0
        self.aligned_anchor=None
        self.observed_jpeg=None
    def guard(self):
        return {'busy':False,'drive':{'active':False},'epoch':1,
            'hardware': {'joints':self.joints,'motors':[0,0,0,0],
                'tof_protocol':1,'tof':{'valid':True,'status':0,'present':True,
                    'mm':self.mm,'seq':1,'age_ms':10}}}
    async def fresh_hardware(self):
        return self.guard()
    async def move_targets(self,targets,*args,**kwargs):
        self.joints.update(targets)
        self.joint_moves.append(dict(targets))
    async def emit(self,*args,**kwargs):
        self.events.append(args)
    def check_rear(self):
        pass
    def clear_overlay(self):
        pass
    async def wait(self,n):
        await asyncio.sleep(0)
    def sensor(self,name):
        return True

def test_config_and_timing():
    c=load_auto_config(PARAMS,ROBOT_CONFIG)['box_tof_flow']
    assert (c['overshoot_back_actual_ms'],c['overshoot_back_max_steps'])==(200,10)
    from mecanum import smooth_timing
    from box_tof_flow import nominal_for_actual
    nominal=nominal_for_actual(c['overshoot_back_actual_ms'],160)
    assert (nominal,smooth_timing(nominal,160)[2])==(100,200)
    assert load_auto_config(PARAMS,ROBOT_CONFIG)['pose_profile']['stages'][0]['forward_ms']==400


def test_rotated_branch_s2_moves_only_and_s1_waits_for_tof():
    async def run():
        a=FakeOwner(mm=390)
        f=BoxToFFlow(a)
        f.last_obj={'bbox_norm':[.40,.3,.6,.7]}
        async def observe():
            return {'bbox_norm':[.40,.3,.6,.7]}
        async def sensor_loop(p1):
            assert p1==2310
            assert f.pre_s2_only and f.tof_pose==2310
            assert a.joints['p1']==2310
            assert a.joints['p10']==950
            assert a.joints['d2']==2100  # no wrist yet; needs in-window ToF
            assert a.vision_paused
            return 'ready'
        f.observe=observe
        f.sensor_distance_loop=sensor_loop
        assert await f.rotated_s2_first()=='ready'
        assert len(a.joint_moves)>1
        assert all('p10' not in step and 'd2' not in step for step in a.joint_moves)
    asyncio.run(run())


def test_final_grasp_rotated_branch_only_s1_after_tof():
    async def run():
        a=FakeOwner(mm=224)
        a.joints.update(p1=2310,p10=950)
        a.vision_paused=True
        f=BoxToFFlow(a)
        f.last_obj={'bbox_norm':[.40,.3,.6,.7]}
        f.stage='tof_approach'
        f.pre_s2_only=True
        f.tof_pose=2310
        f.tof_confirmed=True
        f.tof_confirmed_at=1.0
        # No need to actually wait 300ms in the fake; sensor state stable.
        f.c['ir_confirm_ms']=0
        with pytest.raises(CloseRangeContact):
            await f.final_approach()
        assert not f.pre_s2_only
        assert a.joints['p1']==2310 and a.joints['p10']==650
        assert a.joints['d2']==2300
        assert a.joint_moves==[{'d2':2300},{'p10':650}]
        assert a.search_orientation['applied'] is True
    asyncio.run(run())


def test_overshoot_reverses_up_to_valid_window():
    async def run():
        a=FakeOwner(mm=159)
        a.back_progress=23
        f=BoxToFFlow(a)
        f.stage='tof_approach'
        f.tof_pose=1300
        f.tof_guard_armed=True
        assert await f.overshoot_back_to_window(p1=1300)==205
        assert [p[0] for p in a.brain.moves if p[0]=='BACK']==['BACK','BACK']
        assert [p[1] for p in a.brain.moves if p[0]=='BACK']==[100,100]
        assert a.joints['p10']==950
    asyncio.run(run())


def test_overshoot_no_progress_holds_instead_of_backing_indefinitely():
    async def run():
        a=FakeOwner(mm=160)
        f=BoxToFFlow(a)
        f.stage='tof_approach'
        f.tof_pose=1300
        with pytest.raises(NearHold,match='không tăng khoảng cách'):
            await f.overshoot_back_to_window(p1=1300)
        assert sum(m[0]=='BACK' for m in a.brain.moves)==2
    asyncio.run(run())


def test_back_passes_25cm_window_returns_to_normal_forward_approach():
    async def run():
        a=FakeOwner(mm=180)
        a.back_progress=90
        f=BoxToFFlow(a)
        f.stage='tof_approach'
        assert await f.overshoot_back_to_window(p1=1300)==270
        assert sum(m[0]=='BACK' for m in a.brain.moves)==1
        assert any(event[0]=='auto_tof_back_beyond' for event in a.events)
    asyncio.run(run())
