"""AUTO D2 orientation: D19/final pose, 3 fresh frames, safety gates."""
import asyncio
import time
from unittest.mock import AsyncMock

import pytest

from test_control import robot
from test_box_tof_flow import prepared
from arm_path import targets_at
from auto_controller import NearHold, LostGrip
import operator_control
from grasp_orientation import pulse_for_angle


class FinalPoseVision:
    def __init__(self, boxes=None):
        self.calls=0
        self.boxes=boxes or [[40, 30, 560, 330]]*3

    async def detect(self, fresh_after=None):
        self.calls+=1
        return {'_frame_at':time.monotonic(), '_frame_size':[640,360],
                '_frame_jpeg':b'original-jpeg',
                'objects':[{'bbox': self.boxes[self.calls-1], 'label':'can','bin':4}]}


def setup_final(robot,tmp_path):
    b,hw,a,_=prepared(robot,tmp_path)
    a.cfg['grasp_orientation']['enabled']=True
    a.cfg['grasp_orientation']['calibrated']=True
    hw.state['joints'].update(targets_at(a.cfg['arm_path'],2310))
    hw.state['joints']['d3']=b.config['grip']['open']
    hw.state['sensors']['D19']=0
    b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
    a.vision=FinalPoseVision()
    a.move_targets=AsyncMock()
    a.emit=AsyncMock()
    return b,hw,a


def test_rotates_only_after_three_new_closeup_images(robot,tmp_path,monkeypatch):
    async def run():
        b,hw,a=setup_final(robot,tmp_path)
        axes=iter([20.,21.,20.])
        monkeypatch.setattr(operator_control,'estimate_axis',lambda *args:{'angle_deg':next(axes)})
        await a.operator.rotate_for_grasp(a.target)
        assert a.vision.calls==3
        assert a.move_targets.await_count==1
        assert a.move_targets.await_args.args[0]=={'d2':pulse_for_angle(20.3333333333333)}
        assert a.emit.await_args.args[0]=='auto_rotate_grip'
        # The procedure NEVER commands the D3 closure, which happens in cycle() afterward.
        assert all('d3' not in call.args[0] for call in a.move_targets.await_args_list)
    asyncio.run(run())


def test_no_rotation_until_final_pose_and_confirmed_ir(robot,tmp_path,monkeypatch):
    async def run():
        b,hw,a=setup_final(robot,tmp_path)
        monkeypatch.setattr(operator_control,'estimate_axis',lambda *args:{'angle_deg':10.})
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],2050))
        b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        with pytest.raises(NearHold,match='tư thế'):await a.operator.rotate_for_grasp(a.target)
        a.move_targets.assert_not_awaited()
        hw.state['joints'].update(targets_at(a.cfg['arm_path'],2310))
        hw.state['sensors']['D19']=1
        b.last_hw=hw.request('STATUS');b.updated=time.monotonic()
        with pytest.raises(LostGrip,match='D19'):await a.operator.rotate_for_grasp(a.target)
        a.move_targets.assert_not_awaited()
    asyncio.run(run())


def test_ambiguous_or_moving_closeup_does_not_rotate(robot,tmp_path,monkeypatch):
    async def run():
        b,hw,a=setup_final(robot,tmp_path)
        monkeypatch.setattr(operator_control,'estimate_axis',lambda *args:{'angle_deg':20.})
        async def two(*args,**kwargs):
            return {'_frame_at':time.monotonic(),'_frame_size':[640,360],
                    '_frame_jpeg':b'jpeg','objects':[
                        {'bbox':[40,30,560,330],'label':'can','bin':4},
                        {'bbox':[30,20,510,320],'label':'can','bin':4}]}
        a.vision.detect=two
        with pytest.raises(NearHold,match='đúng 1 vật'):await a.operator.rotate_for_grasp(a.target)
        a.move_targets.assert_not_awaited()
        a.vision=FinalPoseVision(boxes=[[40,30,560,330],[310,30,620,330],[310,30,620,330]])
        with pytest.raises(NearHold,match='dịch chuyển'):await a.operator.rotate_for_grasp(a.target)
        a.move_targets.assert_not_awaited()
    asyncio.run(run())


def test_unreliable_axis_holds_not_guesses(robot,tmp_path,monkeypatch):
    async def run():
        b,hw,a=setup_final(robot,tmp_path)
        monkeypatch.setattr(operator_control,'estimate_axis',lambda *args: (_ for _ in ()).throw(ValueError('Vật bị cắt khỏi ảnh')))
        with pytest.raises(NearHold,match='Vật bị cắt'):await a.operator.rotate_for_grasp(a.target)
        a.move_targets.assert_not_awaited()
    asyncio.run(run())
