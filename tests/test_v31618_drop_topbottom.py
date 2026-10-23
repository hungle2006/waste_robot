"""V3.16.18: derive tilt from bottle, not bbox corners; bounded loss recovery."""
import asyncio
import copy
from unittest.mock import AsyncMock

import numpy as np
import pytest

from auto_controller import LostGrip, NearHold, CloseRangeContact
from box_tof_flow import BoxToFFlow
from grasp_orientation import top_bottom_foreground_axis
from test_control import robot
from test_box_tof_flow import prepared


def test_top_bottom_foreground_measures_left_right_not_midpoint_of_aabb():
    # Synthetic elongated silhouette: x = 100 + slope*(80-y).
    ys=np.repeat(np.arange(30,131),9)
    for slope,kind in [(0.45,'right'),(-0.45,'left'),(0.,'upright')]:
        centers=100+slope*(80-ys)
        xs=centers+np.tile(np.arange(-4,5),101)
        angle,top,bottom=top_bottom_foreground_axis(xs,ys)
        assert top[1]<bottom[1]
        if kind=='right':
            assert top[0]>bottom[0] and 20<angle<30
        elif kind=='left':
            assert top[0]<bottom[0] and -30<angle<-20
        else:
            assert top[0]==pytest.approx(bottom[0]) and abs(angle)<1


def test_drop_retry_once_then_auto_hold(robot,tmp_path):
    async def go():
        _b,_hw,a,_=prepared(robot,tmp_path)
        a.emit=AsyncMock()
        a.wait=AsyncMock()
        a.recover_lost_grip=AsyncMock()
        assert a.drop_failures==0
        await a.handle_lost_grip('D19 mất vật sau đóng kẹp')
        assert a.drop_failures==1
        assert any(call.args[0]=='auto_drop_retry' for call in a.emit.await_args_list)
        with pytest.raises(NearHold,match='2 lần'):
            await a.handle_lost_grip('D19 mất vật khi nâng')
        assert a.drop_failures==2
        assert a.recover_lost_grip.await_count==2
        assert all(call.args[0]!='auto_cycle_done' for call in a.emit.await_args_list)
    asyncio.run(go())


def test_after_close_d19_missing_prevents_lift(robot,tmp_path,monkeypatch):
    async def go():
        b,hw,a,_=prepared(robot,tmp_path)
        a.move_targets=AsyncMock()
        a.emit=AsyncMock()
        a.sensor=lambda _pin: True
        a.confirm_hold=AsyncMock(side_effect=LostGrip('Mất vật ngay sau D3 đóng'))
        bottle=copy.deepcopy(a.target)
        async def fake_approach(flow):
            a.locked_bin=4
            a.locked_label='can'
            a.selected_bin=4
            a.search_orientation={'target_id':a.lock_sequence, 'bin':4,
                                  'applied':True,'pulse_us':hw.state['joints']['d2']}
            raise CloseRangeContact(copy.deepcopy(bottle))
        monkeypatch.setattr(BoxToFFlow,'run',fake_approach)
        with pytest.raises(LostGrip,match='sau D3'):
            await a.cycle()
        actions=[call.args[0] for call in a.move_targets.await_args_list]
        assert {'d3':b.config['grip']['close']} in actions
        assert not any('AUTO nâng' in str(call) or 'AUTO PLACE' in str(call)
                       for call in a.move_targets.await_args_list)
        assert a.holding_object is True
        assert any(call.args[0]=='auto_grip_check' for call in a.emit.await_args_list)
    asyncio.run(go())


def test_d19_loss_blocks_next_arm_move(robot,tmp_path):
    async def go():
        b,hw,a,_=prepared(robot,tmp_path)
        a.holding_object=True
        a.sensor=lambda _pin: False
        a.check_grip_or_drop=AsyncMock(side_effect=LostGrip('D19 mất vật trước chuyển động'))
        old=len(hw.commands)
        with pytest.raises(LostGrip,match='trước chuyển động'):
            await a.move_targets({'p4':b.config['lid']['open']},'AUTO mở nắp')
        assert not any(command.startswith(('P ', 'S ')) for command in hw.commands[old:])
    asyncio.run(go())


def test_drop_recovery_refuses_opening_if_d19_detects_again(robot,tmp_path):
    async def go():
        b,hw,a,_=prepared(robot,tmp_path)
        a.sensor=lambda _pin: True
        a.wait=AsyncMock()
        a.move_targets=AsyncMock()
        a.fresh_hardware=AsyncMock()
        with pytest.raises(NearHold,match='D19 phát hiện vật trở lại'):
            await a.recover_lost_grip('Mất tín hiệu tạm thời')
        assert all(call.args[0].get('d3') != b.config['grip']['open']
                   for call in a.move_targets.await_args_list)
    asyncio.run(go())
