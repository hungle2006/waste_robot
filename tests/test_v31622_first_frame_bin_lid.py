"""V3.16.22: first frame locks front BIN and preopens lid at SEARCH."""
import asyncio
import copy
from unittest.mock import AsyncMock
import pytest

from test_control import robot
from test_box_tof_flow import prepared, SequenceVision, raw
from auto_controller import NearHold
from box_tof_flow import BoxToFFlow
from target_lock import lock_config


def test_config_accepts_one_initial_classification_frame_but_not_zero():
    assert lock_config({'classify_frames': 1})['classify_frames'] == 1
    assert lock_config({'confirm_frames': 2})['confirm_frames'] == 2
    with pytest.raises(ValueError):
        lock_config({'classify_frames': 0})


def test_first_frame_locks_bin_without_3frame_vote(robot, tmp_path):
    async def go():
        _, _, a, _ = prepared(robot,tmp_path)
        a.target=None; a.locked_bin=None; a.locked_label=None
        a.classification_vote=None; a.classification_count=0
        a.vision=SequenceVision([raw(label='plastic bottle',bin=1)])
        obj=await a.detect()
        assert obj['bin']==1 and a.locked_bin==1
        assert a.locked_label=='plastic bottle'
        assert a.vision.calls==1
    asyncio.run(go())


def test_front_bin_closes_turns_opens_before_orientation(robot,tmp_path):
    async def go():
        b, hw, a, _=prepared(robot,tmp_path)
        a.holding_object=False
        a.front_lid_preopened=False
        events=[]
        async def move(targets,*args,**kwargs):
            events.append(('servo',dict(targets)))
            hw.state['joints'].update(targets)
            b.last_hw=hw.request('STATUS')
        a.move_targets=move
        a.emit=AsyncMock()
        # Starting from real SEARCH pose, D3 open.
        await a.prepare_bin(4,open_for_pickup=True)
        assert events==[
            ('servo',{'p4':b.config['lid']['close']}),
            ('servo',{'p9':b.config['bins']['OTHER']}),
            ('servo',{'p4':b.config['lid']['open']})]
        assert a.front_lid_preopened and a.selected_bin==4
        assert hw.state['joints']['p1']==b.config['poses']['SEARCH']['p1']
        assert hw.state['joints']['d3']==b.config['grip']['open']
        assert a.emit.await_args.args[0]=='auto_lid_ready'
    asyncio.run(go())


def test_rear_never_uses_preopen(robot,tmp_path):
    async def go():
        _, _, a, _=prepared(robot,tmp_path)
        a.move_targets=AsyncMock()
        with pytest.raises(Exception,match='D18'):
            await a.prepare_bin(1,rear=True,open_for_pickup=True)
        a.move_targets.assert_not_awaited()
    asyncio.run(go())


def test_search_flow_prepare_bin_before_orientation_and_no_premature_grip(robot,tmp_path):
    class Done(Exception):pass
    async def go():
        _,hw,a,_=prepared(robot,tmp_path)
        events=[]
        a.detect=AsyncMock(return_value=copy.deepcopy(a.target))
        async def bin_prepared(bin, **kwargs):
            events.append(('prepare',bin,kwargs))
        async def measure(obj):
            events.append(('orientation',obj['bin']))
            raise Done()
        a.prepare_bin=bin_prepared
        a.operator.remember_search_orientation=measure
        with pytest.raises(Done):
            await BoxToFFlow(a).run()
        assert events==[('prepare',4,{'open_for_pickup':True}),('orientation',4)]
        assert not any(cmd == 'S 3 2300' for cmd in hw.commands)
    asyncio.run(go())


def test_early_open_rejects_nonsearch_arm(robot,tmp_path):
    async def go():
        b,hw,a,_=prepared(robot,tmp_path)
        hw.state['joints']['p1']=1550
        b.last_hw=hw.request('STATUS')
        a.move_targets=AsyncMock()
        with pytest.raises(NearHold,match='SEARCH'):
            await a.prepare_bin(4,open_for_pickup=True)
        a.move_targets.assert_not_awaited()
    asyncio.run(go())


def test_bad_orientation_closes_preopened_lid_at_search(robot,tmp_path):
    async def go():
        b,hw,a,_=prepared(robot,tmp_path)
        events=[]
        a.detect=AsyncMock(return_value=copy.deepcopy(a.target))
        async def prep(bin,**_):
            events.append('rotate-open')
            a.front_lid_preopened=True
            hw.state['joints']['p4']=b.config['lid']['open']
        a.prepare_bin=prep
        async def orientation(_):
            events.append('bad-angle')
            raise NearHold('Không đo được góc')
        a.operator.remember_search_orientation=orientation
        async def move(targets,*args,**kwargs):
            events.append('close-lid')
            assert targets=={'p4':b.config['lid']['close']}
            hw.state['joints'].update(targets)
        a.move_targets=move
        with pytest.raises(NearHold,match='Không đo được góc'):
            await BoxToFFlow(a).run()
        assert events==['rotate-open','bad-angle','close-lid']
        assert not a.front_lid_preopened
        assert hw.state['joints']['d3']==b.config['grip']['open']
    asyncio.run(go())
