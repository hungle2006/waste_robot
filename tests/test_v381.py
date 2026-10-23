"""Regression cases from the real 60-61% bottom / tiny-object approach log."""
import asyncio,copy,json,time
import pytest
from test_control import robot
from test_target_lock import lock_setup,LockVision,until
from test_auto import finish
from auto_controller import load_auto_config
from approach_progress import ForwardProgress
from target_lock import associate,DEFAULTS
from livekit_modal import LiveKitModal


def geom(bottom,rw=.04,rh=.06):
    return {'extent':{'rw':rw,'rh':rh},'bbox_norm':[.48,bottom-rh,.48+rw,bottom]}


def test_small_cumulative_progress_is_accepted():
    p=ForwardProgress(5,.003)
    for step in range(1,6):
        result=p.observe(geom(.6+(step-1)*.001),geom(.6+step*.001))
        if step<5:assert result is None
    assert result['confirmed'] and result['gains'][2]==pytest.approx(.004)


def test_oscillating_boxes_are_not_counted_as_net_progress():
    p=ForwardProgress(5,.003)
    values=[.600,.601,.599,.601,.599,.600]
    for a,b in zip(values,values[1:]):result=p.observe(geom(a),geom(b))
    assert not result['confirmed']


def test_progress_windows_do_not_accumulate_positive_noise_forever():
    p=ForwardProgress(5,.003)
    for _ in range(2):
        for a,b in zip([.6,.601,.6,.601,.6],[.601,.6,.601,.6,.6]):result=p.observe(geom(a),geom(b))
        assert not result['confirmed'] and len(p.samples)==1


def test_first_target_is_anchored_even_when_a_stale_response_started_waiting(robot,tmp_path):
    b,hw,a,v,_,_=lock_setup(robot,tmp_path)
    a.cfg=load_auto_config(a.path,b.config);a.tracking_waiting=True;a.frame_after=time.monotonic()
    a.observed_frame_size=[640,480]
    r={'found':True,'object':{'label':'bottle','bin':1,'bbox':[300,220,340,260]},'_frame_size':[640,480]}
    one=a.select_locked_target(r)
    assert one and a.target and a.lock_sequence==1
    two=a.select_locked_target(r)
    assert two and a.lock_sequence==1
    r['object']['bbox']=[500,220,540,260]
    assert a.select_locked_target(r) is None and a.lock_sequence==1


def test_matching_failure_reports_geometry_and_color():
    a={'bbox_norm':[.4,.4,.5,.5],'center_norm':[.45,.45],'_appearance':[1.,0.]}
    b=dict(a,_appearance=[0.,1.])
    index,reason=associate(a,[b],DEFAULTS)
    assert index is None and 'màu' in reason and 'IoU=' in reason and 'diện tích=' in reason


def test_true_stall_still_holds_after_five_forward_steps_even_after_reacquisition(robot,tmp_path):
    async def run():
        b,hw,a,v,_,_=lock_setup(robot,tmp_path);v.stuck=True;v.vanish=3
        await a.start();await until(lambda:a.state['state']=='auto_hold',timeout=12)
        assert sum(x.startswith('C 100 0 0 ') for x in hw.commands)==5
        assert v.recoveries==3 and a.running and b.auto_owner==a.token
        assert 'S 3 2300' not in hw.commands
        await a.stop()
    asyncio.run(run())


class TinySlowVision(LockVision):
    async def detect(self,fresh_after=None):
        r=await super().detect(fresh_after)
        pulse,forward,_=self.calls[-1]
        if pulse==1300:
            bottom=.5991+.0002*forward
            target=r['objects'][-1]
            target['bbox']=[.47*640,(bottom-.08)*480,.53*640,bottom*480]
        return r


def test_reaching_bottom_goal_is_checked_before_stall_hold(robot,tmp_path):
    async def run():
        b,hw,a,v,world,c=lock_setup(robot,tmp_path)
        c['approach']['stop_bottom_norm']=.60;a.path.write_text(json.dumps(c))
        a.vision=TinySlowVision(b,hw)
        await a.start();await finish(a,timeout=20)
        assert a.state['state']=='auto_done' and world.released,a.state
        assert sum(x.startswith('C 100 0 0 ') for x in hw.commands)==5
    asyncio.run(run())






def test_classification_name_is_taken_when_bin_is_confirmed_not_from_first_guess(robot,tmp_path):
    async def run():
        b,hw,a,v,_,_=lock_setup(robot,tmp_path)
        detect=v.detect;calls=0
        async def initial_wrong_name(fresh_after=None):
            nonlocal calls
            r=await detect(fresh_after);calls+=1
            r['objects'][-1]['label']='battery' if calls==1 else 'bottle'
            return r
        v.detect=initial_wrong_name
        await a.start();await until(lambda:a.selected_bin==1)
        assert a.locked_label=='bottle' and a.target['label']=='bottle'
        await a.stop()
    asyncio.run(run())


def test_windowed_mode_can_laterally_align_then_move_forward(robot,tmp_path):
    async def run():
        b,hw,a,v,world,_=lock_setup(robot,tmp_path);v.offset=.07
        await a.start();await finish(a,timeout=20)
        assert a.state['state']=='auto_done' and world.released
        left_right=next(i for i,c in enumerate(hw.commands) if c.startswith('C 0 100 0 '))
        forward=next(i for i,c in enumerate(hw.commands) if c.startswith('C 100 0 0 '))
        assert left_right<forward
    asyncio.run(run())


