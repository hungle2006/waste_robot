"""Pose-conditioned image references; no camera intrinsic or Cartesian claims.

Boxes were manually read from the seven supplied USB photographs. Only pulse
commands and image references are interpolated. Distances remain measurements
at the reference photos, never a current S2/floor distance estimate.
"""
import copy
import math
from arm_path import targets_at


def _num(value, low, high, name, integer=False):
    if ((type(value) is not int if integer else type(value) not in (int,float))
            or not math.isfinite(value) or not low<=value<=high):
        raise ValueError('Sai pose_profile.'+name)


def checked_profile(value, path, config):
    if not value:
        return {'enabled':False}
    if not isinstance(value,dict) or type(value.get('enabled')) is not bool:
        raise ValueError('Sai pose_profile.enabled')
    if not value['enabled']:
        return copy.deepcopy(value)
    if not path or config['calibration']['mode']!='relative' or not config['motion']['enabled']:
        raise ValueError('Mốc ảnh cần arm_path, relative và motion.enabled')
    if config.get('distance',{}).get('enabled'):
        raise ValueError('Không dùng ma trận SEARCH cho quỹ đạo camera đang đổi tư thế')
    if not (config.get('confirmed_grasp') or {}).get('enabled'):
        raise ValueError('Mốc ảnh cần confirmed_grasp để kiểm tra tư thế cuối và D19')
    if value.get('frame_size')!=[1920,1080]:
        raise ValueError('Ảnh mốc được đọc ở 1920x1080, không kéo giãn tỷ lệ')
    _num(value.get('top_clip_from_p1'),path['start']['p1'],path['end']['p1'],'top_clip_from_p1',True)
    _num(value.get('max_wheel_p1_us'),path['start']['p1'],path['end']['p1']-1,'max_wheel_p1_us',True)
    for k,lo,hi,integer in [('stable_frames',2,5,True),('max_no_progress',2,6,True),
            ('min_progress_norm',.001,.03,False),('side_margin_norm',.005,.05,False),
            ('bottom_limit_norm',.8,.99,False),('min_visible_rw',.01,.2,False),
            ('min_visible_rh',.02,.3,False),('max_arm_feature_jump_norm',.05,.4,False)]:
        _num(value.get(k),lo,hi,k,integer)
    stages=value.get('stages')
    if not isinstance(stages,list) or len(stages)<2:
        raise ValueError('Cần ít nhất hai mốc pose_profile')
    if [p.get('p1') for p in stages]!=[p['p1'] for p in path.get('knots',[path['start'],path['end']])]:
        raise ValueError('Mốc ảnh phải trùng các mốc xung arm_path.knots')
    for i,stage in enumerate(stages):
        if stage.get('p10')!=targets_at(path,stage['p1'])['p10']:
            raise ValueError('P10 mốc ảnh không khớp quỹ đạo')
        _num(stage.get('lens_cm'),1,200,'lens_cm')
        box=stage.get('bbox_px')
        if not isinstance(box,(list,tuple)) or len(box)!=4:
            raise ValueError('Cần bbox_px của ảnh tham chiếu')
        for x,high in zip(box,[1920,1080,1920,1080]):
            _num(x,0,high,'bbox_px')
        if box[2]<=box[0] or box[3]<=box[1]:
            raise ValueError('bbox_px rỗng/ngược')
        for k in ('x_tolerance_norm','bottom_tolerance_norm'):
            _num(stage.get(k),.02,.15,k)
        if type(stage.get('wheels_enabled')) is not bool or (i==len(stages)-1 and stage['wheels_enabled']):
            raise ValueError('Bánh phải tắt tại mốc gắp cuối')
        for k,lo,hi in [('forward_ms',50,400),('back_ms',50,400),('lateral_ms',50,400),
                ('settle_ms',100,3000),('arm_step_ms',20,200),
                ('max_forward_steps',0,60),('max_back_steps',0,10),('max_lateral_steps',0,30)]:
            if k == 'max_forward_steps' and stage.get(k) is None:
                continue  # unlimited forward count; vision/no-progress safeguards still apply
            _num(stage.get(k),lo,hi,k,True)
    samples=value.get('search_samples',[])
    if not isinstance(samples,list):raise ValueError('Sai search_samples')
    for sample in samples:
        if sample.get('p1')!=path['start']['p1'] or sample.get('p10')!=path['start']['p10']:
            raise ValueError('search_samples khác tư thế SEARCH')
        _num(sample.get('lens_cm'),1,200,'search_samples.lens_cm')
        if not isinstance(sample.get('bbox_px'),list) or len(sample['bbox_px'])!=4:
            raise ValueError('Sai search_samples.bbox_px')
        for n,high in zip(sample['bbox_px'],[1920,1080,1920,1080]):
            _num(n,0,high,'search_samples.bbox_px')
    return copy.deepcopy(value)


def reference_at(profile, p1):
    """Linear image reference between photos; timings use the outgoing span."""
    stages=profile['stages']
    if not stages[0]['p1']<=p1<=stages[-1]['p1']:
        raise ValueError('P1 ngoài mốc ảnh')
    a=b=stages[-1]
    for left,right in zip(stages,stages[1:]):
        if left['p1']<=p1<right['p1']:
            a,b=left,right
            break
    t=0 if a is b else (p1-a['p1'])/(b['p1']-a['p1'])
    result=copy.deepcopy(a)
    result['stage_index']=stages.index(a)
    result['p1']=p1
    for k in ('lens_cm','x_tolerance_norm','bottom_tolerance_norm'):
        result[k]=a[k]+t*(b[k]-a[k])
    result['bbox_norm']=[(x+t*(y-x))/size for x,y,size in zip(a['bbox_px'],b['bbox_px'],[1920,1080,1920,1080])]
    box=result['bbox_norm']
    result['target_x_norm']=(box[0]+box[2])/2
    result['target_bottom_norm']=box[3]
    return result


def view_error(obj, ref):
    x1,_,x2,y2=obj['bbox_norm']
    return ((x1+x2)/2-ref['target_x_norm'],y2-ref['target_bottom_norm'])


def view_problem(obj, profile, p1):
    x1,y1,x2,y2=obj['bbox_norm']
    margin=profile['side_margin_norm']
    if x1<=margin or x2>=1-margin:
        return 'Chai chạm mép ngang; không tự đoán tâm chai bị cắt'
    if y2>=profile['bottom_limit_norm']:
        return 'Đáy chai bị che/sát mép dưới; giữ tay và bánh'
    if y1<=.005 and p1<profile['top_clip_from_p1']:
        return 'Mất đầu chai trước vùng ảnh gần đã tham chiếu'
    if x2-x1<profile['min_visible_rw'] or y2-y1<profile['min_visible_rh']:
        return 'Vật quá nhỏ để hạ tay theo mốc ảnh'
    return None


def wheel_correction(obj, ref, positive_x_move):
    ex,ey=view_error(obj,ref)
    if abs(ex)>ref['x_tolerance_norm']:
        return positive_x_move if ex>0 else ('LEFT' if positive_x_move=='RIGHT' else 'RIGHT')
    if ey < -ref['bottom_tolerance_norm']:
        return 'FORWARD'
    if ey > ref['bottom_tolerance_norm']:
        return 'BACK'
    return None


def correction_error(obj, ref, move):
    ex,ey=view_error(obj,ref)
    return abs(ex) if move in ('LEFT','RIGHT') else abs(ey)


def search_lens_estimate(obj, samples, p1):
    """Empirical top-pixel lookup for the same reference bottle at SEARCH only.

    No extrapolation, no metric control, no use of clipped bounding-box height.
    This lookup does not identify a general-purpose camera matrix.
    """
    if p1!=1300 or len(samples)<2:
        return None
    x1,y1,x2,_=obj['bbox_norm']
    if not .4<(x1+x2)/2<.65 or not .02<x2-x1<.12:
        return None
    points=sorted((s['bbox_px'][1]/1080,s['lens_cm']) for s in samples)
    for (ya,da),(yb,db) in zip(points,points[1:]):
        if ya<=y1<=yb and yb>ya:
            return da+(y1-ya)*(db-da)/(yb-ya)
    return None
