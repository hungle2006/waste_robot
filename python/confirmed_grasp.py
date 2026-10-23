"""A confirmed close-view pose is not a floor-distance calibration."""
import copy
import math


def checked_reference(value, arm_path):
    if not value:
        return None
    if not isinstance(value, dict) or type(value.get('enabled')) is not bool:
        raise ValueError('Sai confirmed_grasp')
    if not value['enabled']:
        return copy.deepcopy(value)
    if not arm_path or value.get('pose_us') != arm_path['end']:
        raise ValueError('Ảnh 27 cm chỉ được xác nhận tại P1=2310/P10=650; đích quỹ đạo đã đổi')
    for key,low,high in [('target_x_norm',.1,.9),('x_tolerance_norm',.01,.15),
                         ('min_rw',.1,.7),('min_rh',.1,.8),('max_rw',.3,.95),
                         ('side_margin_norm',.005,.1),('measured_forward_cm',1,200)]:
        v=value.get(key)
        if type(v) not in (int,float) or not math.isfinite(v) or not low<=v<=high:
            raise ValueError('Sai confirmed_grasp.'+key)
    if value['min_rw']>=value['max_rw']:
        raise ValueError('Sai vùng rộng vật tại tư thế gắp')
    if type(value.get('stable_frames')) is not int or not 2<=value['stable_frames']<=5:
        raise ValueError('Cần ít nhất 2 ảnh xác nhận ở tư thế gắp')
    follow=value.get('base_follow',{'enabled':False})
    if not isinstance(follow,dict) or type(follow.get('enabled')) is not bool:
        raise ValueError('Sai confirmed_grasp.base_follow')
    if follow['enabled']:
        for key,lo,hi in [('pulse_ms',50,120),('max_steps',1,60),('max_no_progress',2,6)]:
            if type(follow.get(key)) is not int or not lo<=follow[key]<=hi:
                raise ValueError('Sai base_follow.'+key)
        if type(follow.get('max_p1_us')) is not int or not arm_path['start']['p1']<=follow['max_p1_us']<arm_path['end']['p1']:
            raise ValueError('Phải ngừng chạy bánh trước xung gắp cuối')
        if type(follow.get('trigger_bottom_norm')) not in (int,float) or not .3<=follow['trigger_bottom_norm']<=.5:
            raise ValueError('Sai base_follow.trigger_bottom_norm')
    return copy.deepcopy(value)


def matches_close_view(obj, joints, reference):
    if not reference or not reference.get('enabled'):
        return False
    if any(joints.get(j)!=v for j,v in reference['pose_us'].items()):
        return False
    x1,y1,x2,y2=obj['bbox_norm']
    rw,rh=x2-x1,y2-y1
    # A top-clipped bottle is expected in the user's confirmed image. Its
    # vertical box center is unreliable; never use it for a wheel command.
    return (x1>reference['side_margin_norm'] and x2<1-reference['side_margin_norm']
            and y2<1-reference['side_margin_norm']
            and reference['min_rw']<=rw<=reference['max_rw'] and rh>=reference['min_rh']
            and abs((x1+x2)/2-reference['target_x_norm'])<=reference['x_tolerance_norm'])
