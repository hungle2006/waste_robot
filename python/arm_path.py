"""Interpolate measured PWM commands, never infer mechanical joint angles."""
import copy


def validate_path(values, robot, final):
    if not values:
        return None
    if not isinstance(values, dict) or not {'start', 'end'} <= set(values) or set(values)-{'start','end','knots'}:
        raise ValueError('arm_path cần start/end')
    start, end = values['start'], values['end']
    joints = {'p1', 'p10', 'p11', 'd2'}
    knots=values.get('knots',[start,end])
    if not isinstance(knots,list) or len(knots)<2 or knots[0]!=start or knots[-1]!=end:
        raise ValueError('arm_path.knots phải bắt đầu/end đúng start/end')
    for pose in knots:
        if not isinstance(pose, dict) or set(pose) != joints:
            raise ValueError('Mỗi mốc arm_path cần P1, P10, P11, D2')
        for j, value in pose.items():
            if type(value) is not int or not robot['limits'][j][0] <= value <= robot['limits'][j][1]:
                raise ValueError('arm_path ngoài giới hạn ' + j)
    if start != {j: robot['poses']['SEARCH'][j] for j in joints}:
        raise ValueError('arm_path.start phải khớp SEARCH trong robot_params.py')
    if end['p1'] != final or end['p1'] <= start['p1']:
        raise ValueError('Đích P1 phải trùng grasp_final_s2_us và lớn hơn SEARCH')
    if any(p[j] != start[j] for p in knots for j in ('p11', 'd2')):
        raise ValueError('Quỹ đạo này chỉ thay đổi P1/P10; giữ trụ P11 và nối D2')
    if any(b['p1']<=a['p1'] for a,b in zip(knots,knots[1:])):
        raise ValueError('arm_path.knots phải tăng P1, không trùng mốc')
    return copy.deepcopy(values)


def targets_at(path, p1):
    start, end = path['start'], path['end']
    if type(p1) is not int or not start['p1'] <= p1 <= end['p1']:
        raise ValueError('P1 nằm ngoài quỹ đạo đã khai báo')
    for a,b in zip(path.get('knots',[start,end]),path.get('knots',[start,end])[1:]):
        if a['p1']<=p1<=b['p1']:
            start,end=a,b
            break
    t = (p1-start['p1'])/(end['p1']-start['p1'])
    return {j: round(start[j] + t*(end[j]-start[j])) for j in start}


def pulse_steps(start, end, step):
    direction = 1 if end > start else -1
    return list(range(start + direction*step, end, direction*step)) + ([end] if end != start else [])
