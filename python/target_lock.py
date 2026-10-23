"""Conservative single-target association across an unordered list of detections.

No server ID or label is trusted as physical identity. Ambiguous matches wait.
An HSV descriptor helps reject lookalikes; it is not a learned instance ID.
"""
import math

DEFAULTS = {'enabled': False, 'continuous_auto': False, 'min_iou': .15, 'max_shift_norm': .22,
            'min_area_ratio': .35, 'max_area_ratio': 3., 'ambiguity_margin': .12,
            'appearance_similarity': .70, 'confirm_frames': 2, 'classify_frames': 3,
            'retry_ms': 500, 'reference_max_age_s': 30, 'overlay_max_age_s': 2.,
            'geometry_overrides_color': False, 'restart_initial_candidate': False,
            'compensate_arm_pose': False}


def lock_config(values):
    if not isinstance(values, dict) or set(values)-set(DEFAULTS):
        raise ValueError('Sai cấu hình target_lock')
    c = dict(DEFAULTS, **values)
    if type(c['enabled']) is not bool or type(c['continuous_auto']) is not bool: raise ValueError('target_lock.enabled/continuous_auto phải boolean')
    for name in ('geometry_overrides_color', 'restart_initial_candidate', 'compensate_arm_pose'):
        if type(c[name]) is not bool: raise ValueError('Sai target_lock.'+name)
    ranges = {'min_iou':(.05,.9), 'max_shift_norm':(.02,.4),
              'min_area_ratio':(.1,.9), 'max_area_ratio':(1.1,5),
              'ambiguity_margin':(.05,.5), 'appearance_similarity':(.5,.99),
              'reference_max_age_s':(5,120), 'overlay_max_age_s':(.2,5)}
    for key,(lo,hi) in ranges.items():
        v = c[key]
        if type(v) not in (int,float) or not math.isfinite(v) or not lo<=v<=hi:
            raise ValueError('Sai target_lock.'+key)
    for key,lo,hi in [('confirm_frames',2,5),('classify_frames',1,7),('retry_ms',100,5000)]:
        if type(c[key]) is not int or not lo<=c[key]<=hi: raise ValueError('Sai target_lock.'+key)
    return c


def iou(a,b):
    inter=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/union if union>0 else 0.


def appearance_similarity(a,b):
    if not a or not b or len(a)!=len(b): return None
    return sum(math.sqrt(max(0,x)*max(0,y)) for x,y in zip(a,b))


def associate(reference, candidates, cfg):
    """Return (index, reason); no guess when two candidates score too closely."""
    ranked=[]
    rejected=[]
    for index,obj in enumerate(candidates):
        a,b=reference['bbox_norm'],obj['bbox_norm']
        ratio=((b[2]-b[0])*(b[3]-b[1]))/((a[2]-a[0])*(a[3]-a[1]))
        shift=math.hypot(*[x-y for x,y in zip(reference['center_norm'],obj['center_norm'])])
        overlap=iou(a,b)
        similarity=appearance_similarity(reference.get('_appearance'),obj.get('_appearance'))
        reasons=[]
        if not cfg['min_area_ratio']<=ratio<=cfg['max_area_ratio']:reasons.append('diện tích')
        if shift>cfg['max_shift_norm']:reasons.append('dịch tâm')
        # A transparent bottle/can crop changes colour with the background.
        # Close geometric continuity can outweigh colour; ambiguity still waits.
        geometric = (cfg.get('geometry_overrides_color', False) and overlap >= .30
                     and shift <= .12 and .5 <= ratio <= 2.)
        if similarity is not None and similarity<cfg['appearance_similarity'] and not geometric:reasons.append('màu')
        # Without appearance, require actual overlap; never jump to an isolated box.
        if overlap<cfg['min_iou'] and (similarity is None or similarity<.9):reasons.append('IoU')
        if reasons:
            color='NA' if similarity is None else f'{similarity:.2f}'
            rejected.append(f'box {index}: {"/".join(reasons)}; IoU={overlap:.2f}, dịch={shift:.3f}, diện tích={ratio:.2f}, màu={color}')
            continue
        score=.65*overlap+.20*(1-shift/cfg['max_shift_norm'])+.15*(similarity or 0)
        ranked.append((score,index))
    ranked.sort(reverse=True)
    if not ranked:return None,'Không có box khớp vật đã khóa · '+(' | '.join(rejected[:3]) if rejected else 'Modal chưa trả box')
    if len(ranked)>1 and ranked[0][0]-ranked[1][0]<cfg['ambiguity_margin']:
        return None,'Hai box tương tự nhau; chưa đủ chắc để chọn'
    return ranked[0][1],'Đã khớp box'


def associate_rolling(reference, candidates, cfg, flow):
    """Match the last measured box, allowing growth/partial views, not a seed size.

    Ambiguous identical objects still wait. Appearance alone or class alone
    cannot switch a track to a disjoint object. No hidden-part inference.
    """
    ranked = []
    a = reference['bbox_norm']
    old_area = (a[2]-a[0])*(a[3]-a[1])
    for index, obj in enumerate(candidates):
        b = obj['bbox_norm']
        ratio = (b[2]-b[0])*(b[3]-b[1])/old_area
        shift = math.hypot(*[x-y for x,y in zip(reference['center_norm'],obj['center_norm'])])
        overlap = iou(a, b)
        similarity = appearance_similarity(reference.get('_appearance'), obj.get('_appearance'))
        if (not flow['track_min_area_ratio'] <= ratio <= flow['track_max_area_ratio']
                or shift > flow['track_max_shift_norm'] or overlap < flow['track_min_iou']):
            continue
        # Strong overlap can survive a transparent crop's background change.
        geometric = overlap >= .30 and shift <= .18
        if similarity is not None and similarity < cfg['appearance_similarity'] and not geometric:
            continue
        score = .65*overlap + .20*(1-shift/flow['track_max_shift_norm']) + .15*(similarity or 0)
        ranked.append((score,index))
    ranked.sort(reverse=True)
    if not ranked:
        return None, 'Không khớp box gần nhất của vật khóa; không cập nhật sang vật khác'
    if len(ranked)>1 and ranked[0][0]-ranked[1][0] < cfg['ambiguity_margin']:
        return None, 'Nhiều box giống nhau; chưa xác định đúng vật đã khóa'
    return ranked[0][1], 'Khớp vật khóa theo box gần nhất, cho phép thay đổi kích thước'


def pose_prediction(reference, before, after):
    """Predict image motion from arm-photo geometry, not object size in cm.

    Used only for a completed <=50 us arm step; a reference-photo box never
    becomes the tracked object's box. Identity must still pass association.
    """
    import copy
    old, new = before['bbox_norm'], after['bbox_norm']
    actual = reference['bbox_norm']
    sx = (new[2]-new[0])/(old[2]-old[0])
    sy = (new[3]-new[1])/(old[3]-old[1])
    cx = (actual[0]+actual[2])/2 + (new[0]+new[2]-old[0]-old[2])/2
    bottom = actual[3]+new[3]-old[3]
    bw, bh = (actual[2]-actual[0])*sx, (actual[3]-actual[1])*sy
    box = [max(0., cx-bw/2), max(0., bottom-bh), min(1., cx+bw/2), min(1., bottom)]
    if box[2] <= box[0] or box[3] <= box[1]:
        return reference
    result = copy.deepcopy(reference)
    result.update(bbox_norm=box, center_norm=[(box[0]+box[2])/2, (box[1]+box[3])/2])
    return result


def attach_appearance(jpeg, objects, frame_size):
    """Describe crops of the exact JPEG sent to Modal; never annotate model input."""
    try:
        import cv2
        import numpy as np
        frame=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        if frame is None or [frame.shape[1],frame.shape[0]]!=frame_size:return
        for obj in objects:
            box=obj.get('bbox')
            if not isinstance(box,list) or len(box)!=4:continue
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in box):continue
            x1,y1,x2,y2=box;w,h=frame_size
            if not (0<=x1<x2<=w and 0<=y1<y2<=h):continue
            crop=frame[int(y1):int(y2),int(x1):int(x2)]
            if not crop.size:continue
            hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
            hist=cv2.calcHist([hsv],[0,1,2],None,[8,4,4],[0,180,0,256,0,256]).flatten()
            total=float(hist.sum())
            if total:obj['_appearance']=(hist/total).tolist()
    except ImportError:
        return  # Conservative geometry association still works in headless tests.
