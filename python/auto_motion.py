"""Conservative motion/visibility settings; no physical distance inference."""
import math

MOTION_DEFAULTS={'enabled':False,'ramp_ms':160,'arm_step_us':8,'arm_step_ms':50}
VISIBILITY_DEFAULTS={'enabled':False,'trigger_bottom_norm':.54,'blur_bottom_norm':.60,
                     'min_bottom_norm':.15,'min_top_norm':.03,'near_forward_ms':60,
                     'near_margin_norm':.06,'min_step_us':5,'prediction_margin_norm':.01,
                     'response_noise_norm':.015,'min_response_norm':.002,'max_no_response':4}


def checked_settings(values, defaults):
    if not isinstance(values,dict) or set(values)-set(defaults):raise ValueError('Sai cấu hình chuyển động/giữ ảnh')
    c=dict(defaults,**values)
    if type(c['enabled']) is not bool:raise ValueError('enabled phải là boolean')
    ranges={'ramp_ms':(40,400),'arm_step_us':(2,100),'arm_step_ms':(20,200),
            'near_forward_ms':(50,200),'min_step_us':(5,100),'max_no_response':(2,10)}
    normalized={'trigger_bottom_norm':(.3,.8),'blur_bottom_norm':(.4,.95),'min_bottom_norm':(.05,.3),
                'min_top_norm':(.01,.15),'near_margin_norm':(.02,.2),'prediction_margin_norm':(.005,.05),
                'response_noise_norm':(.005,.05),'min_response_norm':(.001,.02)}
    for k,v in c.items():
        if k in ranges:
            if type(v) is not int or not ranges[k][0]<=v<=ranges[k][1]:raise ValueError('Sai '+k)
        if k in normalized:
            if type(v) not in (int,float) or not math.isfinite(v) or not normalized[k][0]<=v<=normalized[k][1]:raise ValueError('Sai '+k)
    if 'blur_bottom_norm' in c:
        if c['trigger_bottom_norm']+.02>c['blur_bottom_norm']:raise ValueError('Phải hạ tay trước vùng càng che ít nhất 2% ảnh')
        if c['min_bottom_norm']>=c['trigger_bottom_norm']:raise ValueError('Sai giới hạn giữ vật trong ảnh')
    return c


def visible_step(obj,current,final,previous,motion,visibility):
    """Limit the next S2 step from observed image response, not presumed arm angles."""
    top,bottom=obj['bbox_norm'][1],obj['bbox_norm'][3]
    if top<=visibility['min_top_norm'] or bottom<=visibility['min_bottom_norm']:
        raise ValueError('Vật gần ra mép trên; không hạ tiếp')
    if bottom>visibility['blur_bottom_norm']:
        raise ValueError('Vật đã vào vùng càng che; không hạ mù')
    step=min(motion['arm_step_us'],final-current)
    if step<=0:return 0
    # Slow near the view boundary and before the obscured gripper region.
    if min(top-visibility['min_top_norm'],visibility['blur_bottom_norm']-bottom)<.03:
        step=min(step,visibility['min_step_us'])
    if previous and current>previous[0]:
        slope=(bottom-previous[1]['bbox_norm'][3])/(current-previous[0])
        if slope<0:
            while step>0 and (top+slope*step<=visibility['min_top_norm']+visibility['prediction_margin_norm'] or
                             bottom+slope*step<=visibility['min_bottom_norm']+visibility['prediction_margin_norm']):
                step-=1
    if step<min(visibility['min_step_us'],final-current):
        raise ValueError('Bước hạ dự đoán quá nhỏ cho PCA9685 hoặc làm vật ra khỏi ảnh; giữ tại chỗ')
    return step
