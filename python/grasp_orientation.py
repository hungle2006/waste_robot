"""Unoriented object axis from actual foreground pixels, never an AABB diagonal."""
import math
from collections import deque


def axis_angle(dx, dy):
    if not all(math.isfinite(v) for v in (dx, dy)) or math.hypot(dx, dy) < 1e-8:
        raise ValueError('Trục vật không hợp lệ')
    # Image y points down. Canonical line angle: [-90, 90), + = lean right.
    return (math.degrees(math.atan2(dx, -dy)) + 90) % 180 - 90


def pulse_for_angle(angle):
    if type(angle) not in (int, float) or not math.isfinite(angle):
        raise ValueError('Góc không hợp lệ')
    angle = (angle + 90) % 180 - 90
    # At a positive observed angle above +60°, command the reachable +60°
    # wrist endpoint instead of entering AUTO HOLD. Keep negative calibration.
    angle = min(angle, 60.0)
    return round(2100 + angle * (1600 / 90 if angle < 0 else 400 / 60))


def auto_pulse_for_angle(angle, deadband_deg=10.0):
    """AUTO only: hold the wrist centered for |mapped camera angle| < deadband.

    At exactly +/-deadband_deg, send the normal calibrated pulse. The manual
    WRIST LEFT/RIGHT commands deliberately do not use this deadband.
    """
    if (type(angle) not in (int, float) or not math.isfinite(angle)
            or type(deadband_deg) not in (int, float) or not math.isfinite(deadband_deg)
            or not 0 <= deadband_deg <= 30):
        raise ValueError('Góc/vùng chết AUTO D2 không hợp lệ')
    if abs(angle) < deadband_deg:
        return 2100
    return pulse_for_angle(angle)


def angle_for_pulse(pulse):
    """Commanded wrist angle from the user's three calibration points, not an encoder."""
    if type(pulse) not in (int, float) or not math.isfinite(pulse) or not 500 <= pulse <= 2500:
        raise ValueError('Chưa có xung D2 hợp lệ trong 500–2500 µs')
    return (pulse - 2100) * (90 / 1600 if pulse < 2100 else 60 / 400)


def mean_axis(angles, max_spread=8.):
    if not angles:
        raise ValueError('Chưa đủ góc')
    x=sum(math.cos(math.radians(2*a)) for a in angles)
    y=sum(math.sin(math.radians(2*a)) for a in angles)
    if math.hypot(x,y)/len(angles) < .95:
        raise ValueError('Góc vật chưa ổn định')
    avg=(math.degrees(math.atan2(y,x))/2+90)%180-90
    if any(abs((a-avg+90)%180-90)>max_spread for a in angles):
        raise ValueError('Góc vật dao động quá lớn')
    return avg



def top_bottom_foreground_axis(xs, ys):
    """Use the visible object's TOP/BOTTOM within its detection box.

    A plain axis-aligned bounding rectangle has identical top/bottom X centers
    regardless of tilt. We must find the center of actual bottle pixels in a
    thin upper/lower band of the segmented foreground, not the box midpoints.
    """
    import numpy as np
    if len(xs) < 80 or len(xs) != len(ys):
        raise ValueError('Thiếu pixel chai để tìm điểm trên/dưới')
    ylo, yhi = np.percentile(ys, [3., 97.])
    if yhi-ylo < 12:
        raise ValueError('Chai quá ngang hoặc quá thấp để đo bằng điểm trên/dưới')
    band = max(3., .14*(yhi-ylo))
    up = (ys >= ylo) & (ys <= ylo+band)
    down = (ys >= yhi-band) & (ys <= yhi)
    if np.count_nonzero(up)<12 or np.count_nonzero(down)<12:
        raise ValueError('Không đủ thân chai tại đầu trên/dưới box')
    top = [float(np.median(xs[up])),float(np.median(ys[up]))]
    bottom = [float(np.median(xs[down])),float(np.median(ys[down]))]
    angle = axis_angle(top[0]-bottom[0], top[1]-bottom[1])
    return angle, top, bottom


def estimate_axis(jpeg, bbox):
    import cv2
    import numpy as np
    image=cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None: raise ValueError('Không giải mã được ảnh hướng vật')
    h,w=image.shape[:2]
    if not isinstance(bbox,(list,tuple)) or len(bbox)!=4 or not all(type(x) in (int,float) and math.isfinite(x) for x in bbox):
        raise ValueError('Box không hợp lệ')
    x1,y1,x2,y2=map(int,bbox)
    if not (2<x1<x2<w-2 and 2<y1<y2<h-2) or min(x2-x1,y2-y1)<12:
        raise ValueError('Vật quá nhỏ hoặc bị cắt khỏi ảnh; chưa biết hướng')
    margin=max(4,round(.1*max(x2-x1,y2-y1)))
    left,top=max(0,x1-margin),max(0,y1-margin)
    roi=image[top:min(h,y2+margin),left:min(w,x2+margin)]
    # Bound CPU cost and preserve the aspect ratio.
    scale=min(1.,400/max(roi.shape[:2]))
    roi=cv2.resize(roi,None,fx=scale,fy=scale)
    rect=tuple(round(v*scale) for v in (x1-left,y1-top,x2-x1,y2-y1))
    mask=np.zeros(roi.shape[:2],np.uint8)
    bg=np.zeros((1,65),np.float64);fg=bg.copy()
    cv2.grabCut(roi,mask,rect,bg,fg,3,cv2.GC_INIT_WITH_RECT)
    mask=np.uint8((mask==cv2.GC_FGD)|(mask==cv2.GC_PR_FGD))
    n,labels,stats,_=cv2.connectedComponentsWithStats(mask)
    if n<2: raise ValueError('Chưa tách được vật khỏi nền')
    areas=stats[1:,cv2.CC_STAT_AREA];order=np.argsort(areas)[::-1]
    largest=int(areas[order[0]])
    if largest<80 or largest<.15*rect[2]*rect[3]: raise ValueError('Vùng vật quá ít')
    if len(order)>1 and areas[order[1]]>.25*largest: raise ValueError('Có nhiều vùng vật trong box')
    ys,xs=np.where(labels==int(order[0])+1)
    points=np.column_stack((xs,ys)).astype(np.float64)
    mean,vec,values=cv2.PCACompute2(points,mean=None)
    ratio=float(values[0,0]/max(values[1,0],1e-9))
    # Operator-requested threshold: strictly PCA > 1.0 is accepted.
    # PCA eigenvalues are ordered, so the ratio is mathematically >= 1.0;
    # a perfectly round/symmetric object with ratio == 1.0 has no unique axis.
    # This low threshold still requires foreground segmentation and 3-frame
    # angular agreement before D2 is commanded.
    if ratio <= 1.0: raise ValueError(f'Vật gần tròn hoặc hướng không rõ (PCA={ratio:.4f} <= 1.0000)')
    dx,dy=map(float,vec[0]);pca_angle=axis_angle(dx,dy)
    center=[float(mean[0,0]/scale+left),float(mean[0,1]/scale+top)]
    length=math.sqrt(float(values[0,0]))/scale
    axis_points=[[center[0]-dx*length,center[1]-dy*length],
                 [center[0]+dx*length,center[1]+dy*length]]
    method='pca_horizontal_fallback'
    angle=pca_angle
    # With a near-horizontal bottle, top/bottom pixel bands show thickness,
    # not its long direction; keep the valid PCA axis instead of guessing 0°.
    if abs(pca_angle) <= 55:
        top_bottom_angle, upper, lower = top_bottom_foreground_axis(xs,ys)
        angular_difference = abs((top_bottom_angle-pca_angle+90)%180-90)
        if angular_difference > 15:
            raise ValueError('Điểm trên/dưới box không trùng trục dài chai; không xoay mò')
        angle=top_bottom_angle
        method='top_bottom_foreground'
        axis_points=[[upper[0]/scale+left, upper[1]/scale+top],
                     [lower[0]/scale+left, lower[1]/scale+top]]
    else:
        # Order PCA endpoints consistently from top to bottom in image space.
        axis_points.sort(key=lambda point:point[1])
    return {'angle_deg':angle,'elongation':ratio,'center':center,
            'axis_points':axis_points, 'method':method}


class OrientationWindow:
    def __init__(self):
        self.values=deque(maxlen=3)
        self.key=None

    def add(self, result, key):
        if key!=self.key: self.values.clear();self.key=key
        self.values.append(result['angle_deg'])
        if len(self.values)<3: raise ValueError('Đang lấy trung bình 3 ảnh hướng vật')
        return dict(result,angle_deg=mean_axis(self.values),samples=len(self.values))


def checked_config(value, path):
    if value is None:return {'enabled':False,'calibrated':False}
    if not isinstance(value,dict) or set(value)-{'enabled','calibrated','camera_sign','offset_deg','deadband_deg','calibration_pose'}:
        raise ValueError('Sai grasp_orientation')
    if type(value.get('enabled')) is not bool or type(value.get('calibrated')) is not bool:
        raise ValueError('enabled/calibrated góc phải boolean')
    if not value['enabled']:return dict(value)
    if not path:raise ValueError('Xoay kẹp cần quỹ đạo gắp đã đo')
    if type(value.get('camera_sign')) is not int or value['camera_sign'] not in (-1,1):
        raise ValueError('camera_sign chỉ là -1 hoặc +1')
    offset=value.get('offset_deg')
    if type(offset) not in (int,float) or not math.isfinite(offset) or not -180<=offset<=180:
        raise ValueError('Sai góc lệch camera → càng')
    deadband=value.get('deadband_deg',10.0)
    if (type(deadband) not in (int,float) or not math.isfinite(deadband)
            or not 0 <= deadband <= 30):
        raise ValueError('Vùng chết góc D2 phải nằm trong 0–30 độ')
    if value.get('calibration_pose')!=path['end']:
        raise ValueError('Góc camera cần hiệu chuẩn tại đúng tư thế cuối')
    return dict(value)
