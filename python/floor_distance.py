"""Measured floor coordinates relative to the robot's front reference line.

No assumed bottle size, camera height, servo angle or extrapolated old paper map.
Calibration needs measured fit points and independent check points at SEARCH.
"""
import copy
import math


DEFAULTS = {'enabled': False, 'stop_forward_cm': None, 'tolerance_cm': 3.,
            'slow_zone_cm': 10., 'max_error_cm': 2., 'calibration': None}


def finite(value, lo, hi, name):
    if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError('Sai ' + name)
    return float(value)


def project(matrix, point):
    x, y = point
    d = matrix[2][0]*x + matrix[2][1]*y + matrix[2][2]
    if not math.isfinite(d) or abs(d) < 1e-8:
        raise ValueError('Điểm gần đường chân trời; không có khoảng cách hợp lệ')
    out = [(row[0]*x + row[1]*y + row[2])/d for row in matrix[:2]]
    if not all(math.isfinite(v) for v in out): raise ValueError('Khoảng cách không hợp lệ')
    return out


def fit_calibration(points, frame_size, search_pose, camera_index, max_error_cm):
    import cv2
    import numpy as np
    if not isinstance(points, list) or not 6 <= len(points) <= 30:
        raise ValueError('Cần ít nhất 4 điểm FIT và 2 điểm CHECK đo độc lập')
    w, h = frame_size
    finite(w, 16, 3840, 'rộng ảnh'); finite(h, 16, 2160, 'cao ảnh')
    normalized = []
    for p in points:
        if not isinstance(p, dict) or p.get('kind') not in ('fit', 'check'):
            raise ValueError('Sai loại điểm FIT/CHECK')
        pixel, floor = p.get('pixel'), p.get('floor_cm')
        if not isinstance(pixel, list) or not isinstance(floor, list) or len(pixel) != 2 or len(floor) != 2:
            raise ValueError('Mỗi điểm cần pixel [x,y] và floor_cm [X,Y]')
        x = finite(pixel[0], 0, w-1, 'pixel x')/w
        y = finite(pixel[1], 0, h-1, 'pixel y')/h
        X = finite(floor[0], -300, 300, 'X cm'); Y = finite(floor[1], 1, 500, 'Y cm')
        normalized.append({'image_norm': [x, y], 'floor_cm': [X, Y], 'kind': p['kind']})
    fit = [p for p in normalized if p['kind'] == 'fit']
    check = [p for p in normalized if p['kind'] == 'check']
    if len(fit) < 4 or len(check) < 2: raise ValueError('Thiếu 4 FIT / 2 CHECK')
    src = np.array([p['image_norm'] for p in fit], dtype=np.float64)
    dst = np.array([p['floor_cm'] for p in fit], dtype=np.float64)
    if any(math.dist(a, b) < .005 for i, a in enumerate(src) for b in src[i+1:]):
        raise ValueError('Các điểm FIT trùng hoặc quá sát nhau')
    for p in check:
        if any(math.dist(p['image_norm'], q['image_norm']) < .005 for q in fit):
            raise ValueError('CHECK phải là điểm đo khác FIT')
    hull = cv2.convexHull(src.astype(np.float32)).reshape(-1, 2)
    if abs(cv2.contourArea(hull)) < .005 or abs(cv2.contourArea(cv2.convexHull(dst.astype(np.float32)))) < 25:
        raise ValueError('Vùng đo quá nhỏ hoặc các điểm gần thẳng hàng')
    H, _ = cv2.findHomography(src, dst, method=0)
    if H is None or not np.isfinite(H).all() or np.linalg.matrix_rank(H) != 3:
        raise ValueError('Không lập được bản đồ mặt sàn')
    H = H / H[2, 2]
    errors = []
    for p in normalized:
        if cv2.pointPolygonTest(hull, tuple(p['image_norm']), False) < 0:
            raise ValueError('Điểm CHECK phải nằm trong vùng FIT')
        errors.append(math.dist(project(H.tolist(), p['image_norm']), p['floor_cm']))
    if max(errors) > max_error_cm:
        raise ValueError(f'Sai số kiểm chứng {max(errors):.2f} cm vượt {max_error_cm:g} cm; đo lại')
    # Reject a horizon/singularity inside the region used for motion.
    denominators = [H[2, 0]*x + H[2, 1]*y + H[2, 2] for x, y in hull]
    if min(denominators)*max(denominators) <= 0: raise ValueError('Vùng FIT cắt đường chân trời')
    return {'version': 1, 'origin': 'robot_front_floor', 'matrix_norm_to_cm': H.tolist(),
            'valid_polygon_norm': hull.tolist(), 'frame_size': list(frame_size),
            'search_pose': copy.deepcopy(search_pose), 'camera_index': camera_index,
            'points': normalized, 'max_observed_error_cm': max(errors)}


def distance_config(values, robot_config, camera_index=None):
    if not isinstance(values, dict) or set(values)-set(DEFAULTS): raise ValueError('Sai cấu hình distance')
    c = dict(DEFAULTS, **copy.deepcopy(values))
    if type(c['enabled']) is not bool: raise ValueError('distance.enabled phải là boolean')
    for key, lo, hi in [('tolerance_cm', .5, 10), ('slow_zone_cm', 1, 40), ('max_error_cm', .1, 5)]:
        c[key] = finite(c[key], lo, hi, key)
    if not c['enabled']: return c
    c['stop_forward_cm'] = finite(c['stop_forward_cm'], 3, 200, 'stop_forward_cm: cần đo tầm gắp thực tế')
    if c['stop_forward_cm'] <= c['tolerance_cm']: raise ValueError('Khoảng dừng phải lớn hơn dung sai')
    if c['tolerance_cm'] <= c['max_error_cm']: raise ValueError('Dung sai dừng phải lớn hơn sai số hiệu chuẩn')
    cal = c['calibration']
    if isinstance(cal, dict) and cal.get('version') == 2:
        from paper_distance import rebuild_reference
        c['calibration'] = rebuild_reference(cal, robot_config, camera_index)
        return c
    if not isinstance(cal, dict) or cal.get('version') != 1 or cal.get('origin') != 'robot_front_floor':
        raise ValueError('Chưa hiệu chuẩn khoảng cách cm: mở trang hiệu chuẩn, đo 4 FIT + 2 CHECK')
    expected_pose = {j: robot_config['poses']['SEARCH'][j] for j in ('p1', 'p10', 'p11', 'd2')}
    if cal.get('search_pose') != expected_pose: raise ValueError('SEARCH đã đổi; hiệu chuẩn lại khoảng cách')
    if camera_index is not None and cal.get('camera_index') != camera_index:
        raise ValueError('Camera index đã đổi; hiệu chuẩn lại khoảng cách')
    # Refit from retained measurements rather than trusting an edited matrix/error flag.
    w, h = cal['frame_size']
    points = [{'kind': p['kind'], 'pixel': [p['image_norm'][0]*w, p['image_norm'][1]*h], 'floor_cm': p['floor_cm']}
              for p in cal['points']]
    rebuilt = fit_calibration(points, [w, h], expected_pose, cal['camera_index'], c['max_error_cm'])
    ys = [p['floor_cm'][1] for p in rebuilt['points'] if p['kind'] == 'fit']
    if not min(ys) <= c['stop_forward_cm'] <= max(ys):
        raise ValueError('Khoảng dừng phải nằm trong vùng khoảng cách đã đo FIT')
    c['calibration'] = rebuilt
    return c


def measure_floor(obj, config, joints, frame_size):
    import cv2
    import numpy as np
    cal = config['calibration']
    if any(joints.get(j) != value for j, value in cal['search_pose'].items()):
        raise ValueError('Khoảng cách chỉ hợp lệ khi tay đứng đúng SEARCH')
    w, h = frame_size; cw, ch = cal['frame_size']
    if abs((w/h)/(cw/ch)-1) > .01:
        raise ValueError('Tỷ lệ ảnh đã đổi; không áp dụng hiệu chuẩn cm')
    x1, y1, x2, y2 = obj['bbox_norm']
    point = ((x1+x2)/2, y2)  # Must be the visible floor-contact point.
    hull = np.array(cal['valid_polygon_norm'], dtype=np.float32)
    # The bbox lower edge has jitter; all three nearby samples must be measured,
    # inside the valid region, and on the near side before a handoff is allowed.
    points = [(point[0], point[1]+offset) for offset in (-2/h, 0, 2/h)]
    if any(cv2.pointPolygonTest(hull, p, False) < 0 for p in points):
        if cal.get('version') == 2:
            raise ValueError('Chân chai ngoài vùng giấy–mốc gắp đã ghi; giữ bánh, không ngoại suy cm')
        raise ValueError('Chân chai ngoài vùng FIT; dừng bánh, bổ sung vùng đo, không ngoại suy cm')
    values = [project(cal['matrix_norm_to_cm'], p) for p in points]
    X, Y = values[1]
    uncertainty = config['max_error_cm'] + max(abs(q[1]-Y) for q in values)
    upper, lower = Y+uncertainty, Y-uncertainty
    goal = config['stop_forward_cm']
    preliminary = cal.get('version') == 2
    return {'forward_cm': Y, 'lateral_cm': X, 'floor_range_cm': math.hypot(X, Y),
            'quality': 'preliminary' if preliminary else 'checked',
            'origin': cal.get('origin', 'robot_front_floor'),
            'uncertainty_cm': uncertainty, 'close': upper <= goal+config['tolerance_cm'],
            'too_close': lower < goal-config['tolerance_cm'], 'slow': lower <= goal+config['slow_zone_cm']}
