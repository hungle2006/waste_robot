"""Recovered paper homography, with an explicitly measured grasp reference.

One known ground point anchors translation only; planar robot axes are taken
from local image derivatives. This remains a preliminary estimate, not an
independent validation or a camera intrinsic calibration.
"""
import copy
import math
from floor_distance import project


def floor_point(obj):
    x1, _, x2, y2 = obj['bbox_norm']
    return [(x1+x2)/2, y2]


def paper_position(point_norm, parameters, frame_size):
    w, h = frame_size
    cw, ch = parameters['PAPER_FRAME_SIZE']
    if abs((w/h)/(cw/ch)-1) > .01:
        raise ValueError('Ảnh USB khác tỷ lệ 16:9 của ảnh giấy; không tự suy đoán crop')
    x, y = point_norm
    if not all(type(v) in (int,float) and math.isfinite(v) and 0 <= v <= 1 for v in (x,y)):
        raise ValueError('Điểm ảnh không hợp lệ')
    return project(parameters['PAPER_H_PIXEL_TO_CM'], [x*cw, y*ch])


def create_reference(parameters, point_norm, frame_size):
    import cv2
    import numpy as np
    anchor = paper_position(point_norm, parameters, frame_size)
    cw, ch = parameters['PAPER_FRAME_SIZE']
    H = np.asarray(parameters['PAPER_H_PIXEL_TO_CM'], dtype=float)
    normalized = H @ np.diag([cw, ch, 1.])
    x, y = point_norm
    dx = np.array(paper_position([x+.001,y],parameters,frame_size))-anchor
    right = dx / np.linalg.norm(dx)
    forward = np.array([right[1], -right[0]])
    up = np.array(paper_position([x,y-.001],parameters,frame_size))-anchor
    if forward@up <= 0:
        raise ValueError('Hướng ma trận giấy không khớp camera; không tạo mốc')
    rotation = np.array([right, forward])
    transform = np.eye(3)
    transform[:2,:2] = rotation
    transform[:2,2] = np.array([0.,parameters['GRASP_FORWARD_CM']])-rotation@anchor
    matrix = transform@normalized
    matrix /= matrix[2,2]
    corners = np.array(parameters['PAPER_CORNERS_PX'],dtype=float)/[cw,ch]
    # Only interpolate within the paper-to-reference region. No whole-image extrapolation.
    around = np.array([[x-.025,y-.025],[x+.025,y-.025],[x+.025,y+.025],[x-.025,y+.025]])
    hull = cv2.convexHull(np.vstack([corners,around]).astype(np.float32)).reshape(-1,2)
    denominators = [matrix[2,0]*a+matrix[2,1]*b+matrix[2,2] for a,b in hull]
    if min(denominators)*max(denominators) <= 0:
        raise ValueError('Vùng từ giấy tới điểm gắp cắt đường chân trời')
    robot=parameters['ROBOT_CONFIG']
    return {'version':2, 'origin':'s2_floor_projection', 'quality':'preliminary_reference',
            'anchor_norm':list(point_norm), 'anchor_forward_cm':parameters['GRASP_FORWARD_CM'],
            'paper_frame_size':list(parameters['PAPER_FRAME_SIZE']),
            'paper_corners_px':copy.deepcopy(parameters['PAPER_CORNERS_PX']),
            'paper_matrix':H.tolist(), 'matrix_norm_to_cm':matrix.tolist(),
            'valid_polygon_norm':hull.tolist(), 'frame_size':list(frame_size),
            'search_pose':{j:robot['poses']['SEARCH'][j] for j in ('p1','p10','p11','d2')},
            'camera_index':parameters['CAMERA_INDEX'],
            'point_mode':'bottom_center', 'independent_validation':False,
            'note':'27 cm do bằng thước; pixel USB ghi tự động. Gốc S2 và hướng trục là xấp xỉ từ mốc tâm chai; ma trận giấy vẽ tay chưa kiểm định.'}


def rebuild_reference(reference, robot, camera_index):
    expected={j:robot['poses']['SEARCH'][j] for j in ('p1','p10','p11','d2')}
    if reference.get('origin') != 's2_floor_projection' or reference.get('search_pose') != expected:
        raise ValueError('Mốc 27 cm không khớp SEARCH; ghi lại bằng record_grasp_reference.py')
    if camera_index is not None and reference.get('camera_index') != camera_index:
        raise ValueError('Camera index đã đổi; mốc gắp không còn khớp')
    parameters = {'PAPER_FRAME_SIZE':reference['paper_frame_size'],
                  'PAPER_CORNERS_PX':reference['paper_corners_px'],
                  'PAPER_H_PIXEL_TO_CM':reference['paper_matrix'],
                  'GRASP_FORWARD_CM':reference['anchor_forward_cm'],
                  'ROBOT_CONFIG':robot, 'CAMERA_INDEX':reference['camera_index']}
    return create_reference(parameters,reference['anchor_norm'],reference['frame_size'])
