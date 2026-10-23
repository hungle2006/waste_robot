#!/usr/bin/env python3
"""Convert image pixels to preliminary ground coordinates in cm.

Works only for points on the same flat floor as the 17x17cm square.
Usage:
  python waste_robot_pixel_to_cm.py --point 928 661
  python waste_robot_pixel_to_cm.py --bbox 900 610 990 710
  python waste_robot_pixel_to_cm.py --two-points 928 661 960 680

Note: bottom bbox center must ACTUALLY be an observed floor-contact pixel,
not just a guess for an occluded or inclined object.
"""
import argparse, json, sys
from pathlib import Path
import cv2, numpy as np
DEFAULT_CONFIG=Path(__file__).with_name('waste_robot_search_floor_calibration_preliminary.json')

def pixel_to_ground(x,y,H):
    point=np.array([[[float(x),float(y)]]],dtype=np.float64)
    p=cv2.perspectiveTransform(point,H)[0,0]
    return [float(p[0]),float(p[1])]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--frame-size',nargs=2,type=int,metavar=('WIDTH','HEIGHT'),
                        help='Input frame size if only resized (no crop/rotation), e.g. --frame-size 640 360')
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--point',nargs=2,type=float,metavar=('PIXEL_X','PIXEL_Y'))
    group.add_argument('--bbox',nargs=4,type=float,metavar=('X1','Y1','X2','Y2'))
    group.add_argument('--two-points',nargs=4,type=float,metavar=('X1','Y1','X2','Y2'))
    args=parser.parse_args()
    data=json.loads(args.config.read_text(encoding='utf-8'))
    H=np.array(data['homography_pixel_to_floor_cm'],dtype=np.float64)
    # Input pixel coordinates may come from a uniformly resized, uncropped camera frame.
    # Convert the input pixel coordinates into calibration image pixels using H @ scale.
    if args.frame_size:
        iw,ih=args.frame_size
        if iw<=0 or ih<=0: parser.error('Frame dimensions must be positive')
        sx=data['camera_frame_width_px']/iw
        sy=data['camera_frame_height_px']/ih
        if abs(sx-sy)>1e-3:
            print('WARNING: nonuniform resize; verify the frame was not cropped/letterboxed')
        H=H@np.array([[sx,0,0],[0,sy,0],[0,0,1]],dtype=np.float64)
    print('CALIBRATION STATUS:',data['status'])
    if args.point is not None:
        p=pixel_to_ground(*args.point,H)
        print(f'Point on floor (paper frame): X={p[0]:.2f} cm, Y={p[1]:.2f} cm')
    elif args.bbox is not None:
        x1,y1,x2,y2=args.bbox
        cx=(x1+x2)/2;cy=y2
        p=pixel_to_ground(cx,cy,H)
        print(f'BOTTOM CENTER OF BBOX: pixel ({cx:.1f}, {cy:.1f}) -> X={p[0]:.2f}, Y={p[1]:.2f} cm')
        print('WARNING: This is valid only when that pixel is the true floor-contact point.')
    else:
        p=pixel_to_ground(*args.two_points[:2],H)
        q=pixel_to_ground(*args.two_points[2:],H)
        print(f'Distance between two FLOOR points = {float(np.linalg.norm(np.array(p)-np.array(q))):.2f} cm')
    print('Caution: Blue outline was hand-drawn; test with a second ruler/marker before using for vehicle control.')

if __name__=='__main__':main()
