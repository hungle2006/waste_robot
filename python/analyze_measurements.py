"""Reproduce reference arithmetic offline; never change robot parameters.

    python analyze_measurements.py
    python analyze_measurements.py --json
"""
import argparse
import json
import math
from pathlib import Path
from config_io import read_namespace


def solve(a,b):
    rows=[list(map(float,row))+[float(v)] for row,v in zip(a,b)]
    for c in range(len(b)):
        pivot=max(range(c,len(b)),key=lambda r:abs(rows[r][c]))
        rows[c],rows[pivot]=rows[pivot],rows[c]
        divisor=rows[c][c]
        if abs(divisor)<1e-12:raise ValueError('Bốn góc giấy suy biến')
        rows[c]=[x/divisor for x in rows[c]]
        for r in range(len(b)):
            if r!=c:
                factor=rows[r][c];rows[r]=[x-factor*y for x,y in zip(rows[r],rows[c])]
    return [row[-1] for row in rows]


def paper_homography(corners,side):
    a=[];b=[]
    for (x,y),(u,v) in zip(corners,[(0,0),(side,0),(side,side),(0,side)]):
        a.extend([[x,y,1,0,0,0,-u*x,-u*y],[0,0,0,x,y,1,-v*x,-v*y]])
        b.extend([u,v])
    v=solve(a,b)
    return [v[0:3],v[3:6],v[6:8]+[1.]]


def project(h,x,y):
    z=h[2][0]*x+h[2][1]*y+h[2][2]
    return [(row[0]*x+row[1]*y+row[2])/z for row in h[:2]]


def analyze(p):
    corners=p['PAPER_CORNERS_PX'];h=paper_homography(corners,p['PAPER_SIDE_CM'])
    x=sum(c[0] for c in corners)/4;y=sum(c[1] for c in corners)/4
    u,v=project(h,x,y);z=h[2][0]*x+h[2][1]*y+h[2][2]
    j=[[(h[r][c]-[u,v][r]*h[2][c])/z for c in (0,1)] for r in (0,1)]
    scales=[math.hypot(j[0][i],j[1][i]) for i in (0,1)]
    difference=max(abs(a-b) for row1,row2 in zip(h,p['PAPER_H_PIXEL_TO_CM']) for a,b in zip(row1,row2))
    segments=[]
    for a,b in zip(p['ARM_MEASURED_POINTS'],p['ARM_MEASURED_POINTS'][1:]):
        segments.append(dict(p1_interval=[a[0],b[0]],p10_start=a[1],
            p10_slope=(b[1]-a[1])/(b[0]-a[0]),
            lens_reference_start_cm=a[2],lens_reference_slope=(b[2]-a[2])/(b[0]-a[0])))
    return dict(version='3.12',distance_definition='camera_lens_to_bottle',
        s2_floor_reference_cm=p['GRASP_FORWARD_CM'],grasp_lens_reference_cm=5.,
        mechanical_angles_identified=False,camera_intrinsic_K_identified=False,
        independent_calibration_validation=False,
        usb_frame_size=[1920,1080],usb_manual_samples=p['USB_MEASUREMENTS'],
        arm_piecewise_segments=segments,stage_timing=p['STAGE_TIMING'],
        paper_corners_px=corners,paper_side_cm=p['PAPER_SIDE_CM'],
        recomputed_paper_H_pixel_to_cm=h,max_delta_from_retained_H=difference,
        local_scale_paper_center_px=[x,y],local_cm_per_native_pixel_axis_xy=scales,
        note='Paper H: SEARCH pose and paper plane only; no current S2 or arbitrary-object range inferred.')


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--json',action='store_true')
    ap.add_argument('--parameters',default=str(Path(__file__).with_name('robot_params.py')))
    args=ap.parse_args();result=analyze(read_namespace(args.parameters))
    if args.json:print(json.dumps(result,ensure_ascii=False,indent=2));return
    print('USB: 60/75/90/45/24/12/5 cm đo từ ỐNG KÍNH. S2 -> chai 27 cm lưu riêng.')
    print('P1/P10 nội suy từng đoạn; không suy ra góc cơ khí.')
    for segment in result['arm_piecewise_segments']:print(segment)
    print('Tính lại H từ 4 góc giấy 17x17 cm:')
    for row in result['recomputed_paper_H_pixel_to_cm']:print(row)
    print('Sai khác lớn nhất với H giữ lại:',result['max_delta_from_retained_H'])
    print('cm/pixel gốc tại giữa tờ giấy, theo hướng x/y:',result['local_cm_per_native_pixel_axis_xy'])
    print('Đây là tỷ lệ cục bộ trên mặt phẳng giấy ở SEARCH, không áp sang ảnh hạ tay.')


if __name__=='__main__':main()
