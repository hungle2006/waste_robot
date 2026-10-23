WASTE ROBOT - HIEU CHUAN SO BO CAMERA / SAN 17 x 17 cm

Nguon: anh camera WIN_20261013_21_15_05_Pro(1).jpg (co ve khung xanh), anh camera goc WIN_20261013_21_15_05_Pro.jpg.
Do phan giai file goc: 1920 x 1080 pixel.
Tu the SEARCH: goc hinh hoc S1/S2=195 do (nguoi dung mo ta).
Giay mau: 17 x 17 cm, camera -> tam giay 92-93 cm do bang thuoc (thong so tham khao, khong dung trong H).

Goc pixel (TL,TR,BR,BL), UOC LUONG theo khung ve mau xanh:
P1 = (816, 606) -> (0, 0) cm
P2 = (1007, 594) -> (17, 0) cm
P3 = (1050, 711) -> (17, 17) cm
P4 = (838, 737) -> (0, 17) cm

Cai dat: pip install numpy opencv-python
Thu: python waste_robot_pixel_to_cm.py --point 928 661
     python waste_robot_pixel_to_cm.py --two-points 928 661 960 680
     python waste_robot_pixel_to_cm.py --bbox 900 610 990 710
     python waste_robot_pixel_to_cm.py --frame-size 640 360 --point 311 219

He truc x/y moi: goc to giay, x huong sang phai theo canh tren, y theo canh trai xuong duoi.
KHONG PHAI TRUC ROBOT. X/Y co the am khi diem nam ngoai giay.

LUU Y:
- Day moi la HIEU CHUAN SO BO tu net ve tay, chua chac dung 4 goc GIAY THUC.
- Dung cho cac diem nam TREN CUNG mat phang san, uu tien gan giay.
- Khong the do do cao cua chai, lon, hay khoang cach 3D den be mat vat bang H.
- Chi tinh chenh lech tu camera -> vat khong du: can hieu chuan them diem hinh chieu cang kep khi o tu the APPROACH.
- Khong dung toa do tam bbox cua chai dung thang nhu 1 diem san.
- Neu camera doi tu the / cat anh / zoom hay quay len thi dung H cu khong dung.
- Neu feed camera co kich thuoc khac anh goc, can doi toa do pixel sang 1920x1080 neu chi resize dong nhat va khong crop; can hieu chuan lai neu co cat anh.
- Khong cho robot tu di chuyen bang ban H so bo nay; hay xac nhan bang thuoc/marker khac.

CACH LAM TOT HON:
1. In ArUco marker hoac danh 4 dau mau dam o 4 goc giay 17 x 17cm.
2. Chup anh SEARCH goc moi; xac nhan chinh xac 4 goc (paper nằm trọn trong frame).
3. Them 2-4 diem kiem tra voi khoang cach thuc den dau tren san; do sai so cm.
4. Do va danh dau hinh chieu cang kep o tu the ha tay 90 do de lay toa do grasp target.
5. Thuc thi he thong closed-loop re-detect Modal, dung bang mecanum, roi moi ha canh tay.
