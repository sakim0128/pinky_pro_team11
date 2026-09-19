#!/usr/bin/env python3
"""핑키 카메라 내부 파라미터 캘리브레이션 (PC). 체스보드 프레임 → camera_intrinsics.yaml.

  1) 체스보드(예: 9x6 내부 코너, 한 칸 25 mm) 를 A4 에 인쇄해 판에 붙인다.
  2) 로봇에서 record_drive.py 로 30 초쯤 찍는다. 체스보드를 화면 구석구석·기울여서 보여 준다.
  3) PC: python3 tools/extract_frames.py <mp4> --every 5 --out ~/calib_frames
  4) python3 tools/calib_intrinsics.py --frames ~/calib_frames --cols 9 --rows 6 --square 0.025 \
         --out pinky_lane_station/config/camera_intrinsics.yaml

reprojection RMS 가 0.5 px 이하면 좋고 1 px 이상이면 다시 찍는다. camera_node 의 orient 보정을
거친 이미지(정방향)로 찍어야 한다 — record_drive.py --orient 로 같은 값을 준다.
"""

import argparse
import glob
import os

import cv2
import numpy as np
import yaml


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--frames', required=True)
    ap.add_argument('--cols', type=int, default=9, help='가로 내부 코너 수')
    ap.add_argument('--rows', type=int, default=6)
    ap.add_argument('--square', type=float, default=0.025, help='한 칸 (m)')
    ap.add_argument('--out', default='camera_intrinsics.yaml')
    ap.add_argument('--max', type=int, default=60)
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(os.path.expanduser(args.frames), '*.jpg')) +
                   glob.glob(os.path.join(os.path.expanduser(args.frames), '*.png')))
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    obj_pts, img_pts = [], []
    shape = None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        shape = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), None)
        if not ok:
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
        obj_pts.append(objp)
        img_pts.append(corners)
        if len(obj_pts) >= args.max:
            break
    print(f'체스보드 검출 {len(obj_pts)} / {len(files)} 장')
    if len(obj_pts) < 8:
        raise SystemExit('프레임이 부족합니다 (8 장 이상 필요)')
    rms, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, shape, None, None)
    print(f'RMS {rms:.3f} px  fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}')
    print('dist', np.round(D.ravel(), 4).tolist())
    out = {'calibrated': True, 'width': int(shape[0]), 'height': int(shape[1]),
           'fx': float(K[0, 0]), 'fy': float(K[1, 1]), 'cx': float(K[0, 2]), 'cy': float(K[1, 2]),
           'dist': [float(v) for v in D.ravel()[:5]], 'rms_px': float(rms), 'frames': len(obj_pts)}
    with open(os.path.expanduser(args.out), 'w', encoding='utf-8') as fh:
        yaml.safe_dump(out, fh, sort_keys=False)
    print('저장:', args.out)


if __name__ == '__main__':
    main()
