#!/usr/bin/env python3
"""카메라 외부 파라미터(base_footprint→카메라) 캘리브레이션 (PC). 마커 한 장 → camera_extrinsics.yaml.

  1) 로봇을 바닥에 놓고, 바퀴 축 중심(base_footprint) 에서 정확히 --ahead m 앞(기본 0.30),
     정중앙에 마커(id --id) 를 +x 화살표가 로봇 전방을 향하게 놓는다. 줄자로 잰다.
  2) 로봇 카메라로 정지 프레임을 찍는다 (record_drive.py --snap 또는 ros2 topic 저장).
  3) python3 tools/calib_extrinsics.py --image frame.jpg --intrinsics camera_intrinsics.yaml \
         --markers pinky_lane_station/config/markers.yaml --id 0 --ahead 0.30 \
         --out pinky_lane_station/config/camera_extrinsics.yaml

원리: T_base_marker 를 알고(측정), T_cam_marker 를 PnP 로 얻으니 T_base_cam = T_base_marker · inv(T_cam_marker).
여러 장(거리 0.2/0.3/0.5) 을 --image 로 여러 번 주면 평균한다.
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                'pinky_lane_station'))
from pinky_lane_station.marker_localizer import (  # noqa: E402
    R_BASE_OPT0, CameraIntrinsics, MarkerLocalizer, MarkerMap, inv_T, make_T, rot_z)


def euler_zyx(R):
    """R = Rz(yaw)·Ry(pitch)·Rx(roll) 분해."""
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    yaw = math.atan2(R[1, 0], R[0, 0])
    roll = math.atan2(R[2, 1], R[2, 2])
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image', action='append', required=True)
    ap.add_argument('--ahead', action='append', type=float, help='각 이미지의 마커 거리 (m). 하나면 공통')
    ap.add_argument('--left', type=float, default=0.0, help='마커 중심의 좌측 오프셋 (m)')
    ap.add_argument('--intrinsics', required=True)
    ap.add_argument('--markers', required=True)
    ap.add_argument('--id', type=int, default=0)
    ap.add_argument('--out', default='camera_extrinsics.yaml')
    args = ap.parse_args()

    intr = CameraIntrinsics.load(args.intrinsics)
    with open(os.path.expanduser(args.markers), encoding='utf-8') as fh:
        mcfg = yaml.safe_load(fh)
    aheads = args.ahead or [0.30]
    if len(aheads) == 1:
        aheads = aheads * len(args.image)
    mm = MarkerMap(str(mcfg.get('dictionary', 'DICT_4X4_50')), float(mcfg['size']),
                   float(mcfg.get('white_border', 0.015)), {args.id: (0.0, 0.0, 0.0)})
    # extrinsics 는 아직 모른다 — 검출·PnP 만 쓰므로 아무 값이나
    from pinky_lane_station.marker_localizer import CameraExtrinsics
    loc = MarkerLocalizer(intr, CameraExtrinsics(), mm, min_range=0.05, max_range=2.0, max_reproj=10.0)

    Ts = []
    for path, ahead in zip(args.image, aheads):
        img = cv2.imread(os.path.expanduser(path))
        obs = [o for o in loc.detect(img) if o.id == args.id]
        if not obs:
            print(f'{path}: 마커 {args.id} 못 찾음')
            continue
        o = obs[0]
        T_base_marker = make_T(rot_z(0.0), (ahead, args.left, 0.0))   # +x 화살표 = 로봇 전방
        T_base_cam = T_base_marker @ inv_T(o.T_cam_marker)
        Ts.append(T_base_cam)
        print(f'{path}: range {o.range:.3f} reproj {o.reproj:.2f} px → cam at '
              f'({T_base_cam[0,3]:.3f}, {T_base_cam[1,3]:.3f}, {T_base_cam[2,3]:.3f})')
    if not Ts:
        raise SystemExit('유효한 이미지가 없습니다')
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    R = np.mean([T[:3, :3] for T in Ts], axis=0)
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    # R_base_cam = Rz·Ry·Rx · R_BASE_OPT0  →  Rz·Ry·Rx = R · R_BASE_OPT0ᵀ
    roll, pitch, yaw = euler_zyx(R @ R_BASE_OPT0.T)
    out = {'calibrated': True, 'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2]),
           'roll_deg': math.degrees(roll), 'pitch_deg': math.degrees(pitch), 'yaw_deg': math.degrees(yaw),
           'images': len(Ts)}
    print(f"x={out['x']:.3f} y={out['y']:.3f} z={out['z']:.3f} roll={out['roll_deg']:.1f}° "
          f"pitch={out['pitch_deg']:.1f}° yaw={out['yaw_deg']:.1f}°")
    if not (0.03 < out['z'] < 0.30) or not (5 < out['pitch_deg'] < 60):
        print('경고: 값이 비현실적입니다. 마커 방향(+x 화살표 = 전방)·거리·intrinsics 를 확인하세요')
    with open(os.path.expanduser(args.out), 'w', encoding='utf-8') as fh:
        yaml.safe_dump(out, fh, sort_keys=False)
    print('저장:', args.out)


if __name__ == '__main__':
    main()
