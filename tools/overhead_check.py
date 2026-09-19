#!/usr/bin/env python3
"""천장 웹캠 라이브 확인 (ROS 없이). 기준 마커·H·핑키 map 좌표를 화면에 그린다.

    python3 tools/overhead_check.py --config pinky_lane_station/config/overhead.yaml [--device 0] [--save shot.jpg]

핑키를 줄자로 잰 위치에 놓고 표시 좌표와 비교한다 (목표 5 cm / 5°). 코스 네 구석에서도 본다.
q 종료, s 스냅샷 저장.
"""

import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                'pinky_lane_station'))
from pinky_lane_station.overhead_localizer import OverheadConfig, OverheadLocalizer  # noqa: E402
from pinky_lane_station.overhead_localizer_node import draw_debug, open_camera  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', required=True)
    ap.add_argument('--device', help='overhead.yaml 의 camera.device 대신')
    ap.add_argument('--image', help='웹캠 대신 정지 이미지')
    ap.add_argument('--save', default='overhead_shot.jpg')
    args = ap.parse_args()

    cfg = OverheadConfig.load(args.config)
    if args.device is not None:
        cfg.camera['device'] = args.device
    loc = OverheadLocalizer(cfg)
    if args.image:
        img = cv2.imread(os.path.expanduser(args.image))
        fixes = loc.update(img)
        print(loc.status(), {k: (round(f.x, 3), round(f.y, 3), round(f.yaw, 3)) for k, f in fixes.items()})
        cv2.imwrite(args.save, draw_debug(img, loc, loc.detect(img), fixes))
        print('저장:', args.save)
        return
    cap = open_camera(cfg.camera)
    print('q 종료, s 스냅샷')
    while True:
        ok, img = cap.read()
        if not ok:
            print('프레임 읽기 실패')
            break
        fixes = loc.update(img)
        dbg = draw_debug(img.copy(), loc, loc.detect(img), fixes)
        cv2.imshow('overhead', dbg)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        if k == ord('s'):
            cv2.imwrite(args.save, img)
            print('저장:', args.save, loc.status(),
                  {n: (round(f.x, 3), round(f.y, 3), round(f.yaw, 3)) for n, f in fixes.items()})
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
