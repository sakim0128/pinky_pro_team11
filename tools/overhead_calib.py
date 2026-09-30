#!/usr/bin/env python3
"""천장 웹캠 호모그래피 캘리브레이션 → overhead_tracker.yaml 의 image_to_map_homography (ROS 없이).

    python3 tools/overhead_calib.py --device 0 --corners "40:0.10,0.10;41:2.25,0.10;42:2.25,1.15;43:0.10,1.15"
    python3 tools/overhead_calib.py --image shot.jpg --corners ... --write pinky_fleet_station/config/overhead_tracker.yaml

corners = 지도 네 꼭짓점 마커 id : map 좌표(m). 줄자로 잰 마커 **중심** (라이다 맵 origin 기준).
--write 를 주면 yaml 의 image_to_map_homography 줄만 바꾼다. --check "x,y" 를 주면 로봇 마커(id 1/2) 의
map 좌표를 같이 출력해 줄자 값과 비교할 수 있다.
"""

import argparse
import os
import re
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                'pinky_fleet_station'))
from pinky_fleet_station.overhead_calib import (CalibError, calibrate, detect_centers,  # noqa: E402
                                                parse_corner_map, project, yaml_line)


def grab(device, width, height, n=5):
    cap = cv2.VideoCapture(int(device) if str(device).isdigit() else device)
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    img = None
    for _ in range(n):                       # 자동 노출이 안정될 때까지 몇 장 버린다
        ok, img = cap.read()
        if not ok:
            img = None
    cap.release()
    if img is None:
        raise CalibError(f'카메라 {device} 프레임 읽기 실패')
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--corners', required=True, help='"id:x,y;id:x,y;..." (4개 이상)')
    ap.add_argument('--image', help='정지 이미지. 없으면 --device 에서 촬영')
    ap.add_argument('--device', default='0')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--dictionary', default='DICT_4X4_50')
    ap.add_argument('--write', help='이 yaml 의 image_to_map_homography 줄을 갱신')
    ap.add_argument('--save', default='overhead_calib.jpg', help='검출 표시 이미지 저장')
    ap.add_argument('--robots', default='1,2', help='같이 좌표를 찍어볼 로봇 마커 id')
    args = ap.parse_args()

    corners = parse_corner_map(args.corners)
    img = cv2.imread(os.path.expanduser(args.image)) if args.image else grab(args.device, args.width, args.height)
    if img is None:
        raise SystemExit('이미지를 읽을 수 없습니다')
    try:
        H, rep = calibrate(img, corners, args.dictionary)
    except CalibError as exc:
        raise SystemExit(f'실패: {exc}')

    print('기준 마커 (id: px → 재투영 오차 m)')
    for i, (px, py) in rep['detected'].items():
        print(f'  {i}: ({px:.1f}, {py:.1f}) → {rep["reproj_m"][i] * 100:.1f} cm')
    if rep['missing']:
        print('안 보인 기준 마커:', rep['missing'])
    print(f'최대 재투영 오차 {rep["max_reproj_m"] * 100:.1f} cm  (2 cm 넘으면 좌표 측정을 다시)')
    centers = detect_centers(img, args.dictionary)
    for rid in (int(r) for r in args.robots.split(',') if r.strip()):
        if rid in centers:
            mx, my = project(H, *centers[rid])
            print(f'로봇 마커 {rid}: map ({mx:.3f}, {my:.3f}) m  ← 줄자와 비교')
    line = yaml_line(H)
    print('\n' + line)

    dbg = img.copy()
    for i, (px, py) in centers.items():
        cv2.circle(dbg, (int(px), int(py)), 6, (0, 0, 255) if i in corners else (0, 255, 0), -1)
        cv2.putText(dbg, str(i), (int(px) + 8, int(py) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.imwrite(args.save, dbg)
    print('검출 이미지:', args.save)

    if args.write:
        path = os.path.expanduser(args.write)
        with open(path, encoding='utf-8') as fh:
            text = fh.read()
        new, n = re.subn(r'^(\s*)image_to_map_homography:.*$', lambda m: m.group(1) + line, text, flags=re.M)
        if n == 0:
            raise SystemExit(f'{path} 에 image_to_map_homography 줄이 없습니다')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(new)
        print('갱신:', path)


if __name__ == '__main__':
    main()
