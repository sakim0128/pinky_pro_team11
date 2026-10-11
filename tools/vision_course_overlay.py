#!/usr/bin/env python3
"""비전 미션 코스(vision_course.yaml) 를 docs/map5.png 위에 겹쳐 그린다 — 중심선·빨간 선·정지 위치·지점 확인용 (ROS 없이).

    python3 tools/vision_course_overlay.py                         # → docs/vision_course_overlay.png
    python3 tools/vision_course_overlay.py --route 3 1 --out /tmp/r31.png

초록 = 코스 중심선, 빨강 점 = 빨간 선 가운데, 파랑 삼각형 = 빨간 선·횡단보도·마커 앞 정지 위치(구동축), 숫자 = 지점.
중심선이 흰 선 두 개 사이 가운데에 있는지, 정지 위치가 선 앞에 있는지 눈으로 본다. 틀리면 yaml 의 px 를 고친다.
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'pinky_lane_station'))

from pinky_lane_station.vision_pose import load_vision_course  # noqa: E402

ROUTES = [('2', '1'), ('1', '2'), ('3', '1'), ('1', '3'), ('2', '3'), ('3', '2')]


def draw(course, routes):
    img = cv2.imread(os.path.join(REPO, course.image_file))
    if img is None:
        raise SystemExit(f'그림을 못 읽음: {course.image_file}')
    for a, b in routes:
        r = course.route(a, b)
        px = [tuple(int(round(v)) for v in course.map_to_px(x, y)) for x, y in r.points]
        for p, q in zip(px, px[1:]):
            cv2.line(img, p, q, (40, 170, 40), 1, cv2.LINE_AA)
        stops = [r.s_entry - course.red_stop_back, r.s_exit - course.red_stop_back,
                 r.length - course.marker_stop_back] + [c[0] - course.crosswalk_stop_back for c in r.crosswalks]
        for s in stops:
            x, y, yaw = r.point_at(s)
            cx, cy = course.map_to_px(x, y)
            d = 7
            # map yaw → 그림 방향 (y 축 뒤집힘)
            tip = (int(cx + d * math.cos(yaw)), int(cy - d * math.sin(yaw)))
            l = (int(cx + d * math.cos(yaw + 2.5)), int(cy - d * math.sin(yaw + 2.5)))
            rr = (int(cx + d * math.cos(yaw - 2.5)), int(cy - d * math.sin(yaw - 2.5)))
            cv2.fillPoly(img, [np.array([tip, l, rr], np.int32)], (200, 80, 0))
    for k, (x, y) in course.red.items():
        cx, cy = course.map_to_px(x, y)
        cv2.circle(img, (int(cx), int(cy)), 4, (0, 0, 220), -1)
        cv2.putText(img, k, (int(cx) + 6, int(cy) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 180), 1)
    for pid, p in course.points.items():
        cx, cy = course.map_to_px(*p['arm'][0])
        cv2.putText(img, pid, (int(cx) - 6, int(cy) + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 0, 180), 2)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--course', default=os.path.join(REPO, 'pinky_lane_station', 'config', 'vision_course.yaml'))
    ap.add_argument('--route', nargs=2, metavar=('START', 'GOAL'))
    ap.add_argument('--out', default=os.path.join(REPO, 'docs', 'vision_course_overlay.png'))
    a = ap.parse_args()
    course = load_vision_course(a.course)
    img = draw(course, [tuple(a.route)] if a.route else ROUTES)
    cv2.imwrite(a.out, img)
    print(f'저장: {a.out}')


if __name__ == '__main__':
    main()
