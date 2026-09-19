#!/usr/bin/env python3
"""markers.yaml 의 ArUco 마커를 실제 크기로 인쇄용 PNG(A4, 300 dpi) 로 만든다 (PC).

    python3 tools/print_markers.py --config pinky_lane_station/config/markers.yaml --out ~/markers
    → markers/marker_00_BL.png ... 한 장에 한 마커. 프린터에서 "실제 크기(100 %)" 로 인쇄한다.

각 장에는 id · 노드 이름 · **+x 화살표** 가 찍힌다. 마커를 바닥에 붙일 때 이 화살표가
markers.yaml 의 yaw 방향(map 기준)을 가리키게 놓는다. 인쇄 후 검은 테두리 한 변을 자로 재서
size 와 같은지 확인한다 (프린터 배율이 어긋나면 위치가 그 비율만큼 틀린다).
"""

import argparse
import os

import cv2
import numpy as np
import yaml

DPI = 300
A4 = (int(8.27 * DPI), int(11.69 * DPI))     # (w, h) px


def mm2px(mm):
    return int(round(mm / 25.4 * DPI))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', required=True)
    ap.add_argument('--out', default='markers_print')
    ap.add_argument('--per-page', type=int, default=1, help='1 또는 2 (세로로 2장)')
    args = ap.parse_args()

    with open(os.path.expanduser(args.config), encoding='utf-8') as fh:
        cfg = yaml.safe_load(fh)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg.get('dictionary', 'DICT_4X4_50')))
    size_px = mm2px(float(cfg['size']) * 1000.0)
    border_px = mm2px(float(cfg.get('white_border', 0.015)) * 1000.0)
    os.makedirs(os.path.expanduser(args.out), exist_ok=True)

    for m in cfg['markers']:
        mid = int(m['id'])
        label = m.get('node') or f"({m.get('x')}, {m.get('y')})"
        page = np.full((A4[1], A4[0]), 255, dtype=np.uint8)
        mk = (cv2.aruco.generateImageMarker(d, mid, size_px) if hasattr(cv2.aruco, 'generateImageMarker')
              else cv2.aruco.drawMarker(d, mid, size_px))
        cx, cy = A4[0] // 2, A4[1] // 2
        x0, y0 = cx - size_px // 2, cy - size_px // 2
        page[y0:y0 + size_px, x0:x0 + size_px] = mk
        # 흰 여백 경계 (자르는 선, 연한 회색)
        cv2.rectangle(page, (x0 - border_px, y0 - border_px),
                      (x0 + size_px + border_px, y0 + size_px + border_px), 200, 2)
        # +x 화살표: 마커 프레임 x 는 인쇄물의 오른쪽
        ax = x0 + size_px + border_px + mm2px(8)
        cv2.arrowedLine(page, (ax, cy), (ax + mm2px(30), cy), 0, 6, tipLength=0.3)
        cv2.putText(page, '+x (yaw)', (ax, cy - mm2px(5)), cv2.FONT_HERSHEY_SIMPLEX, 2.0, 0, 4)
        cv2.putText(page, f'id {mid}  {label}  size {float(cfg["size"])*100:.0f} cm  yaw {m.get("yaw", 0.0)} rad',
                    (mm2px(15), mm2px(20)), cv2.FONT_HERSHEY_SIMPLEX, 2.0, 0, 4)
        cv2.putText(page, 'print at 100% (actual size); measure black edge after printing',
                    (mm2px(15), A4[1] - mm2px(15)), cv2.FONT_HERSHEY_SIMPLEX, 1.4, 0, 3)
        name = f"marker_{mid:02d}_{m.get('node', 'free')}.png"
        cv2.imwrite(os.path.join(os.path.expanduser(args.out), name), page)
        print(name)


if __name__ == '__main__':
    main()
