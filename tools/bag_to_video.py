#!/usr/bin/env python3
"""bag 의 카메라 프레임을 센서·상태 오버레이와 함께 mp4 로 (D12, PC, ROS 불필요).

    python3 tools/bag_to_video.py <bag_dir> --out clip.mp4                 # 전체
    python3 tools/bag_to_video.py <bag_dir> --from 120 --to 140 --out clip.mp4   # bag 시작 기준 초
    python3 tools/bag_to_video.py <bag_dir> --station <station_bag_dir>    # 관제 bag 의 lane_path 도 겹친다

오버레이: 시각 · 라이다 전방 최소거리 · 초음파 · cmd_vel v/ω · odom v · drive_state · error_x · quality.
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_report import DRIVE_NAMES, QUALITY_NAMES, front_min, iter_messages  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag_dir')
    ap.add_argument('--robot', default='pinky1')
    ap.add_argument('--station', help='관제 bag 디렉터리 (lane_debug 오버레이 대신 사용)')
    ap.add_argument('--from', dest='t_from', type=float, default=0.0)
    ap.add_argument('--to', dest='t_to', type=float, default=float('inf'))
    ap.add_argument('--fps', type=float, default=10.0)
    ap.add_argument('--out', default='clip.mp4')
    args = ap.parse_args(argv)
    r = args.robot
    cam_topic = f'/{r}/camera/image/compressed'

    # 관제 bag 의 lane_path 는 source_stamp(로봇 이미지 stamp) 로 프레임에 붙인다
    lp_by_stamp = {}
    if args.station:
        for topic, t, msg in iter_messages(os.path.expanduser(args.station), topics=[f'/{r}/lane_path']):
            key = round(msg.source_stamp.sec + msg.source_stamp.nanosec * 1e-9, 3)
            lp_by_stamp[key] = (int(msg.quality), float(msg.error_x_norm), bool(msg.crosswalk_detected))

    state = {'scan': None, 'us': None, 'cmd': (0.0, 0.0), 'odom': 0.0, 'drive': None, 'ex': None,
             'q': None, 'reason': ''}
    writer = None
    t0 = None
    n = 0
    for topic, t, msg in iter_messages(os.path.expanduser(args.bag_dir)):
        if t0 is None:
            t0 = t
        rel = t - t0
        if rel > args.t_to:
            break
        if topic == '/scan':
            state['scan'] = front_min(msg.ranges, msg.angle_min, msg.angle_increment,
                                      range_min=msg.range_min, range_max=msg.range_max)[0]
        elif topic == '/us_sensor/range':
            state['us'] = float(msg.range)
        elif topic == '/cmd_vel':
            state['cmd'] = (float(msg.linear.x), float(msg.angular.z))
        elif topic == '/odom':
            state['odom'] = float(msg.twist.twist.linear.x)
        elif topic == f'/{r}/lane_status':
            state['drive'] = int(msg.drive_state)
            state['ex'] = float(msg.error_x_norm)
            state['q'] = int(msg.lane_quality)
            state['reason'] = msg.state_reason
        elif topic == cam_topic and rel >= args.t_from:
            img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            H, W = img.shape[:2]
            if writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                writer = cv2.VideoWriter(args.out, fourcc, args.fps, (W, H))
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            lp = lp_by_stamp.get(round(stamp, 3))
            q = lp[0] if lp else state['q']
            ex = lp[1] if lp else state['ex']
            scan = state['scan']
            lines = [
                f't={rel:7.2f}s  {DRIVE_NAMES.get(state["drive"], "-")}  {state["reason"][:28]}',
                f'lidar {scan:.2f}m' if scan is not None and math.isfinite(scan) else 'lidar -',
                f'us {state["us"]:.2f}m' if state['us'] is not None and math.isfinite(state['us']) else 'us -',
                f'cmd v={state["cmd"][0]:.2f} w={state["cmd"][1]:+.2f}  odom v={state["odom"]:.2f}',
                f'{QUALITY_NAMES.get(q, "-")} e={ex:+.2f}' if ex is not None else 'lane -',
            ]
            y = 22
            for s in lines:
                cv2.putText(img, s, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                cv2.putText(img, s, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
                y += 22
            if ex is not None:
                cx = int(W / 2 + ex * W / 2)
                cv2.line(img, (W // 2, H - 30), (W // 2, H), (255, 0, 0), 1)
                cv2.circle(img, (cx, H - 15), 6, (0, 0, 255), -1)
            writer.write(img)
            n += 1
    if writer is None:
        raise SystemExit('구간에 카메라 프레임이 없습니다')
    writer.release()
    print(f'{args.out}: {n} 프레임 ({n / args.fps:.1f} s)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
