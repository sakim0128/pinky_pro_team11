#!/usr/bin/env python3
"""블랙박스 bag 요약 리포트 (D12, PC, ROS 불필요).  pip install mcap mcap-ros2-support

    python3 tools/bag_report.py ~/pinky_logs/2026-09-21/143000_pinky_robot_pinky1 [--out report.md] [--events events.csv]
    python3 tools/bag_report.py <bag_dir> --robot pinky1

토픽별 개수·주기·끊김, 라이다 전방 최소거리 급변·무효 급증, 초음파 튐, 명령 대 odom 속도 불일치,
LaneStatus 상태 전이·체류시간, LanePath quality 분포, path_age 통계를 시각순 이벤트와 함께 낸다.
시각은 bag 시작 기준 초(s). bag_to_video.py --from/--to 에 그대로 쓴다.
"""

import argparse
import csv
import glob
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                'pinky_lane_station'))
from pinky_lane_station.bag_analysis import (  # noqa: E402
    dwell, front_min, gap_events, histogram, rate_stats, scalar_jumps, scan_events, transitions,
    velocity_mismatch)

DRIVE_NAMES = {0: 'IDLE', 1: 'CRUISE', 2: 'WAIT_CLEARANCE', 3: 'CROSSWALK_STOP', 4: 'CROSSWALK_CLEAR',
               5: 'OBSTACLE_WAIT', 6: 'LANE_LOST', 7: 'ARRIVED', 8: 'ESTOP', 9: 'LINK_LOST'}
QUALITY_NAMES = {0: 'BOTH', 1: 'SINGLE', 2: 'JUNCTION', 3: 'STALE', 4: 'LOST'}


def open_bag(bag_dir):
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError:
        raise SystemExit('pip install mcap mcap-ros2-support 가 필요합니다')
    files = sorted(glob.glob(os.path.join(os.path.expanduser(bag_dir), '*.mcap')))
    if not files:
        raise SystemExit(f'{bag_dir} 에 .mcap 이 없습니다 (sqlite3 bag 이면 ros2 bag convert 로 mcap 으로)')
    return files, make_reader, DecoderFactory


def iter_messages(bag_dir, topics=None):
    """(topic, t_log[s], msg) 를 시각순으로. 분할된 파일을 차례로 읽는다."""
    files, make_reader, DecoderFactory = open_bag(bag_dir)
    for f in files:
        with open(f, 'rb') as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            for schema, channel, message, ros_msg in reader.iter_decoded_messages(topics=topics):
                yield channel.topic, message.log_time * 1e-9, ros_msg


def stamp_of(msg):
    try:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    except AttributeError:
        return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag_dir')
    ap.add_argument('--robot', default='pinky1')
    ap.add_argument('--out', help='markdown 리포트 경로 (기본: <bag_dir>/report.md)')
    ap.add_argument('--events', help='이벤트 csv (기본: <bag_dir>/events.csv)')
    ap.add_argument('--scan-jump', type=float, default=0.30)
    ap.add_argument('--us-jump', type=float, default=0.15)
    args = ap.parse_args(argv)
    bag = os.path.expanduser(args.bag_dir)
    out_md = args.out or os.path.join(bag, 'report.md')
    out_csv = args.events or os.path.join(bag, 'events.csv')
    r = args.robot

    per_topic = {}
    scan_t, scan_front = [], []
    us_t, us_v = [], []
    cmd_t, cmd_v = [], []
    odom_t, odom_v = [], []
    st_t, st_state, st_age, st_q = [], [], [], []
    lp_q = []
    cam_t = []
    t0 = None
    for topic, t, msg in iter_messages(bag):
        if t0 is None:
            t0 = t
        rel = t - t0
        per_topic.setdefault(topic, []).append(rel)
        if topic == '/scan':
            scan_t.append(rel)
            scan_front.append(front_min(msg.ranges, msg.angle_min, msg.angle_increment,
                                        range_min=msg.range_min, range_max=msg.range_max))
        elif topic == '/us_sensor/range':
            us_t.append(rel)
            us_v.append(float(msg.range))
        elif topic == '/cmd_vel':
            cmd_t.append(rel)
            cmd_v.append(float(msg.linear.x))
        elif topic == '/odom':
            odom_t.append(rel)
            odom_v.append(float(msg.twist.twist.linear.x))
        elif topic == f'/{r}/lane_status':
            st_t.append(rel)
            st_state.append(int(msg.drive_state))
            st_age.append(float(msg.path_age))
            st_q.append(int(msg.lane_quality))
        elif topic == f'/{r}/lane_path':
            lp_q.append(int(msg.quality))
        elif topic == f'/{r}/camera/image/compressed':
            cam_t.append(rel)
    if t0 is None:
        raise SystemExit('메시지가 없습니다')

    events = []
    events += scan_events(scan_t, scan_front, jump=args.scan_jump)
    events += scalar_jumps(us_t, us_v, thresh=args.us_jump, label='us')
    events += velocity_mismatch(cmd_t, cmd_v, odom_t, odom_v)
    events += gap_events(cam_t, 'camera', min_gap=0.5)
    events += gap_events(scan_t, 'scan', min_gap=0.5)
    events += gap_events(per_topic.get(f'/{r}/lane_path', []), 'lane_path', min_gap=0.9)
    trans = transitions(st_t, st_state, DRIVE_NAMES)
    events.sort(key=lambda e: e.t)

    lines = [f'# bag 리포트 — {os.path.basename(bag.rstrip("/"))}', '',
             f'시작 {t0:.3f} (unix), 길이 {max(max(v) for v in per_topic.values()):.1f} s', '',
             '## 토픽', '', '| 토픽 | 개수 | 평균 주기 | 최대 갭 | 갭(>3×) |', '|---|---|---|---|---|']
    for topic in sorted(per_topic):
        s = rate_stats(per_topic[topic])
        mp = '-' if math.isnan(s['mean_period']) else f"{s['mean_period'] * 1000:.0f} ms"
        lines.append(f"| `{topic}` | {s['count']} | {mp} | {s['max_gap']:.2f} s | {len(s['gaps'])} |")
    if scan_front:
        mins = [m for m, _, _ in scan_front if math.isfinite(m)]
        inv = [1 - v / n for _, v, n in scan_front if n]
        lines += ['', '## 라이다 전방 ±35°', '',
                  f'최소거리 중앙값 {sorted(mins)[len(mins) // 2]:.2f} m, 최소 {min(mins):.2f} m, '
                  f'무효 비율 평균 {sum(inv) / len(inv) * 100:.1f} %, 최대 {max(inv) * 100:.0f} %' if mins else '유효 스캔 없음']
    if us_v:
        ok = [v for v in us_v if math.isfinite(v)]
        lines += ['', '## 초음파', '', f'샘플 {len(us_v)}, 무효 {len(us_v) - len(ok)}, '
                  + (f'중앙값 {sorted(ok)[len(ok) // 2]:.2f} m, 최소 {min(ok):.2f} m' if ok else '')]
    if st_t:
        d = dwell(st_t, st_state, DRIVE_NAMES)
        ages = [a for a in st_age if a >= 0]
        lines += ['', '## 주행 상태 (LaneStatus)', '',
                  '체류: ' + ', '.join(f'{k} {v:.1f}s' for k, v in sorted(d.items(), key=lambda kv: -kv[1])), '',
                  'quality(로봇 수신): ' + ', '.join(f'{k} {v}' for k, v in histogram(st_q, QUALITY_NAMES).items())]
        if ages:
            ages.sort()
            lines.append(f'path_age(카메라→로봇 왕복): 중앙값 {ages[len(ages) // 2] * 1000:.0f} ms, '
                         f'p95 {ages[int(len(ages) * 0.95)] * 1000:.0f} ms, 최대 {ages[-1] * 1000:.0f} ms')
        lines += ['', '전이:', ''] + [f'- {t:.1f}s {a} → {b}' for t, a, b in trans]
    if lp_q:
        lines += ['', '## LanePath quality (관제 발행)', '',
                  ', '.join(f'{k} {v}' for k, v in histogram(lp_q, QUALITY_NAMES).items())]
    lines += ['', f'## 이벤트 ({len(events)})', '', '| t(s) | 종류 | 값 | 설명 |', '|---|---|---|---|']
    lines += [f'| {e.t:.2f} | {e.kind} | {e.value:.3f} | {e.detail} |' for e in events]
    lines += ['', '클립: `python3 tools/bag_to_video.py <bag_dir> --from <t-10> --to <t+10> --out clip.mp4`']
    with open(out_md, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n')
    with open(out_csv, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'kind', 'value', 'detail'])
        for e in events:
            w.writerow(e.row())
    print('\n'.join(lines[:6]))
    print(f'... 이벤트 {len(events)}개. 리포트 {out_md}, 이벤트 {out_csv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
