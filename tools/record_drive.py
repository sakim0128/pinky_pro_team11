#!/usr/bin/env python3
"""핑키 주행 영상 수집기 (로봇에서 실행, ROS 불필요).

teleop 로 수동 주행하는 동안 카메라 영상을 mp4 로 저장하고,
프레임마다 촬영 시각을 CSV 로 남긴다. CSV 의 stamp 는 rosbag 의
odom / scan 과 나중에 맞추기 위한 것이다.

    python3 record_drive.py --snap              # 방향 확인용 한 장
    python3 record_drive.py --name course_a     # 본 촬영 (Ctrl+C 로 종료)

카메라가 거꾸로 달려 있다. --orient 로 보정 방식을 고른다.
기본값 rot180 이 기하학적으로 맞는 보정이고, 나머지는 현장에서
--snap 으로 비교해 보기 위한 것이다.
"""
import argparse
import csv
import os
import signal
import sys
import time

import cv2

DEFAULT_W, DEFAULT_H, DEFAULT_FPS = 640, 480, 15
ORIENTS = ('none', 'rot180', 'rot180_mirror', 'vflip', 'hflip')


def apply_orient(frame, orient):
    """카메라 장착 방향 보정. rot180_mirror 는 수업 자료의 LCD 미리보기와 같다."""
    if orient == 'rot180':
        return cv2.rotate(frame, cv2.ROTATE_180)
    if orient == 'rot180_mirror':
        return cv2.flip(cv2.rotate(frame, cv2.ROTATE_180), 1)
    if orient == 'vflip':
        return cv2.flip(frame, 0)
    if orient == 'hflip':
        return cv2.flip(frame, 1)
    return frame


def open_camera(width, height):
    """picamera2 를 연다. RGB888 은 numpy 상에서 BGR 순서라 cv2 에 그대로 쓴다."""
    from picamera2 import Picamera2  # noqa: PLC0415 - 로봇에서만 필요

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(
        main={'format': 'RGB888', 'size': (width, height)}))
    picam2.start()
    time.sleep(1.0)  # 자동 노출이 자리잡을 시간
    return picam2


def snap(args):
    picam2 = open_camera(args.width, args.height)
    try:
        frame = picam2.capture_array()
        os.makedirs(args.out, exist_ok=True)
        for orient in ORIENTS:
            path = os.path.join(args.out, f'snap_{orient}.jpg')
            cv2.imwrite(path, apply_orient(frame, orient))
            print(f'저장: {path}')
        print('\n다섯 장을 보고 실제 장면과 좌우·상하가 맞는 것을 고르세요.')
        print('고른 값을 본 촬영에서 --orient 로 넘깁니다.')
    finally:
        picam2.close()


def record(args):
    session = args.name or time.strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(args.out, session)
    os.makedirs(out_dir, exist_ok=True)
    video_path = os.path.join(out_dir, 'video.mp4')
    csv_path = os.path.join(out_dir, 'frames.csv')

    picam2 = open_camera(args.width, args.height)
    writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'),
                             args.fps, (args.width, args.height))
    if not writer.isOpened():
        print('오류: VideoWriter 를 열 수 없습니다.', file=sys.stderr)
        picam2.close()
        return 1

    stop = {'now': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))

    period = 1.0 / args.fps
    n = 0
    print(f'녹화 시작 → {out_dir}   (Ctrl+C 로 종료)')
    print(f'  {args.width}x{args.height} @ {args.fps}fps, orient={args.orient}')
    try:
        with open(csv_path, 'w', newline='') as fp:
            w = csv.writer(fp)
            w.writerow(['frame', 'stamp_monotonic', 'stamp_unix'])
            next_at = time.monotonic()
            while not stop['now']:
                frame = apply_orient(picam2.capture_array(), args.orient)
                writer.write(frame)
                w.writerow([n, f'{time.monotonic():.6f}', f'{time.time():.6f}'])
                n += 1
                if n % (args.fps * 10) == 0:
                    print(f'  {n} 프레임 ({n / args.fps:.0f}초)')
                next_at += period
                delay = next_at - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_at = time.monotonic()  # 밀렸으면 재동기
    finally:
        writer.release()
        picam2.close()

    print(f'\n완료: {n} 프레임 ({n / args.fps:.1f}초)')
    print(f'  {video_path}')
    print(f'  {csv_path}')
    return 0


def main():
    p = argparse.ArgumentParser(description='핑키 주행 영상 수집기')
    p.add_argument('--out', default=os.path.expanduser('~/pinky_data'),
                   help='저장 루트 (기본: ~/pinky_data)')
    p.add_argument('--name', default='', help='세션 이름 (기본: 날짜시각)')
    p.add_argument('--width', type=int, default=DEFAULT_W)
    p.add_argument('--height', type=int, default=DEFAULT_H)
    p.add_argument('--fps', type=int, default=DEFAULT_FPS)
    p.add_argument('--orient', choices=ORIENTS, default='rot180',
                   help='카메라 장착 방향 보정 (기본: rot180)')
    p.add_argument('--snap', action='store_true',
                   help='보정 방식별로 한 장씩만 찍고 종료')
    args = p.parse_args()
    return snap(args) or 0 if args.snap else record(args)


if __name__ == '__main__':
    sys.exit(main())
