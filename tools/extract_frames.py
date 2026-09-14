#!/usr/bin/env python3
"""주행 영상에서 라벨링용 프레임을 추출한다 (PC 에서 실행).

연속 프레임은 거의 같은 그림이라 전부 라벨링하면 낭비다. --every 로
솎아내고, --blur-thresh 로 흔들린 프레임을 버린다.

    python3 extract_frames.py ~/pinky_data/course_a --every 10
    python3 extract_frames.py ~/pinky_data/course_a --every 10 --max 60
"""
import argparse
import os
import sys

import cv2


def sharpness(frame):
    """라플라시안 분산. 낮을수록 흐릿하다."""
    return cv2.Laplacian(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()


def main():
    p = argparse.ArgumentParser(description='주행 영상 → 라벨링용 프레임')
    p.add_argument('session', help='record_drive.py 가 만든 세션 폴더 또는 mp4 경로')
    p.add_argument('--every', type=int, default=10, help='N 프레임마다 한 장 (기본 10)')
    p.add_argument('--max', type=int, default=0, help='최대 장수 (0 이면 무제한)')
    p.add_argument('--blur-thresh', type=float, default=40.0,
                   help='이 값 미만이면 흔들린 것으로 보고 버린다 (0 이면 끔)')
    p.add_argument('--out', default='', help='출력 폴더 (기본: <세션>/frames)')
    args = p.parse_args()

    video = args.session
    if os.path.isdir(video):
        video = os.path.join(args.session, 'video.mp4')
    if not os.path.isfile(video):
        print(f'오류: 영상을 찾을 수 없습니다 -> {video}', file=sys.stderr)
        return 1

    out_dir = args.out or os.path.join(os.path.dirname(video), 'frames')
    os.makedirs(out_dir, exist_ok=True)

    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        print(f'오류: 영상을 열 수 없습니다 -> {video}', file=sys.stderr)
        return 1

    stem = os.path.basename(os.path.dirname(video)) or 'frame'
    idx = kept = blurred = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.every == 0:
            if args.blur_thresh > 0 and sharpness(frame) < args.blur_thresh:
                blurred += 1
            else:
                path = os.path.join(out_dir, f'{stem}_{idx:06d}.jpg')
                cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                kept += 1
                if args.max and kept >= args.max:
                    break
        idx += 1
    cap.release()

    print(f'총 {idx} 프레임 중 {kept} 장 추출 (흐려서 제외 {blurred} 장)')
    print(f'  → {out_dir}')
    print('\n이 폴더를 Roboflow 에 업로드하고 Polygon Tool 로 left / right 를 그리세요.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
