#!/usr/bin/env python3
"""검출기 추론 벤치 — 관제 PC 에서 CPU / GPU 를 결정한다 (ROS 불필요).

    python3 -m pinky_lane_station.bench_detector --config config/detector_lane.yaml \
        --images ~/drive_data/frames --n 100 [--device cpu|cuda:0] [--imgsz 416]
    ros2 run pinky_lane_station bench_detector -- --config ... --synthetic

이미지 디렉터리가 없으면 --synthetic 으로 합성 프레임을 쓴다. p50 / p95 / fps 를 표로 낸다.
목표: 파이프라인 p95 < 100 ms (0.3 s 주기의 1/3), 아니면 imgsz 를 내리거나 GPU 로.
"""

import argparse
import glob
import os
import sys

import numpy as np
import yaml

from .detectors import create_detector
from .lane_target import LaneTargetEstimator, TargetParams


def load_images(path, n):
    files = sorted(glob.glob(os.path.join(os.path.expanduser(path), '*.jpg')) +
                   glob.glob(os.path.join(os.path.expanduser(path), '*.png')))
    import cv2
    out = []
    for f in files[:n]:
        img = cv2.imread(f)
        if img is not None:
            out.append(img)
    return out


def synthetic_images(n):
    from .synthetic_camera import render_lane_frame
    return [render_lane_frame(lateral=0.04 * np.sin(i / 7.0), heading=0.1 * np.cos(i / 11.0),
                              crosswalk_ahead=(0.3 if i % 20 < 5 else None), noise=6)
            for i in range(n)]


def run(detector, images, warm=3):
    est = LaneTargetEstimator(TargetParams())
    for img in images[:warm]:
        detector.infer(img)
    times, qualities = [], []
    for img in images:
        inst, ms = detector.infer_timed(img)
        r = est.update(inst, img.shape[1], img.shape[0])
        times.append(ms)
        qualities.append(r.quality_name)
    t = np.array(times)
    return {'n': len(t), 'p50_ms': float(np.percentile(t, 50)), 'p95_ms': float(np.percentile(t, 95)),
            'max_ms': float(t.max()), 'fps': 1000.0 / float(t.mean()),
            'both': qualities.count('BOTH'), 'single': qualities.count('SINGLE'),
            'lost': qualities.count('LOST')}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', help='detector_lane.yaml')
    ap.add_argument('--kind', help='config 대신 kind 직접 지정 (classic|stub|ultralytics)')
    ap.add_argument('--model', help='ultralytics 모델 경로')
    ap.add_argument('--device', action='append', help='여러 번 주면 각각 벤치 (cpu, cuda:0)')
    ap.add_argument('--imgsz', type=int, action='append')
    ap.add_argument('--images', help='프레임 디렉터리 (jpg/png)')
    ap.add_argument('--synthetic', action='store_true')
    ap.add_argument('--n', type=int, default=100)
    args = ap.parse_args(argv)

    base = {'kind': 'classic'}
    if args.config:
        with open(os.path.expanduser(args.config), encoding='utf-8') as fh:
            base = dict((yaml.safe_load(fh) or {}).get('detector') or base)
    if args.kind:
        base['kind'] = args.kind
    if args.model:
        base['model'] = args.model
    if 'model' in base:
        base['model'] = os.path.expandvars(os.path.expanduser(str(base['model'])))

    images = load_images(args.images, args.n) if args.images and not args.synthetic else []
    if not images:
        images = synthetic_images(args.n)
        print(f'합성 프레임 {len(images)}장 사용')
    else:
        print(f'{args.images}: {len(images)}장')

    devices = args.device or [base.get('device', 'cpu')]
    sizes = args.imgsz or [base.get('imgsz', 416)]
    rows = []
    for dev in devices:
        for sz in sizes:
            spec = dict(base)
            if spec['kind'] in ('ultralytics', 'yolo'):
                spec['device'], spec['imgsz'] = dev, sz
            try:
                det = create_detector(spec)
            except Exception as exc:  # noqa: BLE001
                print(f'  {spec["kind"]} device={dev} imgsz={sz}: 실패 — {exc}')
                continue
            r = run(det, images)
            det.close()
            rows.append((det.name, dev, sz, r))
            if spec['kind'] not in ('ultralytics', 'yolo'):
                break
        else:
            continue
        break
    print()
    print(f'{"detector":34s} {"device":8s} {"imgsz":>5s} {"p50":>8s} {"p95":>8s} {"max":>8s} {"fps":>6s}  BOTH/SINGLE/LOST')
    for name, dev, sz, r in rows:
        print(f'{name[:34]:34s} {dev:8s} {sz:5d} {r["p50_ms"]:8.1f} {r["p95_ms"]:8.1f} {r["max_ms"]:8.1f} '
              f'{r["fps"]:6.1f}  {r["both"]}/{r["single"]}/{r["lost"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
