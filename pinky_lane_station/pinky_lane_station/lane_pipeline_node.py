#!/usr/bin/env python3
"""관제 PC 인식 파이프라인 — 로봇 카메라 → 검출기 → 차선 목표 → LanePath / SceneState.

로봇마다 estimator 를 따로 두고 검출기는 하나를 공유한다 (순차 추론). 이미지 QoS 가
BEST_EFFORT depth 1 이라 추론이 느리면 낡은 프레임은 자동으로 버려진다.

시각 규칙: LanePath.source_stamp 는 이미지 header.stamp 를 **복사만** 한다 (로봇 시계).
header.stamp 는 관제 시계 — 진단용. pipeline_latency 는 관제 내부 수신→발행 시간.
"""

import os

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from pinky_lane_msgs.msg import LanePath, SceneState

from .detectors import create_detector
from .lane_mission import LaneMissionError, load_lane_mission
from .lane_target import QUALITY_STALE, LaneTargetEstimator, TargetParams
from .pipeline_image import draw_debug, mask_top

try:
    import cv2
except ImportError:              # pragma: no cover
    cv2 = None

BEST_EFFORT_1 = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                           reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           durability=QoSDurabilityPolicy.VOLATILE)


def load_detector_config(path):
    path = os.path.expandvars(os.path.expanduser(str(path)))
    with open(path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    det = dict(data.get('detector') or {'kind': 'stub'})
    if 'model' in det:
        det['model'] = os.path.expandvars(os.path.expanduser(str(det['model'])))
    target = TargetParams(**{k: v for k, v in (data.get('target') or {}).items()
                             if k in TargetParams.__dataclass_fields__})
    pipeline = {'max_rate': 10.0, 'stale_period': 0.3, 'stale_max_seconds': 2.0, 'warmup': True,
                'mask_top_frac': 0.0, 'mask_fill': 0, 'debug_polygons': False}
    pipeline.update(data.get('pipeline') or {})
    return det, target, pipeline


class RobotLane:
    def __init__(self, spec, target_params):
        self.spec = spec
        self.name = spec['name']
        self.est = LaneTargetEstimator(target_params)
        self.seq = 0
        self.last_msg = None            # 마지막으로 보낸 LanePath (STALE 재발행용)
        self.last_frame_time = None     # 관제 시계
        self.last_pub_time = None
        self.fps_t = []


class LanePipeline(Node):

    def __init__(self):
        super().__init__('lane_pipeline')
        self.declare_parameter('mission', '')
        self.declare_parameter('detector_config', '')
        self.declare_parameter('publish_debug_image', False)

        mission = load_lane_mission(self.get_parameter('mission').value)
        det_cfg, target_params, pipe = load_detector_config(
            self.get_parameter('detector_config').value)
        self._max_period = 1.0 / float(pipe['max_rate']) if float(pipe['max_rate']) > 0 else 0.0
        self._stale_period = float(pipe['stale_period'])
        self._stale_max = float(pipe['stale_max_seconds'])

        self._stop_row_frac = float(target_params.crosswalk_stop_row_frac)
        self._mask_frac = float(pipe['mask_top_frac'])          # 모델 학습과 같은 상위 마스킹
        self._mask_fill = int(pipe['mask_fill'])
        self._debug_polygons = bool(pipe['debug_polygons'])
        self.detector = create_detector(det_cfg)
        if pipe.get('warmup', True):
            self.detector.warmup()
        self._det_name = f'lane={self.detector.name}'

        self._robots = {}
        self._path_pubs = {}
        self._scene_pubs = {}
        self._debug_pubs = {}
        debug = bool(self.get_parameter('publish_debug_image').value)
        for spec in mission.robots:
            rl = RobotLane(spec, target_params)
            self._robots[rl.name] = rl
            self._path_pubs[rl.name] = self.create_publisher(LanePath, spec['lane_path_topic'],
                                                             BEST_EFFORT_1)
            self._scene_pubs[rl.name] = self.create_publisher(
                SceneState, f"/{rl.name}/scene_state", 10)
            if debug:
                self._debug_pubs[rl.name] = self.create_publisher(
                    CompressedImage, f"/{rl.name}/lane_debug/compressed", BEST_EFFORT_1)
            self.create_subscription(CompressedImage, spec['image_topic'],
                                     lambda msg, n=rl.name: self._on_image(n, msg), BEST_EFFORT_1)
        self.create_timer(self._stale_period, self._stale_tick)
        self.get_logger().info(
            f'lane_pipeline 시작: detector={self._det_name} robots={list(self._robots)} '
            f'max_rate={pipe["max_rate"]} stale={self._stale_period}s/{self._stale_max}s '
            f'mask_top={self._mask_frac:.2f}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ 프레임

    def _on_image(self, name, msg: CompressedImage):
        rl = self._robots[name]
        t_in = self._now()
        if rl.last_frame_time is not None and self._max_period > 0 \
                and (t_in - rl.last_frame_time) < self._max_period * 0.9:
            return
        rl.last_frame_time = t_in
        img = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f'{name}: 이미지 디코드 실패')
            return
        H, W = img.shape[:2]
        # 추론 입력만 마스킹한다. 오버레이·저장은 원본(img) 그대로
        masked = mask_top(img, self._mask_frac, self._mask_fill) if self._mask_frac > 0 else img
        instances, infer_ms = self.detector.infer_timed(masked)
        r = rl.est.update(instances, W, H)

        rl.seq += 1
        lp = LanePath()
        lp.header.stamp = self.get_clock().now().to_msg()
        lp.header.frame_id = 'camera'
        lp.source_stamp = msg.header.stamp          # 복사만
        lp.robot_name = name
        lp.seq = rl.seq
        lp.quality = int(r.quality)
        lp.error_x_norm = float(r.error_x)
        lp.image_width, lp.image_height = W, H
        lp.target_x, lp.target_y = int(r.target_x), int(r.target_y)
        lp.left_x, lp.right_x = int(r.left_x), int(r.right_x)
        lp.left_seen, lp.right_seen = bool(r.left_seen), bool(r.right_seen)
        lp.half_lane_px = float(r.half_lane_px)
        lp.confidence = float(r.confidence)
        lp.crosswalk_detected = bool(r.crosswalk_detected)
        lp.crosswalk_bottom_y = int(r.crosswalk_bottom_y)
        lp.barricade_detected = bool(r.barricade_detected)
        lp.barricade_bottom_y = int(r.barricade_bottom_y)
        lp.stop_line_detected = bool(r.stop_line_detected)
        lp.stop_line_bottom_y = int(r.stop_line_bottom_y)
        lp.pipeline_latency = float(self._now() - t_in)
        self._path_pubs[name].publish(lp)
        rl.last_msg = lp
        rl.last_pub_time = self._now()

        rl.fps_t = [t for t in rl.fps_t if t_in - t < 2.0] + [t_in]
        sc = SceneState()
        sc.header.stamp = lp.header.stamp
        sc.source_stamp = msg.header.stamp
        sc.robot_name = name
        sc.crosswalk_raw = bool(r.crosswalk_raw)
        sc.crosswalk_detected = bool(r.crosswalk_detected)
        sc.crosswalk_bottom_y = int(r.crosswalk_bottom_y)
        sc.crosswalk_confidence = float(r.crosswalk_confidence)
        sc.lane_count = int(r.lane_count)
        sc.left_count, sc.right_count = int(r.left_count), int(r.right_count)
        sc.pair_rule = str(r.pair_rule)
        sc.barricade_raw = bool(r.barricade_raw)
        sc.barricade_detected = bool(r.barricade_detected)
        sc.barricade_bottom_y = int(r.barricade_bottom_y)
        sc.barricade_confidence = float(r.barricade_confidence)
        sc.stop_line_raw = bool(r.stop_line_raw)
        sc.stop_line_detected = bool(r.stop_line_detected)
        sc.stop_line_bottom_y = int(r.stop_line_bottom_y)
        sc.stop_line_confidence = float(r.stop_line_confidence)
        sc.detector_name = self._det_name
        sc.infer_ms = float(infer_ms)
        sc.fps = len(rl.fps_t) / 2.0
        self._scene_pubs[name].publish(sc)

        if name in self._debug_pubs:
            self._publish_debug(name, img, instances, r, msg.header.stamp, infer_ms)

    def _publish_debug(self, name, img, instances, r, stamp, infer_ms=0.0):
        dbg = draw_debug(img, instances, r, mask_frac=self._mask_frac, infer_ms=infer_ms,
                         stop_row_frac=self._stop_row_frac, draw_polygons=self._debug_polygons)
        ok, buf = cv2.imencode('.jpg', dbg, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        if not ok:
            return
        out = CompressedImage()
        out.header.stamp = stamp
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self._debug_pubs[name].publish(out)

    # ------------------------------------------------------------------ STALE

    def _stale_tick(self):
        """새 프레임이 없을 때 직전 값을 STALE 로 재발행한다 (stale_max 까지만).

        그 뒤에는 침묵한다 — 로봇의 path_timeout 이 LINK_LOST 로 세운다. 카메라가 죽었는데
        맵만 믿고 계속 달리게 두지 않는다.
        """
        now = self._now()
        for name, rl in self._robots.items():
            if rl.last_msg is None or rl.last_frame_time is None:
                continue
            since_frame = now - rl.last_frame_time
            if since_frame < self._stale_period or since_frame > self._stale_max:
                continue
            lp = LanePath()
            lp.header.stamp = self.get_clock().now().to_msg()
            lp.source_stamp = rl.last_msg.source_stamp      # 원본 stamp 그대로 (로봇이 age 로 안다)
            lp.robot_name = name
            lp.seq = rl.last_msg.seq
            lp.quality = QUALITY_STALE
            lp.error_x_norm = rl.last_msg.error_x_norm
            lp.image_width, lp.image_height = rl.last_msg.image_width, rl.last_msg.image_height
            lp.half_lane_px = rl.last_msg.half_lane_px
            lp.crosswalk_detected = False
            self._path_pubs[name].publish(lp)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LanePipeline()
    except (LaneMissionError, FileNotFoundError, RuntimeError) as exc:
        print(f'[lane_pipeline] 시작 실패: {exc}')
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.detector.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
