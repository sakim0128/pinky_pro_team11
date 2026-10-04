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

from .aruco_detector import ArucoMarkerDetector
from .aruco_detector import params_from_dict as aruco_params
from .detectors import create_detector
from .ground_bev import BevLaneFollower, BevParams, GroundCamera, apply_bev, draw_bev, lane_mask_from_instances
from .lane_mission import LaneMissionError, load_lane_mission
from .lane_target import LANE_CLASSES, QUALITY_STALE, LaneTargetEstimator, TargetParams
from .pipeline_image import draw_debug, mask_top
from .red_line_detector import RedLineColorDetector
from .red_line_detector import params_from_dict as red_line_params
from .stop_line_detector import WhiteStopLineDetector
from .stop_line_detector import params_from_dict as stop_line_params

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
    pipeline['stop_line'] = stop_line_params(data.get('stop_line'))
    pipeline['aruco'] = aruco_params(data.get('aruco'))
    pipeline['red_line_color'] = red_line_params(data.get('red_line_color'))
    bev = BevParams(**{k: v for k, v in (data.get('bev') or {}).items() if k in BevParams.__dataclass_fields__})
    calib = os.path.expandvars(os.path.expanduser(str(bev.calib)))
    bev.calib = calib if os.path.isabs(calib) else os.path.join(os.path.dirname(os.path.abspath(path)), calib)
    pipeline['bev'] = bev
    return det, target, pipeline


class RobotLane:
    def __init__(self, spec, target_params, stop_params=None, aruco_params=None, bev=None):
        self.spec = spec
        self.name = spec['name']
        self.est = LaneTargetEstimator(target_params)
        self.bev = bev                  # BevLaneFollower (bev.enabled) — 로봇마다 회전 방향 hint 를 따로 기억
        self.last_bev = None
        self.stop_line = WhiteStopLineDetector(stop_params)
        self.aruco = ArucoMarkerDetector(aruco_params)
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
        self.red_line = RedLineColorDetector(pipe['red_line_color'])   # 상태 없음 — 로봇끼리 공유
        self._mask_fill = int(pipe['mask_fill'])
        self._debug_polygons = bool(pipe['debug_polygons'])
        self.detector = create_detector(det_cfg)
        bp = pipe['bev']
        self._bev_cam = GroundCamera.from_yaml(bp.calib) if bp.enabled else None   # 없으면 시작 실패 (FileNotFoundError)
        self._bev_params = bp
        self.get_logger().info(
            '차선 중앙: ' + (f'BEV 바닥 좌표 (calib={bp.calib}, Ld={bp.lookahead_mm:.0f} mm, 차선 폭 {bp.lane_width_mm:.0f} mm, '
                            f'구동축 {bp.axle_mm:.0f} mm)' if bp.enabled else '영상 샘플 행 (lane_target)'))
        if pipe.get('warmup', True):
            self.detector.warmup()
        self._det_name = f'lane={self.detector.name}'
        names = getattr(self.detector, 'names', None)
        if names:
            self.get_logger().info(f'모델 클래스: {names} → 우리 클래스: {getattr(self.detector, "class_map", {})}')
        for cls in getattr(self.detector, 'unmatched', []):
            self.get_logger().warn(f"class_map 의 '{cls}' 가 모델 클래스와 맞지 않아 검출되지 않는다")
        rc = pipe['red_line_color']
        self.get_logger().info(f"빨간 선: 색 검출 {'켬' if rc.enabled else '끔'} (H≤{rc.h_low_max}|≥{rc.h_high_min}, "
                               f"S≥{rc.s_min}, V≥{rc.v_min}, ROI {rc.roi_top_frac:.2f}·H~)")

        self._robots = {}
        self._path_pubs = {}
        self._scene_pubs = {}
        self._debug_pubs = {}
        debug = bool(self.get_parameter('publish_debug_image').value)
        for spec in mission.robots:
            bev = BevLaneFollower(self._bev_cam, self._bev_params) if self._bev_cam is not None else None
            rl = RobotLane(spec, target_params, pipe['stop_line'], pipe['aruco'], bev)
            if pipe['aruco'].enabled and not rl.aruco.available:
                self.get_logger().error('ArUco 검출 불가 — cv2.aruco 가 없다 (opencv-contrib). 비전 미션 도착 판정이 안 된다')
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
        # 교차로 빨간 테이프 — 모델 클래스가 아니라 색(HSV)으로, 원본 영상에서. lane_target 이 폭·하단 행·확정 프레임을 본다
        instances = list(instances) + self.red_line.detect(img)
        r = rl.est.update(instances, W, H)
        b = None
        if rl.bev is not None:
            b = rl.bev.update(lane_mask_from_instances(instances, img.shape, LANE_CLASSES), img)
            apply_bev(r, b, self._bev_cam, W, H)
        rl.last_bev = b
        sl = rl.stop_line.update(img)            # 목적지 흰 정지선 — 원본(마스킹 전) 영상에서
        mk = rl.aruco.update(img)                # 도착 지점 벽 ArUco — 원본 영상에서 (마스킹된 위쪽에 있다)

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
        lp.red_line_detected = bool(r.red_line_detected)
        lp.red_line_bottom_y = int(r.red_line_bottom_y)
        lp.red_line_xs = [int(b[0]) for b in r.red_line_blobs]
        lp.red_line_ys = [int(b[1]) for b in r.red_line_blobs]
        lp.red_line_widths = [float(b[2]) for b in r.red_line_blobs]
        lp.stop_line_detected = bool(sl.detected)
        lp.stop_line_bottom_y = int(sl.bottom_y)
        lp.marker_ids = [int(i) for i in mk.markers]
        lp.marker_distances = [float(d) for d in mk.markers.values()]
        if b is not None and b.valid:
            fx, fy = b.robot_target_m()
            lp.floor_valid = True
            lp.floor_x, lp.floor_y = float(fx), float(fy)
            lp.floor_curvature = float(b.robot_curvature())
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
        sc.red_line_raw = bool(r.red_line_raw)
        sc.red_line_detected = bool(r.red_line_detected)
        sc.red_line_bottom_y = int(r.red_line_bottom_y)
        sc.red_line_confidence = float(r.red_line_confidence)
        sc.stop_line_raw = bool(sl.raw)
        sc.stop_line_detected = bool(sl.detected)
        sc.stop_line_bottom_y = int(sl.bottom_y)
        sc.stop_line_width_frac = float(sl.width_frac)
        sc.detector_name = self._det_name
        sc.infer_ms = float(infer_ms)
        sc.fps = len(rl.fps_t) / 2.0
        self._scene_pubs[name].publish(sc)

        if name in self._debug_pubs:
            self._publish_debug(name, img, instances, r, msg.header.stamp, infer_ms, sl, mk, b)

    def _publish_debug(self, name, img, instances, r, stamp, infer_ms=0.0, stop_line=None, aruco=None, bev=None):
        dbg = draw_debug(img, instances, r, mask_frac=self._mask_frac, infer_ms=infer_ms,
                         stop_row_frac=self._stop_row_frac, draw_polygons=self._debug_polygons,
                         stop_line=stop_line, aruco=aruco)
        if bev is not None:                       # 바닥 경로를 영상에 투영: 빨강 = 차로 중앙 경로 · 원 = 목표점, 자홍 = 차선
            draw_bev(dbg, self._bev_cam, bev)
            cv2.putText(dbg, f'BEV {bev.mode}' + (f' k={bev.robot_curvature():+.1f}/m' if bev.valid else ''),
                        (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
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
            # red_line_xs 는 비워 둔다 — 새 프레임이 없으니 seek 동작은 '안 보임' 으로 다룬다
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
