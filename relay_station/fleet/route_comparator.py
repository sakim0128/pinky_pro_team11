#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""경로 비교기 (Route Comparator) — RobotState 위치와 LaneStatus 진행도 상호 검증.

관제국(Relay)에서 다음을 종합 평가한다:
1. 물리적 위치(RobotState x, y) 대 기준 경로(Route.waypoints)의 최근접 인덱스 및 횡오차(cross-track error).
2. 로봇 온보드 논리 진행 상태(LaneStatus route_idx, edge_id)와의 일치성.
3. 이탈(off_route) 및 논리-물리 불일치(ROUTE_STATE_MISMATCH) 진단.
4. 다음 분기(junction)까지의 잔여 거리 및 경로 진행률 산출.
"""

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from pinky_lane_station.road_graph import project_to_polyline


@dataclass
class RouteComparisonResult:
    robot_name: str
    nearest_idx: int
    route_progress: float         # 0.0 ~ 1.0
    progress_meters: float        # s (meters along polyline)
    cross_track_error: float      # meters from centerline
    current_edge: str
    next_junction_idx: Optional[int]
    distance_to_junction: Optional[float]
    off_route: bool
    route_mismatch: bool
    status: str                   # 'OK', 'OFF_ROUTE', 'ROUTE_STATE_MISMATCH', 'UNLOCALIZED'


class RouteComparator:
    def __init__(self, off_route_threshold: float = 0.35, mismatch_idx_threshold: int = 5):
        """
        :param off_route_threshold: 중심선과의 거리가 이 값(m) 초과 시 off_route 판정 (도로폭 0.20m 기준 마진 포함)
        :param mismatch_idx_threshold: 온보드 route_idx 와 물리적 nearest_idx 차이가 이 웨이포인트 수(0.10m 간격) 초과 시 불일치
        """
        self.off_route_threshold = float(off_route_threshold)
        self.mismatch_idx_threshold = int(mismatch_idx_threshold)

    def compare(
        self,
        robot_name: str,
        x: float,
        y: float,
        yaw: float,
        route: Any,
        lane_status: Optional[Any] = None,
        localized: bool = True,
        expected_route_seq: Optional[int] = None
    ) -> RouteComparisonResult:
        """
        :param robot_name: 로봇 식별자 (예: 'pinky1', 'pinky2')
        :param x, y, yaw: RobotState 기반 전역 위치 (map frame)
        :param route: RoadGraph Route 객체 또는 waypoints/edge_ids 속성을 갖는 객체
        :param lane_status: 온보드에서 발행된 pinky_lane_msgs/LaneStatus 객체 (선택)
        :param localized: amcl_pose 등 위치 추정이 유효한지 여부
        """
        if not localized or not math.isfinite(x) or not math.isfinite(y):
            return RouteComparisonResult(
                robot_name=robot_name,
                nearest_idx=0,
                route_progress=0.0,
                progress_meters=0.0,
                cross_track_error=float('inf'),
                current_edge='',
                next_junction_idx=None,
                distance_to_junction=None,
                off_route=False,
                route_mismatch=False,
                status='UNLOCALIZED'
            )

        waypoints = route.waypoints
        if not waypoints:
            return RouteComparisonResult(
                robot_name=robot_name,
                nearest_idx=0,
                route_progress=0.0,
                progress_meters=0.0,
                cross_track_error=0.0,
                current_edge='',
                next_junction_idx=None,
                distance_to_junction=None,
                off_route=False,
                route_mismatch=False,
                status='NO_ROUTE'
            )

        # 1. 중심선 투영 및 횡오차
        s, lateral, qx, qy, dist = project_to_polyline(x, y, waypoints)
        cross_track_error = float(dist)
        off_route = bool(dist > self.off_route_threshold)

        # 2. 최근접 웨이포인트 인덱스
        nearest_idx = 0
        min_d2 = float('inf')
        for i, wp in enumerate(waypoints):
            dx = wp[0] - x
            dy = wp[1] - y
            d2 = dx * dx + dy * dy
            if d2 < min_d2:
                min_d2 = d2
                nearest_idx = i

        # 3. 누적 거리 및 진행률
        total_len = max(float(getattr(route, 'length', 0.0)), 1e-6)
        progress_meters = min(float(s), total_len)
        route_progress = min(max(progress_meters / total_len, 0.0), 1.0)

        # 4. 현재 엣지 판정 (edge_end_idx 활용)
        current_edge = ''
        edge_ids = getattr(route, 'edge_ids', [])
        edge_end_idx = getattr(route, 'edge_end_idx', [])
        for k, end_idx in enumerate(edge_end_idx):
            if nearest_idx <= end_idx:
                if k < len(edge_ids):
                    current_edge = edge_ids[k]
                break
        if not current_edge and edge_ids:
            current_edge = edge_ids[-1]

        # 5. 다음 분기(junction) 인덱스 및 잔여 거리
        junction_idx = getattr(route, 'junction_idx', [])
        next_junction_idx = None
        distance_to_junction = None
        cum = route.cumulative() if hasattr(route, 'cumulative') else None

        for j_idx in junction_idx:
            if j_idx > nearest_idx:
                next_junction_idx = j_idx
                if cum is not None and j_idx < len(cum) and nearest_idx < len(cum):
                    distance_to_junction = max(0.0, cum[j_idx] - cum[nearest_idx])
                break

        # 6. 온보드 LaneStatus 와의 상호 검증 (물리 vs 논리)
        route_mismatch = False
        if lane_status is not None:
            # route_seq 비교: 온보드 route_seq 와 관제 route_seq 가 불일치하면 즉시 mismatch
            exp_seq = expected_route_seq
            if exp_seq is None:
                exp_seq = getattr(route, 'route_seq', None)
            ls_seq = getattr(lane_status, 'route_seq', None)
            if exp_seq is not None and ls_seq is not None and ls_seq != exp_seq:
                route_mismatch = True

            # route_idx 비교 (유효 범위 내일 때)
            ls_idx = getattr(lane_status, 'route_idx', -1)
            if ls_idx >= 0:
                idx_diff = abs(ls_idx - nearest_idx)
                if idx_diff > self.mismatch_idx_threshold:
                    route_mismatch = True

            # edge_id 비교 (비어있지 않을 때)
            ls_edge = getattr(lane_status, 'edge_id', '')
            if ls_edge and current_edge and ls_edge != current_edge:
                # 엣지 전환 부근(노드 경계)에서는 약간의 타이밍 차이 허용
                # 하지만 인덱스 차이도 크다면 확실한 불일치
                if abs(ls_idx - nearest_idx) > 2:
                    route_mismatch = True

        # 7. 상태 판정
        if off_route:
            status = 'OFF_ROUTE'
        elif route_mismatch:
            status = 'ROUTE_STATE_MISMATCH'
        else:
            status = 'OK'

        return RouteComparisonResult(
            robot_name=robot_name,
            nearest_idx=nearest_idx,
            route_progress=round(route_progress, 4),
            progress_meters=round(progress_meters, 3),
            cross_track_error=round(cross_track_error, 4),
            current_edge=current_edge,
            next_junction_idx=next_junction_idx,
            distance_to_junction=round(distance_to_junction, 3) if distance_to_junction is not None else None,
            off_route=off_route,
            route_mismatch=route_mismatch,
            status=status
        )
