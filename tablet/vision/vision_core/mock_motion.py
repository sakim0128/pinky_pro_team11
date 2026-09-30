# -*- coding: utf-8 -*-
"""Moving mock poses along the Relay road graph (T-7).

The synthetic camera used to render robots at fixed poses (0,0,0 / 0.5,-0.2,1.57) that
matched no arena node, so a robot driven by Nav2 was pulled back to its start every
second by pose_fuser and goal arrival could never be judged. This module places the
mock robots on `road_graph.yaml` nodes and advances them along a node path at a fixed
speed. Only the *rendered* pose moves: render -> detect -> PoseFix stays the same.

⚠️ All road-graph coordinates are FIELD_CONFIG_PENDING placeholders. Mock output is
   synthetic and must never be stored as ground truth.
"""

import math
import os
from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

Pose = Tuple[float, float, float]

_candidate_graphs = [
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "relay_station", "fleet", "config", "road_graph.yaml")),
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "pinky_lane_station", "config", "road_graph.yaml")),
]
DEFAULT_ROAD_GRAPH = next((p for p in _candidate_graphs if os.path.exists(p)), _candidate_graphs[0])

# Robot -> start node, per lane_mission.yaml (pinky1 START_A, pinky2 START_B).
DEFAULT_START_NODES = {"pinky1": "START_A", "pinky2": "START_B"}


class RoadGraph:
    """Nodes and edge polylines from road_graph.yaml."""

    def __init__(self, nodes: Dict[str, Tuple[float, float]],
                 edges: List[Tuple[str, str, List[Tuple[float, float]], bool]]):
        self.nodes = nodes
        self.edges = edges

    @classmethod
    def load(cls, path: str = DEFAULT_ROAD_GRAPH) -> "RoadGraph":
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        nodes = {n["id"]: (float(n["x"]), float(n["y"])) for n in data.get("nodes", [])}
        edges = []
        for e in data.get("edges", []):
            wps = [(float(p[0]), float(p[1])) for p in e.get("waypoints", [])]
            if not wps:
                wps = [nodes[e["from"]], nodes[e["to"]]]
            edges.append((e["from"], e["to"], wps, bool(e.get("oneway", False))))
        return cls(nodes, edges)

    def path(self, node_ids: Sequence[str]) -> List[Tuple[float, float]]:
        """Polyline through the given nodes, each hop routed by BFS over edges."""
        if len(node_ids) < 2:
            raise ValueError("mock path needs at least two nodes")
        for nid in node_ids:
            if nid not in self.nodes:
                raise KeyError(f"unknown road-graph node: {nid}")
        out: List[Tuple[float, float]] = []
        for a, b in zip(node_ids, node_ids[1:]):
            for pts in self._route(a, b):
                out.extend(pts if not out else [p for p in pts if p != out[-1]])
        return out

    def _route(self, a: str, b: str) -> List[List[Tuple[float, float]]]:
        adj: Dict[str, List[Tuple[str, List[Tuple[float, float]]]]] = {}
        for frm, to, wps, oneway in self.edges:
            adj.setdefault(frm, []).append((to, wps))
            if not oneway:
                adj.setdefault(to, []).append((frm, list(reversed(wps))))
        prev: Dict[str, Optional[Tuple[str, List[Tuple[float, float]]]]] = {a: None}
        q = deque([a])
        while q:
            cur = q.popleft()
            if cur == b:
                break
            for nxt, wps in adj.get(cur, []):
                if nxt not in prev:
                    prev[nxt] = (cur, wps)
                    q.append(nxt)
        if b not in prev:
            raise ValueError(f"no road-graph route {a} -> {b}")
        hops = []
        cur = b
        while prev[cur] is not None:
            parent, wps = prev[cur]
            hops.append(wps)
            cur = parent
        return list(reversed(hops))


class PathMotion:
    """Constant-speed motion along a polyline; yaw follows the segment heading."""

    def __init__(self, polyline: Sequence[Tuple[float, float]], speed_m_s: float):
        if len(polyline) < 2:
            raise ValueError("polyline needs at least two points")
        if speed_m_s < 0:
            raise ValueError("speed must be >= 0")
        self.pts = [tuple(map(float, p)) for p in polyline]
        self.speed = float(speed_m_s)
        self.seg_len = [math.dist(p, q) for p, q in zip(self.pts, self.pts[1:])]
        self.length = sum(self.seg_len)

    def pose_at(self, t_s: float) -> Pose:
        """Pose after t_s seconds; holds the final pose once the path is done."""
        s = min(max(t_s, 0.0) * self.speed, self.length)
        for (p, q), L in zip(zip(self.pts, self.pts[1:]), self.seg_len):
            if L <= 0:
                continue
            yaw = math.atan2(q[1] - p[1], q[0] - p[0])
            if s <= L:
                f = s / L
                return (p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]), yaw)
            s -= L
        p, q = self.pts[-2], self.pts[-1]
        return (q[0], q[1], math.atan2(q[1] - p[1], q[0] - p[0]))

    def done(self, t_s: float) -> bool:
        return t_s * self.speed >= self.length


class MockPoses:
    """Mock robot poses: static at start nodes, pinky1 optionally moving along a path."""

    def __init__(self, graph: RoadGraph, path_nodes: Optional[Sequence[str]] = None,
                 speed_m_s: float = 0.0, moving_robot: str = "pinky1",
                 start_nodes: Optional[Dict[str, str]] = None):
        self.static: Dict[str, Pose] = {
            name: (*graph.nodes[nid], 0.0)
            for name, nid in (start_nodes or DEFAULT_START_NODES).items()
        }
        self.moving_robot = moving_robot
        self.motion: Optional[PathMotion] = None
        if speed_m_s > 0:
            self.motion = PathMotion(graph.path(path_nodes or ["START_A", "GOAL_C"]), speed_m_s)

    def poses(self, t_s: float) -> Dict[str, Pose]:
        out = dict(self.static)
        if self.motion is not None:
            out[self.moving_robot] = self.motion.pose_at(t_s)
        return out
