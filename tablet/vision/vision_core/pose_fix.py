# -*- coding: utf-8 -*-
"""Canonical PoseFix contract for External Vision Compute.

1:1 semantic correspondence with Team11 PoseFix.msg:
    std_msgs/Header header              # frame_id = map
    bool    stamp_is_robot_clock        # false for overhead (station clock)
    string  robot_name
    uint32  seq
    float64 x
    float64 y
    float64 yaw
    int32   marker_id
    float32 marker_range
    float32 reproj_error
    int32   n_markers
    float32 pipeline_latency
"""

from dataclasses import asdict, dataclass, field
import time
from typing import Any, Dict, Optional


@dataclass
class Header:
    frame_id: str = "map"
    stamp_sec: int = 0
    stamp_nanosec: int = 0

    @classmethod
    def now(cls, frame_id: str = "map") -> "Header":
        return cls.from_timestamp(time.time(), frame_id=frame_id)

    @classmethod
    def from_timestamp(cls, ts: float, frame_id: str = "map") -> "Header":
        sec = int(ts)
        nanosec = int((ts - sec) * 1e9)
        return cls(frame_id=frame_id, stamp_sec=sec, stamp_nanosec=nanosec)

    @property
    def stamp_float(self) -> float:
        return self.stamp_sec + (self.stamp_nanosec * 1e-9)


@dataclass
class PoseFix:
    """Canonical representation of vision-derived robot pose measurement."""
    robot_name: str
    x: float
    y: float
    yaw: float
    seq: int = 0
    header: Header = field(default_factory=lambda: Header.now("map"))
    stamp_is_robot_clock: bool = False  # Overhead uses station/tablet clock
    marker_id: int = 0
    marker_range: float = 0.0           # 0.0 for overhead camera
    reproj_error: float = 0.0           # Reprojection error in pixels
    n_markers: int = 0                  # Visible reference markers count
    pipeline_latency: float = 0.0       # Processing latency in seconds

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary matching PoseFix.msg field names."""
        return {
            "header": {
                "frame_id": self.header.frame_id,
                "stamp": {
                    "sec": self.header.stamp_sec,
                    "nanosec": self.header.stamp_nanosec,
                },
            },
            "stamp_is_robot_clock": self.stamp_is_robot_clock,
            "robot_name": self.robot_name,
            "seq": self.seq,
            "x": round(float(self.x), 5),
            "y": round(float(self.y), 5),
            "yaw": round(float(self.yaw), 5),
            "marker_id": int(self.marker_id),
            "marker_range": float(self.marker_range),
            "reproj_error": round(float(self.reproj_error), 3),
            "n_markers": int(self.n_markers),
            "pipeline_latency": round(float(self.pipeline_latency), 4),
        }

    def to_legacy_dict(self) -> Dict[str, Any]:
        """Convert to legacy JSON payload for POST /api/vision/pose (backward compatibility)."""
        return {
            "robotId": self.robot_name,
            "x": round(float(self.x), 4),
            "y": round(float(self.y), 4),
            "yaw": round(float(self.yaw), 4),
            "computedAtMs": int(self.header.stamp_float * 1000),
            "markerId": int(self.marker_id),
            "reprojError": round(float(self.reproj_error), 3),
            "nMarkers": int(self.n_markers),
            "seq": self.seq,
            "pipelineLatency": round(float(self.pipeline_latency), 4),
        }

    def to_relay_dict(self) -> Dict[str, Any]:
        """Alias for to_legacy_dict() for backward compatibility."""
        return self.to_legacy_dict()

    def to_ros_msg(self) -> Optional[Any]:
        """Convert to pinky_lane_msgs.msg.PoseFix if installed, else None."""
        try:
            from pinky_lane_msgs.msg import PoseFix as RosPoseFix  # type: ignore
            msg = RosPoseFix()
            msg.header.frame_id = self.header.frame_id
            msg.header.stamp.sec = self.header.stamp_sec
            msg.header.stamp.nanosec = self.header.stamp_nanosec
            msg.stamp_is_robot_clock = self.stamp_is_robot_clock
            msg.robot_name = self.robot_name
            msg.seq = self.seq
            msg.x = float(self.x)
            msg.y = float(self.y)
            msg.yaw = float(self.yaw)
            msg.marker_id = int(self.marker_id)
            msg.marker_range = float(self.marker_range)
            msg.reproj_error = float(self.reproj_error)
            msg.n_markers = int(self.n_markers)
            msg.pipeline_latency = float(self.pipeline_latency)
            return msg
        except ImportError:
            return None
