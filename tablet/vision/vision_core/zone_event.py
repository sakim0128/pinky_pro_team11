# -*- coding: utf-8 -*-
"""Start / Goal Camera Vision Zone Event Generator (Phase 6).

Responsible for:
    Camera Frame -> Robot/Marker check -> VisionZoneEvent (PRESENT / ENTER / EXIT)

Includes:
    - Debounce (frame-based thresholding)
    - Duplicate suppression
    - Clean event contract with timestamps, sequence, and confidence.
Does NOT modify Fleet state directly (Relay consumes as mission evidence).
"""

from dataclasses import dataclass, field
import time
from typing import Any, Dict, List, Optional, Set


@dataclass
class VisionZoneEvent:
    """Canonical event emitted when a robot enters, exits, or is present in a designated zone."""
    robot_name: str
    camera_id: str
    zone_id: str
    event_type: str        # 'ENTER', 'PRESENT', 'EXIT'
    confidence: float
    timestamp: float
    sequence: int

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robot_name": self.robot_name,
            "camera_id": self.camera_id,
            "zone_id": self.zone_id,
            "event_type": self.event_type,
            "confidence": round(float(self.confidence), 3),
            "timestamp": round(float(self.timestamp), 4),
            "sequence": self.sequence,
        }


@dataclass
class ZoneConfig:
    """Configuration for a single monitored zone."""
    zone_id: str
    camera_id: str
    target_markers: Dict[int, str] = field(default_factory=lambda: {30: "pinky1", 31: "pinky2"})
    enter_debounce_frames: int = 3
    exit_debounce_frames: int = 4
    min_confidence: float = 0.8


class ZoneDetector:
    """Tracks presence of robots in monitored zones with debounce and duplicate suppression."""

    def __init__(self, configs: List[ZoneConfig]):
        self.configs = {c.zone_id: c for c in configs}
        # State tracking: (zone_id, robot_name) -> state dict
        self._states: Dict[tuple, Dict[str, Any]] = {}
        self._seq = 0

        for zone_id, cfg in self.configs.items():
            for robot_name in cfg.target_markers.values():
                key = (zone_id, robot_name)
                self._states[key] = {
                    "seen_count": 0,
                    "miss_count": 0,
                    "is_present": False,
                    "last_event_type": None,
                }

    def update(
        self,
        camera_id: str,
        detected_marker_ids: Set[int],
        timestamp: Optional[float] = None,
        confidence: float = 1.0,
    ) -> List[VisionZoneEvent]:
        """Update zone tracking with detected markers for a specific camera frame.

        Args:
            camera_id: Identifier of camera (e.g. 'START_A', 'START_B', 'GOAL')
            detected_marker_ids: Set of ArUco marker IDs detected in this frame
            timestamp: Timestamp in seconds (defaults to current time)
            confidence: Detection confidence (0.0 to 1.0)

        Returns:
            List of emitted VisionZoneEvents (if any transition occurred).
        """
        if timestamp is None:
            timestamp = time.time()

        emitted: List[VisionZoneEvent] = []

        for zone_id, cfg in self.configs.items():
            if cfg.camera_id != camera_id:
                continue

            for mid, robot_name in cfg.target_markers.items():
                key = (zone_id, robot_name)
                st = self._states[key]
                # Positive detection requires both marker match and confidence >= min_confidence
                saw_robot = (mid in detected_marker_ids) and (confidence >= cfg.min_confidence)

                if saw_robot:
                    st["seen_count"] += 1
                    st["miss_count"] = 0
                else:
                    st["miss_count"] += 1
                    st["seen_count"] = 0

                # Check for ENTER transition
                if not st["is_present"]:
                    if st["seen_count"] >= cfg.enter_debounce_frames:
                        st["is_present"] = True
                        if st["last_event_type"] != "ENTER":
                            self._seq += 1
                            st["last_event_type"] = "ENTER"
                            emitted.append(
                                VisionZoneEvent(
                                    robot_name=robot_name,
                                    camera_id=camera_id,
                                    zone_id=zone_id,
                                    event_type="ENTER",
                                    confidence=confidence,
                                    timestamp=timestamp,
                                    sequence=self._seq,
                                )
                            )
                # Check for EXIT transition
                else:
                    if st["miss_count"] >= cfg.exit_debounce_frames:
                        st["is_present"] = False
                        if st["last_event_type"] != "EXIT":
                            self._seq += 1
                            st["last_event_type"] = "EXIT"
                            emitted.append(
                                VisionZoneEvent(
                                    robot_name=robot_name,
                                    camera_id=camera_id,
                                    zone_id=zone_id,
                                    event_type="EXIT",
                                    confidence=confidence,
                                    timestamp=timestamp,
                                    sequence=self._seq,
                                )
                            )

        return emitted

    def is_in_zone(self, zone_id: str, robot_name: str) -> bool:
        """Query if robot is currently debounced as present in zone."""
        key = (zone_id, robot_name)
        return self._states.get(key, {}).get("is_present", False)
