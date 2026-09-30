# -*- coding: utf-8 -*-
"""Tablet Vision Runtime Service (Phase 4, 5, 7).

Coordinates:
    1. Overhead camera frame ingestion & local processing (ArUco + Homography)
    2. Robot pose generation (PoseFix) with occlusion handling (stale poses NEVER republished)
    3. Start / Goal camera zone event detection (VisionZoneEvent with debounce)
    4. Comprehensive health metrics reporting
    5. Clean egress dispatch (Relay HTTP API / ROS 2 domain topic)
"""

from dataclasses import dataclass, field
import json
import logging
import math
import time
from typing import Any, Callable, Dict, List, Optional, Set

import numpy as np

from ..vision_core.marker_localizer import Fix
from ..vision_core.overhead_localizer import OverheadConfig, OverheadLocalizer
from ..vision_core.pose_fix import Header, PoseFix
from ..vision_core.zone_event import VisionZoneEvent, ZoneConfig, ZoneDetector

logger = logging.getLogger("TabletVisionService")


class TrackingState:
    TRACKED = "TRACKED"
    LOST = "LOST"
    NO_REFERENCE = "NO_REFERENCE"
    BAD_HOMOGRAPHY = "BAD_HOMOGRAPHY"
    HIGH_REPROJ_ERROR = "HIGH_REPROJ_ERROR"


class TabletVisionService:
    """External Vision Compute service for Lenovo Y700 Tablet."""

    def __init__(
        self,
        overhead_cfg: OverheadConfig,
        zone_configs: Optional[List[ZoneConfig]] = None,
        pose_callback: Optional[Callable[[PoseFix], None]] = None,
        zone_event_callback: Optional[Callable[[VisionZoneEvent], None]] = None,
    ):
        self.overhead_cfg = overhead_cfg
        self.localizer = OverheadLocalizer(overhead_cfg)
        self.zone_detector = ZoneDetector(zone_configs or [])
        self.pose_callback = pose_callback
        self.zone_event_callback = zone_event_callback

        # Per-robot tracking state and sequences
        self._seq: Dict[str, int] = {name: 0 for name in overhead_cfg.robots}
        self._last_pose_time: Dict[str, Optional[float]] = {name: None for name in overhead_cfg.robots}
        self._tracking_states: Dict[str, str] = {name: TrackingState.LOST for name in overhead_cfg.robots}

        # Performance & Health tracking
        self.camera_connected = False
        self._frame_times: List[float] = []
        self._detect_times: List[float] = []
        self._last_detector_latency = 0.0
        self._last_pipeline_latency = 0.0
        self._frames_total = 0
        self._posefix_times: List[float] = []

        # Validate reference height policy (Phase 3)
        if hasattr(overhead_cfg, "reference_height"):
            ref_h = getattr(overhead_cfg, "reference_height", 0.10)
            logger.info(f"Initialized with reference height {ref_h}m")

    def process_overhead_frame(self, img: np.ndarray, capture_stamp: Optional[float] = None) -> Dict[str, PoseFix]:
        """Process an overhead camera frame.

        Args:
            img: BGR or grayscale numpy image
            capture_stamp: Tablet Ingress / Frame-Read Started Timestamp (recorded on
                local station clock when camera capture read began; not HW shutter time)

        Returns:
            Dict of {robot_name: PoseFix} ONLY for currently detected robots.
            Missing robots are NOT included in the returned dict (stale poses are NEVER republished).
        """
        now = time.time()
        self.camera_connected = True
        t0 = time.perf_counter()

        # Update FPS history window (2 seconds)
        self._frame_times = [t for t in self._frame_times if now - t < 2.0] + [now]
        self._frames_total += 1

        fixes = self.localizer.update(img)
        self._detect_times = [t for t in self._detect_times if now - t < 2.0] + [now]

        t1 = time.perf_counter()
        self._last_detector_latency = t1 - t0
        self._last_pipeline_latency = (
            time.time() - capture_stamp
            if capture_stamp is not None
            else self._last_detector_latency
        )

        out_fixes: Dict[str, PoseFix] = {}

        # Evaluate homography and reference health
        st = self.localizer.status()
        base_state = TrackingState.TRACKED
        if not st["homography"]:
            base_state = TrackingState.NO_REFERENCE if st["reference_seen"] < self.overhead_cfg.min_reference else TrackingState.BAD_HOMOGRAPHY
        elif self.localizer.reproj > self.overhead_cfg.max_reproj:
            base_state = TrackingState.HIGH_REPROJ_ERROR

        for name in self.overhead_cfg.robots:
            if name in fixes and base_state == TrackingState.TRACKED:
                f: Fix = fixes[name]
                self._seq[name] += 1
                self._last_pose_time[name] = now
                self._tracking_states[name] = TrackingState.TRACKED

                header = (
                    Header.from_timestamp(capture_stamp, "map")
                    if capture_stamp is not None
                    else Header.now("map")
                )
                pipeline_latency = self._last_pipeline_latency

                pf = PoseFix(
                    robot_name=name,
                    x=f.x,
                    y=f.y,
                    yaw=f.yaw,
                    seq=self._seq[name],
                    header=header,
                    stamp_is_robot_clock=False,
                    marker_id=f.marker_id,
                    marker_range=0.0,
                    reproj_error=f.reproj,
                    n_markers=f.n_markers,
                    pipeline_latency=pipeline_latency,
                )
                out_fixes[name] = pf
                self._posefix_times = [t for t in self._posefix_times if now - t < 2.0] + [now]
                if self.pose_callback:
                    self.pose_callback(pf)
            else:
                # Occlusion / Marker missing: transition to LOST or reference error
                self._tracking_states[name] = base_state if base_state != TrackingState.TRACKED else TrackingState.LOST

        return out_fixes

    def process_zone_frame(
        self,
        camera_id: str,
        detected_marker_ids: Set[int],
        timestamp: Optional[float] = None,
        confidence: float = 1.0,
    ) -> List[VisionZoneEvent]:
        """Process detections from a Start / Goal camera frame and emit zone events."""
        events = self.zone_detector.update(
            camera_id=camera_id,
            detected_marker_ids=detected_marker_ids,
            timestamp=timestamp,
            confidence=confidence,
        )
        for ev in events:
            if self.zone_event_callback:
                self.zone_event_callback(ev)
        return events

    def detect_markers(self, img: np.ndarray) -> Set[int]:
        """Detect ArUco marker IDs present in an image frame."""
        detected = self.localizer.detect(img)
        return set(detected.keys())

    def process_zone_image(
        self,
        camera_id: str,
        img: np.ndarray,
        timestamp: Optional[float] = None,
        confidence: float = 1.0,
    ) -> List[VisionZoneEvent]:
        """Detect ArUco markers in a zone camera frame and process zone transitions."""
        marker_ids = self.detect_markers(img)
        return self.process_zone_frame(
            camera_id=camera_id,
            detected_marker_ids=marker_ids,
            timestamp=timestamp,
            confidence=confidence,
        )

    def get_health(self) -> Dict[str, Any]:
        """Collect tablet health metrics as specified in Section 10 (Phase 7)."""
        now = time.time()
        loc_st = self.localizer.status()

        frame_fps = len(self._frame_times) / 2.0 if self._frame_times else 0.0
        detect_fps = len(self._detect_times) / 2.0 if self._detect_times else 0.0

        last_pose_age = {}
        for name, last_t in self._last_pose_time.items():
            last_pose_age[name] = round(now - last_t, 2) if last_t is not None else None

        return {
            "camera_connected": self.camera_connected,
            "frame_fps": round(frame_fps, 1),
            "marker_detect_fps": round(detect_fps, 1),
            "reference_seen": loc_st["reference_seen"],
            "homography_valid": loc_st["homography"],
            "reproj_error": loc_st["reproj_px"],
            "robot1_tracking": self._tracking_states.get("pinky1", TrackingState.LOST),
            "robot2_tracking": self._tracking_states.get("pinky2", TrackingState.LOST),
            "pipeline_latency": round(self._last_pipeline_latency, 4),
            "detector_latency": round(self._last_detector_latency, 4),
            "last_pose_age": last_pose_age,
            # T-8 health contract fields (no Relay route consumes them yet; uplink is an R-5 follow-up)
            "input_fps": round(frame_fps, 1),
            "frames_total": self._frames_total,
            "markers_seen": list(self.localizer.last_ids),
            "reference_ok": loc_st["reference_seen"] >= self.overhead_cfg.min_reference,
            "homography_ok": bool(loc_st["homography"]),
            "posefix_rate": round(len([t for t in self._posefix_times if now - t < 2.0]) / 2.0, 1),
        }
