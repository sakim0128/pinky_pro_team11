# -*- coding: utf-8 -*-
"""Executable runtime entrypoint for Y700 Tablet External Vision Compute (T-D7).

Coordinates:
    - Configuration loading (cameras.yaml, markers.yaml, zones.yaml)
    - Camera capture ingestion (V4L2 device, HTTP MJPEG, or synthetic mock)
    - Start / Goal zone camera ingestion and event detection (START_A, START_B, GOAL_C)
    - TabletVisionService execution
    - HttpVisionTransport dispatch (/api/vision/pose_fix, /api/vision/zone_event)
    - Safety invariants:
        * Egress blocked if configuration status != READY (unless --allow-pending-config)
        * Automatic mock fallback on live camera open failure is FORBIDDEN (fail-closed)
        * Mock mode disables Relay egress by default (unless --mock-egress)
    - Periodic health diagnostics logging
    - Graceful SIGINT/SIGTERM shutdown
"""

import argparse
import json
import logging
import os
import signal
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

if __package__ is None or __package__ == "":
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))
    from tablet.vision.vision_core.overhead_localizer import OverheadConfig
    from tablet.vision.vision_core.synthetic_camera import render_overhead
    from tablet.vision.vision_core.mock_motion import DEFAULT_ROAD_GRAPH, MockPoses, RoadGraph
    from tablet.vision.vision_core.zone_event import ZoneConfig
    from tablet.vision.runtime.http_transport import HttpVisionTransport
    from tablet.vision.runtime.tablet_vision_service import TabletVisionService
else:
    from ..vision_core.overhead_localizer import OverheadConfig
    from ..vision_core.synthetic_camera import render_overhead
    from ..vision_core.mock_motion import DEFAULT_ROAD_GRAPH, MockPoses, RoadGraph
    from ..vision_core.zone_event import ZoneConfig
    from .http_transport import HttpVisionTransport
    from .tablet_vision_service import TabletVisionService

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S%z",
)
logger = logging.getLogger("TabletVisionRunner")


def detect_git_sha() -> str:
    """Short SHA of the running checkout, suffixed '-dirty' when the tree has local changes."""
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    try:
        sha = subprocess.run(["git", "-C", repo, "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=5).stdout.strip()
        dirty = subprocess.run(["git", "-C", repo, "status", "--porcelain", "--untracked-files=no"],
                               capture_output=True, text=True, timeout=5).stdout.strip()
        return (sha + ("-dirty" if dirty else "")) if sha else "unknown"
    except Exception:
        return "unknown"


def build_vision_health(service_health: Dict[str, Any], post_stats: Optional[Dict[str, Any]],
                        config_statuses: Dict[str, str], mode: str, git_sha: str) -> Dict[str, Any]:
    """T-8 health contract, logged as one `VISION_HEALTH` JSON line.

    Nothing on the Relay consumes it yet: `/api/ops/vision` is the planned name, the uplink
    route does not exist (R-5 follow-up).

    Questions: is camera input arriving · how many markers · is H up · are PoseFixes produced ·
    do POSTs succeed. `last_post` is None when egress is off (not "failed").
    """
    return {
        "input_fps": service_health["input_fps"],
        "frames_total": service_health["frames_total"],
        "markers_seen": service_health["markers_seen"],
        "reference_ok": service_health["reference_ok"],
        "homography_ok": service_health["homography_ok"],
        "posefix_rate": service_health["posefix_rate"],
        "last_post": None if post_stats is None else {"status": post_stats["status"],
                                                      "age_s": post_stats["age_s"]},
        "config_status": "READY" if all(v == "READY" for v in config_statuses.values()) else "PENDING",
        "mode": mode,
        "git_sha": git_sha,
    }


def load_configs(config_dir: str) -> Tuple[OverheadConfig, dict, List[ZoneConfig], Dict[str, str]]:
    """Load camera, marker, and zone configurations and their status indicators."""
    markers_path = os.path.join(config_dir, "markers.yaml")
    cameras_path = os.path.join(config_dir, "cameras.yaml")
    zones_path = os.path.join(config_dir, "zones.yaml")

    with open(markers_path, "r", encoding="utf-8") as fh:
        markers_data = yaml.safe_load(fh) or {}

    with open(cameras_path, "r", encoding="utf-8") as fh:
        cameras_data = yaml.safe_load(fh) or {}

    overhead_cfg = OverheadConfig.from_dict(markers_data)
    overhead_cam = cameras_data.get("cameras", {}).get("overhead", {})
    overhead_cfg.camera = overhead_cam

    zone_configs = []
    zones_data = {}
    if os.path.exists(zones_path):
        with open(zones_path, "r", encoding="utf-8") as fh:
            zones_data = yaml.safe_load(fh) or {}
            for z in zones_data.get("zones", []):
                zone_configs.append(ZoneConfig(
                    zone_id=z["zone_id"],
                    camera_id=z["camera_id"],
                    target_markers=z.get("target_markers", {}),
                    enter_debounce_frames=z.get("enter_debounce_frames", 3),
                    exit_debounce_frames=z.get("exit_debounce_frames", 4),
                    min_confidence=z.get("min_confidence", 0.8),
                ))

    statuses = {
        "markers": markers_data.get("status", "FIELD_CONFIG_PENDING"),
        "cameras": cameras_data.get("status", "FIELD_CONFIG_PENDING"),
        "zones": zones_data.get("status", "FIELD_CONFIG_PENDING"),
    }

    return overhead_cfg, cameras_data, zone_configs, statuses


class CameraIngest:
    """Wrapper for camera frame acquisition with Tablet Ingress timestamping."""

    def __init__(self, dev_spec: Any, width: int = 1280, height: int = 720, fps: int = 15):
        self.dev_spec = dev_spec
        self.width = width
        self.height = height
        self.fps = fps
        self.cap: Optional[cv2.VideoCapture] = None

    def open(self):
        src = int(self.dev_spec) if str(self.dev_spec).isdigit() else str(self.dev_spec)
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 장치를 열 수 없습니다: {self.dev_spec}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read_frame(self) -> Tuple[bool, Optional[np.ndarray], float]:
        """Read frame and record tablet ingress timestamp immediately before acquisition."""
        if self.cap is None:
            return False, None, 0.0
        # Ingress timestamp: recorded on Tablet station clock when frame-read initiates
        t_capture = time.time()
        ok, frame = self.cap.read()
        return ok, frame, t_capture

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def main(argv=None):
    parser = argparse.ArgumentParser(description="Y700 Tablet External Vision Compute Runner")
    parser.add_argument("--config-dir", default=os.path.join(os.path.dirname(__file__), "..", "config"),
                        help="Configuration files directory")
    parser.add_argument("--relay-url", default=os.getenv("RELAY_URL", ""),
                        help="Relay station gateway URL (http://<중계 PC>:8889). 비어 있으면 송신하지 않는다(dry-run)")
    parser.add_argument("--no-relay", action="store_true", help="Disable HTTP egress (dry-run)")
    parser.add_argument("--allow-pending-config", action="store_true",
                        help="Permit Relay egress even when configuration status is FIELD_CONFIG_PENDING")
    parser.add_argument("--mock", action="store_true", help="Use synthetic camera rendering loop")
    parser.add_argument("--mock-egress", action="store_true",
                        help="Permit Relay egress in mock mode (disabled by default for safety)")
    parser.add_argument("--enable-zone-cameras", action="store_true",
                        help="Enable Start/Goal zone camera ingestion and event detection")
    parser.add_argument("--once", action="store_true", help="Process single frame and exit (test/CI)")
    parser.add_argument("--mock-motion", type=float, default=0.0, metavar="V_M_S",
                        help="Mock only: advance pinky1 along --mock-path at V m/s (0 = static at start nodes)")
    parser.add_argument("--mock-path", default="BL,TR",
                        help="Mock only: comma-separated road-graph nodes for the moving robot")
    parser.add_argument("--road-graph", default=DEFAULT_ROAD_GRAPH,
                        help="Mock only: road_graph.yaml providing start nodes and path waypoints")
    parser.add_argument("--api-key", default=os.getenv("RELAY_VISION_API_KEY") or os.getenv("VISION_API_KEY"),
                        help="Relay Vision API Key for authenticated HTTP egress")
    args = parser.parse_args(argv)

    overhead_cfg, cameras_data, zone_configs, config_statuses = load_configs(args.config_dir)

    # ----------------------------------------------------------------------
    # Safety Invariant Checks (Fail-Closed Egress)
    # ----------------------------------------------------------------------
    configs_ready = all(st == "READY" for st in config_statuses.values())
    egress_allowed = True

    if not configs_ready and not args.allow_pending_config:
        logger.warning(
            f"Configuration status is pending ({config_statuses}). "
            "Relay egress is DISABLED (fail-closed) to prevent emitting unmeasured coordinates. "
            "Pass --allow-pending-config to override."
        )
        egress_allowed = False
    elif args.mock and not args.mock_egress:
        logger.warning(
            "Mock mode active: Relay egress is DISABLED by default to prevent synthetic "
            "coordinates from reaching live Relay. Pass --mock-egress to override."
        )
        egress_allowed = False
    elif args.no_relay or not args.relay_url:
        logger.info("Relay egress is disabled by configuration flag (--no-relay or empty URL).")
        egress_allowed = False

    transport: Optional[HttpVisionTransport] = None
    if egress_allowed and args.relay_url:
        transport = HttpVisionTransport(
            relay_base_url=args.relay_url,
            allow_egress=True,
            async_pose=True,
            api_key=args.api_key,
        )
        logger.info(f"Relay HTTP transport ENABLED: {args.relay_url} (async latest-only worker)")
    else:
        logger.info("Relay HTTP transport is INACTIVE.")

    service = TabletVisionService(
        overhead_cfg=overhead_cfg,
        zone_configs=zone_configs,
        pose_callback=(transport.send_pose_fix if transport else None),
        zone_event_callback=(transport.send_zone_event if transport else None),
    )

    running = True

    def _sig_handler(signum, frame):
        nonlocal running
        logger.info("Shutdown signal received, exiting frame loop...")
        running = False

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    # ----------------------------------------------------------------------
    # Overhead Camera Ingestion Setup (Fail-Closed, No Mock Fallback)
    # ----------------------------------------------------------------------
    cam_ingest: Optional[CameraIngest] = None
    if not args.mock:
        dev = overhead_cfg.camera.get("device", 0)
        cam_ingest = CameraIngest(
            dev_spec=dev,
            width=overhead_cfg.camera.get("width", 1280),
            height=overhead_cfg.camera.get("height", 720),
            fps=overhead_cfg.camera.get("fps", 15),
        )
        try:
            cam_ingest.open()
            logger.info(f"Connected to live overhead camera device: {dev}")
        except Exception as exc:
            # P0: Live camera failure must NEVER automatically fallback to mock mode
            logger.error(
                f"Failed to open live overhead camera device ({exc}). "
                "Fail-closed: automatic mock fallback is disabled. Exiting."
            )
            return 1

    # ----------------------------------------------------------------------
    # Start / Goal Zone Camera Ingestion Setup (START_A, START_B, GOAL_C)
    # ----------------------------------------------------------------------
    zone_cams: Dict[str, dict] = {}
    for name, cinfo in cameras_data.get("cameras", {}).items():
        if cinfo.get("role") == "zone":
            cam_id = cinfo.get("camera_id", f"CAM_{name.upper()}")
            zone_id = cinfo.get("zone_id", name.upper())
            enabled = cinfo.get("enabled", False) or args.enable_zone_cameras
            zone_cams[cam_id] = {
                "name": name,
                "camera_id": cam_id,
                "zone_id": zone_id,
                "device": cinfo.get("device"),
                "fps": cinfo.get("fps", 5),
                "enabled": enabled,
                "ingest": None,
                "last_poll": 0.0,
            }

            if enabled and not args.mock:
                zingest = CameraIngest(
                    dev_spec=cinfo.get("device"),
                    width=640,
                    height=480,
                    fps=cinfo.get("fps", 5),
                )
                try:
                    zingest.open()
                    zone_cams[cam_id]["ingest"] = zingest
                    logger.info(f"Connected to zone camera {cam_id} ({zone_id}) at {cinfo.get('device')}")
                except Exception as exc:
                    logger.warning(f"Zone camera {cam_id} ({cinfo.get('device')}) unavailable: {exc}")

    active_zone_cam_count = sum(1 for zc in zone_cams.values() if zc["enabled"])
    logger.info(
        f"Tablet External Vision Compute service started (mode={'mock' if args.mock else 'live'}, "
        f"zone_cameras={active_zone_cam_count})."
    )

    last_health_log = time.time()
    git_sha = detect_git_sha()
    mode = "mock" if args.mock else "live"
    mock: Optional[MockPoses] = None
    if args.mock:
        # Start nodes / path come from the Relay road graph so synthetic fixes land on arena nodes.
        mock = MockPoses(RoadGraph.load(args.road_graph),
                         path_nodes=[n.strip() for n in args.mock_path.split(",") if n.strip()],
                         speed_m_s=args.mock_motion)
        logger.info(f"Mock poses from road graph: start={mock.static} motion={args.mock_motion} m/s "
                    f"path={args.mock_path if args.mock_motion > 0 else '-'} (PLACEHOLDER coordinates)")
    t_mock0 = time.monotonic()

    try:
        while running:
            # 1. Overhead frame acquisition & processing
            if args.mock:
                t_capture = time.time()
                _, frame = render_overhead(overhead_cfg, mock.poses(time.monotonic() - t_mock0))
                ok = True
                time.sleep(0.066)  # ~15 FPS
            else:
                ok, frame, t_capture = cam_ingest.read_frame()

            if ok and frame is not None:
                service.process_overhead_frame(frame, capture_stamp=t_capture)

            # 2. Start / Goal Zone camera ingest & detection
            now = time.time()
            for cam_id, zc in zone_cams.items():
                if not zc["enabled"]:
                    continue
                poll_interval = 1.0 / max(1, zc["fps"])
                if now - zc["last_poll"] >= poll_interval:
                    zc["last_poll"] = now
                    if args.mock:
                        # Synthetic zone marker detection for mock verification
                        simulated_markers = set()
                        if zc["zone_id"] == "START_A":
                            simulated_markers.add(30)
                        elif zc["zone_id"] == "GOAL_C":
                            simulated_markers.add(31)
                        service.process_zone_frame(
                            camera_id=cam_id,
                            detected_marker_ids=simulated_markers,
                            timestamp=now,
                        )
                    elif zc["ingest"] is not None:
                        ok_z, zframe, zt = zc["ingest"].read_frame()
                        if ok_z and zframe is not None:
                            service.process_zone_image(
                                camera_id=cam_id,
                                img=zframe,
                                timestamp=zt,
                            )

            # 3. Health diagnostics periodic log
            if now - last_health_log >= 5.0:
                health = service.get_health()
                logger.info(
                    f"Health: seen={health['reference_seen']}/4 H={health['homography_valid']} "
                    f"reproj={health['reproj_error']}px R1={health['robot1_tracking']} "
                    f"R2={health['robot2_tracking']} "
                    f"lat_pipe={health['pipeline_latency']}s lat_det={health['detector_latency']}s"
                )
                vh = build_vision_health(health, transport.post_stats() if transport else None,
                                         config_statuses, mode, git_sha)
                logger.info("VISION_HEALTH " + json.dumps(vh, separators=(",", ":")))
                last_health_log = now

            if args.once:
                break
    finally:
        if cam_ingest:
            cam_ingest.release()
        for zc in zone_cams.values():
            if zc.get("ingest") is not None:
                zc["ingest"].release()
        if transport:
            transport.close()
        logger.info("Tablet External Vision Compute service shutdown cleanly.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
