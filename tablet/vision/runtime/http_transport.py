# -*- coding: utf-8 -*-
"""HTTP transport adapter for Tablet External Vision Compute (T-D2).

Transports:
    - PoseFix -> POST /api/vision/pose_fix (canonical)
    - VisionZoneEvent -> POST /api/vision/zone_event (canonical)

Invariants:
    1. Stale PoseFix is NEVER queued or retried after network recovery.
       If POST fails, it is dropped immediately to prevent stale telemetry replay.
    2. PoseFix dispatch is asynchronous (latest-only worker) by default so network
       latency/outages do NOT block the vision frame loop. Stale intermediate poses
       are overwritten by newer poses and never queued.
    3. VisionZoneEvent preserves its original sequence and timestamp across retries,
       with bounded queue capacity and exponential backoff. Stale events exceeding
       max_event_age (e.g., 3.0s) are dropped to prevent stale mission transitions.
    4. When allow_egress is False (e.g., FIELD_CONFIG_PENDING or Mock mode without
       explicit override), all egress is fail-closed.
"""

from collections import deque
import json
import logging
import threading
import time
from typing import Any, Deque, Dict, Optional
import urllib.error
import urllib.request

from ..vision_core.pose_fix import PoseFix
from ..vision_core.zone_event import VisionZoneEvent

logger = logging.getLogger("HttpVisionTransport")


class HttpVisionTransport:
    """HTTP client for dispatching vision telemetry to Relay Gateway."""

    def __init__(
        self,
        relay_base_url: str,
        timeout: float = 2.0,
        max_zone_queue: int = 50,
        max_event_age: float = 3.0,
        allow_egress: bool = True,
        async_pose: bool = True,
        opener: Optional[Any] = None,
        api_key: Optional[str] = None,
    ):
        import os
        self.relay_base_url = relay_base_url.rstrip("/")
        self.timeout = timeout
        self.max_zone_queue = max_zone_queue
        self.max_event_age = max_event_age
        self.allow_egress = allow_egress
        self.async_pose = async_pose
        self.api_key = api_key or os.environ.get("RELAY_VISION_API_KEY") or os.environ.get("VISION_API_KEY")
        self._opener = opener or urllib.request.build_opener()

        # POST outcome bookkeeping for health (T-8). status = HTTP code or exception class name.
        self._stats_lock = threading.Lock()
        self._last_post_status: Optional[Any] = None
        self._last_post_time: Optional[float] = None
        self._post_counts: Dict[str, int] = {"ok": 0, "fail": 0}

        # Bounded retry queue for zone events: preserves (event, retry_count, next_retry_time)
        self._zone_retry_queue: Deque[Dict[str, Any]] = deque(maxlen=max_zone_queue)
        self._is_closed = False

        # Latest-only PoseFix asynchronous dispatch
        self._pose_lock = threading.Lock()
        self._pose_event = threading.Event()
        self._latest_poses: Dict[str, PoseFix] = {}
        self._is_sending = False
        self._worker_thread: Optional[threading.Thread] = None

        if self.async_pose:
            self._worker_thread = threading.Thread(
                target=self._pose_worker_loop,
                daemon=True,
                name="PoseFixWorker",
            )
            self._worker_thread.start()

    def _post_json(self, path: str, payload: dict) -> Dict[str, Any]:
        """Perform HTTP POST with JSON body."""
        url = f"{self.relay_base_url}{path}"
        data = json.dumps(payload).encode("utf-8")
        headers = {"Content-Type": "application/json", "User-Agent": "Y700-Tablet-Vision/1.0"}
        if self.api_key:
            headers["X-API-Key"] = self.api_key
        req = urllib.request.Request(
            url=url,
            data=data,
            headers=headers,
            method="POST",
        )
        try:
            with self._opener.open(req, timeout=self.timeout) as resp:
                status = resp.status if hasattr(resp, "status") else resp.getcode()
                body = resp.read().decode("utf-8")
                res_json = json.loads(body) if body else {}
        except urllib.error.HTTPError as exc:
            self._record_post(exc.code)
            raise
        except Exception as exc:
            self._record_post(type(exc).__name__)
            raise
        self._record_post(status)
        return {"status": status, "data": res_json, "ok": 200 <= status < 300}

    def _record_post(self, status: Any) -> None:
        ok = isinstance(status, int) and 200 <= status < 300
        with self._stats_lock:
            changed = status != self._last_post_status
            self._last_post_status = status
            self._last_post_time = time.time()
            self._post_counts["ok" if ok else "fail"] += 1
        # Failures are otherwise DEBUG-only (e.g. a wrong key gives silent 401s); surface transitions.
        if changed:
            (logger.info if ok else logger.warning)(f"Relay POST status changed -> {status}")

    def post_stats(self) -> Dict[str, Any]:
        """last_post{status, age_s} plus ok/fail counters for the health contract."""
        with self._stats_lock:
            age = None if self._last_post_time is None else round(time.time() - self._last_post_time, 2)
            return {"status": self._last_post_status, "age_s": age, **self._post_counts}

    def _send_pose_fix_sync(self, pose_fix: PoseFix) -> bool:
        """Internal synchronous send for a single PoseFix."""
        payload = pose_fix.to_dict()
        try:
            res = self._post_json("/api/vision/pose_fix", payload)
            if res.get("ok"):
                return True
            logger.warning(f"Relay rejected pose_fix seq={pose_fix.seq}: {res.get('data')}")
            return False
        except Exception as exc:
            # Dropped immediately by design (no retry for pose fixes)
            logger.debug(f"PoseFix delivery failed (dropped): {exc}")
            return False

    def _pose_worker_loop(self):
        """Background worker loop dispatching latest-only PoseFix without stalling vision loop."""
        while not self._is_closed:
            signaled = self._pose_event.wait(timeout=0.1)
            if self._is_closed:
                break
            if not signaled:
                continue

            with self._pose_lock:
                poses = list(self._latest_poses.values())
                self._latest_poses.clear()
                self._pose_event.clear()
                self._is_sending = bool(poses)

            for pose in poses:
                if not self.allow_egress:
                    break
                try:
                    self._send_pose_fix_sync(pose)
                except Exception as exc:
                    logger.debug(f"PoseFix delivery failed in async worker: {exc}")

            with self._pose_lock:
                self._is_sending = False

    def send_pose_fix(self, pose_fix: PoseFix, sync: bool = False) -> bool:
        """Send PoseFix to Relay via POST /api/vision/pose_fix.

        Invariants:
            - If allow_egress is False or transport is closed: drops and returns False.
            - Stale PoseFix is NEVER queued or retried.
            - Asynchronous latest-only by default: returns True immediately without
              blocking the vision frame loop.
            - If sync=True, sends synchronously (useful for verification/tests).
        """
        if self._is_closed or not self.allow_egress:
            return False

        if sync or not self.async_pose:
            return self._send_pose_fix_sync(pose_fix)

        with self._pose_lock:
            # Overwrite any previous pending pose for this robot (latest-only)
            self._latest_poses[pose_fix.robot_name] = pose_fix
            self._pose_event.set()
        return True

    def send_zone_event(self, event: VisionZoneEvent) -> bool:
        """Send VisionZoneEvent to Relay via POST /api/vision/zone_event.

        Invariants:
            - If allow_egress is False: drops immediately and returns False.
            - Stale events older than max_event_age are dropped and never retried.
            - Retries preserve original sequence and timestamp.
        """
        if self._is_closed or not self.allow_egress:
            return False

        now = time.time()
        # Drop stale event immediately if it already exceeded max_event_age
        if now - event.timestamp > self.max_event_age:
            logger.warning(
                f"Dropping stale ZoneEvent seq={event.sequence} "
                f"(age {now - event.timestamp:.2f}s > max {self.max_event_age}s)"
            )
            return False

        # Try to flush pending queued events first
        self.flush_zone_events()

        payload = event.to_dict()
        try:
            res = self._post_json("/api/vision/zone_event", payload)
            if res.get("ok"):
                return True
            logger.warning(f"Relay rejected zone_event seq={event.sequence}: {res.get('data')}")
        except Exception as exc:
            logger.debug(f"ZoneEvent POST failed (enqueuing retry): {exc}")

        # Check age again before enqueuing
        if time.time() - event.timestamp > self.max_event_age:
            logger.warning(
                f"Dropping stale ZoneEvent seq={event.sequence} before retry enqueue "
                f"(age > {self.max_event_age}s)"
            )
            return False

        # Enqueue for bounded retry, preserving sequence and timestamp
        self._zone_retry_queue.append({
            "event": event,
            "retries": 1,
            "next_retry": time.time() + 0.5,
        })
        return False

    def flush_zone_events(self) -> int:
        """Attempt to flush pending zone events in retry queue.

        Stale events older than max_event_age are discarded rather than retried.
        """
        if not self._zone_retry_queue or self._is_closed or not self.allow_egress:
            return 0

        now = time.time()
        flushed = 0
        pending_items = list(self._zone_retry_queue)
        self._zone_retry_queue.clear()

        for item in pending_items:
            event: VisionZoneEvent = item["event"]

            # Expiration check: drop stale events exceeding max_event_age
            event_age = now - event.timestamp
            if event_age > self.max_event_age:
                logger.warning(
                    f"Discarding expired ZoneEvent {event.event_type} for {event.robot_name} "
                    f"in {event.zone_id} (age {event_age:.2f}s > max {self.max_event_age}s, "
                    f"seq={event.sequence})"
                )
                continue

            if now < item["next_retry"]:
                self._zone_retry_queue.append(item)
                continue

            try:
                res = self._post_json("/api/vision/zone_event", event.to_dict())
                if res.get("ok"):
                    flushed += 1
                    continue
            except Exception:
                pass

            # Exponential backoff up to 8s
            retries = item["retries"] + 1
            if retries <= 5:
                backoff = min(8.0, 0.5 * (2 ** (retries - 1)))
                item["retries"] = retries
                item["next_retry"] = now + backoff
                self._zone_retry_queue.append(item)
            else:
                logger.warning(f"Dropping ZoneEvent seq={event.sequence} after 5 failed retries")

        return flushed

    def drain_poses(self, timeout: float = 1.0) -> bool:
        """Wait for any pending latest poses in worker queue to be processed."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._pose_lock:
                if not self._latest_poses and not self._pose_event.is_set() and not self._is_sending:
                    return True
            time.sleep(0.01)
        return False

    @property
    def pending_zone_events_count(self) -> int:
        return len(self._zone_retry_queue)

    def close(self):
        """Close transport and worker threads cleanly."""
        self.drain_poses(timeout=0.5)
        self._is_closed = True
        self._pose_event.set()
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
        with self._pose_lock:
            self._latest_poses.clear()
        self._zone_retry_queue.clear()
