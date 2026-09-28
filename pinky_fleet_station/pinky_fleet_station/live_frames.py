"""Bounded latest-JPEG storage for the read-only web monitor."""
import threading
import time


class FrameStore:
    def __init__(self, names, timeout=2.0, maximum_bytes=2_000_000, clock=time.monotonic):
        self.names = tuple(names) + ('overhead',)
        self.timeout = timeout
        self.maximum_bytes = maximum_bytes
        self.clock = clock
        self.lock = threading.Lock()
        self.frames = {}

    def update_jpeg(self, name, data):
        payload = bytes(data)
        if name not in self.names or len(payload) < 4 or len(payload) > self.maximum_bytes:
            return False
        if not (payload.startswith(b'\xff\xd8') and payload.endswith(b'\xff\xd9')):
            return False
        with self.lock:
            self.frames[name] = (payload, self.clock())
        return True

    def get(self, name):
        with self.lock:
            item = self.frames.get(name)
            if item is None or self.clock() - item[1] > self.timeout:
                return None
            return item[0]

    def snapshot(self):
        with self.lock:
            now = self.clock()
            return {name: dict(status=('live' if name in self.frames and now-self.frames[name][1] <= self.timeout else 'stale' if name in self.frames else 'missing'), age_seconds=(round(max(0., now-self.frames[name][1]), 3) if name in self.frames else None)) for name in self.names}
