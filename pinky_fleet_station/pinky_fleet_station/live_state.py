"""Thread-safe, ROS-independent read-only telemetry snapshots."""
import copy
import math
import threading
import time


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(item) for item in value]
    return value


class StateStore:
    def __init__(self, names, timeout=2.0, clock=time.monotonic):
        if not names or len(set(names)) != len(names):
            raise ValueError('robot_names must be nonempty and unique')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('state_timeout must be finite and positive')
        self.names = tuple(names)
        self.timeout = timeout
        self.clock = clock
        self.lock = threading.Lock()
        self.samples = {}

    def update(self, key, data):
        with self.lock:
            self.samples[key] = (finite_json(copy.deepcopy(data)), self.clock())

    def snapshot(self):
        with self.lock:
            now = self.clock()

            def sample(key):
                item = self.samples.get(key)
                if item is None:
                    return dict(status='missing', age_seconds=None, data=None)
                data, received = item
                age = max(0.0, now - received)
                return dict(status='live' if age <= self.timeout else 'stale',
                            age_seconds=round(age, 3), data=copy.deepcopy(data))

            robots = [dict(name=name, state=sample((name, 'state')),
                           lane=sample((name, 'lane')),
                           amcl=sample((name, 'amcl')),
                           overhead=sample((name, 'overhead'))) for name in self.names]
            mission = sample('mission')
            streams = [mission] + [r[k] for r in robots for k in ('state', 'lane')]
            health = ('online' if all(s['status'] == 'live' for s in streams)
                      else 'waiting' if all(s['status'] == 'missing' for s in streams)
                      else 'degraded')
            return dict(schema_version=1, mode='live', read_only=True,
                        health=health, timeout_seconds=self.timeout,
                        robots=robots, mission=mission)
