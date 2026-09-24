"""Validated HTTP-to-ROS control requests; publishing happens only on the ROS thread."""
import queue


class ControlQueue:
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.requests = queue.SimpleQueue()

    def submit(self, payload):
        if not self.enabled or not isinstance(payload, dict):
            return False
        action = payload.get('action')
        if action not in {'start', 'pause', 'resume', 'reset', 'speed'}:
            return False
        if action == 'speed':
            if payload.get('robot') not in {'pinky1', 'pinky2'}:
                return False
            try:
                linear, angular = float(payload['linear']), float(payload['angular'])
            except (KeyError, TypeError, ValueError):
                return False
            if not (0 <= linear <= .3 and 0 <= angular <= 2.):
                return False
        self.requests.put(payload)
        return True
