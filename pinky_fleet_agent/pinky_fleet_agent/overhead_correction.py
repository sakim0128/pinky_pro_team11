"""항공뷰 이탈 보정 (2026-10-01) — 관제가 보낸 차선 중앙 이탈량으로 차선 주행 조향에 작은 보정을 더한다. ROS 에 의존하지 않는다.

관제(relay vision_coordinator)가 항공뷰 좌표(태블릿 → /pinkyN/overhead_pose)를 코스 중심선(vision_course.yaml)에 겹쳐
/pinkyN/lane_correction (geometry_msgs/Vector3Stamped) 으로 10 Hz 보낸다.

    vector.x = 옆 이탈 (m, 진행 방향 왼쪽 +)
    vector.y = 방향 오차 (rad, 코스 방향보다 왼쪽을 보면 +)
    vector.z = 1 이면 보정, 0 이면 보정 없음 (교차로 구간 · 이탈 3 cm 미만 · 항공뷰 끊김)

    ω_corr = −(k_lat · x + k_head · y)   (|ω_corr| ≤ max_omega)

차선 주행(CRUISE) 중에만 더한다. timeout 넘게 새 값이 없으면 보정하지 않는다 — 항공뷰가 끊기면 카메라 차선 주행만 남는다.
"""

from dataclasses import dataclass


@dataclass
class CorrectionParams:
    enabled: bool = True
    k_lat: float = 2.0              # rad/s per m — 8 cm 이탈이면 0.16 rad/s
    k_head: float = 0.5             # rad/s per rad
    max_omega: float = 0.3          # 보정 각속도 상한 (rad/s) — 카메라 조향을 이기지 않게
    timeout: float = 0.5            # 이만큼(s) 새 값이 없으면 보정 안 함


class OverheadCorrection:
    def __init__(self, params=None):
        self.p = params or CorrectionParams()
        self.clear()

    def clear(self):
        self._t = None
        self.lateral = 0.0
        self.heading = 0.0
        self.active = False

    def update(self, now, lateral, heading, active):
        self._t = float(now)
        self.lateral = float(lateral)
        self.heading = float(heading)
        self.active = bool(active)

    def fresh(self, now):
        return self._t is not None and float(now) - self._t <= self.p.timeout

    def omega(self, now):
        """더할 각속도 (rad/s). 보정할 게 없으면 None."""
        if not self.p.enabled or not self.active or not self.fresh(now):
            return None
        w = -(self.p.k_lat * self.lateral + self.p.k_head * self.heading)
        m = abs(self.p.max_omega)
        return max(-m, min(m, w))
