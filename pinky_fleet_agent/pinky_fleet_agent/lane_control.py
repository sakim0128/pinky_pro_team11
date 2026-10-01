"""속도·각속도 명령 계산 — 경로 pure-pursuit + 카메라 ErrorX 보정 블렌딩.

ROS 에 의존하지 않는다. 20 Hz 로 호출된다.

    ω = ω_route + w_cam · ω_cam
    ω_cam = −(Kp·e + Kd·Δe/Δt)          e = error_x_norm (-1..+1)
                                          차선 중앙이 화면 오른쪽에 보이면(= 로봇이 왼쪽으로
                                          치우침) 양수 → 우회전(ω<0) 이 나와야 한다
    v = v_max · min(1, 1 − k_err·|e|, 1 − k_curv·|ω|/ω_max)  → 정지 지점 앞에서 √(2·a·d) 로 감속

카메라 ↔ 조향 부호는 실차에서 한 번 확정한다 (params 의 cam_sign). 로봇을 차선 왼쪽에
놓으면 e > 0 이고 우회전(ω < 0)이 나와야 한다.
"""

from dataclasses import dataclass

QUALITY_BOTH, QUALITY_SINGLE, QUALITY_JUNCTION, QUALITY_STALE, QUALITY_LOST = 0, 1, 2, 3, 4
CAM_WEIGHT = {QUALITY_BOTH: 1.0, QUALITY_SINGLE: 0.5, QUALITY_JUNCTION: 0.0,
              QUALITY_STALE: 0.0, QUALITY_LOST: 0.0}


@dataclass
class ControlParams:
    v_max: float = 0.15
    omega_max: float = 1.2
    kp: float = 1.2                 # ω_cam per unit error
    kd: float = 0.15
    k_err: float = 0.5              # |e| = 1 이면 v 를 절반으로
    k_curv: float = 0.5             # |ω| = ω_max 이면 v 를 절반으로
    a_decel: float = 0.6            # 정지 지점 접근 감속 (m/s²)
    stop_tolerance: float = 0.03    # 이 거리 안이면 정지
    accel_slew: float = 0.4         # v 변화율 상한 (m/s per s)
    cam_sign: float = 1.0           # 실차에서 확정. -1 이면 부호 반전
    v_min_moving: float = 0.04      # 정지 지점 접근 중 최저 속도 (정지 직전까지 굼뜨지 않게)
    stale_max_age: float = 0.9      # 이 이상 오래된 error_x 는 쓰지 않는다
    single_weight: float = 0.5      # 차선이 하나(SINGLE)일 때 카메라 보정 가중치 (BOTH = 1.0)
    error_alpha: float = 1.0        # 새 측정마다 e_f = α·e + (1−α)·e_f (1.0 = 필터 없음). 목표점 점프·검출 잡음 완화


class LaneController:
    def __init__(self, params=None):
        self.p = params or ControlParams()
        self.reset()

    def reset(self):
        self._prev_error = None
        self._v_cmd = 0.0
        self._meas_stamp = None     # 마지막으로 반영한 측정(LanePath) 의 stamp
        self._meas_e = None         # 필터된 error
        self._meas_de = 0.0         # 측정 간 기울기 (다음 측정까지 유지)

    def _measured(self, error_x, error_stamp, active):
        """측정 단위 PD 입력 (e, de). 새 LanePath(stamp 변경) 에서만 필터·기울기를 갱신한다.

        제어는 20 Hz, 측정은 ≤ 10 Hz 라 틱마다 de 를 구하면 새 프레임이 올 때마다 de 가 튀고(Δe/0.05 s) 다음 틱엔 0 이 돼
        D 항이 프레임마다 조향을 툭툭 친다. 측정 간격으로 나눈 기울기를 다음 측정까지 유지한다.
        """
        p = self.p
        if not active:
            self._meas_stamp, self._meas_e, self._meas_de = None, None, 0.0
            return float(error_x or 0.0), 0.0
        e_raw = float(error_x)
        if error_stamp != self._meas_stamp:
            a = min(1.0, max(0.0, float(p.error_alpha)))
            e_new = e_raw if self._meas_e is None else a * e_raw + (1.0 - a) * self._meas_e
            if self._meas_e is not None and self._meas_stamp is not None and error_stamp > self._meas_stamp:
                self._meas_de = (e_new - self._meas_e) / (error_stamp - self._meas_stamp)
            else:
                self._meas_de = 0.0
            self._meas_e, self._meas_stamp = e_new, error_stamp
        return self._meas_e, self._meas_de

    def command(self, omega_route, error_x, quality, dt, dist_to_stop, speed_factor=1.0,
                turn_in_place=False, error_stamp=None):
        """(v, ω). dist_to_stop 은 정지 지점까지 남은 호길이 (None 이면 제한 없음).

        error_stamp: error_x 를 만든 측정의 시각(LanePath.source_stamp). 주면 D 항·필터를 측정 단위로 계산한다.
        """
        p = self.p
        dt = max(1e-3, float(dt))
        if error_x is None:
            w_cam = 0.0
        elif int(quality) == QUALITY_SINGLE:
            w_cam = float(p.single_weight)
        else:
            w_cam = CAM_WEIGHT.get(int(quality), 0.0)
        if error_stamp is not None and error_x is not None:
            e, de = self._measured(error_x, float(error_stamp), w_cam > 0)
        else:
            e = float(error_x or 0.0)
            de = 0.0 if self._prev_error is None else (e - self._prev_error) / dt
            self._prev_error = e if w_cam > 0 else None
        omega_cam = -(p.kp * e + p.kd * de) * p.cam_sign
        if turn_in_place:
            w_cam = 0.0          # 제자리 회전 중엔 기하가 무너져 카메라 항이 뜻이 없다
        omega = float(omega_route) + w_cam * omega_cam
        omega = max(-p.omega_max, min(p.omega_max, omega))

        if turn_in_place:
            v_target = 0.0
        else:
            v_target = p.v_max * min(1.0,
                                     max(0.0, 1.0 - p.k_err * abs(e) * w_cam),
                                     max(0.2, 1.0 - p.k_curv * abs(omega) / p.omega_max))
            v_target *= max(0.0, float(speed_factor))
            if dist_to_stop is not None:
                d = max(0.0, float(dist_to_stop))
                if d <= p.stop_tolerance:
                    v_target = 0.0
                else:
                    v_target = min(v_target, max(p.v_min_moving, (2.0 * p.a_decel * d) ** 0.5))
        # 가속 슬루 (감속은 즉시)
        if v_target > self._v_cmd:
            v_target = min(v_target, self._v_cmd + p.accel_slew * dt)
        self._v_cmd = v_target
        if v_target <= 0.0 and not turn_in_place:
            omega = 0.0
        return v_target, omega

    def follow_curvature(self, kappa, dt, speed_factor=1.0):
        """(v, ω) — 기억한 차선 중앙을 pure pursuit 곡률 κ 로 따라간다 (lane_memory). ω = κ·v.

        속도는 command() 와 같은 규칙(v_max · 곡률 감속 · speed_factor · 가속 슬루). ω 가 omega_max 를 넘으면 v 를 줄인다.
        """
        p = self.p
        dt = max(1e-3, float(dt))
        k = float(kappa)
        self._prev_error = None
        self._meas_stamp, self._meas_e, self._meas_de = None, None, 0.0
        v_target = p.v_max * max(0.2, 1.0 - p.k_curv * abs(k) * p.v_max / p.omega_max)
        v_target *= max(0.0, float(speed_factor))
        if abs(k) * v_target > p.omega_max:
            v_target = p.omega_max / abs(k)
        if v_target > self._v_cmd:
            v_target = min(v_target, self._v_cmd + p.accel_slew * dt)
        self._v_cmd = v_target
        omega = max(-p.omega_max, min(p.omega_max, k * v_target))
        return v_target, omega
