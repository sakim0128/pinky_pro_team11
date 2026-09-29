# -*- coding: utf-8 -*-
"""중계 상태 한 장 — GET /api/relay/health (개편 2단계, 2026-09-29).

주 대시보드는 팀11 live 웹(:8080)이다. 이 응답은 그 화면이 보여 주지 않는 **중계 자신의 일**만 답한다:
브리지가 로봇 토픽을 관제 도메인에 올리고 있나 · 코디네이터가 떠 있나 · 누가 제어권을 쥐었나 · 로봇 온보드
진단(`/pinkyN/diag`)이 무엇을 말하나. 중계 콘솔(static/relay_console.html)과 `:18081` 보기 전용이 이것만 본다.

ROS 를 모른다 — 게이트웨이가 잰 값을 받아 모양만 만든다(시험이 ROS 없이 돈다).

## 정직성
- 못 잰 값은 `None` 이다. 0 이나 false 로 채우지 않는다(발행자 수를 못 셌으면 "없다" 가 아니라 "모른다").
- 진단은 **중계가 받은 시각**으로 신선도를 잰다. 로봇 벽시계(stamp)는 기기마다 어긋나 쓰지 않는다.
- `ok` 는 셋이 다 참일 때만 참이다: 브리지로 상태가 올라온다 · 코디네이터가 그 로봇을 최근에 들었다 · 진단이 신선하다.
"""
import os

# live 웹이 로봇마다 구독하는 이름 — 브리지가 이것을 올려야 화면이 채워진다(tests/test_live_web_contract.py 와 같은 목록).
CONTRACT_TOPICS = ('state', 'lane_status', 'diag', 'amcl_pose', 'camera/image/compressed')
DIAG_FRESH_S = 3.0          # 진단은 1 Hz — 세 번 못 받으면 낡은 것
HEARD_FRESH_S = 2.0         # RobotState · LaneStatus 는 10 Hz (게이트웨이 ROBOT_HEARD_RECENTLY_SEC 와 같다)


def contract_topics(robot):
    return ['/%s/%s' % (robot, t) for t in CONTRACT_TOPICS]


def _diag_row(item, now):
    """(진단 dict, 받은 벽시계) → 콘솔 한 줄에 쓸 요약. 없으면 None."""
    if not item:
        return None
    d, got = item
    nav2 = d.get('nav2') or {}
    gate = d.get('gate') or {}
    tf = d.get('tf') or {}
    agent = d.get('agent') or {}
    age = None if got is None else max(0.0, now - got)
    return {
        'age_s': None if age is None else round(age, 1),
        'fresh': age is not None and age <= DIAG_FRESH_S,
        'pose_source': d.get('pose_source'),
        'tf_fresh': tf.get('fresh'),
        'nav2_active': nav2.get('active'),
        'nav2_total': nav2.get('total'),
        'nav2_not_active': list(nav2.get('not_active') or []),
        'estop': d.get('estop'),
        'gate_source': gate.get('source'),
        'map_load': agent.get('map_load'),
        'link_lost': agent.get('link_lost'),        # 링크유실 래치 — 로봇 재개 필요
    }


def _robot_row(name, link, coord_robot, diag_item, now):
    pubs = (link or {}).get(name) or {}
    counts = {t: pubs.get(t) for t in contract_topics(name)}
    state_pubs = counts.get('/%s/state' % name)
    heard = None if coord_robot is None else coord_robot.get('last_heard_sec')
    lane = (coord_robot or {}).get('lane_status') or {}
    diag = _diag_row(diag_item, now)
    uplink = None if state_pubs is None else state_pubs > 0
    heard_ok = None if heard is None else heard <= HEARD_FRESH_S
    ok = bool(uplink) and bool(heard_ok) and bool(diag and diag['fresh'])
    return {
        'name': name,
        'ok': ok,
        'uplink': uplink,
        'publishers': counts,
        'last_heard_sec': heard,
        'held': (coord_robot or {}).get('held'),
        'held_reason': (coord_robot or {}).get('held_reason'),
        'map_check': (coord_robot or {}).get('map_check'),
        'drive_state': lane.get('drive_state'),
        'state_reason': lane.get('state_reason'),
        'diag': diag,
    }


def build(robots, *, now, relay_domain=None, dds_uri=None, link=None, coord_status=None,
          coord_error=None, diag=None, control=None, live_web_up=None, live_web_port=8080):
    """게이트웨이가 잰 값들 → 한 장. robots = 로봇 이름 목록(코디네이터가 없어도 줄은 선다)."""
    coord_robots = (coord_status or {}).get('robots') or {}
    rows = [_robot_row(n, link, coord_robots.get(n), (diag or {}).get(n), now) for n in robots]
    coordinator = None
    if coord_status is not None:
        prof = coord_status.get('profile') or {}
        coordinator = {
            'present': True,
            'mission_state': coord_status.get('mission_state'),
            'estop_latched': coord_status.get('estop_latched'),
            'warning': coord_status.get('warning'),
            'profile': prof.get('active') if isinstance(prof, dict) else prof,
        }
    else:
        coordinator = {'present': False, 'error': coord_error}
    return {
        'timestamp': now,
        'relay_domain': relay_domain,
        'dds_profile': None if not dds_uri else os.path.basename(str(dds_uri).replace('file://', '')),
        'coordinator': coordinator,
        'control': control,
        'live_web': {'port': live_web_port, 'up': live_web_up},
        'robots': rows,
        'ok': bool(coordinator.get('present')) and all(r['ok'] for r in rows),
    }
