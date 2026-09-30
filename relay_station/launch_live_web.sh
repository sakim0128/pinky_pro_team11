#!/usr/bin/env bash
# =============================================================================
# 주 대시보드(팀11 live 웹)를 중계 PC 의 관제 도메인 8 에서 띄운다 — 개편 1단계 (2026-09-29).
#
#   relay_station/launch_live_web.sh              # 지도 = 지금 고른 중계 좌표 프로파일의 지도
#   MAP_YAML=/path/map5.yaml relay_station/launch_live_web.sh
#   LIVE_WEB_CONTROL=true relay_station/launch_live_web.sh   # 미션 버튼 켜기 (아래 ⚠️)
#
# 왜 도메인 8 인가: 로봇 상태(/pinkyN/state · lane_status · amcl_pose · camera)는 중계 브리지가 10·11 → 8 로
# 올리고, 플릿 코디네이터(relay_station/fleet)도 8 에서 /fleet/lane/control · /fleet/lane/status 를 쓴다.
# live 웹은 그 토픽을 그대로 구독하므로 같은 도메인·같은 DDS 프로파일에 서기만 하면 된다.
#
# 지도: live 웹은 로봇이 보고한 지도(RobotState.map_*)가 자기 map_yaml 과 **원점·크기까지 같을 때만** 로봇을 그린다.
#   그래서 기본값은 중계가 지금 고른 좌표 프로파일의 지도(map4 등)다 — 로봇에게 ② 로봇 지도 전환으로 내려 준 것과 같다.
#   프로파일이 지도를 싣지 않으면(legacy) 팀11 기본 map5 로 뜬다.
#
# ⚠️ 미션 버튼(enable_control): live 웹 버튼은 ROS 로 바로 발행해 게이트웨이의 제어권 문을 지나지 않는다.
#   그래서 기본은 꺼짐(조회 전용)이고, 켜는 것은 **중계 PC 자신이 쓸 때만**이다 — 중계 PC 콘솔과 같은 급
#   ("로컬이 언제나 이긴다"). 팀원 노트북의 움직이는 명령은 :8889 (control_allow.json 허용 주소)로 낸다.
#   host 는 0.0.0.0 이라 팀원 노트북도 :8080 을 **본다** — 켜 두면 그들도 버튼을 누를 수 있으니 주의.
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export REPO_ROOT="${REPO_ROOT:-$(cd "$HERE/.." && pwd)}"
PINKY_WS="${PINKY_WS:-$HOME/pinky_pro}"

[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
[ -f "$PINKY_WS/install/setup.bash" ] && source "$PINKY_WS/install/setup.bash"
# DDS 프로파일은 브리지·게이트웨이와 같은 규칙으로 고른다 (다르면 같은 기계의 두 서비스가 서로 못 본다).
source "$HERE/domain_bridge/bridge_env.sh" >/dev/null
export ROS_DOMAIN_ID=8

if [ -z "${MAP_YAML:-}" ]; then
    MAP_YAML="$(cd "$HERE" && python3 - <<'PY'
import sys, types
pkg = types.ModuleType('fleet'); pkg.__path__ = ['fleet']; sys.modules['fleet'] = pkg   # __init__(rclpy) 없이 profiles 만
from fleet.profiles import load_profiles, resolve_active
default, profs = load_profiles()
name, _ = resolve_active(default, profs) if profs else (None, None)
p = profs.get(name) if name else None
print(p.map_yaml if (p is not None and getattr(p, 'map_yaml', None)) else '')
PY
)"
fi

# live_web.launch.xml 은 enable_control 을 인자로 받지 않는다 — 같은 노드를 같은 이름으로 직접 띄운다.
echo "[live-web] 도메인 $ROS_DOMAIN_ID · 지도 ${MAP_YAML:-(팀11 기본 map5)} · 미션 버튼 ${LIVE_WEB_CONTROL:-false}" >&2

exec ros2 run pinky_fleet_station live_web_node --ros-args -r __node:=fleet_live_web \
    -p host:=0.0.0.0 -p port:="${LIVE_WEB_PORT:-8080}" \
    ${MAP_YAML:+-p map_yaml:="$MAP_YAML"} \
    -p enable_control:="${LIVE_WEB_CONTROL:-false}"
