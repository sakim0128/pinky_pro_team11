#!/usr/bin/env bash
# =============================================================================
# 🌉 도메인 브릿지 설정 생성기
#
#   생성물 (relay_station/domain_bridge/configs/)
#     pinkyN_control.yaml  로봇 도메인(10~13) <-> 관제 도메인(8)  [비대칭]
#     team_mirror.yaml     관제 도메인(8)      -> 팀원 도메인(9)   [읽기 전용]
#
#   로봇 이름은 pinkyN 하나다(2026-09-29): 토픽 접두어·파일 이름·유닛 인스턴스 이름 전부.
#   로봇은 lane_agent_node 뿐이다 — Nav2 직접 목표(goal_pose·mission_cmd)·관제 teleop(cmd_vel→cmd_vel_teleop 게이트)·
#   nav_status 는 받는 쪽도 내는 쪽도 없어 지웠다. 움직이는 명령은 Route·LaneCommand·FleetCommand(플릿 코디네이터)뿐.
#
#   ⚠️ 비대칭 원칙 — 이 설계의 안전 근간
#     10~13 -> 8 : 센서/상태 전량 (업링크)
#     8 -> 10~13 : 제어 화이트리스트만 (route / lane_command / pose_fix / overhead_pose / command)
#     8 -> 9     : 읽기 전용 미러
#     9 -> 8     : **teleop 이름만** (2026-09-12 사용자 지시로 신설 · 기본 꺼짐)
#
#   ⭐ 원래 원칙은 `9 -> 어디든 : 없음` 이었고 근거는 "팀원 시뮬레이션 cmd_vel 이
#      실물 로봇으로 나가는 사고 차단" 이었다. 막아야 할 것은 **팀원**이 아니라
#      **시뮬 트래픽**이므로, 길을 막는 대신 **이름을 가른다** (아래 벌 2 참고).
#
#   ⚠️ QoS 를 전부 명시하는 이유
#     domain_bridge 는 기동 시점에 기존 publisher 의 QoS 를 탐지한다.
#     로봇보다 브릿지가 먼저 뜨면 기본값(RELIABLE)으로 굳어져 BEST_EFFORT 로
#     발행되는 /scan·카메라와 매칭에 실패하고, 에러 한 줄 없이 조용히 빈 토픽이
#     된다. 아래처럼 명시하면 기동 순서에 의존하지 않는다.
#
#   ⚠️ TF 주의
#     브릿지는 토픽 이름은 바꿔도 메시지 내부 frame_id 는 바꾸지 못한다.
#     로봇 4대의 /tf 를 도메인 8 에 모으려면 발행 측 frame_prefix 가 필수이며,
#     pinky_bringup 의 frame_prefix 파라미터와 robotN.env 의 FRAME_PREFIX 가
#     이를 담당한다.
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$SCRIPT_DIR/configs"
mkdir -p "$OUT"

RELAY_DOMAIN=8
TEAM_DOMAIN=9
ROBOTS=(1 2 3 4)
DOMAINS=(10 11 12 13)

# ── 감싼 서비스·액션 (topic_wrap.py) ────────────────────────────────────────
#
# 브리지는 토픽만 나른다. 서비스·액션은 로봇 위의 `topic_wrap.py` 가 **토픽 쌍으로
# 감싸서** 내놓고, 여기서는 그 토픽을 나르기만 한다. 이름 규칙은 `topic_wrap.py` 의
# `topic_names()` 가 정하고 **여기는 그것을 따른다** — 두 곳에 따로 적으면 조용히
# 어긋나고, 증상은 "브리지가 안 나른다" 로 보인다
# (시험 `test_topic_wrap.py::test_생성기와_어댑터가_같은_이름을_쓴다` 가 고정).
#
#   pinkyN/svc/<이름>/request   8 -> N   부른다
#   pinkyN/svc/<이름>/result    N -> 8   답한다
#   pinkyN/act/<이름>/goal·cancel   8 -> N
#   pinkyN/act/<이름>/feedback·result  N -> 8
#
# ⚠️ 팀원(도메인 9)에게 여는 것은 **기본이 아니다.** 열려면 WRAP_SERVICES_TEAM 에
#    적고, 그러면 이름이 `pinkyN/teleop/svc/...` 로 나간다 — 벌 2 는 teleop 접두어만
#    나른다는 규칙을 그대로 둔 채 들어간다.
WRAP_SERVICES=(led)          # 관제국이 부를 수 있는 것
WRAP_ACTIONS=()              # 아직 없음 — 액션은 실기 검증 전이다
WRAP_SERVICES_TEAM=()        # 팀원에게 연 것 (기본: 없음)

# ── 로봇별 제어 브릿지 (N <-> 8) ─────────────────────────────────────────────
for i in "${!ROBOTS[@]}"; do
    n="${ROBOTS[$i]}"
    d="${DOMAINS[$i]}"
    cat > "$OUT/pinky${n}_control.yaml" <<EOF
# 자동 생성: generate_configs.sh — 직접 편집하지 말 것
# 로봇 pinky${n} <-> 관제 평면 브릿지 (도메인 ${d} <-> ${RELAY_DOMAIN})
name: pinky_bridge_pinky${n}
from_domain: ${d}
to_domain: ${RELAY_DOMAIN}

topics:

  # ══ 업링크: 로봇(${d}) → 관제(${RELAY_DOMAIN}) ══════════════════════════════

  pinky${n}/odom:
    type: nav_msgs/msg/Odometry
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}

  pinky${n}/pose:
    type: geometry_msgs/msg/PoseStamped
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}

  # 로봇 안에서는 표준 이름 /scan 을 유지하고(온보드 SLAM·Nav2 호환),
  # 관제 도메인에 합류할 때만 로봇별 이름으로 분리한다.
  scan:
    type: sensor_msgs/msg/LaserScan
    remap: pinky${n}/scan
    qos: {reliability: best_effort, durability: volatile, history: keep_last, depth: 5}

  # 원본(image_raw)은 절대 중계하지 않는다. 2.4GHz 채널 11에서 4대분 원본을
  # 8·9 두 도메인으로 복제하면 AP가 즉시 포화된다. 압축본만 통과시킨다.
  pinky${n}/camera/image_raw/compressed:
    type: sensor_msgs/msg/CompressedImage
    qos: {reliability: best_effort, durability: volatile, history: keep_last, depth: 2}

  pinky${n}/battery_state:
    type: sensor_msgs/msg/BatteryState
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 5}

  # frame_prefix 로 pinky${n}/ 접두사가 붙어 있어야 4대 병합 시 충돌하지 않는다.
  tf:
    type: tf2_msgs/msg/TFMessage
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 100}

  tf_static:
    type: tf2_msgs/msg/TFMessage
    qos: {reliability: reliable, durability: transient_local, history: keep_last, depth: 100}

  # ══ 다운링크: 관제(${RELAY_DOMAIN}) → 로봇(${d}) — 화이트리스트 ═══════════════
  # 관제국은 로봇별로 구분된 이름으로 발행하고, 브릿지가 해당 로봇 도메인
  # 안에서만 표준 이름으로 되돌린다. 이렇게 해야 하나의 명령이 4대
  # 전부에게 동시에 전달되는 사고가 발생하지 않는다.
  # (goal_pose·mission_cmd·cmd_vel 다운링크는 2026-09-29 에 지웠다 — 로봇은 lane_agent_node 뿐이다.)

EOF

    # ══ Track R (Mini Project 2) 신규 5대 계약 토픽은 오직 pinky1 & pinky2 에만 적용 ══
    # pinky3, pinky4 는 본 프로젝트 운영 범위에서 제외되므로 이전 기준을 그대로 보존한다.
    if [ "$n" -eq 1 ] || [ "$n" -eq 2 ]; then
        cat >> "$OUT/pinky${n}_control.yaml" <<EOF

  # ══ 2026-09-22 신설: Team11 멀티로봇 관제 연동 5대 핵심 계약 토픽 (pinky1/2 한정) ══
  # 1) RobotState (업링크: 로봇 도메인 -> 관제 도메인 8)
  /pinky${n}/state:
    type: pinky_fleet_msgs/msg/RobotState
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}

  # 2) LaneStatus (업링크: 로봇 도메인 -> 관제 도메인 8)
  /pinky${n}/lane_status:
    type: pinky_lane_msgs/msg/LaneStatus
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}

  # 3) Route (다운링크: 관제 도메인 8 -> 로봇 도메인)
  /pinky${n}/route:
    type: pinky_lane_msgs/msg/Route
    from_domain: ${RELAY_DOMAIN}
    to_domain: ${d}
    qos: {reliability: reliable, durability: transient_local, history: keep_last, depth: 1}

  # 4) LaneCommand (다운링크: 관제 도메인 8 -> 로봇 도메인)
  /pinky${n}/lane_command:
    type: pinky_lane_msgs/msg/LaneCommand
    from_domain: ${RELAY_DOMAIN}
    to_domain: ${d}
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}

  # 5) PoseFix (다운링크: 관제 도메인 8 -> 로봇 도메인)
  /pinky${n}/pose_fix:
    type: pinky_lane_msgs/msg/PoseFix
    from_domain: ${RELAY_DOMAIN}
    to_domain: ${d}
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 5}

  # 5-b) 항공뷰 위치 (다운링크: 관제 도메인 8 -> 로봇 도메인) — 팀11 overhead_tracker_node(ArUco) 가 내고
  #      로봇 pose_fuser_node(lane_robot.launch.xml use_overhead) 가 map->odom TF 로 쓴다 (2026-09-29).
  #      overhead_tracker 가 도메인 0 에서 돌면 0->8 미러(팀 브리지)를 먼저 거쳐야 한다.
  /pinky${n}/overhead_pose:
    type: geometry_msgs/msg/PoseStamped
    from_domain: ${RELAY_DOMAIN}
    to_domain: ${d}
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}

  # 6) FleetCommand (다운링크: 관제 도메인 8 -> 로봇 도메인, pinky1/2 한정 관리/주행 명령 브리지) [R-D3]
  #    Team11 lane_agent_node 연동용 (CMD_SET_INITIAL_POSE, CMD_SET_MAP, STOP/RESUME/HEARTBEAT).
  #    RelayFleetCoordinator 가 경로(Route)·허가(LaneCommand)·관리 명령(FleetCommand)을 낸다.
  /pinky${n}/command:
    type: pinky_fleet_msgs/msg/FleetCommand
    from_domain: ${RELAY_DOMAIN}
    to_domain: ${d}
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}

  # 7) 진단 (업링크: 로봇 도메인 -> 관제 도메인 8) [R-5, 2026-09-24]
  #    RobotState 는 팀11 과 바이트 동일해야 해 늘릴 수 없다 → 별도 JSON std_msgs/String, 1 Hz.
  #    fix_status · Nav2 lifecycle · /estop · 게이트 소스 · 에이전트 상태 — 관제 화면이 "어느 구간이 끊겼나"
  #    에 답하는 재료. 스키마는 pinky_fleet_agent/pinky_fleet_agent/diag.py 가 정본이다.
  /pinky${n}/diag:
    type: std_msgs/msg/String
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 5}

  # 8) 차선 인식 (2026-09-29): 관제 PC 의 lane_pipeline_node(pinky_lane_station)가 **도메인 8** 에서 돈다.
  #    로봇 camera_node 의 압축 영상(업링크, 이름·QoS 는 pinky_lane_station/config/bridge_pinkyN_up.yaml 과 같다)과
  #    파이프라인이 내는 LanePath(다운링크). 이 둘이 없으면 레인 로봇은 차선을 못 본다.
  /pinky${n}/camera/image/compressed:
    type: sensor_msgs/msg/CompressedImage
    qos: {reliability: best_effort, durability: volatile, history: keep_last, depth: 1}

  /pinky${n}/lane_path:
    type: pinky_lane_msgs/msg/LanePath
    from_domain: ${RELAY_DOMAIN}
    to_domain: ${d}
    qos: {reliability: best_effort, durability: volatile, history: keep_last, depth: 1}

  # 9) 비전 미션 (2026-09-30): 코디네이터 비전 모드가 내는 교차로 고정 동작·도착 정지선 수 (다운링크, 늦게 떠도 받게)
  /pinky${n}/junction_plan:
    type: pinky_lane_msgs/msg/JunctionPlan
    from_domain: ${RELAY_DOMAIN}
    to_domain: ${d}
    qos: {reliability: reliable, durability: transient_local, history: keep_last, depth: 1}

  # 10) 항공뷰 이탈 보정 (2026-10-01): 비전 코디네이터가 항공뷰 좌표로 잰 차선 중앙 이탈 (다운링크, 10 Hz, 낡은 건 버린다)
  /pinky${n}/lane_correction:
    type: geometry_msgs/msg/Vector3Stamped
    from_domain: ${RELAY_DOMAIN}
    to_domain: ${d}
    qos: {reliability: best_effort, durability: volatile, history: keep_last, depth: 1}
EOF
    fi


    # ── 감싼 서비스·액션 (topic_wrap.py 가 로봇 위에서 내놓는 토픽) ──────────
    for s in ${WRAP_SERVICES[@]+"${WRAP_SERVICES[@]}"}; do
        cat >> "$OUT/pinky${n}_control.yaml" <<EOF

  # 감싼 서비스 '${s}' — 로봇 위 topic_wrap.py 가 서비스를 이 토픽 쌍으로 바꿔 준다.
  # 이름 규칙의 주인은 topic_wrap.py 의 topic_names() 다.
  pinky${n}/svc/${s}/request:
    type: std_msgs/msg/String
    from_domain: ${RELAY_DOMAIN}
    to_domain: ${d}
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}

  pinky${n}/svc/${s}/result:
    type: std_msgs/msg/String
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}
EOF
    done
    for a in ${WRAP_ACTIONS[@]+"${WRAP_ACTIONS[@]}"}; do
        cat >> "$OUT/pinky${n}_control.yaml" <<EOF

  # 감싼 액션 '${a}'. ⚠️ 취소는 브리지를 건너느라 늦는다 — 비상 정지로 쓰지 말 것.
  #    비상 정지는 teleop_out 유닛을 끄는 것이고, 그건 바퀴로 가는 길을 끊는다.
  pinky${n}/act/${a}/goal:
    type: std_msgs/msg/String
    from_domain: ${RELAY_DOMAIN}
    to_domain: ${d}
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 5}

  pinky${n}/act/${a}/cancel:
    type: std_msgs/msg/String
    from_domain: ${RELAY_DOMAIN}
    to_domain: ${d}
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 5}

  pinky${n}/act/${a}/feedback:
    type: std_msgs/msg/String
    qos: {reliability: best_effort, durability: volatile, history: keep_last, depth: 5}

  pinky${n}/act/${a}/result:
    type: std_msgs/msg/String
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 5}
EOF
    done
    echo "  ● $OUT/pinky${n}_control.yaml   (도메인 ${d} <-> ${RELAY_DOMAIN})"
done

# ── 팀원 미러 브릿지 (8 → 9, 전 로봇 일괄) ──────────────────────────────────
{
cat <<EOF
# 자동 생성: generate_configs.sh — 직접 편집하지 말 것
# 팀원 읽기 전용 미러 (도메인 ${RELAY_DOMAIN} -> ${TEAM_DOMAIN})
#
# 팀원이 도메인 ${TEAM_DOMAIN} 한 곳에 머문 채로 로봇 4대를 동시에 관측한다.
# 로봇 도메인마다 옮겨 다닐 필요가 없다.
#
# ⚠️ 2026-09-12 정정 — 이 자리에 예전에는 이렇게 적혀 있었다:
#     "반대 방향 경로가 존재하지 않으므로 팀원 쪽에서 실물 로봇으로 명령이 나가는
#      일이 **구조적으로 불가능**하다."
#    사용자 지시로 팀원 주행 관제가 생기면서 **그 문장은 더 이상 사실이 아니다.**
#    없는 보호를 있다고 적어 두는 것이 보호가 없는 것보다 나쁘므로 지운다.
#
#    지금의 보호는 "경로가 없다" 가 아니라 **"이름이 갈려 있다"** 이다:
#      팀원 시뮬이 쓰는 pinkyN/cmd_vel        -> 아무 데도 안 간다 (여기에도 없다)
#      조종기 전용 pinkyN/teleop/cmd_vel      -> 벌 2 가 나른다 (pinkyN_teleop_*.yaml)
#    그리고 벌 2 는 **기본이 꺼짐**이라, 켜는 행위가 곧 "이 로봇을 넘긴다" 는 뜻이다.
#
# 이 미러는 여전히 **읽기 전용**이다. 여기 나열된 토픽이 팀원에게 노출되는 전부이고,
# 이 목록에 명령이 들어가서는 안 된다 (test_bridge_two_sets.py 가 고정).
name: pinky_bridge_team_mirror
from_domain: ${RELAY_DOMAIN}
to_domain: ${TEAM_DOMAIN}

topics:

  tf:
    type: tf2_msgs/msg/TFMessage
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 100}

  tf_static:
    type: tf2_msgs/msg/TFMessage
    qos: {reliability: reliable, durability: transient_local, history: keep_last, depth: 100}

  map:
    type: nav_msgs/msg/OccupancyGrid
    qos: {reliability: reliable, durability: transient_local, history: keep_last, depth: 1}

  bridge/health:
    type: std_msgs/msg/String
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 5}
EOF
for n in "${ROBOTS[@]}"; do
cat <<EOF

  pinky${n}/odom:
    type: nav_msgs/msg/Odometry
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}

  pinky${n}/pose:
    type: geometry_msgs/msg/PoseStamped
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}

  pinky${n}/scan:
    type: sensor_msgs/msg/LaserScan
    qos: {reliability: best_effort, durability: volatile, history: keep_last, depth: 5}

  pinky${n}/camera/image_raw/compressed:
    type: sensor_msgs/msg/CompressedImage
    qos: {reliability: best_effort, durability: volatile, history: keep_last, depth: 2}

  pinky${n}/battery_state:
    type: sensor_msgs/msg/BatteryState
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 5}
EOF
done
} > "$OUT/team_mirror.yaml"
echo "  ● $OUT/team_mirror.yaml         (도메인 ${RELAY_DOMAIN} -> ${TEAM_DOMAIN}, 읽기 전용)"

# ═══════════════════════════════════════════════════════════════════════════
#  벌 2 — 팀원 주행 관제 (사용자 지시 2026-09-12)
# ═══════════════════════════════════════════════════════════════════════════
#
# 사용자 지시: "팀원 노트북도 영상 처리는 안 되어도 **직접 주행에 대한 관제는
# 기능 구현이 되어야 한다.**"
#
# 이 파일 머리말의 비대칭 원칙은 `9 -> 어디든 : 없음` 이었고, 그 근거는
# **"팀원 시뮬레이션 cmd_vel 이 실물 로봇으로 나가는 사고 차단"** 이었다.
# 그 사고는 지금도 막아야 한다. 그래서 길을 여는 대신 **이름을 가른다.**
#
#   팀원이 시뮬에 쓰는 이름   pinky1/cmd_vel      <- 벌 2 가 **안 본다**
#   주행 관제 전용 이름       pinky1/teleop/cmd_vel <- 벌 2 가 이것만 본다
#
# 조종기가 일부러 이 이름으로 쏘지 않는 한 아무것도 안 나간다. 시뮬을 아무리
# 돌려도 실물은 안 움직인다 — 원래 막으려던 사고가 그대로 막힌다.
#
# ⭐ 그리고 **두 벌로 쪼갠다.** 한 벌이 아니라 들어오는 다리와 나가는 다리를
#    따로 둔다:
#
#     pinkyN_teleop_in.yaml    9 -> 8   팀원이 보낸 것을 **관제국이 본다**
#     pinkyN_teleop_out.yaml   8 -> N   그것을 **로봇에게 내보낸다**
#
#    이렇게 하면 강사가 `_out` 만 끌 수 있다. 그러면 팀원이 무엇을 시키려는지는
#    화면에 그대로 보이면서 **로봇은 안 움직인다** — 관측을 잃지 않는 정지다.
#    반대로 `_in` 만 끄면 관제국이 못 보게 되므로, 끄는 순서는 항상 `_out` 이 먼저다.
#
# ⚠️ 루프 주의: 미러는 8 -> 9 다. 만약 미러 목록에 `teleop` 이름이 들어가면
#    9 -> 8 -> 9 로 메시지가 돌게 된다. 미러 목록과 teleop 이름은 **겹치면 안 된다**
#    (시험 `test_bridge_two_sets.py::test_미러와_teleop_이름이_안_겹친다` 가 고정).
#
# ⚠️ 기본은 꺼짐이다. 유닛을 `enable` 하는 것이 곧 "이 로봇을 팀원에게 넘긴다" 는
#    행위다. `install_bridge.sh` 는 이 벌을 **자동으로 켜지 않는다.**
for i in "${!ROBOTS[@]}"; do
    n="${ROBOTS[$i]}"
    d="${DOMAINS[$i]}"

    cat > "$OUT/pinky${n}_teleop_in.yaml" <<EOF
# 자동 생성: generate_configs.sh — 직접 편집하지 말 것
# 벌 2-A · 팀원(${TEAM_DOMAIN}) → 관제(${RELAY_DOMAIN})  [주행 명령을 관제국에 보인다]
#
# 여기까지는 로봇에 닿지 않는다. 내보내는 것은 pinky${n}_teleop_out.yaml 이다.
name: pinky_teleop_in_pinky${n}
from_domain: ${TEAM_DOMAIN}
to_domain: ${RELAY_DOMAIN}

topics:

  # 팀원 조종기 전용 이름. 팀원이 시뮬에 쓰는 pinky${n}/cmd_vel 과 **다른 이름**이라
  # 시뮬 트래픽은 여기에 걸리지 않는다.
  pinky${n}/teleop/cmd_vel:
    type: geometry_msgs/msg/Twist
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}

  # 누가 몰고 있는지. 조종기가 자기 이름을 함께 낸다 — 화면이 "누구의 명령인가" 를
  # 말할 수 있어야 강사가 감독할 수 있다.
  pinky${n}/teleop/operator:
    type: std_msgs/msg/String
    qos: {reliability: reliable, durability: transient_local, history: keep_last, depth: 1}
EOF
    # 팀원에게 연 감싼 서비스 — teleop 밑으로 들어가므로 벌 2 의 규칙이 안 깨진다
    for s in ${WRAP_SERVICES_TEAM[@]+"${WRAP_SERVICES_TEAM[@]}"}; do
        cat >> "$OUT/pinky${n}_teleop_in.yaml" <<EOF

  pinky${n}/teleop/svc/${s}/request:
    type: std_msgs/msg/String
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}
EOF
        cat >> "$OUT/pinky${n}_teleop_out.yaml" <<EOF

  pinky${n}/teleop/svc/${s}/request:
    type: std_msgs/msg/String
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}
EOF
    done
    echo "  ● $OUT/pinky${n}_teleop_in.yaml   (도메인 ${TEAM_DOMAIN} -> ${RELAY_DOMAIN})"

    cat > "$OUT/pinky${n}_teleop_out.yaml" <<EOF
# 자동 생성: generate_configs.sh — 직접 편집하지 말 것
# 벌 2-B · 관제(${RELAY_DOMAIN}) → 로봇 pinky${n}(${d})  [실제로 바퀴가 도는 다리]
#
# ⚠️ 이 유닛을 끄면 로봇이 즉시 안 움직인다. 관측과 화면은 그대로다.
#    팀원에게 로봇을 넘기고 회수하는 손잡이가 바로 이것이다.
name: pinky_teleop_out_pinky${n}
from_domain: ${RELAY_DOMAIN}
to_domain: ${d}

topics:

  # 로봇 안에서는 표준 이름으로 되돌린다 — 온보드는 고칠 것이 없다.
  pinky${n}/teleop/cmd_vel:
    type: geometry_msgs/msg/Twist
    remap: cmd_vel
    qos: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}
EOF
    echo "  ● $OUT/pinky${n}_teleop_out.yaml  (도메인 ${RELAY_DOMAIN} -> ${d})"
done

echo
echo "생성 완료. 적용: relay_station/domain_bridge/install_bridge.sh"
echo "팀원 주행 벌은 기본으로 안 켠다 — 넘길 때만:"
echo "  systemctl --user start pinky-domain-bridge@pinky1_teleop_in"
echo "  systemctl --user start pinky-domain-bridge@pinky1_teleop_out"
