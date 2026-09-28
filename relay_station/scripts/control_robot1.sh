#!/usr/bin/env bash
# =============================================================================
# 🚀 Pinky #1 Terminal Mission & Navigation Controller
# 현장 중계 노트북 터미널에서 로봇 1호기를 직접 조작하는 대화형 & 원클릭 CLI 도구
# =============================================================================

# ROS 2 환경 로드
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$HOME/pinky_pro/install/setup.bash" ]; then
    source $HOME/pinky_pro/install/setup.bash
fi

# 관제 콘솔은 관제 평면(도메인 8)에서 동작한다.
# 여기서 발행한 /robot1/cmd_vel · /robot1/mission_cmd 를 도메인 브릿지가
# 로봇 1호기 도메인(10) 안의 /cmd_vel · /robot1/mission_cmd 로 내려보낸다.
# 로봇별로 이름을 분리했기 때문에 하나의 명령이 4대에 동시에 나가는 일이 없다.
export ROS_DOMAIN_ID=8
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/pinky_pro/src/pinky_pro_team11/configs/cyclonedds.xml"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
# 게이트웨이 주소 — 시험은 가짜 서버로 바꾼다(라이브 게이트웨이에 명령을 보내지 않게).
GATEWAY_URL="${GATEWAY_URL:-http://127.0.0.1:8889}"
# 비상정지의 0 속도 직접 발행(python) 상한(초). 검수 실측 0.66~2.2 s(제3자 검수 G-8 · REVIEW_20260926 §6.4).
# 멈추면(hang) 이 시간 뒤 끊고 "시간 초과" 로 알린다 — 정지 API 는 이것을 기다리지 않는다.
# 실측 최악(2.22 s) 가까이로 내리면 부하 걸린 노트북에서 제대로 나가던 0 속도를 끊는다 — 시험이 2.5 s 로 붙잡는다(검수 R-script-2).
ESTOP_PUB_TIMEOUT_SEC=5

# 종료코드 (제3자 검수 G-12 — 예전엔 거절·실패에도 늘 0 이었다):
#   0 = 게이트웨이가 처리했다고 답했다(200). 비상정지는 0 속도 발행 성공 + 정지 확인(200) 둘 다일 때만.
#   1 = 실패 · 거절 · 보내지 못함 · 잘못된 입력
#   2 = 보냈으나 결과를 모른다(202 — 게이트웨이가 그렇게 말한다)

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# 게이트웨이에 POST 하고 "HTTP코드 본문" 을 찍는다. 게이트웨이에 닿지 못하면 000.
# ⚠️ 2026-09-26 (관제 검수 S1): 게이트웨이가 ESTOP·정지·HOLD 중 목표를 409 로 거절한다. 예전 스크립트는
#    curl 종료코드만 봐서 409 에도 "전송 완료" 를 찍었다. 그리고 **거절을 실패로 보고 토픽 직접 발행으로
#    넘어가면 거절을 우회한다**.
# ⚠️ 제3자 검수 G-7 (REVIEW_20260926): 예전엔 닿지 못할 때(000)만 목표·미션을 토픽으로 직접 발행했다. 그 순간은
#    게이트웨이(=플릿 코디네이터) 프로세스가 죽은 때라 플릿 래치(비상정지·정지·HOLD)가 전부 없다 — 이제 움직이는
#    명령은 **어떤 경우에도** 직접 발행하지 않는다. 직접 발행은 멈추는 쪽(비상정지의 0 속도)만 남는다.
_post() {
    local out rc
    out=$(curl -s -m "${3:-2}" -w $'\n%{http_code}' -X POST "${GATEWAY_URL}$1" \
          -H "Content-Type: application/json" -d "$2" 2>/dev/null)
    rc=$?
    if [ $rc -ne 0 ]; then
        # 000 = 게이트웨이에 **닿지 못했다**(연결 거부 7 · 이름 못 풂 6).
        # 느리거나(28) 응답이 끊긴(52·56) 게이트웨이는 살아 있다 — 이미 처리했을 수도, 거절했을 수도 있다
        # (직렬 검토: 그걸 닿지 못한 것으로 치면 409 를 우회하거나 목표가 두 번 간다).
        case $rc in 6|7) echo "000" ;; *) echo "ERR curl=$rc (게이트웨이는 살아 있을 수 있다 — 처리했는지 모른다)" ;; esac
        return
    fi
    echo "$(printf '%s' "$out" | tail -1) $(printf '%s' "$out" | sed '$d')"
}

# 게이트웨이에 닿지 못했을 때 움직이는 명령을 거절한다(제3자 검수 G-7). 호출자는 1 로 끝난다.
_refuse_unreachable() {
    echo -e "${RED}⛔ 게이트웨이(${GATEWAY_URL})에 닿지 못했다(연결 거부·이름 못 풂) — $1 을(를) 보내지 않았다.${NC}"
    echo -e "${RED}   게이트웨이(=플릿 코디네이터)가 죽은 동안은 비상정지·정지·HOLD 래치가 없다 — 토픽 직접 발행으로 움직이지 않는다.${NC}"
    echo -e "${RED}   게이트웨이를 살린 뒤 다시 보낸다. 멈추는 것은 된다: $0 stop (0 속도 직접 발행).${NC}"
}

# 도 → 라디안. 숫자가 아니면 1. 제3자 검수 G-12: 예전엔 이 python 의 종료코드를 안 봐서 빈 값이
# send_goal 의 기본값 0.0 이 되었다 — 잘못 친 Heading 이 조용히 0° 목표로 나갔다.
# 검수 R-script-1: 입력이 아니라 **결과**가 유한해야 한다 — 1e308° 는 곱하다 inf 가 되어 "inf" 로 나갔다.
_deg2rad() {
    python3 -c 'import math, sys
r = float(sys.argv[1]) * math.pi / 180.0
if not math.isfinite(r):
    sys.exit(1)
print(r)' "$1" 2>/dev/null
}

# 목표 JSON 본문. X·Y·Yaw(rad) 가 전부 유한한 수일 때만 만든다 — 아니면 1(검수 R-script-1).
# 예전엔 셋을 JSON 에 그대로 끼워 넣었다: 게이트웨이는 깨진 본문을 {} 로 보고 x·y·yaw 기본값 0 을 쓴다 —
# 잘못 친 X('abc'·'0.5m')나 메뉴에서 Enter 만 친 X 가 **(0,0,0) 목표**로 나가고 "완료"·종료코드 0 이었다.
# NaN·1e999 는 그 JSON 파서가 받아 NaN·Infinity 목표가 됐고, '0.5, "x": 0.1' 은 다른 키를 끼워 넣었다.
_goal_body() {
    python3 -c 'import json, math, sys
v = [float(a) for a in sys.argv[1:4]]
if not all(math.isfinite(a) for a in v):
    sys.exit(1)
print(json.dumps({"x": v[0], "y": v[1], "yaw": v[2]}))' "$1" "$2" "$3" 2>/dev/null
}

send_goal() {
    local X="$1"
    local Y="$2"
    local YAW="${3:-0.0}"
    local body r code
    if ! body=$(_goal_body "$X" "$Y" "$YAW"); then
        echo -e "${RED}⛔ 잘못된 좌표: X='${X}' Y='${Y}' Yaw='${YAW}'(rad) — 유한한 수가 아니다. 목표를 보내지 않는다${NC}"
        return 1
    fi
    echo -e "${YELLOW}⏳ [목표 전송] X=${X}m, Y=${Y}m, Yaw=${YAW}rad -> 전송 중...${NC}"

    r=$(_post /api/robot1/goal "$body")
    code=${r%% *}
    if [ "$code" = "200" ]; then
        echo -e "${GREEN}✅ 게이트웨이 경유 목표 전송 완료! (A* 경로 생성 및 실물/가제보 동시 주행)${NC}"
        return 0
    elif [ "$code" = "ERR" ]; then
        echo -e "${RED}⚠️ 게이트웨이 응답 없음·끊김 — 직접 발행하지 않는다: ${r#* }${NC}"
    elif [ "$code" != "000" ]; then
        echo -e "${RED}⛔ 게이트웨이가 거절 (HTTP $code) — 직접 발행하지 않는다: ${r#* }${NC}"
    else
        # 제3자 검수 G-7: 예전엔 여기서 /robot1/goal_pose 로 직접 발행했다(플릿 래치가 전부 없는 순간).
        _refuse_unreachable "목표"
    fi
    return 1
}

send_mission() {
    local MISSION="$1"
    echo -e "${YELLOW}⏳ [미션 전송] '${MISSION}' -> 전송 중...${NC}"
    local r code
    r=$(_post /api/robot1/mission "{\"mission\": \"$MISSION\"}")
    code=${r%% *}
    if [ "$code" = "200" ] || [ "$code" = "202" ]; then
        # 202 = 보냈지만 받는 쪽은 모른다(게이트웨이가 그렇게 말한다) — 성공이라 하지 않는다(종료코드 2)
        echo -e "${YELLOW}📨 미션 보냄 (HTTP $code): ${r#* }${NC}"
        [ "$code" = "200" ] && return 0
        return 2
    elif [ "$code" = "ERR" ]; then
        echo -e "${RED}⚠️ 게이트웨이 응답 없음·끊김 — 직접 발행하지 않는다: ${r#* }${NC}"
    elif [ "$code" != "000" ]; then
        echo -e "${RED}⛔ 게이트웨이가 거절 (HTTP $code) — 직접 발행하지 않는다: ${r#* }${NC}"
    else
        # 제3자 검수 G-7: 예전엔 여기서 /robot1/mission_cmd 로 직접 발행했다(플릿 래치가 전부 없는 순간).
        _refuse_unreachable "미션 '${MISSION}'"
    fi
    return 1
}

emergency_stop() {
    echo -e "${RED}🛑 [긴급 정지 발동] 모터 정지 및 모든 경로 취소 중...${NC}"
    # ⚠️ 관제 검수 §3.4: 예전엔 정지 API 를 먼저 불렀다 — 그 API 는 로봇의 멈춤 보고를 최대 1 s 기다린다.
    # ⚠️ 제3자 검수 G-8 (REVIEW_20260926): 그래서 0 속도 python 을 먼저 끝내고 API 를 불렀더니, Nav2 목표를 취소하는
    #    HOLD 가 python 시간(검수 실측 0.66~2.2 s)만큼 늦게 나갔고, python 이 멈추면(타임아웃 없음) API 는 끝내 안 불렸다.
    #    → 둘을 **동시에** 보낸다: 0 속도는 API 를, API 는 0 속도를 기다리지 않는다. 둘 다 상한이 있다
    #      (python = timeout ESTOP_PUB_TIMEOUT_SEC · API = curl -m 3). 결과는 둘 다 끝난 뒤 종료코드를 보고 말한다(G-12).
    local py_pid py_rc r code rc=0
    timeout -k 1 "$ESTOP_PUB_TIMEOUT_SEC" python3 -c "
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

rclpy.init()
node = Node('term_stop_pub')
pub_v = node.create_publisher(Twist, '/robot1/cmd_vel', 10)
pub_m = node.create_publisher(String, '/robot1/mission_cmd', 10)
t = Twist()
s = String()
s.data = 'stop'
for _ in range(10):
    pub_v.publish(t)
    pub_m.publish(s)
    rclpy.spin_once(node, timeout_sec=0.02)
node.destroy_node()
rclpy.shutdown()
" &
    py_pid=$!
    r=$(_post /api/robot1/stop '{}' 3)
    wait "$py_pid"
    py_rc=$?
    code=${r%% *}

    # 제3자 검수 G-12: 예전엔 python 이 실패해도 "0 속도 직접 발행 완료" 를 찍었다.
    case $py_rc in
        0)  echo -e "${RED}🛑 0 속도 직접 발행 완료 (V=0, W=0)${NC}" ;;
        124|137)
            echo -e "${RED}⛔ 0 속도 직접 발행 시간 초과(${ESTOP_PUB_TIMEOUT_SEC} s) — 나갔는지 모른다${NC}"
            rc=1 ;;
        *)
            echo -e "${RED}⛔ 0 속도 직접 발행 실패 (python 종료코드 $py_rc) — 0 속도가 나갔다고 보지 않는다${NC}"
            # CYCLONEDDS_URI 는 위에서 현장 NIC 에 묶인 설정으로 고정했다 — 그 NIC 가 없는 곳에선 도메인을 못 만든다.
            echo -e "${RED}   CYCLONEDDS_URI(${CYCLONEDDS_URI})는 현장 NIC 에 묶여 있다 — 현장 밖(그 NIC 가 없는 곳)에서는 rclpy 가 실패한다${NC}"
            rc=1 ;;
    esac
    if [ "$code" = "200" ]; then
        echo -e "${RED}   정지 API: HTTP 200 ${r#* }${NC}"
    elif [ "$code" = "202" ]; then
        echo -e "${YELLOW}⚠️ 정지 API: HTTP 202 — 보냈으나 로봇이 멈췄는지 확인하지 못했다: ${r#* }${NC}"
        [ $rc -eq 0 ] && rc=2
    elif [ "$code" = "000" ]; then
        echo -e "${RED}⛔ 정지 API: 게이트웨이(${GATEWAY_URL})에 닿지 못했다 — 플릿 경로 HOLD(Nav2 목표 취소)가 나가지 않았다${NC}"
        rc=1
    elif [ "$code" = "ERR" ]; then
        # 느리거나(28) 끊긴(52·56) 게이트웨이 — HOLD 를 냈을 수도 안 냈을 수도 있다(G-12: HTTP 코드처럼 찍지 않는다).
        echo -e "${RED}⛔ 정지 API 응답 없음·끊김 — 플릿 경로 HOLD(Nav2 목표 취소)가 나갔는지 모른다: ${r#* }${NC}"
        rc=1
    else
        echo -e "${RED}⛔ 정지 API 실패: HTTP ${r%% *} ${r#* }${NC}"
        rc=1
    fi
    return $rc
}

show_status() {
    echo -e "${CYAN}============================================================"
    echo -e " 📊 [실시간 로봇 위치 및 실측 편차 HUD]"
    echo -e "============================================================${NC}"
    curl -s http://127.0.0.1:8889/api/status | python3 -c "
import sys, json
try:
    data = json.load(sys.stdin)
    r1 = data.get('robots', {}).get('robot1', {})
    gz = data.get('robots', {}).get('gazebo_sim', {})
    disc = data.get('discrepancy', {})
    nav = data.get('robot1_nav', {})
    fc = data.get('fleet_comm', {})

    r1_x = r1.get('x', 0.0)
    r1_y = r1.get('y', 0.0)
    r1_yaw = r1.get('yaw', 0.0) * 180 / 3.14159
    gz_x = gz.get('x', 0.0)
    gz_y = gz.get('y', 0.0)
    gz_yaw = gz.get('yaw', 0.0) * 180 / 3.14159

    derr = disc.get('dist_err_cm', 0.0)
    yerr = disc.get('dyaw_deg', 0.0)
    state = nav.get('state', 'IDLE')
    r1_rtt = fc.get('robot1', {}).get('rtt_ms', '--')

    print(f' 🚗 [실물 로봇 1] X={r1_x:6.2f}m, Y={r1_y:6.2f}m, Yaw={r1_yaw:6.1f}° (RTT: {r1_rtt}ms)')
    print(f' 🏭 [가제보 트윈] X={gz_x:6.2f}m, Y={gz_y:6.2f}m, Yaw={gz_yaw:6.1f}°')
    print(f' 📏 [실측 오차]   거리 편차={derr:5.1f}cm, 각도 편차={yerr:5.1f}°')
    print(f' 🧭 [내비게이션] 상태: {state}')
except Exception as e:
    print('상태 조회 대기중...', e)
"
    echo ""
}

check_health() {
    echo -e "${CYAN}============================================================"
    echo -e " 🩺 [시스템 전체 서비스 및 토픽 건전성 진단]"
    echo -e "============================================================${NC}"
    echo -n "1. 게이트웨이 웹서버 (:8889) : "
    if curl -s -m 1 http://127.0.0.1:8889/ >/dev/null 2>&1; then
        echo -e "${GREEN}🟢 정상 작동중${NC}"
    else
        echo -e "${RED}🔴 응답 없음 (재기동 필요)${NC}"
    fi

    echo -n "2. Gazebo 3D 시뮬레이션       : "
    if pgrep -f "gz sim" >/dev/null 2>&1; then
        echo -e "${GREEN}🟢 가동중${NC}"
    else
        echo -e "${RED}🔴 미기동${NC}"
    fi

    echo -n "3. Robot #1 실물 SSH (198.51.100.5:22): "
    if nc -zv -w 1 198.51.100.5 22 >/dev/null 2>&1; then
        echo -e "${GREEN}🟢 SSH 연결 양호${NC}"
    else
        echo -e "${RED}🔴 연결 실패 (Wi-Fi 확인)${NC}"
    fi

    echo -n "4. Robot #1 모터 & 카메라 데몬: "
    ssh -o StrictHostKeyChecking=no -o ConnectTimeout=2 pinky@198.51.100.5 \
        "ps aux | grep -E 'bringup|stream_cam|navigator' | grep -v grep | wc -l" 2>/dev/null | {
        read count
        if [ "$count" -ge 3 ]; then
            echo -e "${GREEN}🟢 모든 데몬(모터/카메라/내비게이터 3종) 정상 실행중 ($count개)${NC}"
        else
            echo -e "${YELLOW}🟡 일부 데몬 미실행 ($count/3)${NC}"
        fi
    }

    echo -n "5. Gazebo 탑뷰 카메라 (/camera): "
    if curl -s -m 1 http://127.0.0.1:8889/gazebo_feed | head -c 20 | grep -q "JFIF"; then
        echo -e "${GREEN}🟢 영상 스트리밍 양호${NC}"
    else
        echo -e "${YELLOW}🟡 스트림 대기중${NC}"
    fi

    echo -n "6. Robot #1 온보드 카메라 스트림: "
    if curl -s -m 1 "http://127.0.0.1:8889/robot_camera_feed?id=robot1" | head -c 20 | grep -q "JFIF"; then
        echo -e "${GREEN}🟢 영상 스트리밍 양호${NC}"
    else
        echo -e "${YELLOW}🟡 스트림 대기중${NC}"
    fi
    echo ""
}

# -----------------------------------------------------------------------------
# CLI 원클릭 인자 처리 (비대화형 실행 지원)
# -----------------------------------------------------------------------------
if [ -n "$1" ]; then
    case "$1" in
        goal)
            if [ -z "$2" ] || [ -z "$3" ]; then
                echo "사용법: $0 goal <X> <Y> [Yaw(deg)]"
                exit 1
            fi
            yaw_deg=${4:-0}
            if ! yaw_rad=$(_deg2rad "$yaw_deg"); then
                echo -e "${RED}⛔ 잘못된 Yaw(deg): '${yaw_deg}' — 목표를 보내지 않는다${NC}"
                exit 1
            fi
            send_goal "$2" "$3" "$yaw_rad"
            exit $?
            ;;
        mission1|1)
            send_mission "mission1"
            exit $?
            ;;
        mission2|2)
            send_mission "mission2"
            exit $?
            ;;
        point1)
            send_mission "point1"
            exit $?
            ;;
        point2)
            send_mission "point2"
            exit $?
            ;;
        point3)
            send_mission "point3"
            exit $?
            ;;
        stop|halt)
            emergency_stop
            exit $?
            ;;
        status|hud)
            show_status
            exit 0
            ;;
        health|check)
            check_health
            exit 0
            ;;
        help|--help|-h)
            echo -e "${CYAN}사용법: $0 [명령]${NC}"
            echo "  인자 없이 실행 시 대화형 메뉴 모드로 진입합니다."
            echo ""
            echo "  $0 goal <X> <Y> [Yaw(deg)]   - 임의 좌표로 목표 전송"
            echo "  $0 mission1                 - 미션 1 실행 (1 -> 2 -> 1)"
            echo "  $0 mission2                 - 미션 2 실행 (1 -> 2 -> 3 -> 1)"
            echo "  $0 point1                   - 출발 원점 (0, 0) 복귀"
            echo "  $0 point2                   - 좌측 작업대 (-1.4, 0) 이동"
            echo "  $0 point3                   - 우측 작업대 (1.3, -0.3) 이동"
            echo "  $0 stop                     - 긴급 정지 (STOP)"
            echo "  $0 status                   - 실시간 위치 및 오차 HUD 조회"
            echo "  $0 health                   - 시스템 및 카메라 헬스체크"
            echo ""
            echo "  목표·미션은 게이트웨이로만 보낸다 — 게이트웨이에 닿지 못하면 거절한다(토픽 직접 발행 없음)."
            echo "  stop 은 0 속도 직접 발행과 정지 API 를 동시에 보낸다."
            echo "  종료코드: 0 = 처리 확인(200) · 1 = 실패·거절·보내지 못함 · 2 = 보냈으나 결과 모름(202)"
            exit 0
            ;;
        *)
            echo "알 수 없는 명령: $1 (도움말: $0 --help)"
            exit 1
            ;;
    esac
fi

# -----------------------------------------------------------------------------
# 대화형 대화형 TUI 메뉴 루프
# -----------------------------------------------------------------------------
while true; do
    echo -e "${BLUE}============================================================"
    echo -e " 🛰️  Pinky #1 실물 로봇 & 가제보 통합 터미널 조작기"
    echo -e "     [ROS_DOMAIN_ID=8 관제 평면 | 브릿지 경유 1번 로봇 관제]"
    echo -e "============================================================${NC}"
    echo -e "  ${GREEN}1)${NC} 🎯 임의 목표 좌표 (X, Y, Yaw) 직접 전송"
    echo -e "  ${GREEN}2)${NC} 🚩 미션 1 실행 (1 -> 2 -> 1 번 복귀)"
    echo -e "  ${GREEN}3)${NC} 🚩 미션 2 실행 (1 -> 2 -> 3 -> 1 번 복귀)"
    echo -e "  ${GREEN}4)${NC} 📍 Point 1 (출발 원점 0.0, 0.0) 이동"
    echo -e "  ${GREEN}5)${NC} 📍 Point 2 (좌측 작업대 -1.4, 0.0) 이동"
    echo -e "  ${GREEN}6)${NC} 📍 Point 3 (우측 작업대 1.3, -0.3) 이동"
    echo -e "  ${RED}7)${NC} 🛑 긴급 정지 (EMERGENCY STOP)"
    echo -e "  ${CYAN}8)${NC} 📊 실시간 로봇 위치 및 실측 오차 모니터 (HUD)"
    echo -e "  ${PURPLE}9)${NC} 🩺 시스템 전체 서비스 & 카메라 헬스체크"
    echo -e "  ${YELLOW}0)${NC} 🚪 종료 (Exit)"
    echo -e "${BLUE}============================================================${NC}"
    read -p "명령 번호를 선택하세요 [0-9]: " choice

    case "$choice" in
        1)
            echo ""
            read -p "목표 X 좌표(m) 입력 [-1.9 ~ 1.9]: " gx
            read -p "목표 Y 좌표(m) 입력 [-1.3 ~ 1.4]: " gy
            read -p "목표 Heading(각도 deg) [-180 ~ 180, 기본 0]: " gyaw_deg
            gyaw_deg=${gyaw_deg:-0}
            if gyaw_rad=$(_deg2rad "$gyaw_deg"); then
                send_goal "$gx" "$gy" "$gyaw_rad"
            else
                echo -e "${RED}⛔ 잘못된 Heading(deg): '${gyaw_deg}' — 목표를 보내지 않는다${NC}"
            fi
            ;;
        2)
            send_mission "mission1"
            ;;
        3)
            send_mission "mission2"
            ;;
        4)
            send_mission "point1"
            ;;
        5)
            send_mission "point2"
            ;;
        6)
            send_mission "point3"
            ;;
        7)
            emergency_stop
            ;;
        8)
            show_status
            ;;
        9)
            check_health
            ;;
        0|q|Q)
            echo -e "${GREEN}터미널 조작기를 종료합니다.${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}잘못된 번호입니다. [0-9] 중에서 선택해주세요.${NC}"
            ;;
    esac
    echo ""
    read -p "계속하려면 [Enter] 키를 누르세요..."
    clear
done

