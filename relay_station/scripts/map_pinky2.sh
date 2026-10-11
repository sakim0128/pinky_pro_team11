#!/bin/bash
###############################################################################
#  map_pinky2.sh — 로봇 #2 키보드 조작 & SLAM 2D 맵핑 통합 스크립트
#
#  기능:
#    1. slam_toolbox 기반 고정밀 2D 실시간 매핑 (로봇 2호기 전용)
#    2. teleop_twist_keyboard 통합 (터미널에서 즉시 i,j,k,l 키로 주행)
#    3. /odom 및 /pinky2/odom 자동 감지 매핑 지원
#    4. 종료 시 대화형 맵 저장 (.pgm + .yaml + .png)
#    5. 웹 대시보드(/my_map.png) 원클릭 동기화 지원
#
#  사용법:
#    ~/map_pinky2.sh             # 맵핑 & 키보드 조작 시작 (기본)
#    ~/map_pinky2.sh save [이름]  # 현재 맵 즉시 저장 (별도 터미널)
#    ~/map_pinky2.sh status      # 현재 맵핑 상태 확인
#    ~/map_pinky2.sh stop        # 맵핑 종료 및 저장
#    ~/map_pinky2.sh teleop      # SLAM 없이 순수 키보드 조작만 실행
###############################################################################

# ── ROS 2 환경 설정 ──────────────────────────────────────────
# [도메인 예외 규정]
#   SLAM 매핑은 /scan 원본의 지연과 QoS가 결과 품질을 직접 좌우한다.
#   따라서 이 스크립트만은 관제 평면(8)을 경유하지 않고 로봇 고유 도메인에
#   직접 참여한다. 브릿지 한 단계를 건너뛰어 지연과 재전송을 없애기 위함이다.
#   설계 근거: docs/DOMAIN_ARCHITECTURE.md
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source $HOME/pinky_pro/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=11   # 로봇 2호기 고유 도메인 (직접 참여)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/pinky_pro/src/pinky_pro_team11/configs/cyclonedds.xml"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# ── 경로 및 설정값 ───────────────────────────────────────────
MAP_SAVE_DIR="$HOME/maps"
WEB_MAP_PATH="$HOME/field_gateway_relay/static/my_map.png"
SLAM_PARAMS_FILE="/tmp/slam_mapping_params_pinky2.yaml"
SLAM_PID_FILE="/tmp/slam_toolbox_pinky2.pid"

# ── ANSI 색상 코드 ───────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

# ── SLAM 파라미터 파일 생성 ──────────────────────────────────
create_slam_params() {
    local odom_topic="${1:-/odom}"
    local scan_topic="${2:-/scan}"

    cat > "$SLAM_PARAMS_FILE" << YAML
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    mode: mapping

    # 소형 교육장 정밀도 최적화: 2.5cm/격자
    resolution: 0.025
    min_laser_range: 0.05
    max_laser_range: 6.0
    minimum_time_interval: 0.2
    transform_timeout: 0.3
    tf_buffer_duration: 15.0

    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: $scan_topic
    use_scan_matching: true
    use_scan_barycenter: true

    # 이동 감지 임계값 (저속 정밀 매핑)
    minimum_travel_distance: 0.08
    minimum_travel_heading: 0.10
    scan_buffer_size: 15
    scan_buffer_maximum_scan_distance: 4.0
    link_match_minimum_response_fine: 0.12
    link_scan_maximum_distance: 2.5
    loop_search_maximum_distance: 3.5

    # 루프 클로저 (출발지로 복귀 시 오차 누적 보정)
    do_loop_closing: true
    loop_match_minimum_chain_size: 8
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    map_update_interval: 1.5
    stack_size_to_use: 40000000
    use_map_saver: true
YAML
}

# ── PGM -> PNG 변환기 ────────────────────────────────────────
convert_pgm_to_png() {
    local pgm_file="$1"
    local png_file="$2"
    if [ -f "$pgm_file" ]; then
        python3 -c "
from PIL import Image
import sys
try:
    im = Image.open('$pgm_file')
    im.save('$png_file')
    print('  [PNG 변환 완료] $png_file')
except Exception as e:
    print('  [PNG 변환 실패]', e, file=sys.stderr)
" 2>/dev/null || true
    fi
}

# ── 맵 저장 함수 ─────────────────────────────────────────────
save_map() {
    mkdir -p "$MAP_SAVE_DIR"
    local default_name="pinky2_map_$(date +%Y%m%d_%H%M%S)"
    local save_name="${1:-$default_name}"
    local save_path="$MAP_SAVE_DIR/$save_name"

    echo ""
    echo -e "${CYAN}💾 [맵 저장 시작] 경로: ${BOLD}$save_path${NC}"
    echo -e "   nav2_map_server map_saver_cli 호출 중..."

    ros2 run nav2_map_server map_saver_cli \
        -f "$save_path" \
        --ros-args -p save_map_timeout:=8.0 2>&1

    if [ -f "${save_path}.pgm" ] && [ -f "${save_path}.yaml" ]; then
        echo -e "${GREEN}✅ [성공] 로봇 #2 맵 저장 완료!${NC}"
        echo -e "   - PGM: ${save_path}.pgm ($(du -h "${save_path}.pgm" 2>/dev/null | cut -f1))"
        echo -e "   - YAML: ${save_path}.yaml"

        # PNG 변환
        convert_pgm_to_png "${save_path}.pgm" "${save_path}.png"

        # 메타 정보 출력
        echo -e "\n${BOLD}[맵 파라미터 요약]${NC}"
        grep -E 'resolution|origin|occupied_thresh' "${save_path}.yaml" 2>/dev/null || cat "${save_path}.yaml"

        # 웹 대시보드 적용 질문 (대화형인 경우)
        if [ -t 0 ]; then
            echo ""
            read -p "🌐 이 맵을 게이트웨이 웹 대시보드(my_map.png)에 바로 적용하시겠습니까? (y/N): " apply_web
            if [[ "$apply_web" =~ ^[Yy]$ ]]; then
                cp -f "${save_path}.png" "$WEB_MAP_PATH" 2>/dev/null || cp -f "${save_path}.pgm" "$WEB_MAP_PATH"
                echo -e "${GREEN}✅ 웹 대시보드 맵 업데이트 완료! 브라우저를 새로고침(F5)하세요.${NC}"
            fi
        fi
        return 0
    else
        echo -e "${RED}❌ [오류] 맵 저장 실패! /map 토픽이 발행되고 있는지 확인하세요.${NC}"
        return 1
    fi
}

# ── 상태 확인 함수 ───────────────────────────────────────────
view_status() {
    echo -e "${BOLD}${CYAN}====================================================${NC}"
    echo -e "${BOLD}  🗺️  로봇 #2 SLAM 매핑 상태 모니터링${NC}"
    echo -e "${BOLD}${CYAN}====================================================${NC}"

    # SLAM 실행 여부
    local slam_pid
    slam_pid=$(pgrep -f "slam_toolbox.*slam_mapping_params_pinky2" | head -1)
    if [ -z "$slam_pid" ]; then
        slam_pid=$(pgrep -f "async_slam_toolbox_node" | head -1)
    fi

    if [ -n "$slam_pid" ]; then
        echo -e "  ● SLAM 노드:  ${GREEN}정상 가동 중${NC} (PID: $slam_pid)"
    else
        echo -e "  ● SLAM 노드:  ${RED}중지됨${NC}"
    fi

    # 핵심 토픽 점검
    echo -e "\n${BOLD}[핵심 토픽 상태]${NC}"
    for t in "/scan" "/pinky2/scan" "/odom" "/pinky2/odom" "/cmd_vel" "/pinky2/cmd_vel" "/map"; do
        local cnt
        cnt=$(ros2 topic info "$t" 2>/dev/null | grep "Publisher count" | awk '{print $3}')
        if [ -n "$cnt" ] && [ "$cnt" != "0" ]; then
            echo -e "  - $t: ${GREEN}발행 중${NC} (퍼블리셔 $cnt 개)"
        fi
    done

    # 저장된 맵 목록
    echo -e "\n${BOLD}[저장된 맵 목록 (${MAP_SAVE_DIR})]${NC}"
    if [ -d "$MAP_SAVE_DIR" ] && [ "$(ls -A "$MAP_SAVE_DIR" 2>/dev/null)" ]; then
        ls -lht "$MAP_SAVE_DIR"/*.yaml 2>/dev/null | head -5 | awk '{print "  - " $9 " (" $5 ", " $6 " " $7 " " $8 ")"}'
    else
        echo "  (아직 저장된 맵이 없습니다)"
    fi
    echo -e "${BOLD}${CYAN}====================================================${NC}"
}

# ── SLAM 종료 함수 ───────────────────────────────────────────
stop_slam() {
    echo -e "${YELLOW}🛑 로봇 #2 SLAM 프로세스 정리 중...${NC}"
    pkill -f "slam_toolbox.*slam_mapping_params_pinky2" 2>/dev/null || true
    pkill -f "async_slam_toolbox_node" 2>/dev/null || true
    pkill -f "teleop_twist_keyboard" 2>/dev/null || true
    rm -f "$SLAM_PID_FILE" 2>/dev/null || true
    echo -e "${GREEN}✅ 모든 SLAM 및 키보드 조작 프로세스가 종료되었습니다.${NC}"
}

# ── 순수 키보드 텔레옵만 실행 ────────────────────────────────
run_teleop_only() {
    local cmd_topic="/cmd_vel"
    if ros2 topic list 2>/dev/null | grep -q "^/pinky2/cmd_vel$"; then
        cmd_topic="/pinky2/cmd_vel"
    fi

    echo -e "${BOLD}${CYAN}🎮 로봇 #2 키보드 수동 주행 모드 ($cmd_topic)${NC}"
    echo -e "${YELLOW}속도: 전진 0.12 m/s, 회전 0.8 rad/s (정지: Space/k, 종료: Ctrl+C)${NC}\n"
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args \
        -r cmd_vel:="$cmd_topic" \
        -p speed:=0.12 \
        -p turn:=0.8 \
        -p speed_limit:=0.25 \
        -p turn_limit:=1.5
}

# ── 메인 매핑 + 키보드 통합 실행 ─────────────────────────────
start_mapping() {
    clear 2>/dev/null || true
    echo -e "${BOLD}${CYAN}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║       🗺️  로봇 #2 (Pinky Sub) SLAM 맵핑 & 키보드 조작         ║"
    echo "║       소형 교육장(2.6m x 1.25m) 신규 맵 생성 모드            ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"

    # 1. 센서 및 토픽 점검 & 토픽 네임스페이스 감지
    echo -e "${CYAN}[1/4] 센서 및 토픽 사전 연결 점검...${NC}"

    local scan_topic="/scan"
    if ! ros2 topic list 2>/dev/null | grep -q "^/scan$" && ros2 topic list 2>/dev/null | grep -q "^/pinky2/scan$"; then
        scan_topic="/pinky2/scan"
    fi

    local odom_topic="/odom"
    if ! ros2 topic list 2>/dev/null | grep -q "^/odom$" && ros2 topic list 2>/dev/null | grep -q "^/pinky2/odom$"; then
        odom_topic="/pinky2/odom"
    fi

    local cmd_topic="/cmd_vel"
    if ! ros2 topic list 2>/dev/null | grep -q "^/cmd_vel$" && ros2 topic list 2>/dev/null | grep -q "^/pinky2/cmd_vel$"; then
        cmd_topic="/pinky2/cmd_vel"
    fi

    echo -e "  - LiDAR 토픽: ${GREEN}$scan_topic${NC}"
    echo -e "  - Odom 토픽:  ${GREEN}$odom_topic${NC}"
    echo -e "  - 제어 토픽:  ${GREEN}$cmd_topic${NC}"

    local scan_pub
    scan_pub=$(ros2 topic info "$scan_topic" 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    if [ -z "$scan_pub" ] || [ "$scan_pub" = "0" ]; then
        echo -e "${RED}⚠️  [경고] $scan_topic 라이다 토픽 퍼블리셔가 감지되지 않습니다!${NC}"
        echo -e "   로봇 2호기에서 bringup 서비스 기동 여부를 확인하세요."
        read -p "   그래도 SLAM을 시작하시겠습니까? (y/N): " cont_ans
        if [[ ! "$cont_ans" =~ ^[Yy]$ ]]; then
            echo "맵핑을 취소합니다."
            exit 0
        fi
    else
        echo -e "  ✓ LiDAR 연결됨 (퍼블리셔 $scan_pub 개)"
    fi

    # 2. 잔여 프로세스 정리
    echo -e "\n${CYAN}[2/4] 이전 잔여 프로세스 정리...${NC}"
    pkill -f "slam_toolbox.*slam_mapping_params_pinky2" 2>/dev/null || true
    pkill -f "teleop_twist_keyboard" 2>/dev/null || true
    sleep 1

    # 3. SLAM Toolbox 백그라운드 구동
    echo -e "\n${CYAN}[3/4] SLAM 백엔드 (slam_toolbox) 초기화...${NC}"
    create_slam_params "$odom_topic" "$scan_topic"
    mkdir -p "$MAP_SAVE_DIR"

    ros2 launch slam_toolbox online_async_launch.py \
        slam_params_file:="$SLAM_PARAMS_FILE" \
        use_sim_time:=false \
        > /tmp/slam_mapping_r2.log 2>&1 &
    SLAM_PID=$!
    echo "$SLAM_PID" > "$SLAM_PID_FILE"

    echo -n "  SLAM 엔진 로딩 대기 "
    for i in {1..6}; do
        echo -n "."
        sleep 1
    done
    echo ""

    if kill -0 $SLAM_PID 2>/dev/null; then
        echo -e "  ✓ SLAM 노드: ${GREEN}가동 성공${NC} (PID: $SLAM_PID)"
    else
        echo -e "  ❌ ${RED}SLAM 노드 기동 실패!${NC} 로그: /tmp/slam_mapping_r2.log"
        tail -15 /tmp/slam_mapping_r2.log
        exit 1
    fi

    # 4. 키보드 조작 안내 및 진입
    echo -e "\n${CYAN}[4/4] 키보드 조작 인터페이스 활성화${NC}"
    echo -e "${BOLD}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║  ⌨️  로봇 #2 키보드 주행 조작 키 안내:                        ║"
    echo "║                                                              ║"
    echo "║        u    i    o         ↖  전진  ↗                        ║"
    echo "║        j    k    l    →    ←  정지  →                        ║"
    echo "║        m    ,    .         ↙  후진  ↘                        ║"
    echo "║                                                              ║"
    echo "║  정지: [스페이스바] 또는 [k]                                 ║"
    echo "║  종료 & 맵 저장: [Ctrl + C]                                  ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"

    trap 'echo ""; handle_exit' INT

    handle_exit() {
        echo -e "\n${YELLOW}⏸️  키보드 조작이 중단되었습니다.${NC}"
        echo -e "${BOLD}생성된 맵을 지금 파일로 저장하시겠습니까? (Y/n): ${NC}"
        read -r do_save
        if [[ ! "$do_save" =~ ^[Nn]$ ]]; then
            echo -e "저장할 맵 이름을 입력하세요 (엔터 시 기본 자동 생성): "
            read -r user_map_name
            if [ -n "$user_map_name" ]; then
                save_map "$user_map_name"
            else
                save_map
            fi
        else
            echo "맵을 저장하지 않았습니다."
        fi

        echo ""
        read -p "SLAM 백그라운드 프로세스를 종료하시겠습니까? (Y/n): " do_stop
        if [[ ! "$do_stop" =~ ^[Nn]$ ]]; then
            stop_slam
        fi
        exit 0
    }

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args \
        -r cmd_vel:="$cmd_topic" \
        -p speed:=0.12 \
        -p turn:=0.8 \
        -p speed_limit:=0.25 \
        -p turn_limit:=1.5
}

case "${1:-start}" in
    start)
        start_mapping
        ;;
    save)
        save_map "$2"
        ;;
    status|view)
        view_status
        ;;
    stop)
        stop_slam
        ;;
    teleop)
        run_teleop_only
        ;;
    help|--help|-h)
        echo "사용법: $0 {start|save [이름]|status|stop|teleop|help}"
        ;;
    *)
        echo -e "${RED}알 수 없는 서브커맨드: $1${NC}"
        exit 1
        ;;
esac
