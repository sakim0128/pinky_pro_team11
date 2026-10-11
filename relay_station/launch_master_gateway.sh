#!/usr/bin/env bash
# =============================================================================
# 🚀 Educational Field Relay & Robot Gateway Master Launcher
# 1. IP & 네트워크 연결 진단 (LAN, Tailscale, 태블릿, 로봇 1/2)
# 2. 로봇 SSH 포트 포워딩 터널 데몬 기동 (socat 2205/2206)
# 3. 도메인 브릿지 기동 (로봇 10~13 <-> 관제 8 -> 팀원 미러 9)
# 4. Gazebo 3D 시뮬레이션 및 디지털 트윈 동기화 기동
# 5. 현장 중계 웹 서버 기동 (포트 8889)
#
# [도메인 배치]  관제 8 / 팀원 미러 9 / 로봇 10·11·12·13 / 개인 실습 41~49
# 상세 설계: docs/DOMAIN_ARCHITECTURE.md
# =============================================================================

set -e

SCRIPT_DIR="$HOME/field_gateway_relay"
PINKY_DIR="$HOME/pinky_pro"

# 1. ROS 2 및 워크스페이스 환경 로드
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$PINKY_DIR/install/setup.bash" ]; then
    source "$PINKY_DIR/install/setup.bash"
fi

REPO_DIR="$HOME/pinky_pro/src/pinky_pro_team11"
if [ -f "$REPO_DIR/install/setup.bash" ]; then
    source "$REPO_DIR/install/setup.bash"
fi

# 중계 노트북은 관제 평면(도메인 8)에 머문다.
# 로봇 데이터는 도메인 브릿지가 10~13에서 끌어와 8로 올려주고,
# 관제국이 8에서 내보낸 제어 명령은 브릿지가 각 로봇 도메인으로 내려보낸다.
# 도메인 8은 방화벽으로 이 노트북 자신에게만 열려 있다.
#   relay_station/network/domain_firewall.sh
export ROS_DOMAIN_ID=8
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# DDS 인터페이스 바인딩은 장소마다 다르다.
# 현장 설정(configs/cyclonedds.xml)은 교육장 유선 NIC 에 바인딩을 박아 두는데,
# 그 어댑터가 없는 곳에서 쓰면 CycloneDDS 가 도메인 생성에 실패하고
# ROS 노드가 하나도 안 만들어져 게이트웨이가 통째로 죽는다 (2026-09-09 집에서 실측).
# bridge_env.sh 와 같은 규칙 — 환경변수로 바꿀 수 있다(관제 검수 P3: 예전엔 여기만 박혀 있어 둘이 갈렸다)
FIELD_NIC="${FIELD_NIC:-enx001122334455}"
_DDS_CFG_DIR="${REPO_ROOT:-$HOME/pinky_pro/src/pinky_pro_team11}/configs"
# R-2 (2026-09-24): RELAY_DDS_PROFILE 로 명시할 수 있다 — 규칙은 domain_bridge/bridge_env.sh 와 같다
# (둘이 다르면 같은 기계의 두 서비스가 다른 DDS 에 선다 · 시험이 대사한다). tailnet 은 명시할 때만.
_dds_die() { echo "[launcher] 🔴 DDS 설정 오류: $* (exit 78 — 재시작해도 같다)" >&2; exit 78; }
_profile_req="${RELAY_DDS_PROFILE:-auto}"
TAILNET_NIC="${TAILNET_NIC:-tailscale0}"
case "$_profile_req" in
    auto)
        if ip link show "$FIELD_NIC" >/dev/null 2>&1; then
            export CYCLONEDDS_URI="file://${_DDS_CFG_DIR}/cyclonedds.xml"
            DDS_PROFILE="field ($FIELD_NIC)"
        else
            export CYCLONEDDS_URI="file://${_DDS_CFG_DIR}/cyclonedds-offsite.xml"
            DDS_PROFILE="offsite (autodetermine, loopback peers)"
        fi ;;
    field)
        ip link show "$FIELD_NIC" >/dev/null 2>&1 || _dds_die "RELAY_DDS_PROFILE=field 인데 $FIELD_NIC 이 없다"
        export CYCLONEDDS_URI="file://${_DDS_CFG_DIR}/cyclonedds.xml"
        DDS_PROFILE="field ($FIELD_NIC, 명시)" ;;
    offsite)
        export CYCLONEDDS_URI="file://${_DDS_CFG_DIR}/cyclonedds-offsite.xml"
        DDS_PROFILE="offsite (명시)" ;;
    tailnet)
        ip link show "$TAILNET_NIC" >/dev/null 2>&1 || _dds_die "RELAY_DDS_PROFILE=tailnet 인데 $TAILNET_NIC 이 없다"
        [ -n "${FARM_HOST_TAILNET_IP:-}" ] || _dds_die "RELAY_DDS_PROFILE=tailnet 인데 FARM_HOST_TAILNET_IP 가 비었다"
        # CycloneDDS 는 ${FARM_HOST_TAILNET_IP} 를 **프로세스 환경**에서 편다 — 셸 변수로만 있으면 peer 가 빈다
        export FARM_HOST_TAILNET_IP
        export CYCLONEDDS_URI="file://${_DDS_CFG_DIR}/cyclonedds-tailnet.xml"
        DDS_PROFILE="tailnet ($TAILNET_NIC, peer=localhost+\$FARM_HOST_TAILNET_IP)" ;;
    *)
        _dds_die "RELAY_DDS_PROFILE 은 auto|field|offsite|tailnet 중 하나다 (받은 값: $_profile_req)" ;;
esac
# 고른 xml 이 없으면 CycloneDDS 가 도메인 생성에 실패해 게이트웨이가 통째로 죽는다 — bridge_env.sh 와 같이 78 로 멈춘다(관제 검수 P3)
[ -f "${CYCLONEDDS_URI#file://}" ] || _dds_die "DDS 프로파일 파일이 없다: ${CYCLONEDDS_URI#file://}"
# 관제 검수 REVIEW_20260925 §3.4: FIELD_NIC 를 바꿔도 현장 xml 은 NIC 이름을 박아 두고 있다 — 둘이 다르면 엉뚱한 NIC 에 선다.
# xml 을 고치지 않고 FIELD_NIC 만 바꾼 경우를 78 로 멈춘다(bridge_env.sh 와 같은 규칙).
case "$CYCLONEDDS_URI" in *"/cyclonedds.xml")
    _xml_nic=$(grep -o 'NetworkInterface name="[^"]*"' "${CYCLONEDDS_URI#file://}" | head -1 | cut -d'"' -f2)
    [ "$_xml_nic" = "$FIELD_NIC" ] || _dds_die "FIELD_NIC=$FIELD_NIC 인데 configs/cyclonedds.xml 은 $_xml_nic 에 묶여 있다 — 둘을 같이 바꾼다" ;;
esac
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export PYTHONUNBUFFERED=1
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix pinky_description 2>/dev/null)/share/pinky_description/..:$(ros2 pkg prefix pinky_gz_sim 2>/dev/null)/share/pinky_gz_sim/models:${HOME}/.gazebo/models:${GZ_SIM_RESOURCE_PATH}"
export LIBGL_DRI3_DISABLE=1
export XDG_RUNTIME_DIR="/run/user/1000"
export WAYLAND_DISPLAY="wayland-0"
export DISPLAY=":0"
export QT_QPA_PLATFORM="wayland"

echo "============================================================"
echo " 🛰️  [1/5] 중계 노트북 네트워크 및 장비 연결 진단"
echo "============================================================"

# 유선 랜 IP 자동 보장 및 활성화 (재부팅/재연결 시 누락 방지)
if ! ip -4 addr show enx001122334455 2>/dev/null | grep -q "198.51.100.3"; then
    echo "  - ⚠️ [NetworkManager] 유선 랜(198.51.100.3) 누락 감지 ➔ 자동 복구 실행..."
    nmcli connection up "Wired connection 1" >/dev/null 2>&1 || true
fi

LAN_IP=$(ip -4 addr show enx001122334455 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || echo "198.51.100.3")
WIFI_IP=$(ip -4 addr show wlp0s20f3 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || echo "연결안됨")
TS_IP="100.64.0.81"

echo " [호스트 IP]"
echo "  - LAN 유선 IP     : $LAN_IP"
echo "  - Wi-Fi IP        : $WIFI_IP"
echo "  - Tailscale VPN IP: $TS_IP"
echo "  - DDS 프로파일     : $DDS_PROFILE"

# 게이트웨이 실행본은 ~/doc/src/field_gateway_relay 이다.
# 2026-09-09 부터 실행본은 레포의 심링크다 - 사본이 하나도 없다.
# 그래서 검사는 "내용이 같은가"가 아니라 "사본이 다시 생겼는가"를 본다.
# 누가 심링크를 실물로 덮으면 그 순간부터 다시 갈리기 시작하므로 기동마다 확인한다.
# 사본이 생겨도 기동은 막지 않는다 - 알리기만 한다.
_SYNC_CHECK="${REPO_ROOT:-$HOME/pinky_pro/src/pinky_pro_team11}/relay_station/scripts/check_gateway_sync.py"
if [ -f "$_SYNC_CHECK" ]; then
    python3 "$_SYNC_CHECK" || echo "  ⚠️ 실행본에 사본이 생겼습니다 - 위 목록을 확인하세요"
fi

# G-D — 게이트웨이 본체가 **기동 가능한 상태**인지 본다(문법 + 형제 import 이름).
# 2026-09-10: 다중행 import 가 깨졌는데 테스트 156개가 전부 통과했다 —
# conftest 가 gateway_web_server.py 를 일부러 import 하지 않아 사각지대였다.
# 실행 중 프로세스는 이미 로드해 둔 모듈로 돌아 증상이 없었고, 다음 재기동에 터졌을 것이다.
_IMPORT_CHECK="${REPO_ROOT:-$HOME/pinky_pro/src/pinky_pro_team11}/relay_station/scripts/check_gateway_imports.py"
# 제3자 재검 GW-R2: 플릿 코디네이터 import 는 게이트웨이를 실제로 띄우는 폴더(아래 cd "$SCRIPT_DIR")의 realpath 로 본다.
# 레포 폴더만 보면 실행본 본체가 사본이 됐을 때 게이트웨이는 fleet 을 못 찾는데 검사는 '정상' 이라 한다.
if [ -f "$_IMPORT_CHECK" ]; then
    python3 "$_IMPORT_CHECK" --exec-dir "$SCRIPT_DIR" || echo "  ⚠️ 게이트웨이 소스가 깨졌습니다 - 기동이 실패합니다"
fi
echo ""

echo " [현장 디바이스 Ping & Port 확인]"
# 태블릿 확인
if curl -s -m 1 http://198.51.100.2:8080/ >/dev/null 2>&1; then
    echo "  - 📱 Lenovo Y700 태블릿 (198.51.100.2:8080) : 🟢 정상 연결됨"
else
    echo "  - 📱 Lenovo Y700 태블릿 (198.51.100.2:8080) : 🟡 대기중 (카메라 앱 확인 요망)"
fi

# 로봇 #1 확인
if nc -zv -w 1 198.51.100.5 22 >/dev/null 2>&1; then
    echo "  - 🤖 Pinky #1 실물 로봇 (198.51.100.5:22)   : 🟢 SSH 정상 응답"
else
    echo "  - 🤖 Pinky #1 실물 로봇 (198.51.100.5:22)   : 🔴 응답 없음 (Wi-Fi 확인)"
fi

# 로봇 #2 확인
if nc -zv -w 1 198.51.100.6 22 >/dev/null 2>&1; then
    echo "  - 🤖 Pinky #2 실물 로봇 (198.51.100.6:22)   : 🟢 SSH 정상 응답"
else
    echo "  - 🤖 Pinky #2 실물 로봇 (198.51.100.6:22)   : 🔴 응답 없음 (Wi-Fi 확인)"
fi

# 로봇 #3 확인
if nc -zv -w 1 198.51.100.8 22 >/dev/null 2>&1; then
    echo "  - 🤖 Pinky #3 실물 로봇 (198.51.100.8:22)   : 🟢 SSH 정상 응답"
else
    echo "  - 🤖 Pinky #3 실물 로봇 (198.51.100.8:22)   : 🔴 응답 없음 (Wi-Fi 확인)"
fi
echo "============================================================"

# 2. 로봇 SSH 포트 포워딩 터널 데몬 (socat 2205/2206)
echo -e "\n🛰️  [2/5] 실물 로봇 SSH 포트 포워딩 터널 기동"
pkill -f "socat.*2205" 2>/dev/null || true
pkill -f "socat.*2206" 2>/dev/null || true
pkill -f "socat.*2207" 2>/dev/null || true
pkill -f "socat.*8005" 2>/dev/null || true
pkill -f "socat.*18081" 2>/dev/null || true
sleep 0.5

socat TCP4-LISTEN:2205,fork,reuseaddr TCP4:198.51.100.5:22 &
SOCAT_2205=$!
socat TCP4-LISTEN:2206,fork,reuseaddr TCP4:198.51.100.6:22 &
SOCAT_2206=$!
socat TCP4-LISTEN:2207,fork,reuseaddr TCP4:198.51.100.8:22 &
SOCAT_2207=$!
socat TCP4-LISTEN:8005,fork,reuseaddr TCP4:198.51.100.5:8888 &
SOCAT_8005=$!
# 18081 원격 보기 경로는 보기 전용 — 제어(움직이는 명령)는 현장 노트북에서 :8889 로 한다(통합 검토 OPS-1).
# 게이트웨이는 발신 주소로 로컬 제어를 가른다(LOCAL_CONTROL_IPS). 이 중계는 그 목록에 없는 루프백 주소 127.0.0.2 에서
# 게이트웨이에 붙는다 — 이 길로 온 요청은 원격으로 본다: 화면·상태·영상은 되고, 움직이는 명령은 403, 멈추는 명령은 받는다.
socat TCP4-LISTEN:18081,fork,reuseaddr TCP4:127.0.0.1:8889,bind=127.0.0.2 &
SOCAT_18081=$!
echo "  - 포트 2205  -> 198.51.100.5:22 (Robot #1 Pinky Main SSH) [PID: $SOCAT_2205]"
echo "  - 포트 2206  -> 198.51.100.6:22 (Robot #2 Pinky Sub SSH)  [PID: $SOCAT_2206]"
echo "  - 포트 2207  -> 198.51.100.8:22 (Robot #3 PP25022 SSH)   [PID: $SOCAT_2207]"
echo "  - 포트 8005  -> 198.51.100.5:8888 (Robot #1 Web/Jupyter)  [PID: $SOCAT_8005]"
echo "  - 포트 18081 -> 127.0.0.1:8889 (카메라 스트림 / 게이트웨이 웹 — 원격 보기 전용, 제어는 이 노트북 :8889) [PID: $SOCAT_18081]"

# 2-B. 도메인 브릿지 기동 — 관제/팀원 화면을 먹여살리는 필수 경로
echo -e "\n🌉 [3/5] 도메인 브릿지 기동 (로봇 10~13 <-> 관제 8 -> 팀원 미러 9)"
BRIDGE_DIR="${REPO_ROOT:-$HOME/pinky_pro/src/pinky_pro_team11}/relay_station/domain_bridge"
if [ -x /opt/ros/jazzy/lib/domain_bridge/domain_bridge ]; then
    # systemd 유저 유닛이 이미 설치돼 있으면 그쪽에 맡긴다(Restart=always 로 자동 복구).
    if systemctl --user list-unit-files 'pinky-domain-bridge@.service' >/dev/null 2>&1; then
        systemctl --user start pinky-domain-bridge@pinky1_control.service 2>/dev/null || true
        systemctl --user start pinky-domain-bridge@pinky2_control.service 2>/dev/null || true
        systemctl --user start pinky-domain-bridge@team_mirror.service    2>/dev/null || true
        systemctl --user start pinky-bridge-watchdog.service              2>/dev/null || true
    fi
    for U in pinky-domain-bridge@pinky1_control pinky-domain-bridge@pinky2_control \
             pinky-domain-bridge@team_mirror pinky-bridge-watchdog; do
        if systemctl --user is-active --quiet "$U.service" 2>/dev/null; then
            echo "  - 🟢 $U"
        else
            echo "  - 🔴 $U (미기동) → $BRIDGE_DIR/install_bridge.sh 로 설치하세요"
        fi
    done
else
    echo "  - ⚠️ ros-jazzy-domain-bridge 미설치 → 대시보드에 로봇 데이터가 올라오지 않습니다."
    echo "       설치: $BRIDGE_DIR/install_bridge.sh"
fi

# 3. 종료 시 모든 프로세스 정리 핸들러
cleanup() {
    # T10 — 종료 코드를 보존한다. 예전엔 이 함수가 exit 0 으로 끝나서
    # 게이트웨이가 죽어도 systemd 가 **성공 종료**로 봤고, Restart=on-failure 가
    # 영원히 안 걸렸다(설정했는데 그 자리에서 유효하지 않은 것).
    # 이 값이 systemd 의 재기동 판정 입력이다. 정상 정지(0)·SIGINT(130)·SIGTERM(143)은
    # 유닛의 SuccessExitStatus 가 성공으로 친다.
    local rc=$?
    echo -e "\n[INFO] Shutting down all Gateway, Gazebo, and Tunnel processes... (rc=$rc)"
    kill $SOCAT_2205 $SOCAT_2206 $SOCAT_2207 $SOCAT_8005 $SOCAT_18081 2>/dev/null || true
    pkill -9 -f "socat.*2205" 2>/dev/null || true
    pkill -9 -f "socat.*2206" 2>/dev/null || true
    pkill -9 -f "socat.*2207" 2>/dev/null || true
    pkill -9 -f "socat.*8005" 2>/dev/null || true
    pkill -9 -f "socat.*18081" 2>/dev/null || true
    if [ -n "$GZ_PID" ]; then
        kill "$GZ_PID" 2>/dev/null || true
    fi
    pkill -9 -f "gz sim" >/dev/null 2>&1 || true
    pkill -9 -f "ros_gz_bridge" >/dev/null 2>&1 || true
    pkill -9 -f "ros_gz_image" >/dev/null 2>&1 || true
    fuser -k 8889/tcp >/dev/null 2>&1 || true
    echo "[INFO] All systems cleanly terminated. (rc=$rc)"
    exit "$rc"
}
trap cleanup SIGINT SIGTERM EXIT

# 4. Gazebo 3D 시뮬레이터 백그라운드 기동
echo -e "\n🛰️  [4/5] Gazebo 3D 시뮬레이터 및 디지털 트윈 엔진 기동"
pkill -9 -f "gz sim" >/dev/null 2>&1 || true
pkill -9 -f "ros_gz_bridge" >/dev/null 2>&1 || true
sleep 1
ros2 launch pinky_gz_sim launch_sim.launch.xml >/tmp/launch_sim.log 2>&1 &
GZ_PID=$!
echo "  - Gazebo 3D Engine started (pinky_factory.world, PID: $GZ_PID)"
echo "  - 3초간 물리 엔진 및 카메라 브릿지 초기화 대기 중..."
sleep 3
ros2 run ros_gz_image image_bridge /camera >/tmp/image_bridge.log 2>&1 &

# 서브 로봇(Pinky Sub) 가상 월드에 자동 스폰
# [Solo Mode] echo "  - 서브 로봇(Pinky #2 Sub) 3D 모델 자동 스폰 중..."
# [Solo Mode] ros2 run ros_gz_sim create -name pinky_sub -topic /robot_description -x 0.0 -y 0.4 -z 0.05 >/dev/null 2>&1 &
sleep 1
echo "  - 메인 로봇(Pinky #1) & 서브 로봇(Pinky #2) 듀얼 디지털 트윈 준비 완료!"

# 5. 게이트웨이 웹 서버 기동
echo -e "\n🛰️  [5/5] 중계 게이트웨이 웹 서버 기동 (포트 8889)"
fuser -k 8889/tcp >/dev/null 2>&1 || true
sleep 0.5

cd "$SCRIPT_DIR"
# (옛 relay_controller_gui.py — Nav2 클릭 목표 데스크톱 창 — 은 지웠다(2026-09-29). 로봇은 lane_agent_node 뿐이라 목표 API 가 없다.
#  제어는 브라우저 http://<중계 PC>:8889/fleet_control_v2.html 의 "미션 배정" 카드 · 플릿 시작/정지.)

# MCV-1C: 로컬 캠은 필요할 때만 열린다(lazy open + idle release) - LED 는 유휴에 꺼진다.
# 장치를 절대 열지 않으려면 인자로 --no-camera 를 준다. pull 소스는 그 플래그와 무관하다.
python3 gateway_web_server.py "$@"
