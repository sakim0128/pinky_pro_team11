#!/usr/bin/env bash
# =============================================================================
# 🌉 도메인 브릿지 설치 · 기동 스크립트 (중계 관제 노트북 전용)
#
#   ./install_bridge.sh              # 설치 + pinky1,pinky2 + 미러 + 워치독 기동
#   ./install_bridge.sh 1 2 3        # 기동할 로봇 번호 직접 지정
#   ./install_bridge.sh status       # 현재 상태만 확인
#   ./install_bridge.sh stop         # 전체 중지
# =============================================================================
set -euo pipefail
# 규칙: pipefail 인 이 스크립트에서 **명령을 `grep -q` 로 파이프하지 않는다.**
#
# `grep -q` 는 매치하는 순간 끝나며 상류에 SIGPIPE 를 던지고, `pipefail` 이 그 신호를
# 파이프라인 실패로 올린다. 즉 **조건이 참일 때 오히려 실패한다.**
#
# 2026-09-12 실측 (같은 조건 25회씩, 매치가 맨 앞, `set -euo pipefail`):
#   상류 288 KB   파이프                  실패 25/25   <- 파이프 용량(64K) 초과로 막힌다
#   상류  48 KB   파이프                  실패  0/25   <- 한 번에 써서 안 막힌다
#   느리거나 여러 프로세스인 상류          실패 21~25/25 (크기와 무관)
#   변수에 담고 `printf | grep -q`         1.9MB 에서 실패 25/25  <- 변수도 안 낫다
#   변수에 담고 `grep -q ... <<< "$var"`   1.9MB 에서 실패  0/25  <- 이 형태를 쓴다
#
# ⭐⭐ 방아쇠가 **둘**이다. "출력이 작으니 괜찮다" 는 판단이 반쪽인 이유다:
#     ① 출력이 파이프 용량을 넘어 상류가 막힌다
#     ② 상류가 일찍 flush 하고 **계속 산다**(느린 명령·다단 파이프) — 크기와 무관
#     제보한 세션의 `ros2 topic info`(0.8초)가 ②라서 출력이 세 줄인데도 80% 터졌다.
#
# ⭐ 위 `0/25` 는 통계지만, herestring 이 안전한 것은 **기전**이다 — 그래서 상한을
#    다시 잴 필요가 없다. bash 는 파이프에 못 담을 크기가 되면 **임시 파일로 바꾼다**:
#        10 B   -> stdin = pipe:[...]              한 번에 써서 쓰기가 즉시 끝난다
#        100 KB -> stdin = /tmp/sh-thd.* (deleted)
#        3 MB   -> stdin = /tmp/sh-thd.* (deleted)  쓰는 프로세스가 **아예 없다**
#    쓰는 프로세스가 없으면 SIGPIPE 가 날 곳이 없다. 중간 구간이 없다.
#
# ⭐ `[[ $out =~ ^pat ]]` 로는 못 바꾼다. grep 의 `^` 는 **줄 머리**이고 bash 의 `^` 는
#    **문자열 처음**이라 의미가 조용히 달라진다(실측 확인).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_DIR="$HOME/.config/systemd/user"
G='\033[0;32m'; Y='\033[1;33m'; R='\033[0;31m'; C='\033[0;36m'; B='\033[1m'; N='\033[0m'

UNITS_FOR() {
    local -a u=()
    for n in "$@"; do u+=("pinky-domain-bridge@pinky${n}_control.service"); done
    u+=("pinky-domain-bridge@team_mirror.service" "pinky-bridge-watchdog.service")
    printf '%s\n' "${u[@]}"
}

ACTION="${1:-install}"

if [ "$ACTION" = "status" ]; then
    systemctl --user list-units 'pinky-*' --all --no-pager || true
    echo
    echo -e "${B}브릿지 건강 상태 (도메인 8):${N}"
    echo "  source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=8 ros2 topic echo /bridge/health --once"
    exit 0
fi

if [ "$ACTION" = "stop" ]; then
    mapfile -t ALL < <(UNITS_FOR 1 2 3 4)
    systemctl --user stop "${ALL[@]}" 2>/dev/null || true
    echo -e "${G}전체 브릿지 중지 완료${N}"
    exit 0
fi

# 기동 대상 로봇 (기본: 현장 가동 중인 1, 2호기)
ROBOTS=()
for a in "$@"; do
    [ "$a" = "install" ] && continue          # 'install' 은 동작 이름이지 로봇 번호가 아니다
    case "$a" in [1-4]) ROBOTS+=("$a") ;; *) echo "무시된 인자: $a" ;; esac
done
[ ${#ROBOTS[@]} -eq 0 ] && ROBOTS=(1 2)

echo -e "${B}============================================================${N}"
echo -e " 🌉 도메인 브릿지 설치 — 대상 로봇: ${C}${ROBOTS[*]}${N}"
echo -e "${B}============================================================${N}"

# ── 1. domain_bridge 패키지 ─────────────────────────────────────────────────
if [ ! -x /opt/ros/jazzy/lib/domain_bridge/domain_bridge ]; then
    echo -e "${Y}[1/5]${N} ros-jazzy-domain-bridge 미설치 → 설치를 시도합니다 (sudo 필요)"
    sudo apt-get update -qq
    sudo apt-get install -y ros-jazzy-domain-bridge
else
    echo -e "${G}[1/5]${N} ros-jazzy-domain-bridge 설치 확인됨"
fi

# ── 2. 설정 파일 생성 ───────────────────────────────────────────────────────
echo -e "${G}[2/5]${N} 브릿지 설정 생성"
bash "$SCRIPT_DIR/generate_configs.sh" >/dev/null
for n in "${ROBOTS[@]}"; do
    f="$SCRIPT_DIR/configs/pinky${n}_control.yaml"
    [ -f "$f" ] || { echo -e "${R}설정 없음: $f${N}"; exit 1; }
done

# ── 3. systemd 유저 유닛 설치 ───────────────────────────────────────────────
echo -e "${G}[3/5]${N} systemd 유저 유닛 설치: $UNIT_DIR"
mkdir -p "$UNIT_DIR"
cp "$SCRIPT_DIR/systemd/pinky-domain-bridge@.service"   "$UNIT_DIR/"
cp "$SCRIPT_DIR/systemd/pinky-bridge-watchdog.service"  "$UNIT_DIR/"
systemctl --user daemon-reload

# 노트북 화면을 닫거나 로그아웃해도 브릿지가 살아 있어야 한다.
# `|| true` 가 필요하다: set -e 아래에서 대입이 실패하면 스크립트가 죽는다.
# 예전 파이프라인은 `if` 조건이라 set -e 가 안 걸렸는데, 형태를 바꾸면 그 보호가 사라진다.
LINGER_OUT="$(loginctl show-user "$USER" -p Linger 2>/dev/null || true)"
if ! grep -q 'Linger=yes' <<< "$LINGER_OUT"; then
    echo -e "     ${Y}로그아웃 후에도 유지하려면:${N} sudo loginctl enable-linger $USER"
fi

# ── 3.5 DDS 프로파일 사전 점검 ──────────────────────────────────────────────
# ⭐ 유닛이 뜨기 **전에** 어느 프로파일을 고를지 여기서 한 번 찍어 본다.
#    고를 수 없으면(78) 유닛을 켜 봐야 failed 로 갈 뿐이라, 먼저 말하고 멈춘다.
echo -e "${G}[3.5]${N} DDS 프로파일 판정"
if ! ENV_OUT="$(bash "$SCRIPT_DIR/bridge_env.sh" 2>&1)"; then
    echo "$ENV_OUT" | sed 's/^/     /'
    echo -e "${R}DDS 프로파일을 고를 수 없다 — 유닛을 켜도 같은 이유로 실패한다.${N}"
    exit 78
fi
echo "$ENV_OUT" | sed 's/^/     /'

# ── 4. 기동 ─────────────────────────────────────────────────────────────────
echo -e "${G}[4/5]${N} 브릿지 기동"
mapfile -t UNITS < <(UNITS_FOR "${ROBOTS[@]}")
systemctl --user enable --now "${UNITS[@]}"

sleep 3

# ── 5. 검증 ─────────────────────────────────────────────────────────────────
echo -e "${G}[5/5]${N} 상태 확인"
FAIL=0
for u in "${UNITS[@]}"; do
    if systemctl --user is-active --quiet "$u"; then
        echo -e "     ${G}●${N} $u"
    else
        echo -e "     ${R}●${N} $u  ${R}(실패)${N}  → journalctl --user -u $u -n 30"
        FAIL=1
    fi
done

echo
echo -e "${B}검증 명령${N}"
echo -e "  ${C}ROS_DOMAIN_ID=8 ros2 topic list${N}          관제 평면에 로봇 토픽이 모였는지"
echo -e "  ${C}ROS_DOMAIN_ID=9 ros2 topic list${N}          팀원 미러에 읽기 전용으로 보이는지"
echo -e "  ${C}ROS_DOMAIN_ID=8 ros2 topic echo /bridge/health --once${N}"
echo
[ $FAIL -eq 0 ] && echo -e "${G}브릿지 정상 기동 완료${N}" || echo -e "${R}일부 유닛 실패 — 위 journalctl 명령으로 확인하세요${N}"
exit $FAIL
