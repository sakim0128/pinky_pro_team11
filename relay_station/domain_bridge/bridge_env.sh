#!/usr/bin/env bash
# =============================================================================
# 🌉 브리지가 쓸 DDS 프로파일을 고른다 — **런처와 같은 방식으로**
#
#   유닛에서:  source .../bridge_env.sh && exec ros2 run domain_bridge ...
#   손으로:    bash relay_station/domain_bridge/bridge_env.sh    # 골라진 것만 본다
#
# ## 왜 있나 (D-3)
#
# 예전 유닛은 DDS 설정을 **박아 뒀다**:
#     Environment=CYCLONEDDS_URI=file://…/configs/cyclonedds.xml
# 그 파일은 교육장 유선 NIC 에 바인딩을 박아 둔 **현장 전용**이다. 현장 밖에서 쓰면
# CycloneDDS 가 도메인 생성에 실패하고(2026-09-09 실측:
# `does not match an available interface`), `Restart=always` 라 **조용히 재시작
# 루프**를 돈다. 같은 기계의 `launch_master_gateway.sh` 는 NIC 유무를 보고 고르는데
# 브리지만 박혀 있었다.
#
# ⭐ **같은 기계의 두 서비스가 같은 질문에 다르게 답하면 안 된다.**
#
# ## 두 가지 실패를 가른다 — 이게 이 파일의 요점이다
#
#     일시적 실패(상대가 아직 안 떴다·망이 깜빡였다)  → 재시작이 맞다. 그대로 둔다.
#     설정 실패(고른 파일이 없다·ROS 가 없다)          → 재시작해도 같다. **말하고 멈춘다.**
#
# 후자는 `78`(EX_CONFIG, sysexits.h)로 죽고, 유닛의 `RestartPreventExitStatus=78`
# 이 그걸 받아 유닛을 `failed` 로 세운다. `Restart=always` 는 **그대로 둔다** —
# 문제는 재시작이 아니라 **실패를 안 말하는 것**이었다.
#
# ## ⚠️ NIC 이름이 런처와 여기 두 곳에 있다
#
# 한 곳으로 모으려면 런처를 고쳐야 하는데, 런처는 현장의 살아 있는 기동 경로다.
# 여기서는 실기 검증 없이 그걸 건드리지 않는다. 대신 **두 값이 같은지를 시험이
# 고정한다**(`test_bridge_env.py::test_런처와_같은_NIC_을_본다`). 한쪽만 바뀌면
# 배포 전에 빨개진다.
# =============================================================================

_BRIDGE_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${REPO_ROOT:-$(cd "$_BRIDGE_ENV_DIR/../.." && pwd)}"
_DDS_CFG_DIR="$REPO_ROOT/relay_station/configs"

# 런처(`launch_master_gateway.sh`)와 **같은 값이어야 한다.** 위 ⚠️ 참고.
FIELD_NIC="${FIELD_NIC:-enx001122334455}"

# ⭐ `return` 이 되면 source 된 것이다. source 면 부모를 죽여야 하고(유닛 경로),
#    단독 실행이면 그냥 exit 다. 둘 다 78 로 끝난다.
_bridge_env_die() {
    echo "[bridge-env] 🔴 설정 오류: $*" >&2
    echo "[bridge-env]    재시작해도 같은 결과다 — 사람이 고쳐야 한다 (exit 78)." >&2
    exit 78
}

# 프로파일 선택 (R-2, 2026-09-24). 기본(auto)은 예전과 같다 — 현장 NIC 가 있으면 field, 없으면 offsite.
# RELAY_DDS_PROFILE 로 명시할 수 있다. ⭐ tailnet 은 **명시할 때만** 탄다 — 자동으로 고르면 중계가
# 예고 없이 홈랩 팜의 DDS 에 합류한다. 명시했는데 조건이 안 되면 다른 프로파일로 조용히 떨어지지
# 않고 78 로 멈춘다(떨어지면 "tailnet 으로 붙었다" 고 믿는데 실제론 안 붙은 상태가 된다).
# ⚠️ launch_master_gateway.sh 가 같은 규칙을 갖는다 — 둘이 다르면 같은 기계가 두 답을 낸다(시험이 대사).
_profile_req="${RELAY_DDS_PROFILE:-auto}"
TAILNET_NIC="${TAILNET_NIC:-tailscale0}"
case "$_profile_req" in
    auto)
        if ip link show "$FIELD_NIC" >/dev/null 2>&1; then
            _bridge_profile="field ($FIELD_NIC)"
            _bridge_uri="$_DDS_CFG_DIR/cyclonedds.xml"
        else
            _bridge_profile="offsite ($FIELD_NIC 없음 → 자동탐지)"
            _bridge_uri="$_DDS_CFG_DIR/cyclonedds-offsite.xml"
        fi ;;
    field)
        ip link show "$FIELD_NIC" >/dev/null 2>&1 || _bridge_env_die "RELAY_DDS_PROFILE=field 인데 $FIELD_NIC 이 없다"
        _bridge_profile="field ($FIELD_NIC, 명시)"
        _bridge_uri="$_DDS_CFG_DIR/cyclonedds.xml" ;;
    offsite)
        _bridge_profile="offsite (명시)"
        _bridge_uri="$_DDS_CFG_DIR/cyclonedds-offsite.xml" ;;
    tailnet)
        ip link show "$TAILNET_NIC" >/dev/null 2>&1 || _bridge_env_die "RELAY_DDS_PROFILE=tailnet 인데 $TAILNET_NIC 이 없다"
        [ -n "${FARM_HOST_TAILNET_IP:-}" ] || _bridge_env_die "RELAY_DDS_PROFILE=tailnet 인데 FARM_HOST_TAILNET_IP 가 비었다 — peer 가 비면 팜을 못 찾는데도 조용히 뜬다"
        # ⚠️ 검사만 하고 export 하지 않으면 `FARM_HOST_TAILNET_IP=… source bridge_env.sh` 나 셸 변수로 준
        #    경우 여기선 값이 보이는데 뒤에 뜨는 ros2 의 환경에는 없다 → CycloneDDS 가 ${…} 를 빈 값으로 편다
        #    (관제 검수 미검증 항목, 2026-09-25 실측으로 확인). 그래서 여기서 내보낸다.
        #    `export` 만으로는 접두 대입(`VAR=… source …`)의 임시 범위와 함께 사라진다 → 전역으로 올려 내보낸다.
        _farm_ip="$FARM_HOST_TAILNET_IP"
        declare -gx FARM_HOST_TAILNET_IP="$_farm_ip"
        _bridge_profile="tailnet ($TAILNET_NIC, peer=localhost+\$FARM_HOST_TAILNET_IP)"
        _bridge_uri="$_DDS_CFG_DIR/cyclonedds-tailnet.xml" ;;
    *)
        _bridge_env_die "RELAY_DDS_PROFILE 은 auto|field|offsite|tailnet 중 하나다 (받은 값: $_profile_req)" ;;
esac

[ -f "$_bridge_uri" ] || _bridge_env_die "DDS 프로파일 파일이 없다: $_bridge_uri"
# 관제 검수 REVIEW_20260925 §3.4: FIELD_NIC 만 바꾸고 현장 xml 의 NIC 이름을 안 바꾸면 엉뚱한 NIC 에 선다 — 78.
case "$_bridge_uri" in *"/cyclonedds.xml")
    _xml_nic=$(grep -o 'NetworkInterface name="[^"]*"' "$_bridge_uri" | head -1 | cut -d'"' -f2)
    [ "$_xml_nic" = "$FIELD_NIC" ] || _bridge_env_die "FIELD_NIC=$FIELD_NIC 인데 configs/cyclonedds.xml 은 $_xml_nic 에 묶여 있다 — 둘을 같이 바꾼다" ;;
esac

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$_bridge_uri"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RCUTILS_LOGGING_BUFFERED_STREAM=0

# 워크스페이스 공통 메시지(shared_msgs) 패키지 소싱
if [ -f "$REPO_ROOT/install/setup.bash" ]; then
    source "$REPO_ROOT/install/setup.bash"
fi

# ⭐ 어느 프로파일을 왜 골랐는지 **매 기동마다** 남긴다. 09-09 에 조용히 돌던
#    재시작 루프는 이 한 줄이 없어서 아무도 못 봤다.
echo "[bridge-env] DDS 프로파일 = $_bridge_profile" >&2
echo "CYCLONEDDS_URI=$CYCLONEDDS_URI"
