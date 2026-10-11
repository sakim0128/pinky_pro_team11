#!/usr/bin/env bash
# T10 — 게이트웨이 systemd --user 유닛을 설치한다.
#
# ⭐ 복사가 아니라 **심링크**다. 2026-09-09 에 실행본 사본을 0 으로 만든 것과 같은 이유 —
#    사본이 둘이면 "어느 쪽이 권위인가"를 파일이 답하지 못하고, 손이 틀린 쪽을 고른다.
#    (domain_bridge/install_bridge.sh 는 아직 cp 를 쓴다. 그건 별건이다)
#
# ⚠️ Linger 는 여기서 못 켠다 — root 권한이 필요하다. 안 켜면 헤드리스 재부팅에서
#    user manager 자체가 안 떠서 이 유닛도 안 뜬다. 마지막에 안내한다.
set -u

UNIT_NAME="field-master-gateway.service"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SRC="$REPO_ROOT/relay_station/systemd/$UNIT_NAME"
UNIT_DIR="$HOME/.config/systemd/user"
DST="$UNIT_DIR/$UNIT_NAME"

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}"

echo "==========================================================="
echo " T10  게이트웨이 systemd --user 유닛 설치"
echo "==========================================================="
echo "  레포 원본 : $SRC"
echo "  설치 위치 : $DST"

if [ ! -f "$SRC" ]; then
    echo "  🔴 레포에 유닛 파일이 없습니다: $SRC"
    exit 1
fi

# ---- 프리플라이트: 카메라 장치 권한 (R-4) ---------------------------------
#
# 🔴 2026-09-14 실측: USB 웹캠과 내장 캠이 **둘 다** 안 열렸다. 장치를 잡고 있는
#    프로세스는 없었고, cv2.VideoCapture 를 by-id 경로 · /dev/videoN · 정수 인덱스
#    넷 다 시도해 전부 isOpened=False 였다. 원인은 권한이다:
#      /dev/video* 는 root:video 0660 + ACL 이고, logind 는 **활성 seat 세션**에만
#      uaccess ACL 을 준다. Chrome Remote Desktop 세션은 seat 가 없어 영영 못 받는다
#      (그때 활성 seat0 는 gdm 이 쥐고 있었다).
#    `setfacl` 로 즉시 열리긴 하지만 세션이 바뀌거나 재부팅하면 지워진다.
#    **정식 자리는 `video` 그룹**이다.
#
# ⚠️ 이 스크립트는 sudo 를 치지 않는다 — 장비에서 직접 고치지 않는 것이 이 레포의 규칙이다.
#    말만 하고 넘어간다. 유닛 설치 자체는 카메라 권한과 무관하게 성공해야 한다.
#
# ⭐⭐ **두 가지를 따로 본다.** `usermod` 만 하고 재부팅을 안 하면
#    계정 정보(`id -nG "$USER"`)에는 video 가 보이는데 **이미 떠 있는 프로세스**는
#    옛 그룹이라 카메라가 여전히 안 열린다. 그 중간 상태를 "됐다"로 읽으면
#    같은 자리에서 또 한 번 태운다. 둘을 갈라서 각각 다른 말을 한다.
#
# ⚠️ `… | grep -q` 를 쓰지 않는다 — pipefail 아래에서 **매치할 때 오히려 실패**한다
#    (grep 이 일찍 끝나며 상류에 SIGPIPE). herestring 은 쓰는 프로세스가 없어 안전하다.
preflight_video_group() {
    local db_groups proc_groups
    db_groups="$(id -nG "$USER" 2>/dev/null || true)"      # 계정 정보 (usermod 즉시 반영)
    proc_groups="$(id -nG 2>/dev/null || true)"             # 지금 이 프로세스의 실제 그룹

    if ! grep -qw video <<< "$db_groups"; then
        echo "  🔴 카메라 권한 없음 — 로컬 캠(relay-cam·relay-cam-internal)이 안 열립니다."
        echo "     고치기:  sudo usermod -aG video $USER     그리고 **재부팅**"
        echo "     (그룹은 새 로그인 세션에만 적용됩니다. setfacl 은 세션마다 지워집니다)"
        return 0
    fi
    if ! grep -qw video <<< "$proc_groups"; then
        echo "  🟡 video 그룹은 등록됐으나 **지금 세션에는 아직 안 붙었습니다.**"
        echo "     이 상태로는 카메라가 여전히 안 열립니다 — **재부팅**하세요."
        echo "     (systemd --user 매니저도 옛 그룹으로 떠 있습니다)"
        return 0
    fi
    return 0
}
preflight_video_group

mkdir -p "$UNIT_DIR"

# 기존이 실물 파일이면 보관한다. 지우지 않는다 — 되돌릴 수 있어야 한다.
if [ -e "$DST" ] && [ ! -L "$DST" ]; then
    STAMP=$(date +%Y%m%d-%H%M%S)
    mv "$DST" "$UNIT_DIR/.stale-$UNIT_NAME.$STAMP"
    echo "  기존 실물 유닛 보관: $UNIT_DIR/.stale-$UNIT_NAME.$STAMP"
fi

ln -sfn "$SRC" "$DST.newlink"
mv -T "$DST.newlink" "$DST"
echo "  링크: $UNIT_NAME -> $(readlink "$DST")"

systemctl --user daemon-reload || { echo "  🔴 daemon-reload 실패"; exit 1; }
systemctl --user enable "$UNIT_NAME" >/dev/null 2>&1 || true

echo
echo " [설치 후 실측]"
for p in UnitFileState Restart RestartUSec SuccessExitStatus TimeoutStopUSec FragmentPath ActiveState MainPID; do
    echo "  $(systemctl --user show "$UNIT_NAME" -p "$p" 2>/dev/null)"
done

echo
LINGER=$(loginctl show-user "$USER" -p Linger --value 2>/dev/null || echo unknown)
echo " [재부팅 자동 기동 조건]  Linger=$LINGER"
if [ "$LINGER" != "yes" ]; then
    echo "  🟡 Linger 가 꺼져 있습니다. **헤드리스 재부팅이면 이 유닛은 안 뜹니다**"
    echo "     (그래픽 로그인을 하면 그때 뜹니다)"
    echo "     켜려면 root 권한이 필요합니다:"
    echo "         sudo loginctl enable-linger $USER"
else
    echo "  ✅ 로그인 없이도 user manager 가 뜹니다"
fi

echo
echo " ⚠️ 이 스크립트는 실행 중인 게이트웨이를 **재기동하지 않습니다.**"
echo "    새 유닛 설정은 다음 기동부터 적용됩니다. 지금 적용하려면:"
echo "        systemctl --user restart $UNIT_NAME"
echo "==========================================================="
