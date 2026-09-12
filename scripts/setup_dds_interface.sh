#!/usr/bin/env bash
#
# Fast DDS 가 쓸 네트워크 인터페이스를 한 개로 고정한다.
# 관제 PC 와 핑키 2대에서 각각 한 번씩 실행한다.
#
#     bash scripts/setup_dds_interface.sh            # 인터페이스 자동 탐지
#     bash scripts/setup_dds_interface.sh wlan0      # 직접 지정
#
# 왜 필요한가
#   Fast DDS 는 멀티캐스트로 자기를 알릴 때 "모든" 네트워크 인터페이스의 주소를 함께
#   광고하고, 상대는 그중 하나로 유니캐스트 응답을 보낸다. 관제 PC 의 tailscale0 나
#   핑키의 ap0(자체 핫스팟) 처럼 서로 닿지 않는 주소가 섞여 있으면 그쪽을 골라
#   핸드셰이크가 끝나지 않는다.
#
#   증상이 고약하다 — `ros2 multicast send/receive` 는 양방향으로 잘 통하는데
#   `ros2 node list` 에는 상대 노드가 하나도 안 보인다. 에러도 없다.
#
# 주의: IP 가 바뀌면(DHCP) 다시 실행해야 한다. 공유기에서 고정 IP 를 주는 편이 낫다.

set -euo pipefail

PROFILE="$HOME/fastdds_wifi.xml"
BASHRC="$HOME/.bashrc"
EXPORT_LINE='export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_wifi.xml'

if ! command -v ip >/dev/null 2>&1; then
    echo "'ip' 명령을 찾을 수 없습니다 (iproute2). 설치 후 다시 실행하세요." >&2
    exit 1
fi

# --- 인터페이스 결정 ---------------------------------------------------
if [ $# -ge 1 ]; then
    IFACE="$1"
else
    # 기본 경로(default route)가 나가는 인터페이스를 쓴다.
    IFACE="$(ip -4 route show default | awk '{print $5; exit}')"
    if [ -z "$IFACE" ]; then
        echo "인터페이스를 자동으로 찾지 못했습니다. 이름을 직접 주세요:" >&2
        ip -4 -o addr show | awk '{print "  " $2 "  " $4}' >&2
        echo "  예)  bash $0 wlan0" >&2
        exit 1
    fi
fi

MY_IP="$(ip -4 -o addr show "$IFACE" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1)"
if [ -z "$MY_IP" ]; then
    echo "'$IFACE' 에 IPv4 주소가 없습니다." >&2
    ip -4 -o addr show | awk '{print "  " $2 "  " $4}' >&2
    exit 1
fi

echo "인터페이스: $IFACE"
echo "IP        : $MY_IP"

OTHERS="$(ip -4 -o addr show | awk -v k="$IFACE" '$2 != "lo" && $2 != k {print "  " $2 "  " $4}')"
if [ -n "$OTHERS" ]; then
    echo
    echo "DDS 에서 제외되는 인터페이스:"
    echo "$OTHERS"
fi

# --- 프로파일 생성 -----------------------------------------------------
# 127.0.0.1 을 반드시 포함한다. 빼면 같은 기계 안의 노드끼리(로봇의 Nav2 내부 통신)
# 서로를 못 찾는다.
cat > "$PROFILE" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<!-- scripts/setup_dds_interface.sh 가 생성. 인터페이스: $IFACE -->
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_fixed</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>127.0.0.1</address>
          <address>$MY_IP</address>
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="fixed_iface" is_default_profile="true">
      <rtps>
        <userTransports><transport_id>udp_fixed</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
EOF

echo
echo "생성: $PROFILE"

# --- .bashrc 등록 ------------------------------------------------------
if grep -qF 'FASTRTPS_DEFAULT_PROFILES_FILE' "$BASHRC" 2>/dev/null; then
    echo "이미 $BASHRC 에 FASTRTPS_DEFAULT_PROFILES_FILE 이 있습니다 — 건너뜁니다."
    grep -n 'FASTRTPS_DEFAULT_PROFILES_FILE' "$BASHRC"
else
    printf '\n# Fast DDS 인터페이스 고정 (setup_dds_interface.sh)\n%s\n' "$EXPORT_LINE" >> "$BASHRC"
    echo "$BASHRC 에 추가했습니다."
fi

cat <<'NOTE'

다음으로 할 것
  1) ros2 daemon stop            # 옛 발견 정보 캐시를 버린다
  2) 돌고 있는 launch 를 전부 Ctrl+C
  3) 새 터미널을 연다             # .bashrc 를 다시 읽어야 적용된다
  4) echo $FASTRTPS_DEFAULT_PROFILES_FILE   # 경로가 찍히는지 확인

세 대(관제 PC, 핑키 2대) 모두에서 실행해야 한다. 한 대라도 빠지면 발견이 안 된다.
NOTE
