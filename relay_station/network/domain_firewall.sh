#!/usr/bin/env bash
# =============================================================================
# 🔒 도메인 격리 방화벽 (중계 관제 노트북)
#
#   ROS_DOMAIN_ID 는 접근 제어 수단이 아니다. UDP 포트 오프셋일 뿐이라
#   누구든 `export ROS_DOMAIN_ID=8` 한 줄이면 관제 평면에 들어올 수 있다.
#   도메인을 실제로 격리하는 방법은 포트 대역 방화벽 하나뿐이다.
#
#   RTPS 포트 대역 = 7400 + 250 × 도메인ID  ~  +249
#     도메인  8 (관제) : 9400 – 9649   ← 중계 노트북 자신만 허용
#     도메인  9 (팀원) : 9650 – 9899
#     도메인 10 (로봇1): 9900 – 10149
#     도메인 11 (로봇2): 10150 – 10399
#     도메인 12 (로봇3): 10400 – 10649
#     도메인 13 (로봇4): 10650 – 10899
#
#   관제 도메인 8은 중계 노트북 내부(브릿지 ↔ 대시보드) 전용이다.
#   로봇도 팀원도 8번에 들어올 이유가 없으므로 자기 자신만 허용한다.
#
#   사용법:
#     ./domain_firewall.sh            규칙 미리보기 (아무것도 바꾸지 않음)
#     sudo ./domain_firewall.sh apply 규칙 적용
#     sudo ./domain_firewall.sh clear 규칙 제거
#     ./domain_firewall.sh show       현재 적용 상태 확인
# =============================================================================
set -euo pipefail

CHAIN="PINKY_DOMAIN8"
RELAY_IP="198.51.100.3"
PORT_LO=9400
PORT_HI=9649

G='\033[0;32m'; Y='\033[1;33m'; C='\033[0;36m'; B='\033[1m'; N='\033[0m'

rules() {
cat <<EOF
iptables -N $CHAIN
iptables -F $CHAIN
# 중계 노트북 자신(로컬 및 유선 IP)만 관제 도메인 8에 참여할 수 있다.
iptables -A $CHAIN -i lo -j RETURN
iptables -A $CHAIN -s 127.0.0.1 -j RETURN
iptables -A $CHAIN -s $RELAY_IP -j RETURN
# 그 외 전부 차단 — 팀원 노트북(198.51.100.10~50), 로봇, Tailscale 원격 포함
iptables -A $CHAIN -j DROP
# 관제 도메인 포트 대역을 체인으로 보낸다
iptables -A INPUT -p udp --dport $PORT_LO:$PORT_HI -j $CHAIN
EOF
}

ACTION="${1:-preview}"

case "$ACTION" in
  preview)
    echo -e "${B}적용될 규칙 (미리보기 — 시스템을 변경하지 않습니다)${N}"
    echo
    rules | sed 's/^/  /'
    echo
    echo -e "${Y}적용하려면:${N} sudo $0 apply"
    echo -e "${Y}주의:${N} 방화벽 변경은 시스템 설정 변경입니다. 위 규칙을 확인한 뒤 직접 실행하세요."
    ;;

  apply)
    [ "$(id -u)" -eq 0 ] || { echo "sudo 로 실행하세요: sudo $0 apply"; exit 1; }
    iptables -N "$CHAIN" 2>/dev/null || true
    iptables -F "$CHAIN"
    iptables -A "$CHAIN" -i lo -j RETURN
    iptables -A "$CHAIN" -s 127.0.0.1 -j RETURN
    iptables -A "$CHAIN" -s "$RELAY_IP" -j RETURN
    iptables -A "$CHAIN" -j DROP
    iptables -C INPUT -p udp --dport "$PORT_LO:$PORT_HI" -j "$CHAIN" 2>/dev/null \
        || iptables -A INPUT -p udp --dport "$PORT_LO:$PORT_HI" -j "$CHAIN"
    echo -e "${G}관제 도메인 8 (UDP $PORT_LO-$PORT_HI) 격리 적용 완료${N}"
    echo -e "${Y}재부팅 시 사라집니다. 영구 적용은 iptables-persistent 를 검토하세요.${N}"
    ;;

  clear)
    [ "$(id -u)" -eq 0 ] || { echo "sudo 로 실행하세요: sudo $0 clear"; exit 1; }
    iptables -D INPUT -p udp --dport "$PORT_LO:$PORT_HI" -j "$CHAIN" 2>/dev/null || true
    iptables -F "$CHAIN" 2>/dev/null || true
    iptables -X "$CHAIN" 2>/dev/null || true
    echo -e "${G}격리 규칙 제거 완료${N}"
    ;;

  show)
    echo -e "${B}현재 $CHAIN 체인${N}"
    sudo iptables -L "$CHAIN" -n -v 2>/dev/null || echo "  (미적용)"
    echo
    echo -e "${B}INPUT 연결 지점${N}"
    sudo iptables -L INPUT -n | grep -E "$CHAIN|udp dpt" || echo "  (없음)"
    ;;

  *)
    echo "사용법: $0 [preview|apply|clear|show]"; exit 1 ;;
esac
