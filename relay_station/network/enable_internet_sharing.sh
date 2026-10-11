#!/usr/bin/env bash
# =============================================================================
# 🌐 공유기 인터넷 연결 공유 (NAT Masquerade & Packet Forwarding)
# - 수신: enx001122334455 (유선 LAN, 교육장 공유기 198.51.100.x)
# - 송출: wlp0s20f3 (무선 Wi-Fi, 외부 인터넷)
# =============================================================================

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] sudo 권한으로 실행해야 합니다: sudo $0"
    exit 1
fi

echo "[1/3] IP 포워딩 활성화..."
sysctl -w net.ipv4.ip_forward=1 >/dev/null
echo "net.ipv4.ip_forward = 1" > /etc/sysctl.d/99-ipforward.conf

echo "[2/3] iptables NAT 및 패킷 포워딩 규칙 적용..."
# 중복 추가 방지를 위해 기존 규칙 제거 후 추가
iptables -t nat -D POSTROUTING -o wlp0s20f3 -j MASQUERADE 2>/dev/null || true
iptables -D FORWARD -i enx001122334455 -o wlp0s20f3 -j ACCEPT 2>/dev/null || true
iptables -D FORWARD -i wlp0s20f3 -o enx001122334455 -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true

iptables -t nat -A POSTROUTING -o wlp0s20f3 -j MASQUERADE
iptables -A FORWARD -i enx001122334455 -o wlp0s20f3 -j ACCEPT
iptables -A FORWARD -i wlp0s20f3 -o enx001122334455 -m state --state RELATED,ESTABLISHED -j ACCEPT

echo "[3/3] iptables 영구 저장 (재부팅 시 자동 유지)..."
if which netfilter-persistent >/dev/null 2>&1; then
    netfilter-persistent save >/dev/null 2>&1
    echo "  - netfilter-persistent에 영구 저장 완료!"
else
    # iptables-save 파일로 저장하고 rc.local 또는 systemd로 복구하도록 설정
    mkdir -p /etc/iptables
    iptables-save > /etc/iptables/rules.v4
    echo "  - /etc/iptables/rules.v4 저장 완료!"
fi

echo "============================================================"
echo " 🟢 공유기(198.51.100.x) 인터넷 연결 공유가 정상 활성화되었습니다!"
echo "============================================================"
