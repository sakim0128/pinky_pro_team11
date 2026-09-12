#!/usr/bin/env bash
#
# 한 도메인의 노드 파라미터를 전부 YAML 로 덤프한다.
#
#   tools/dump_nav2_params.sh <ROS_DOMAIN_ID> [출력디렉터리]
#   tools/dump_nav2_params.sh 10
#   tools/dump_nav2_params.sh 11 /tmp/pinky2
#
# 왜 필요한가
#   rqt_reconfigure 로 바꾼 값은 **노드를 재시작하면 사라진다.** ROS2 의 rqt 에는
#   ROS1 의 dynamic_reconfigure 같은 save 버튼이 없다. 지금 값을 파일로 남기려면
#   `ros2 param dump` 를 써야 하는데, Jazzy 에서는 stdout 으로만 나오고
#   (--output-dir / --print 옵션은 제거됐다) 노드도 8~10개라 손으로 치면 오타가 난다.
#
# 덤프한 파일을 그대로 params 파일로 쓰지 말 것
#   덤프에는 use_sim_time, qos_overrides.*, 플러그인이 런타임에 추가한 항목까지
#   전부 들어 있다. 원본 params 파일의 주석과 구조도 사라진다.
#   **덤프는 "내가 뭘 바꿨는지" 를 찾는 용도로 쓰고, 바뀐 항목만 원본에 옮긴다.**

set -euo pipefail

if [ $# -lt 1 ]; then
    cat >&2 <<'USAGE'
사용법: dump_nav2_params.sh <ROS_DOMAIN_ID> [출력디렉터리]

  예) tools/dump_nav2_params.sh 10
      tools/dump_nav2_params.sh 11 /tmp/pinky2

출력디렉터리를 생략하면 ./nav2_params_dump_domain<ID> 에 만든다.
USAGE
    exit 1
fi

DOMAIN="$1"
OUT="${2:-nav2_params_dump_domain${DOMAIN}}"

if ! [[ "$DOMAIN" =~ ^[0-9]+$ ]]; then
    echo "오류: ROS_DOMAIN_ID 는 숫자여야 합니다 (받은 값: $DOMAIN)" >&2
    exit 1
fi

export ROS_DOMAIN_ID="$DOMAIN"
# WiFi 너머 로봇을 보려면 디스커버리가 localhost 로 막혀 있으면 안 된다.
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

echo "도메인 ${DOMAIN} 의 노드를 찾는 중..."
# 우리 도구와 뷰어는 뺀다. 하드코딩한 목록보다 실제 노드를 찾는 쪽이 튼튼하다.
mapfile -t NODES < <(ros2 node list 2>/dev/null \
    | grep -vE '^/(preflight_|fleet_|rviz|rqt_|_ros2cli)' \
    | sed '/^$/d')

if [ "${#NODES[@]}" -eq 0 ]; then
    echo "도메인 ${DOMAIN} 에 노드가 없습니다." >&2
    echo "  로봇에서 pinky_navigation 이 떠 있는지, 도메인이 맞는지 확인하세요." >&2
    echo "  ros2 run pinky_fleet preflight  로 한 번에 진단할 수 있습니다." >&2
    exit 1
fi

mkdir -p "$OUT"
echo "노드 ${#NODES[@]}개 → ${OUT}/"

ok=0
skipped=0
for node in "${NODES[@]}"; do
    # /local_costmap/local_costmap -> local_costmap__local_costmap.yaml
    name="${node#/}"
    file="${OUT}/${name//\//__}.yaml"
    if ros2 param dump "$node" --timeout 3 > "$file" 2>/dev/null && [ -s "$file" ]; then
        printf '  OK    %-44s -> %s\n' "$node" "$(basename "$file")"
        ok=$((ok + 1))
    else
        printf '  건너뜀 %-44s (파라미터 서비스 무응답)\n' "$node"
        rm -f "$file"
        skipped=$((skipped + 1))
    fi
done

echo
echo "완료: ${ok}개 저장, ${skipped}개 건너뜀"
echo
echo "다음에 할 일 — 덤프를 통째로 쓰지 말고, 바꾼 항목만 찾아서 원본에 옮깁니다."
echo
echo "  # 예: 29강에서 만지는 값들"
echo "  grep -n 'inflation_radius\\|cost_scaling_factor' ${OUT}/*costmap*.yaml"
echo
echo "  # 로봇의 원본 params 파일 위치와 launch 인자 확인"
echo "  ssh pinky@<로봇IP> \"find \\\$(ros2 pkg prefix pinky_navigation)/share -name '*.yaml'\""
echo "  ros2 launch pinky_navigation bringup_launch.xml --show-args"
echo
echo "  # 두 로봇에 똑같이 적용했는지 마지막에 확인"
echo "  ROS_DOMAIN_ID=10 ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius"
echo "  ROS_DOMAIN_ID=11 ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius"
