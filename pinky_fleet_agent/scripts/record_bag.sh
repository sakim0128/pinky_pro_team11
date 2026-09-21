#!/usr/bin/env bash
# 블랙박스 기록 (D12). 로봇·관제 PC 양쪽에서 쓴다. launch 의 record:=True 가 이 스크립트를 부른다.
#
#   record_bag.sh robot   pinky1 [out_root]     # 로봇: 라이다·odom·cmd_vel·초음파·카메라·상태
#   record_bag.sh station pinky1 [out_root]     # 관제: 브릿지 토픽 + 인식 결과 + 관제 상태 (두 로봇 모두)
#
# 출력: <out_root>/<YYYY-MM-DD>/<HHMMSS>_<host>_<mode>_<name>/   (기본 out_root = ~/pinky_logs)
# 5 분 단위로 파일을 나눈다 (--max-bag-duration 300) — 크래시가 나도 마지막 5 분만 잃는다.
# 목록에 있지만 아직 없는 토픽은 나타나는 순간부터 기록된다 (rosbag2 가 주기적으로 발견).
# 용량: 카메라 10 fps JPEG ≈ 1.4 GB/h, 나머지 < 100 MB/h. 지우는 법: tools/purge_logs.sh
set -euo pipefail

mode="${1:-}"
name="${2:-pinky1}"
root="${3:-$HOME/pinky_logs}"
if [[ "$mode" != "robot" && "$mode" != "station" ]]; then
    echo "usage: $0 robot|station <robot_name> [out_root]" >&2
    exit 1
fi

day="$(date +%F)"
stamp="$(date +%H%M%S)"
host="$(hostname -s)"
out="$root/$day/${stamp}_${host}_${mode}_${name}"
mkdir -p "$root/$day"

topics=()
if [[ "$mode" == "robot" ]]; then
    topics+=(/scan /odom /tf /tf_static /cmd_vel
             /us_sensor/range /ir_sensor/range /batt_state /battery/percent
             /initialpose /amcl_pose
             "/$name/camera/image/compressed" "/$name/state" "/$name/command"
             "/$name/lane_status" "/$name/lane_command" "/$name/lane_path" "/$name/route"
             "/$name/pose_fix" "/$name/fix_status")
else
    for r in pinky1 pinky2; do
        topics+=("/$r/camera/image/compressed" "/$r/state" "/$r/command" "/$r/plan"
                 "/$r/lane_status" "/$r/lane_command" "/$r/lane_path" "/$r/route" "/$r/pose_fix"
                 "/$r/lane_debug/compressed" "/$r/scene_state")
    done
    topics+=(/fleet/lane/status /fleet/lane/control /fleet/coordinator_status
             /overhead/status /overhead/debug/compressed)
fi

echo "[record_bag] $mode $name → $out" >&2
echo "[record_bag] 토픽 ${#topics[@]}개, 5 분 분할. 중지: Ctrl+C. 삭제: tools/purge_logs.sh $day" >&2
exec ros2 bag record -o "$out" --max-bag-duration 300 "${topics[@]}"
