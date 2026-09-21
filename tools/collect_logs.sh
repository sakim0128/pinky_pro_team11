#!/usr/bin/env bash
# 로봇의 블랙박스 로그를 관제 PC 로 가져온다 (D12). 작업이 끝난 뒤 PC 에서 실행.
#
#   tools/collect_logs.sh                          # pinky@192.168.4.1:~/pinky_logs/ → ~/pinky_logs/robot_192.168.4.1/
#   tools/collect_logs.sh --delete-remote          # 복사가 체크섬으로 검증된 뒤에만 로봇 쪽을 지운다
#   tools/collect_logs.sh [--delete-remote] pinky@192.168.4.1 '~/pinky_logs/' ~/pinky_logs/robot_pinky1
#
# rsync 는 이어받기(--partial)를 하므로 Wi-Fi 가 끊기면 다시 실행하면 된다.
set -euo pipefail

delete_remote=0
if [[ "${1:-}" == "--delete-remote" ]]; then
    delete_remote=1
    shift
fi
host="${1:-pinky@192.168.4.1}"
remote="${2:-~/pinky_logs/}"
local_dir="${3:-$HOME/pinky_logs/robot_${host#*@}}"
mkdir -p "$local_dir"

echo "[collect] $host:$remote → $local_dir"
rsync -avh --progress --partial "$host:$remote" "$local_dir/"

if [[ $delete_remote -eq 1 ]]; then
    echo "[collect] 체크섬 검증..."
    pending="$(rsync -a --checksum --itemize-changes --dry-run "$host:$remote" "$local_dir/" | grep -E '^>f' || true)"
    if [[ -n "$pending" ]]; then
        echo "[collect] 아직 다르거나 못 받은 파일이 있어 로봇 쪽을 지우지 않습니다:" >&2
        echo "$pending" >&2
        exit 2
    fi
    echo "[collect] 검증 OK — 로봇의 $remote 내용을 삭제합니다"
    ssh "$host" "rm -rf $remote/*"
    echo "[collect] 삭제 완료"
fi
du -sh "$local_dir"
