#!/usr/bin/env bash
# 블랙박스 로그 삭제 (D12). 로봇·관제 PC 공용. 자동으로 지우지 않는다 — 작업 후 직접 실행.
#
#   tools/purge_logs.sh                 # 날짜별 용량만 보여준다
#   tools/purge_logs.sh 2026-09-21      # 그 날짜 폴더 삭제 (확인 질문)
#   tools/purge_logs.sh all --yes       # 전부, 확인 없이
#   tools/purge_logs.sh all --yes ~/pinky_logs/robot_192.168.4.1   # 다른 루트
set -euo pipefail

target="${1:-}"
yes=0
root="$HOME/pinky_logs"
for arg in "${@:2}"; do
    case "$arg" in
        --yes) yes=1 ;;
        *) root="$arg" ;;
    esac
done

if [[ ! -d "$root" ]]; then
    echo "[purge] $root 없음"
    exit 0
fi
echo "[purge] $root"
du -sh "$root"/* 2>/dev/null || echo "(비어 있음)"
[[ -z "$target" ]] && exit 0

if [[ "$target" == "all" ]]; then
    victims=("$root"/*)
else
    victims=("$root/$target")
    [[ -d "$root/$target" ]] || { echo "[purge] $root/$target 없음" >&2; exit 1; }
fi
echo "[purge] 삭제 대상: ${victims[*]}"
if [[ $yes -ne 1 ]]; then
    read -r -p "정말 삭제할까요? [y/N] " ans
    [[ "$ans" == "y" || "$ans" == "Y" ]] || { echo "취소"; exit 0; }
fi
rm -rf "${victims[@]}"
echo "[purge] 완료"
du -sh "$root" 2>/dev/null || true
