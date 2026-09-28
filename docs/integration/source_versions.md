# 작업 기준

- 최종 저장소(upstream): sakim0128/pinky_pro_team11
- 개발 포크(origin): 0gpublike/pinky_pro_team10
- PR 대상: upstream mini_project_2
- 기능 브랜치: feat/live-web-state
- 시작 커밋: 1f505cb88719c44391cc2781dcc3c5f09c9b7795
- 기존 웹 참고: team10 pinky_pro/web-fleet (762ecd2)
- 다른 참고 저장소: 0gpublike/robot_mini_project_pinky (이번 변경에 코드 이식 없음)

기존 웹 브랜치의 미커밋 지도 설정과 백업 파일은 원래 checkout에 보존했습니다.
별도 worktree에서 개발했습니다. push/PR 생성/merge는 수행하지 않았습니다.
PR 제출 전 upstream 최신 변경을 다시 확인해야 합니다.

# 작업 기준 — relay_station · hybrid 로봇 스택 (rkd1rjs2 팀원)

- 최종 저장소(upstream): sakim0128/pinky_pro_team11 · PR 대상: mini_project_2
- 시작 커밋: 1f505cb88719c44391cc2781dcc3c5f09c9b7795 → 2026-09-28 upstream `5faddf1`(PR #3) 병합
- 원 저장소: rkd1rjs2/robot_mini_project_pinky main `f8fc97d`(relay_station export 기준) · 에이전트 커밋 `6f28c63` `d0352f4` `2750fe0` `46757f9` 반영
- 가져온 원본: mini_project_2_aerial_view `21ebc31` 의 `PoseFix.msg` · `pose_fuser.py` · `pose_fuser_node.py` · `test_pose_fuser.py`(바이트 동일)
- 팀11 파일 변경은 `pinky_lane_msgs/CMakeLists.txt` · `pinky_fleet_agent/setup.py` · `pinky_fleet_agent/package.xml` 세 개(각 한두 줄)
