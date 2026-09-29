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

- 최종 저장소(upstream): sakim0128/pinky_pro_team11 · PR 대상: `mini_project_integration`(09-28 저녁 upstream 이 만든 브랜치, = `main` `094e5d7`; 처음 계획은 mini_project_2)
- 시작 커밋: 1f505cb88719c44391cc2781dcc3c5f09c9b7795 → 2026-09-28 upstream `5faddf1`(PR #3) 병합 → 같은 날 저녁 `main` `094e5d7`(PR #2 · #4, mini_project_1: `pinky_fleet_sim` · nav2 튠 · 파라미터 감사) 병합 `f2d4e5b`, 충돌 0
- 원 저장소: rkd1rjs2/robot_mini_project_pinky main `f8fc97d`(relay_station export 기준) · 에이전트 커밋 `6f28c63` `d0352f4` `2750fe0` `46757f9` 반영
- 2026-09-28 저녁 추가: 원 저장소 main `1eed3f8`(제어권 정책 — `relay_station/gateway_web/control_policy.py` · `configs/control_allow.json` · V2 화면 · 시험 20) 을 같은 규칙으로 옮김 (`docs/integration/control_policy.md`)
- 가져온 원본: mini_project_2_aerial_view `21ebc31` 의 `PoseFix.msg` · `pose_fuser.py` · `pose_fuser_node.py` · `test_pose_fuser.py`(바이트 동일)
- 팀11 코드·설정 변경은 `pinky_lane_msgs/CMakeLists.txt` · `pinky_fleet_agent/setup.py` · `pinky_fleet_agent/package.xml` 세 개(각 한두 줄); 문서는 이 파일과 `status.md` 에 절을 덧붙였다
- 2026-09-29 중계 브리지 개편: 1단계 `be5256a`(live 웹 계약) · 2단계 `47ee78b`(중계 화면·비전 월드·캘리브레이션 삭제, 중계 콘솔) — 원 저장소 main 은 아직 옛 화면을 싣는다(다시 옮길 때 제외 목록)
