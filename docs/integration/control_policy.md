# 제어권 정책 (2026-09-28) — 팀원 노트북도 중계를 거쳐 움직이는 명령을 낸다 (기본은 닫힘)

> **결정**: 사용자 2026-09-28 저녁 — "중계 장비를 통하되 명령을 팀원들 노트북에서 내릴 수 있게".
> **구현**(rkd1rjs2 팀원, 원 저장소 main `1eed3f8` 을 이 저장소 배치로 옮김): `relay_station/gateway_web/control_policy.py`(순수) + `gateway_web_server.py` 배선 + V2 화면 + `relay_station/configs/control_allow.json`.
> **바뀌지 않은 것**: 멈추는 명령은 어디서든(OPS-1·D7) · 관측·캘리브레이션·비전 세계는 현장 노트북만(`LOCAL_CONTROL_IPS`) · 액션은 토픽으로 감싸 브리지를 건넌다(D6) — 이 정책은 그 앞의 **HTTP 문**만 바꾼다.
> **2026-09-29**: 로봇 이름은 `pinkyN` 하나(`/api/pinkyN/stop|resume`), 시작·목적지는 `/api/fleet/assign`(V2 "미션 배정" 카드) — 움직이는 명령의 목록이 그만큼 줄었다.

## 1. 규칙 (다섯 줄)

1. **기본은 닫힘.** `relay_station/configs/control_allow.json` 이 없거나 깨졌거나 `enabled` 항목이 없으면 예전과 같다 — 중계 노트북(로컬)만 움직이는 명령을 낸다. 환경변수로는 열리지 않는다.
2. **허용 목록은 파일.** 중계 운영자가 현장에서 주소·이름을 넣고 `enabled: true` 로 바꾼다. **재기동 없이** 먹는다(mtime) — 재기동은 로봇을 래치시키므로.
3. **한 번에 한 사람.** 허용 주소 중 처음 움직이는 명령을 낸 쪽이 제어권을 쥔다. 다른 허용 주소는 **409 `CONTROL_HELD`**(누가 쥐었는지 이름·주소). 쥔 쪽이 `ttl_s`(30 s) 동안 아무 요청도 없으면 만료 — V2 는 2 s 마다 `/api/status` 를 부르니 화면이 열려 있는 한 유지되고 노트북을 닫으면 30 s 뒤 풀린다.
4. **로컬 콘솔이 언제나 이긴다.** 중계 노트북에서 낸 움직이는 명령은 제어권을 가져온다(팀원 화면은 "제어권: 중계 노트북(로컬)" 로 바뀌고 버튼이 잠긴다).
5. **멈추는 명령은 정책 밖.** 일시정지·비상정지·로봇 정지는 누구든, 언제든.

## 2. 접점

| 것 | 어디 | 무엇 |
| :-- | :-- | :-- |
| 정책 | `gateway_web/control_policy.py` | `AllowList`(파일, mtime 재읽기, 깨지면 빈 목록 + 오류 문구) · `ControlPolicy.may_move / acquire / release / touch / status` |
| 배선 | `gateway_web_server.py` `_deny_if_cannot_move` | 로봇 재개(`/api/pinkyN/resume`) · 좌표 전환 ①②③ · 플릿 start/resume/assign 세 경로(2026-09-29: 옛 우회 목표·미션 경로 `/api/robotN/goal`·`/mission` 은 지웠다 — 로봇은 `lane_agent_node` 뿐). 거절 = 403(`FORBIDDEN`) / 409(`CONTROL_HELD`), 본문에 `reason`·`message` |
| 상태 | `GET /api/status` | `view_only`(정책) + `control {holder, holder_ip, mine, holder_ttl_s, ttl_s, allow_source, allow_count, allow_error, controller_name}` |
| 제어권 | `GET /api/control` · `POST /api/control/acquire {name}` · `POST /api/control/release` | 잡기는 허용 주소(로컬 포함)만 · 놓기는 쥔 쪽(로컬 포함)만 |
| 화면 | V2 헤더 `#control-pill` · `#control-acquire` · `#control-release` | 나=초록 · 비어 있음=회색 · 남이 쥠=노랑 + `body.control-held` 로 움직이는 버튼 **잠금(숨기지 않음)** · 보기 전용 주소·데모에서는 제어권 UI 없음 |
| 설정 | `relay_station/configs/control_allow.json` | `ttl_s` · `controllers[{ip,name,enabled}]`. 기본 파일은 `enabled` 0, 주소는 문서용 `198.51.100.x` |

## 3. 시험

- `relay_station/tests/test_control_policy.py` — 순수 12: 닫힘 기본 · enabled 아님 · 깨진 파일 · 첫 명령이 잡음/409 · ttl 만료·touch 연장 · touch 는 쥔 쪽만 · 놓기 권한 · acquire 이름/409/403 · 로컬 우선 · 파일 재읽기 · 상태 필드 · 기본 파일 닫힘.
- `relay_station/tests/test_control_0928_control_policy_wiring.py` — 정적 8: 세 경로 배선(우회 목표·미션 경로가 없음도 잰다) · 로봇 정지 정책 밖 · 관측 등 로컬만 · 403/409 · status/control 엔드포인트 · 화면 잠금 · 기본 파일.
- 실물/격리 netns: 중계 노트북이 `relay_station/tests` 전부(rclpy 필요) + 화면 확인 — 허용 주소 노트북에서 `주행 시작` 이 먹고, 두 번째 노트북은 409 와 노랑 알약, 중계 노트북이 누르면 제어권이 넘어온다.

## 4. 운영 (중계 운영자)

```bash
# 팀원 노트북을 허용 (재기동 없음)
$EDITOR relay_station/configs/control_allow.json     # {"ip": "<노트북 LAN IP>", "name": "팀원 A 노트북", "enabled": true}
curl -s http://127.0.0.1:8889/api/control                  # allow_count 가 1 이상, allow_error null
```

팀원 노트북(같은 Wi-Fi)에서 `http://<중계 IP>:8889/fleet_control_v2.html` → 헤더에 "제어권 비어 있음 — 첫 명령이 잡는다" → 주행 시작을 누르면 제어권을 잡는다. 다른 팀원은 노랑 알약과 잠긴 버튼을 본다. 되돌리려면 `enabled: false`.

## 5. 남긴 것

- 제어권은 게이트웨이 메모리에만 있다(재기동하면 비워진다 — 재기동은 어차피 로봇 재개가 필요한 사건).
- 허용은 주소 단위다. 이름은 화면 표시용이지 인증이 아니다 — 같은 Wi-Fi 안의 신뢰를 전제한다(교육장). 인증이 필요해지면 토큰을 따로 둔다.
- `:18081` 원격 경로는 socat 이 127.0.0.2 로 붙어 들어오므로 여전히 보기 전용이다(허용 목록에 127.0.0.2 를 넣지 않는 한).
