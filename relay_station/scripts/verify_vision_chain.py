#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""영상 → 좌표 → 중계 → 관제 평면. **그 사슬이 통했는지 한 번에 판정한다.**

    python3 relay_station/scripts/verify_vision_chain.py
    python3 relay_station/scripts/verify_vision_chain.py --base http://198.51.100.3:8889

2026-09-19 사용자 확정 종료지점의 수락 2~5 를 재고 표로 찍는다. 수락 1(좌표가
프레임에서 유도된다)은 사람이 카메라 앞에서 물체를 옮겨 눈으로 보는 것이라 여기 없다.

## 🔴 왜 스크립트여야 하나 — 손으로는 못 잰다

`STALE_AFTER_MS` 가 **500 ms** 다. 그런데 셸에서 `curl` 한 번, `python` 한 번을
따로 띄우면 **프로세스 시작만으로 그 문턱을 넘는다**(2026-09-19 도커 복제본 실측:
손으로 친 POST → status 사이가 1,768 ms 였다). 그래서 손으로 재면 **언제나 STALE 이
나오고**, "신선한 적이 없다" 와 "낡았다" 를 구분할 수 없다.

같은 프로세스 안에서 재면 왕복이 40~50 ms 라 FRESH 가 또렷이 보인다. 판정 도구가
관측 대상보다 느리면 그 판정은 도구를 재는 것이다.

## 무엇을 판정하나

| 수락 | 여기서 재는 것 |
| :--- | :--- |
| 2 | POST 응답의 `accepted` · `published` · `hasReceiver` 셋이 **다 참** |
| 3·4 | 보낸 값이 `/api/status.visionPose` 에 그대로 실린다(도메인 유량은 `ros2 topic hz` 로 따로) |
| 5 | 보내기를 멈추면 문턱 뒤 `VISION_STALE` 이 되고 **좌표 키가 사라진다** |

⚠️ 수락 2 의 `hasReceiver` 는 **그 순간 구독자가 있어야** 참이다. 지금 이 토픽의
생산 소비자는 0 이므로, 재기 전에 구독자를 하나 띄워 둔다:

    ROS_DOMAIN_ID=8 ros2 topic echo /robot1/vision_pose geometry_msgs/msg/PoseStamped

🔴 **타입을 반드시 붙인다.** 안 붙이면 첫 발행 전에는
`WARNING: topic does not appear to be published yet / Could not determine the type`
로 **조용히 죽는다**(2026-09-19 복제본 실측). 그러면 구독자가 없는 채로 재게 되고
`hasReceiver=false` 가 나오는데, 그건 사슬이 끊긴 것이 아니라 **내 시험 도구가 안 뜬
것**이다. 띄운 뒤 `ros2 topic info` 로 `Subscription count: 1` 을 먼저 확인하라.

안 띄우면 `hasReceiver=false` 가 나오는데 그건 **결함이 아니라 아무도 안 듣는 것**이다.
이 스크립트는 그 둘을 갈라서 말한다.
"""

import argparse
import json
import sys
import time
import urllib.error
import urllib.request

DEFAULT_BASE = "http://127.0.0.1:8889"
TIMEOUT = 5.0


def _post(base, payload):
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(base + "/api/vision/pose", data=body,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT) as res:
            return res.status, json.loads(res.read().decode("utf-8") or "{}")
    except urllib.error.HTTPError as exc:
        try:
            return exc.code, json.loads(exc.read().decode("utf-8") or "{}")
        except Exception:
            return exc.code, {}


def _vision_state(base, robot_id):
    with urllib.request.urlopen(base + "/api/status", timeout=TIMEOUT) as res:
        st = json.loads(res.read().decode("utf-8"))
    vp = (st.get("visionPose") or {}).get(robot_id)
    if vp is None:
        raise SystemExit("🔴 /api/status 에 visionPose.%s 가 없다 — 게이트웨이가 "
                         "이 변경을 안 들고 있다(심링크·재기동 확인)." % robot_id)
    return vp


def main(argv=None):
    ap = argparse.ArgumentParser(description="영상→좌표→중계 사슬 판정")
    ap.add_argument("--base", default=DEFAULT_BASE, help="게이트웨이 주소")
    ap.add_argument("--robot", default="robot1")
    ap.add_argument("--x", type=float, default=0.42)
    ap.add_argument("--y", type=float, default=-0.13)
    ap.add_argument("--yaw", type=float, default=1.57)
    args = ap.parse_args(argv)

    print("관측점 판정 — %s · %s" % (args.base, time.strftime("%Y-%m-%dT%H:%M:%S%z")))
    print()

    rows = []          # (수락, 판정, 근거)
    payload = {"robotId": args.robot, "x": args.x, "y": args.y, "yaw": args.yaw,
               "computedAtMs": int(time.time() * 1000)}

    t0 = time.time()
    code, body = _post(args.base, payload)
    rtt_post = (time.time() - t0) * 1000.0

    if code != 200 or not body.get("accepted"):
        rows.append(("2", "FAIL", "POST HTTP %s · %s" % (code, body.get("reason") or body)))
        _report(rows)
        return 1

    published = bool(body.get("published"))
    has_recv = bool(body.get("hasReceiver"))
    subs = body.get("subscribers")
    if published and has_recv:
        rows.append(("2", "PASS", "accepted·published·hasReceiver 셋 다 참 (구독자 %s)" % subs))
    elif published:
        rows.append(("2", "부분", "발행은 됐는데 **아무도 안 듣는다**(구독자 %s). "
                                 "`ros2 topic echo /%s/vision_pose` 를 띄우고 다시 재라."
                     % (subs, args.robot)))
    else:
        rows.append(("2", "FAIL", "published=false — 게이트웨이의 ROS 노드가 없다"))

    if body.get("clockSuspect"):
        print("⚠️  연산 노드 시계가 중계와 %s ms 어긋난다 — 좌표는 받았다."
              % body.get("clockOffsetMs"))

    # 수락 3·4 — 보낸 값이 그대로 실리는가 (같은 프로세스라 문턱 안에 든다)
    t1 = time.time()
    vp = _vision_state(args.base, args.robot)
    rtt_status = (time.time() - t1) * 1000.0

    if vp.get("state") != "VISION_FRESH":
        rows.append(("3·4", "FAIL", "보낸 직후인데 %s 다 (왕복 %.0f+%.0f ms). "
                                    "문턱 %s ms 보다 느리면 이 도구가 관측 대상보다 느린 것이다."
                     % (vp.get("state"), rtt_post, rtt_status, body.get("staleAfterMs"))))
    elif abs(vp.get("x", 1e9) - args.x) > 1e-6 or abs(vp.get("y", 1e9) - args.y) > 1e-6:
        rows.append(("3·4", "FAIL", "값이 다르다 — 보낸 (%s, %s) / 받은 (%s, %s)"
                     % (args.x, args.y, vp.get("x"), vp.get("y"))))
    else:
        rows.append(("3·4", "PASS", "값 일치 (x=%s y=%s · ageMs=%s · 왕복 %.0f+%.0f ms)"
                     % (vp.get("x"), vp.get("y"), vp.get("ageMs"), rtt_post, rtt_status)))

    # 수락 5 — 멈추면 사라지는가. **뮤테이션이 통과 조건이다.**
    stale_after = int(body.get("staleAfterMs") or 500)
    time.sleep((stale_after + 400) / 1000.0)
    vp2 = _vision_state(args.base, args.robot)
    if vp2.get("state") == "VISION_STALE" and "x" not in vp2:
        rows.append(("5", "PASS", "멈춘 뒤 %s → 좌표 키 사라짐 (ageMs=%s)"
                     % (vp2.get("state"), vp2.get("ageMs"))))
    elif "x" in vp2:
        rows.append(("5", "FAIL", "🔴 낡았는데 좌표가 **아직 실려 있다** — 화면이 "
                                  "한 번 보낸 흔적을 계속 보여 준다 (state=%s ageMs=%s)"
                     % (vp2.get("state"), vp2.get("ageMs"))))
    else:
        rows.append(("5", "FAIL", "state=%s (기대 VISION_STALE)" % vp2.get("state")))

    return _report(rows)


def _report(rows):
    print()
    print("  수락 | 판정 | 근거")
    print("  ---- | ---- | ----")
    worst = 0
    for acc, verdict, why in rows:
        print("  %-4s | %-4s | %s" % (acc, verdict, why))
        if verdict == "FAIL":
            worst = 1
        elif verdict == "부분" and worst == 0:
            worst = 2
    print()
    if worst == 0:
        print("✅ 사슬이 통했다. 남은 것은 수락 1(좌표가 프레임에서 유도되는가) — 사람이 본다.")
    elif worst == 2:
        print("🟡 사슬은 이어졌으나 듣는 쪽이 없다. 구독자를 띄우고 다시 재라.")
    else:
        print("🔴 사슬이 끊겼다. 위 근거의 자리부터 본다.")
    return 0 if worst == 0 else worst


if __name__ == "__main__":
    sys.exit(main())
