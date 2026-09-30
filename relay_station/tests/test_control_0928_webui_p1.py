# -*- coding: utf-8 -*-
"""관제 U-1~U-6 (REQ_20260927_RELAY_WEBUI_CEO_REVIEW §3 P1) — V2 화면의 경영진용 첫 화면.

정적 시험(JS 런타임 없이 소스 구조를 잰다 — test_v2_front_honesty 와 같은 방식). 실제 브라우저 시험은
test_review_0926_ui2.py 의 Chrome 픽스처에 붙어 있다(격리 netns 에서만 돈다).

U-1 보기 전용: :18081 · ?view=1 · /api/status.view_only 면 움직이는 조작만 숨고 멈추는 조작은 남는다.
U-2 요약 띠: 한 문장 + 안전/진행/데이터 신호등 — /api/fleet/status 만으로, 값 없으면 미수신.
U-3 대시보드 용어: 로봇 카드에 영문 상태 상수를 그대로 쓰지 않는다.
U-4 데모 = map5 임시 도로망(BL·BR·TR·J): 옛 경기장 노드(START_A·START_B·GOAL_C)가 데모·기본값에 없다. 비전은 미수신.
U-5 영상 없음: <img data-media> 가 실패하면 문구로 바뀐다(검은 상자 금지).
U-6 부제 = 프로파일 label · 경과 시간(화면 기준).
"""
import io
import os
import re

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STATIC = os.path.join(REPO, "relay_station", "gateway_web", "static")


def _read(name):
    return io.open(os.path.join(STATIC, name), encoding="utf-8").read()


def _code():
    """주석을 걷어 낸 JS."""
    src = re.sub(r"/\*.*?\*/", "", _read("fleet_control_v2.js"), flags=re.S)
    return "\n".join(re.sub(r"(^|[^:\"'])//.*$", r"\1", line) for line in src.splitlines())


def _fn(code, head):
    body = code[code.index(head):]
    return body[:body.index("\n  }\n")]


# ---- U-1 --------------------------------------------------------------------------------------------------------

def test_U1_보기_전용은_18081_포트·view_1·서버_플래그_셋_중_하나면_켜진다():
    code = _code()
    assert 'location.port === "18081"' in code and 'PARAMS.get("view") === "1"' in code
    assert "state.gateway?.view_only === true" in _fn(code, "function viewOnly")
    apply = _fn(code, "function applyViewOnly")
    assert 'classList.toggle("view-only", on)' in apply and 'pill.hidden = !on' in apply


def test_U1_움직이는_조작에만_data_moving_이_있고_멈추는_조작에는_없다():
    html = _read("fleet_control_v2.html")
    def btn(cmd):
        m = re.search(r'<button[^>]*data-command="%s"[^>]*>' % cmd, html)
        assert m, cmd
        return m.group(0)
    assert "data-moving" in btn("start") and "data-moving" in btn("resume")
    assert "data-moving" not in btn("stop") and "data-moving" not in btn("estop")
    for i in ("profile-select", "profile-switch", "profile-robot-maps", "profile-initial-poses"):
        assert re.search(r'id="%s"[^>]*data-moving' % i, html) or re.search(r'data-moving[^>]*id="%s"' % i, html), i
    cards = _fn(_code(), "function renderDashboardRobots")
    assert 'data-robot-cmd="resume" data-robot="${i}" data-moving="1"' in cards
    assert re.search(r'data-robot-cmd="stop" data-robot="\$\{i\}" \$\{DEMO', cards)     # 로봇 정지엔 없다
    css = _read("fleet_control_v2.css")
    assert "body.view-only [data-moving] { display: none !important; }" in css
    assert ".status-pill[hidden] { display: none; }" in css                              # inline-flex 가 hidden 을 이기던 것
    assert 'id="view-only-pill" hidden' in html and "제어는 현장 노트북 :8889" in html


# ---- U-2 --------------------------------------------------------------------------------------------------------

def test_U2_요약_띠는_대시보드_맨_위에_있고_문장_하나와_신호등_셋이다():
    html = _read("fleet_control_v2.html")
    dash = html[html.index('id="tab-dashboard"'):html.index('id="tab-mission"')]
    assert dash.index('id="summary-band"') < dash.index("mission-toolbar")
    for i in ("summary-sentence", "light-safety", "light-progress", "light-data"):
        assert 'id="%s"' % i in dash, i
    assert html.index('id="alert-banner"') < html.index('id="summary-band"')            # 알림 띠 아래


def test_U2_요약은_순수_함수이고_값이_없으면_미수신·끊기면_옛_값·경고면_빨강():
    code = _code()
    body = _fn(code, "function buildSummary")
    assert '"플릿 상태 미수신' in body and 'm.link === "waiting" || !m.mission' in body
    assert 'if (m.link === "no_coordinator")' in body and '["bad", "코디네이터 없음"]' in body   # 검토 P3: 연결부터 가른다
    assert 'if (m.link === "down")' in body and '"옛 값"' in body
    assert 'if (m.warning) progress = ["bad"' in body
    # 검토 P1: 신선한 보고(fresh)가 없는 로봇이 있으면 안전·진행은 초록이 아니다 — known(목록에 있음)만으로는 부족
    assert "const unknown = robots.filter(r => !r.fresh || r.localized === false)" in body
    assert 'unknown.length ? ["neutral"' in body and body.index('unknown.length ? ["neutral"') < body.index('["ok", "래치·정지 없음"]')
    assert "robotProgressTone(r)" in body and 'none.length ? ["neutral"' in body and 'warn.length ? ["warn"' in body
    assert body.index('bad.length ? ["bad"') < body.index('none.length ? ["neutral"') < body.index('warn.length ? ["warn"') < body.index('["ok", "전부 주행·도착"]')
    derive = _fn(code, "function deriveRobot")
    assert "fresh: Boolean(fleetRobot(index)) && !f.is_stale && f.lane_status != null" in derive
    band = _fn(code, "function renderSummaryBand")
    assert 'warning: link === "ok" ? (state.fleet?.warning || null) : null' in band       # renderLink 와 같은 규칙
    assert "window.__v2 = {buildSummary" in code


def test_U2_로봇_한_구절은_응답_없음과_위험한_상태를_먼저_말한다():
    body = _fn(_code(), "function robotPhrase")
    order = [body.index(k) for k in ("r.linkDown", "!r.fresh", "r.driveCode === 8", "r.driveCode === 9", "r.held", '"해제 보류"',
                                     "r.localized === false", "r.startGaveUp", '"ARRIVAL_CONFIRMED"', "r.waitingFor", "r.driveCode === 1")]
    assert order == sorted(order), order
    assert '"출발 대기"' in body and 'ko("drive", r.driveCode)' in body                   # 모르는 번호를 "출발 대기" 로 꾸미지 않는다
    tone = _fn(_code(), "function robotProgressTone")
    assert 'if (!r.fresh) return "neutral";' in tone and tone.index('"neutral"') < tone.index('"bad"') < tone.index('"ok"')


# ---- U-3 --------------------------------------------------------------------------------------------------------

def test_U3_대시보드_카드에_영문_상태_상수를_그대로_쓰지_않는다():
    cards = _fn(_code(), "function renderDashboardRobots")
    # 보이는 글(<strong>…</strong>·알약 본문)은 우리말 함수를 거친다 — 원문 상수는 title 툴팁에만 남는다
    assert '<strong title="${esc(r.navStatus)}">${esc(ko("nav", r.navStatus))}</strong>' in cards
    assert '<strong title="${esc(r.arrival)}">${esc(ko("arrival", r.arrival))}</strong>' in cards
    assert '${esc(driveText(r))}</span>' in cards and 'title="${esc(r.reason || r.driveState)}"' in cards
    assert "r.vision?.state ||" not in cards and 'ko("vision", r.vision.state)' in cards
    for ko_call in ('ko("pose", r.poseSource || "미수신")', "clearanceText(r, state.fleet?.mission_state)", 'ko("reason", r.reason)'):
        assert ko_call in cards, ko_call
    # 보이는 자리에 원문이 그대로 나오는 옛 꼴이 없다
    for raw in ('">${esc(r.driveState)}</span>', '<strong>${esc(r.navStatus)}</strong>', '<strong>${esc(r.arrival)}</strong>', '${esc(r.reason || "—")}'):
        assert raw not in cards, raw
    code = _code()
    drive = _fn(code, "function driveText")
    assert '!r.known || typeof r.driveCode !== "number") return "미수신"' in drive and '`끊김 · 옛 값: ${base}`' in drive
    clear = _fn(code, "function clearanceText")
    # 검토 P2: 숫자가 있다고 "허가됨" 이 아니다 — RUNNING·신선·진행 인덱스보다 앞일 때만
    assert 'if (!r.fresh) return "미수신";' in clear and 'if (mission !== "RUNNING") return "없음";' in clear
    assert "r.clearUntil > (r.routeIdx ?? 0)" in clear and clear.index('"정지 중"') < clear.index("허가됨 (")
    assert "통행 허가</span><strong>${r.clearUntil" not in cards                        # 인덱스 숫자를 그대로 보이던 것
    assert 'Math.round(r.cte*100)+" cm"' in cards


def test_U3_상태_번호표는_LaneStatus_열_값을_다_우리말로_갖는다():
    code = _code()
    table = code[code.index("const PHRASES"):code.index("function ko(")]
    for k in range(10):                                                                   # 0~9 — 3~6 은 팀11 레인 로봇(검토 P3)
        assert re.search(r"\b%d:\"[^\"]+\"" % k, table), k
    assert "reason:" in table and '"E-STOP":"비상정지"' in table
    assert 'PoseFuser:"위치 융합기(PoseFuser)"' in table                                  # 외부 fix 없이도 이 이름이라 "외부 비전" 단정 금지
    ko = _fn(code, "function ko")
    assert 'return "미수신"' in ko and "?? String(v)" in ko                                # 모르는 값은 원문 그대로, 없으면 미수신


# ---- U-4 --------------------------------------------------------------------------------------------------------

def test_U4_데모와_지도_기본값에_옛_경기장_노드가_없다():
    code = _code()
    demo = code[code.index("const DEMO_NODES"):code.index("async function refresh")]
    for old in ("START_A", "START_B", "GOAL_C", "JUNCTION_CORE", "2.34m", "ARUCO 2/2"):
        assert old not in demo, old
    for node in ("BL", "BR", "TR", "J"):
        assert '{id:"%s"' % node in demo, node
    for old_node in ("JS", "JW", "CW1", "TC", "RE", "MC"):                       # map4 도로망 노드는 데모에도 없다
        assert '{id:"%s"' % old_node not in demo, old_node
    assert "x_min:-0.01,x_max:2.35,y_min:-0.01,y_max:1.27" in demo                # map5: 원점 −0.01, 2.36 × 1.28
    assert 'start_node:"BL",goal_node:"TR"' in demo and 'start_node:"BR",goal_node:"BL"' in demo
    assert 'tracking:"미수신' in demo and "zone_events:[]" in demo
    # 검토 P2: 없는 태블릿 PoseFix 사슬을 데모에서도 LIVE 로 두지 않는다 — 위치 출처는 AMCL
    for fake in ("PoseFix", "Y700", 'pose_source:"PoseFuser"', 'pose_fuser:"LIVE"', "tablet_to_relay:{state:\"LIVE\""):
        assert fake not in demo, fake
    assert demo.count('pose_source:"AMCL"') == 2 and "traces:[]" in demo
    m = _fn(code, "function collectMapData")
    assert "zones: n.zones || []" in m and "START_A" not in m
    assert "state.profiles?.map?.bounds" in m and '"team11_map5" ? {x_min:-0.01' in m       # 지도 규격은 프로파일이 싣는다
    assert "team11_map4" not in m and "legacy_bottom_left" not in m


def test_U4_데모_모드는_프로파일도_싣는다():
    ref = _fn(_code(), "async function refresh")
    assert "state.profiles=d.profiles" in ref


# ---- U-5 --------------------------------------------------------------------------------------------------------

def test_U5_영상_img_는_data_media_를_달고_인라인_onerror_가_없다():
    html = _read("fleet_control_v2.html")
    assert "onerror=" not in html
    imgs = re.findall(r"<img[^>]*>", html)
    assert len(imgs) >= 4 and all("data-media=" in i and "data-missing-why=" in i for i in imgs), imgs
    assert "항공뷰(태블릿)" not in html and "중계 탑뷰(USB 웹캠)" in html                  # relay-cam 은 중계 노트북 웹캠이다(검토 P2)
    code = _code()
    media = _fn(code, "function initMedia")
    assert 'box.className = "media-missing"' in media and "img.replaceWith(box)" in media
    assert "img.complete && img.naturalWidth === 0" in media                             # 스크립트 전에 실패한 것도
    assert "img.dataset.missingWhy" in media
    # 검토 P1: 실물 MJPEG 경로는 프레임 없이 200 으로 열려 있어 error 가 안 온다 — 첫 프레임 기한
    assert "setTimeout(() => { if (img.isConnected && img.naturalWidth === 0) fail(); }, MEDIA_FIRST_FRAME_MS)" in media
    assert re.search(r"const MEDIA_FIRST_FRAME_MS = [3-9]\d{3};", code)
    assert ".media-missing {" in _read("fleet_control_v2.css")


# ---- U-6 --------------------------------------------------------------------------------------------------------

def test_U6_부제는_프로파일_label_이고_경과는_화면_기준이라고_말한다():
    code = _code()
    html = _read("fleet_control_v2.html")
    assert 'id="subtitle"' in html and 'id="mission-elapsed"' in html
    sub = _fn(code, "function renderSubtitle")
    assert "profileLabel()" in sub and '"좌표 프로파일 · "' in sub and '" (옛 값)"' in sub   # 끊기면 옛 label 이라고 말한다(검토 P3)
    el = _fn(code, "function elapsedText")
    assert "(화면 기준)" in el and '"소요 "' in el and "state.runningFrozenMs" in el          # 완료·정지 뒤엔 시계가 선다(검토 P2)
    track = _fn(code, "function trackRunning")
    assert 'mission === "RUNNING"' in track and 'mission === "ASSIGNED"' in track
    assert 'mission === "DONE" || mission === "STOPPED" || mission === "ESTOP"' in track
    ref = _fn(code, "async function refresh")
    assert "trackRunning" not in ref                                                      # OPS-2 의 refresh 줄은 그대로(시험이 잠갔다)
    # 검토 P3: 새 그리기는 try/catch 안, 1 s 시계는 연결 판정(OPS-2)을 먼저 돌린다
    extras = _fn(code, "function renderControlExtras")
    assert "try {" in extras and "renderSummaryBand();" in extras
    tick = code[code.index("setInterval(()=>{"):code.index("},1000);")]
    assert tick.index("const linkNow = renderLink();") < tick.index("renderSummaryBand()") and "if (linkNow !== state.renderedLink) renderAll();" in tick
    band = _fn(code, "function renderSummaryBand")
    assert "if (el.textContent !== s.sentence)" in band                                  # 바뀐 것만 쓴다(검토 P2 — 높이·aria-live)
    css = _read("fleet_control_v2.css")
    assert "-webkit-line-clamp: 2" in css and "min-height: 2.9em; max-height: 2.9em" in css   # 띠 높이 고정 — 아래 정지 버튼이 안 움직인다
