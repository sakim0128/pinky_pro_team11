(() => {
  "use strict";

  const PARAMS = new URLSearchParams(location.search);
  const DEMO = PARAMS.get("demo") === "1";
  // U-1(REQ_20260927_RELAY_WEBUI_CEO_REVIEW): :18081 원격 보기 경로는 서버가 움직이는 명령을 거절한다(OPS-1 `74123be`).
  // 화면도 그걸 알아야 한다 — 움직이는 조작(주행 시작·재시작·로봇 재개·좌표 전환·초기 위치)은 숨기고 멈추는 조작은 남긴다.
  // 서버가 /api/status 에 view_only 를 실어 주면 그것도 본다(viewOnly()).
  const VIEW_ONLY = location.port === "18081" || PARAMS.get("view") === "1";
  // U-3: 첫 화면(대시보드)은 우리말 — 원문 상수는 title 툴팁·진단 탭에 남긴다. 표에 없는 값은 원문 그대로, 없는 값은 "미수신".
  const PHRASES = {
    // LaneStatus.drive_state — 3~6 은 팀11 레인 로봇만 낸다(Nav2 에이전트는 0·1·2·7·8·9)
    drive: {0:"대기", 1:"주행 중", 2:"통행 대기", 3:"횡단보도 정지", 4:"횡단보도 통과", 5:"장애물 대기", 6:"차선 놓침", 7:"도착", 8:"비상정지", 9:"연결 끊김 — 재개 필요",
            10:"바리게이트 대기", 11:"차선 탐색 회전", 12:"교차로 정지", 13:"교차로 통과"},
    nav: {ACTIVE:"이동 중", NAVIGATING:"이동 중", HOLD:"멈춤", WAITING:"대기", IDLE:"대기", SUCCEEDED:"도착", ABORTED:"실패", CANCELED:"취소됨"},
    arrival: {NOT_ARRIVED:"아직", ARRIVAL_PENDING:"도착 확인 중", ARRIVAL_CONFIRMED:"도착 확정"},
    // 검토(P3): PoseFuser 는 외부 fix 를 받은 적 없어도 이 이름으로 보고된다 — "외부 비전" 이라 단정하지 않는다
    pose: {PoseFuser:"위치 융합기(PoseFuser)", AMCL:"로봇 자체 추정(AMCL)", odom:"주행계만", vision:"외부 비전"},
    vision: {TRACKED:"추적 중", LOST:"놓침", STALE:"오래됨"},
    // 에이전트가 사유에 영문 토큰만 실을 때(route_chain: RESUME·STOP·E-STOP·HOLD) — 우리말 문장은 그대로 지나간다
    reason: {RESUME:"재개", STOP:"정지", "E-STOP":"비상정지", HOLD:"보류", IDLE:"대기", "FLEETCOMMAND STOP":"플릿 정지", "LANECOMMAND STOP":"레인 정지"},
    mission: {IDLE:"미션 없음", ASSIGNED:"출발 대기", RUNNING:"주행 중", STOPPED:"일시정지", ESTOP:"비상정지", DONE:"완료"}
  };
  function ko(kind, v) {
    if (v === null || v === undefined || v === "" || v === "—") return "미수신";
    const t = PHRASES[kind] || {};
    return t[v] ?? t[String(v).toUpperCase()] ?? String(v);
  }
  const state = {
    fleet: null,
    gateway: null,
    ops: {
      overview: null, mission: null, navigation: null, vision: null,
      communications: null, diagnostics: null, events: null, config: null
    },
    logs: [],
    lastUpdate: 0,          // 마지막으로 /api/fleet/status 를 **받은** 시각(ms) — 실패한 갱신은 이 값을 바꾸지 않는다
    fleetError: null,       // 마지막 /api/fleet/status 가 실패했으면 그 이유
    renderedLink: null,     // 카드를 마지막으로 그릴 때의 연결 상태 — 바뀌면 다시 그린다
    layers: { route: true, robot: true, vision: true, zones: true },
    runningSince: null,     // U-6: 화면이 RUNNING 을 처음 본 시각(ms) — 코디네이터의 시작 시각이 아니다(그 값은 아직 API 에 없다)
    runningFrozenMs: null,  // RUNNING 을 벗어난 순간의 소요 시간(ms) — 완료·정지 뒤에도 시계가 도는 것을 막는다(검토 P2)
    profiles: null
  };
  // 통합 검토 OPS-2: 플릿 상태를 이만큼 못 받으면 화면 값은 옛 값이다(갱신 2 s · 응답이 매달린 요청도 여기서 잡는다)
  const FLEET_STALE_MS = 5000;

  // 단일 정본 관제 맵 배경 이미지 (/api/fleet/profile_map.png 또는 /my_map.png)
  const MAP_BG_IMG = new Image();
  MAP_BG_IMG.src = "/api/fleet/profile_map.png";

  const $ = (id) => document.getElementById(id);
  const fmt = (v, digits = 2) => typeof v === "number" && Number.isFinite(v) ? v.toFixed(digits) : "—";
  const nowText = () => new Date().toLocaleTimeString("ko-KR", {hour12:false});
  const ageText = (sec) => typeof sec === "number" && Number.isFinite(sec) ? (sec < 1 ? Math.round(sec * 1000) + " ms" : sec.toFixed(1) + " s") : "미측정";
  const esc = (v) => String(v ?? "").replace(/[&<>"']/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#039;"}[c]));

  // 통합 검토 OPS-2: 예전엔 코디네이터가 없어도(FLEET_COORDINATOR_UNAVAILABLE, HTTP 200) 게이트웨이가 죽어도 초록
  // 'RELAY CONNECTED' 에 마지막 값이 얼어 있었다. 'ok' · 'no_coordinator' · 'down'(못 받음·옛 값) · 'waiting'(아직 한 번도 못 받음)
  function fleetLink() {
    if (DEMO) return "ok";
    const age = state.lastUpdate ? Date.now() - state.lastUpdate : null;
    if (state.fleetError || age === null || age > FLEET_STALE_MS) return state.lastUpdate || state.fleetError ? "down" : "waiting";
    if (state.fleet?.status === "FLEET_COORDINATOR_UNAVAILABLE") return "no_coordinator";
    return "ok";
  }

  function fleetAgeText() {
    return state.lastUpdate ? ageText((Date.now() - state.lastUpdate) / 1000) + " 전" : "받은 적 없음";
  }

  function setPill(el, text, tone="neutral") {
    if (!el) return;
    el.textContent = text;
    el.className = "status-pill " + tone;
  }

  async function getJson(url) {
    const res = await fetch(url, {cache:"no-store"});
    if (!res.ok) throw new Error(url + " -> HTTP " + res.status);
    return res.json();
  }

  async function postJson(url, payload={}) {
    const res = await fetch(url, {
      method:"POST",
      headers:{"Content-Type":"application/json"},
      body:JSON.stringify(payload)
    });
    let body = {};
    try { body = await res.json(); } catch (_) {}
    if (!res.ok) throw new Error(body.message || body.error || ("HTTP " + res.status));
    return body;
  }

  function initTabs() {
    document.querySelectorAll(".nav-item").forEach(btn => {
      btn.addEventListener("click", () => {
        document.querySelectorAll(".nav-item").forEach(x => x.classList.remove("active"));
        document.querySelectorAll(".tab-page").forEach(x => x.classList.remove("active"));
        btn.classList.add("active");
        const pane = $("tab-" + btn.dataset.tab);
        if (pane) pane.classList.add("active");
        requestAnimationFrame(drawMaps);
      });
    });
  }

  function initControls() {
    const robotsHost = $("dashboard-robots");
    if (robotsHost) robotsHost.addEventListener("click", (ev) => {   // 카드 버튼은 위임으로 받는다(카드는 없을 때 한 번 만든다)
      const btn = ev.target.closest("[data-robot-cmd]");
      if (!btn || btn.disabled) return;
      robotAction(Number(btn.dataset.robot), btn.dataset.robotCmd);
    });
    document.querySelectorAll("[data-command]").forEach(btn => {
      btn.addEventListener("click", async () => {
        const cmd = btn.dataset.command;
        btn.disabled = true;
        try {
          const body = await postJson("/api/fleet/" + cmd, {});
          // 통합 검토 OPS-8: 202(코디네이터가 처리했는지 모름)가 200 과 똑같이 조용했다 — 로봇 버튼·전환 버튼처럼 말한다
          if (body.applied === null || body.success === false) {
            alert(`⚠️ ${btn.textContent.trim()}: ` + (body.message || "") + " (적용 여부 모름)");
          }
          await refresh();
        } catch (e) {
          alert("명령 전송 실패: " + e.message);
        } finally {
          btn.disabled = false;
        }
      });
    });
    document.querySelectorAll("[data-layer]").forEach(cb => {
      cb.addEventListener("change", () => {
        state.layers[cb.dataset.layer] = cb.checked;
        drawMaps();
      });
    });
    $("refresh-events")?.addEventListener("click", refresh);
    $("event-filter")?.addEventListener("change", renderEvents);
    $("control-acquire")?.addEventListener("click", () => controlAction("acquire"));
    $("control-release")?.addEventListener("click", () => controlAction("release"));
    $("btn-fault-pose-block")?.addEventListener("click", async () => {
      try {
        await postJson("/api/fault/toggle_pose_fix", {});
        await refresh();
      } catch (e) {
        alert("관제 위치 차단/복구 제어 실패: " + e.message);
      }
    });
  }

  function robotName(key) {
    return key === "pinky1" ? "Pinky 1" : "Pinky 2";
  }

  function robotKey(index) { return index === 1 ? "pinky1" : "pinky2"; }

  function fleetRobot(index) {
    const robots = state.fleet?.robots || {};
    return robots[robotKey(index)] || null;
  }

  function navRobot(index) {
    const nav = state.ops.navigation?.robots || {};
    return nav[robotKey(index)] || null;
  }

  function diagRobot(index) {
    const d = state.ops.diagnostics?.robots || {};
    return d[robotKey(index)] || null;
  }

  function missionRobot(index) {
    const m = state.ops.mission?.robots || {};
    return m[robotKey(index)] || null;
  }

  function deltaText(r) {
    const vx = r.extVision?.x ?? r.vision?.x;
    const vy = r.extVision?.y ?? r.vision?.y;
    if (typeof r.x === "number" && typeof r.y === "number" && typeof vx === "number" && typeof vy === "number") {
      const d = Math.hypot(r.x - vx, r.y - vy);
      return `|Δ| ${(d * 100).toFixed(1)} cm (실시간 융합)`;
    }
    return "—";
  }

  function deriveRobot(index) {
    const key = robotKey(index);
    const f = fleetRobot(index) || {};
    const n = navRobot(index) || {};
    const s = f.state || {};
    const ls = f.lane_status || {};
    const gRobot = (state.gateway?.robots || {})[key] || {};
    const gComm = ((state.gateway?.fleet_comm || {})[key]) || {};
    const runtime = gRobot.runtime || gComm.runtime || "OFFLINE";
    const poseFuser = gRobot.pose_fuser || (diagRobot(index) || {}).detail?.fix_status || null;
    const fixWord = gRobot.fix_status_word || (diagRobot(index) || {}).pose_fuser || "미수신";
    const extVision = (state.gateway?.visionPose || {})[key] || n.external_vision || null;

    let runtimeBadge = "";
    if (runtime === "PHYSICAL") {
      runtimeBadge = `<span class="status-pill ok" style="font-size:0.72rem;padding:2px 6px;margin-left:6px;" title="실물 로봇 온보드 하드웨어 직결">🤖 실물</span>`;
    } else if (runtime === "DOCKER") {
      runtimeBadge = `<span class="status-pill warn" style="font-size:0.72rem;padding:2px 6px;margin-left:6px;" title="도커 가상 로봇 팜(HIL) 컨테이너">🐳 도커 가상</span>`;
    } else {
      runtimeBadge = `<span class="status-pill neutral" style="font-size:0.72rem;padding:2px 6px;margin-left:6px;" title="통신 대기 중">미연결</span>`;
    }

    return {
      index,
      known: Boolean(fleetRobot(index)),
      runtime,
      runtimeBadge,
      poseFuser,
      fixWord,
      extVision,
      poseSource: (diagRobot(index) || {}).pose_source || null,
      name: robotName(key),
      localized: s.localized ?? !f.is_unlocalized,
      x: s.x,
      y: s.y,
      yaw: s.yaw,
      navStatus: n.nav_status ?? s.nav_status ?? "—",
      driveState: ls.state_reason || ls.drive_state || "—",
      driveCode: ls.drive_state,              // LaneStatus.drive_state (8 ESTOP · 9 LINK_LOST · 2 WAIT)
      reason: String(ls.state_reason || ""),
      routeProgress: n.route_progress ?? f.comparator?.route_progress,
      cte: n.cross_track_error ?? f.comparator?.cross_track_error,
      clearUntil: f.clear_until_idx,
      waitingFor: f.waiting_for,
      blockedBy: f.blocked_by,
      stale: Boolean(f.is_stale),
      linkDown: fleetLink() === "down",       // 통합 검토 OPS-2: 게이트웨이에서 못 받는다 — 카드 값은 옛 값이다
      arrival: f.arrival_status || "NOT_ARRIVED",
      vision: extVision,
      held: Boolean(f.held),                  // 로봇별 정지 — 운영자·재시작 뒤 ESTOP·링크유실 래치(REVIEW_20260926 G)
      heldReason: f.held_reason || null,
      startNode: f.start_node || null,        // U-2: 한 문장에 "BL→TC" 로 쓴다
      goalNode: f.goal_node || null,
      startAck: Boolean(f.start?.acknowledged),
      startGaveUp: Boolean(f.start?.gave_up),
      stallSec: typeof f.stall_sec === "number" ? f.stall_sec : null,
      fresh: Boolean(fleetRobot(index)) && !f.is_stale && f.lane_status != null,
      lastHeard: typeof f.last_heard_sec === "number" ? f.last_heard_sec : null,
      routeIdx: typeof ls.route_idx === "number" ? ls.route_idx : null
    };
  }

  function toneForRobot(r) {
    if (!r.known) return "pending";          // 🔴 예전엔 데이터가 없어도 기본값 "ok"(초록)였다
    if (r.stale || r.linkDown || r.localized === false) return "bad";
    // 🔴 E-STOP·링크 유실을 초록으로 칠하던 것을 고친다 — 사유 문구가 아니라 로봇이 보고한 상태 번호로 가른다
    if (r.driveCode === 8 || r.driveCode === 9) return "bad";
    if (r.driveCode === 2) return "warn";
    if (r.held) return "warn";
    // 통합 검토 N1: 래치는 풀렸지만 에이전트가 Nav2 취소를 확인하기 전(해제 보류) — /estop 을 쥐고 목표를 취소한다. 초록이 아니다
    if (r.reason.startsWith("해제 보류")) return "warn";
    if (String(r.driveState).includes("WAIT") || r.waitingFor) return "warn";
    return "ok";
  }

  function renderDashboardRobots() {
    const host = $("dashboard-robots");
    if (!host) return;
    // 통합 검토 OPS-4: 버튼은 한 번만 만든다. 예전엔 갱신(2 s)마다 카드 전체를 innerHTML 로 다시 만들어, 누르는 사이 갱신이
    // 끼면 누른 버튼이 문서에서 빠져 클릭이 조용히 사라졌다(정지도). 갱신은 카드의 값 칸만 바꾼다.
    if (!host.querySelector("[data-robot-card]")) {
      host.innerHTML = [1,2].map(i => `
        <article class="card robot-card" data-robot-card="${i}">
          <div class="robot-card-data"></div>
          <div class="robot-actions">
            <button class="btn" data-robot-cmd="stop" data-robot="${i}" ${DEMO ? "disabled" : ""}>로봇 정지</button>
            <button class="btn" data-robot-cmd="resume" data-robot="${i}" data-moving="1" ${DEMO ? "disabled" : ""}>로봇 재개</button>
          </div>
        </article>`).join("");
    }
    [1,2].forEach(i => {
      const r = deriveRobot(i);
      const progress = typeof r.routeProgress === "number" ? Math.round(r.routeProgress * (r.routeProgress <= 1 ? 100 : 1)) + "%" : "—";
      const pos = (typeof r.x === "number" && typeof r.y === "number") ? `x ${fmt(r.x)} · y ${fmt(r.y)} · yaw ${fmt(r.yaw)}` : "위치 대기";
      const leg = r.startNode && r.goalNode ? `${esc(r.startNode)} → ${esc(r.goalNode)}` : "경로 미배정";
      const data = host.querySelector(`[data-robot-card="${i}"] .robot-card-data`);
      if (!data) return;
      // U-3: 영문 상태 상수는 첫 화면에 안 쓴다 — 원문은 title 툴팁(마우스를 올리면 보인다)과 진단 탭에
      data.innerHTML = `
          <div class="robot-head">
            <div class="robot-title"><span class="robot-index r${i}">${i}</span><div><strong>${r.name}</strong> ${r.runtimeBadge}<p>${leg}</p></div></div>
            <span class="status-pill ${toneForRobot(r)}" title="${esc(r.reason || r.driveState)}">${r.linkDown ? "끊김 · 옛 값: " : ""}${esc(driveText(r))}</span>
          </div>
          <div class="robot-kpis">
            <div class="kpi"><span>이동</span><strong title="${esc(r.navStatus)}">${esc(ko("nav", r.navStatus))}</strong></div>
            <div class="kpi"><span>관제 수락 / 거부</span><strong style="color:var(--green);">${r.poseFuser ? `${r.poseFuser.accepted ?? 0}회 / ${r.poseFuser.rejected ?? 0}` : "—"}</strong></div>
            <div class="kpi"><span>수신 신선도</span><strong class="${r.poseFuser && typeof r.poseFuser.fix_age_s === "number" && r.poseFuser.fix_age_s <= 1.0 ? "ok" : "bad"}">${r.poseFuser && typeof r.poseFuser.fix_age_s === "number" ? r.poseFuser.fix_age_s.toFixed(1) + "s" : "—"}</strong></div>
            <div class="kpi"><span>도착</span><strong title="${esc(r.arrival)}">${esc(ko("arrival", r.arrival))}</strong></div>
          </div>
          <div class="robot-detail-grid">
            <div class="data-row"><span>운영 위치</span><strong>${r.localized === false ? "위치 모름(미정위)" : pos}</strong></div>
            <div class="data-row"><span>관제 위치 융합</span><strong>${esc(r.fixWord)}</strong></div>
            <div class="data-row"><span>관제 ↔ 오돔 오차</span><strong>${deltaText(r)}</strong></div>
            <div class="data-row"><span>경로 진행</span><strong>${progress}</strong></div>
            <div class="data-row"><span>경로 이탈</span><strong>${typeof r.cte === "number" ? Math.round(r.cte*100)+" cm" : "—"}</strong></div>
            <div class="data-row"><span>대기 사유</span><strong>${r.waitingFor ? esc(r.waitingFor) + (r.blockedBy ? ` (${esc(r.blockedBy)} 통과 중)` : "") : "없음"}</strong></div>
            <div class="data-row"><span>로봇별 정지</span><strong>${r.held ? `<span class="status-pill warn">세움</span> ${esc(r.heldReason || "")}` : "없음"}</strong></div>
            <div class="data-row"><span>상태 사유</span><strong class="muted" title="${esc(r.reason)}">${esc(r.reason ? ko("reason", r.reason) : "—")}</strong></div>
          </div>`;
    });
  }

  // U-3: 로봇이 보고한 상태 번호를 우리말로. 번호가 없으면(한 번도 못 들음) "미수신". 오래된 보고(is_stale)면 옛 값이라고 붙인다.
  function driveText(r) {
    if (!r.known || typeof r.driveCode !== "number") return "미수신";
    const base = ko("drive", r.driveCode);
    return r.stale ? `끊김 · 옛 값: ${base}` : base;
  }

  // U-3: "통행 허가 42"(인덱스) 대신 "허가됨 / 대기(누구 때문)". 검토 P2: 코디네이터는 clear_until_idx 를 늘 숫자로 보내고
  // ESTOP·ASSIGNED 에선 0 으로 민다 — 숫자가 있다고 "허가됨" 이 아니다. RUNNING 이고 신선하고 진행 인덱스보다 앞일 때만.
  function clearanceText(r, mission) {
    if (!r.fresh) return "미수신";
    if (r.driveCode === 8 || r.driveCode === 9 || r.held) return "정지 중";
    if (r.waitingFor) return "대기" + (r.blockedBy ? ` — ${r.blockedBy} 통과 중` : "");
    if (r.arrival === "ARRIVAL_CONFIRMED") return "도착";
    if (mission !== "RUNNING") return "없음";
    if (typeof r.clearUntil === "number" && r.clearUntil > (r.routeIdx ?? 0)) return `허가됨 (${r.clearUntil})`;
    return "없음";
  }

  // U-1: 보기 전용 — body 에 view-only 를 달면 CSS 가 [data-moving] 을 숨긴다(멈추는 버튼은 data-moving 이 없다)
  function viewOnly() {
    return VIEW_ONLY || state.gateway?.view_only === true;
  }

  function applyViewOnly() {
    const on = viewOnly();
    document.body.classList.toggle("view-only", on);
    const pill = $("view-only-pill");
    if (pill) pill.hidden = !on;
    renderControl(on);
  }

  // 관제 2026-09-28 제어권: 서버(/api/status.control)가 말하는 대로만 그린다 — holder·mine·남은 시간.
  //   mine 이면 초록 "제어권: 나" · 비어 있으면 회색 "첫 명령이 잡는다" · 남이 쥐면 노랑 + 움직이는 버튼 잠금(body.control-held).
  //   보기 전용 주소면 제어권 UI 자체를 숨긴다(잡을 수 없는 주소다).
  function renderControl(viewOnlyOn) {
    const c = state.gateway?.control || null;
    const pill = $("control-pill"), acq = $("control-acquire"), rel = $("control-release");
    if (!pill) return;
    const known = !!c && !viewOnlyOn && !DEMO;
    pill.hidden = !known; if (acq) acq.hidden = !known; if (rel) rel.hidden = !known;
    let held = false;
    if (known) {
      if (c.mine) {
        setPill(pill, `제어권: 나 — ${c.holder} · ${c.holder_ttl_s ?? "—"} s`, "ok");
      } else if (c.holder) {
        setPill(pill, `제어권: ${c.holder} — ${c.holder_ttl_s ?? "—"} s 뒤 만료`, "warn");
        held = true;
      } else {
        setPill(pill, "제어권 비어 있음 — 첫 명령이 잡는다", "neutral");
      }
      if (acq) acq.disabled = !!c.mine;
      if (rel) rel.disabled = !c.mine;
    }
    document.body.classList.toggle("control-held", held);
  }

  async function controlAction(cmd) {
    if (DEMO) return;
    let payload = {};
    if (cmd === "acquire") {
      let name = "";
      try { name = localStorage.getItem("v2.controller.name") || ""; } catch (_) {}
      name = window.prompt("이 노트북을 뭐라고 부를까요(팀원 화면에 이 이름이 보입니다)", name) ?? null;
      if (name === null) return;
      try { localStorage.setItem("v2.controller.name", name); } catch (_) {}
      payload = {name};
    }
    try {
      const body = await postJson("/api/control/" + cmd, payload);
      state.gateway = {...(state.gateway || {}), control: body.control};
      renderControl(viewOnly());
    } catch (e) {
      alert(`⛔ 제어권 ${cmd === "acquire" ? "잡기" : "놓기"} 실패: ` + e.message);
      refresh();
    }
  }

  // U-2: 경영진용 한 문장 + 신호등 셋(안전·진행·데이터). 순수 함수 — 입력은 /api/fleet/status 에서 나온 값뿐이다.
  // 규칙: 값이 없으면 "미수신"(초록 금지) · 끊겼으면 옛 값이라 안전·진행을 칠하지 않는다 · 경고가 있으면 진행은 빨강.
  function robotPhrase(r) {
    if (!r.known) return `${r.name} 미수신`;
    const leg = r.startNode && r.goalNode ? ` ${r.startNode}→${r.goalNode}` : "";
    if (r.linkDown) return `${r.name}${leg} 옛 값`;
    // 검토 P1: 신선한 보고가 없는 로봇은 무엇을 하는지 모른다 — 주행·대기 구절을 만들지 않는다
    if (!r.fresh) return `${r.name}${leg} 응답 없음` + (r.lastHeard != null ? `(${Math.round(r.lastHeard)} s)` : "(보고 없음)");
    if (r.driveCode === 8) return `${r.name}${leg} 비상정지`;
    if (r.driveCode === 9) return `${r.name}${leg} 연결 끊김(재개 필요)`;
    if (r.held) return `${r.name}${leg} 정지(${r.heldReason || "운영자"})`;
    if (r.reason.startsWith("해제 보류")) return `${r.name}${leg} 해제 보류(Nav2 취소 확인 중)`;
    if (r.localized === false) return `${r.name}${leg} 위치 모름`;
    if (r.startGaveUp) return `${r.name}${leg} START 실패(재전송 소진)`;
    if (r.arrival === "ARRIVAL_CONFIRMED") return `${r.name}${leg} 도착`;
    if (r.arrival === "ARRIVAL_PENDING") return `${r.name}${leg} 도착 확인 중`;
    if (r.waitingFor) return `${r.name}${leg} ${r.waitingFor} 앞 대기` + (r.blockedBy ? `(${r.blockedBy} 통과 중)` : "");
    const pct = typeof r.routeProgress === "number" ? ` ${Math.round(r.routeProgress * (r.routeProgress <= 1 ? 100 : 1))}%` : "";
    if (r.driveCode === 1) return `${r.name}${leg} 주행 중${pct}`;
    if (r.driveCode === 2) return `${r.name}${leg} 통행 대기`;
    if (r.driveCode === 7) return `${r.name}${leg} 도착 보고`;
    if (r.driveCode === 0) return `${r.name}${leg} ${r.startAck ? "출발 대기(START 받음)" : "출발 대기"}`;
    return `${r.name}${leg} ${ko("drive", r.driveCode)}`;                 // 3~6(레인 로봇) — 모르는 번호도 "출발 대기" 로 꾸미지 않는다
  }

  // 로봇 하나의 진행 판정: ok(주행·도착) · warn(대기·정지 상태·출발 전) · bad(래치·정지·해제 보류·START 실패) · neutral(응답 없음)
  function robotProgressTone(r) {
    if (!r.fresh) return "neutral";
    if (r.driveCode === 8 || r.driveCode === 9 || r.held || r.reason.startsWith("해제 보류") || r.startGaveUp) return "bad";
    if (r.localized === false) return "bad";
    if (r.arrival === "ARRIVAL_CONFIRMED" || r.arrival === "ARRIVAL_PENDING" || r.driveCode === 1 || r.driveCode === 7 || r.driveCode === 4) return "ok";
    return "warn";                                                          // 0 대기 · 2 통행 대기 · 3 횡단보도 · 5 장애물 · 6 차선 놓침 · waiting_for
  }

  function buildSummary(m) {
    const robots = m.robots || [];
    const ageText = m.ageText || "받은 적 없음";
    // 검토 P3: 연결 상태부터 가른다 — 끊김은 빨강, 코디네이터 없음은 그 말로. "아직 받지 못했다" 는 한 번도 못 받았을 때만.
    if (m.link === "down" && !m.mission) {
      return {sentence: `게이트웨이 끊김 — 마지막 수신 ${ageText}`, safety: ["neutral", "옛 값"], progress: ["neutral", "옛 값"], data: ["bad", `끊김 · 마지막 ${ageText}`]};
    }
    if (m.link === "no_coordinator") {
      return {sentence: "플릿 코디네이터 없음 — 플릿 상태 미수신(게이트웨이는 응답한다)", safety: ["neutral", "미수신"], progress: ["neutral", "미수신"], data: ["bad", "코디네이터 없음"]};
    }
    if (m.link === "waiting" || !m.mission) {
      return {sentence: "플릿 상태 미수신 — 게이트웨이에서 아직 받지 못했다", safety: ["neutral", "미수신"], progress: ["neutral", "미수신"], data: ["neutral", "미수신"]};
    }
    const missionKo = ko("mission", m.mission);
    const sentence = `${m.label || "좌표 프로파일 미수신"} · ${missionKo}${m.link === "down" ? "(옛 값)" : ""} · ` + robots.map(robotPhrase).join(" · ");
    if (m.link === "down") {
      return {sentence, safety: ["neutral", `옛 값 · 마지막 ${ageText}`], progress: ["neutral", "옛 값"], data: ["bad", `끊김 · 마지막 ${ageText}`]};
    }
    // 안전: 확실한 위험(래치·정지·해제 보류·비상정지)이 빨강, 신선한 보고가 없거나 위치를 모르면 회색·이유, 전부 신선하고 위험 없음일 때만 초록
    const unsafe = robots.filter(r => r.fresh && (r.driveCode === 8 || r.driveCode === 9 || r.held || r.reason.startsWith("해제 보류")));
    const unknown = robots.filter(r => !r.fresh || r.localized === false);
    const safety = (m.estopLatched || m.mission === "ESTOP") ? ["bad", "비상정지 래치"]
      : unsafe.length ? ["bad", unsafe.map(r => r.name + " " + (r.driveCode === 8 ? "비상정지" : r.driveCode === 9 ? "연결 끊김" : r.held ? "정지" : "해제 보류")).join(", ")]
      : unknown.length ? ["neutral", unknown.map(r => r.name + (r.fresh ? " 위치 모름" : " 응답 없음")).join(", ")]
      : robots.length ? ["ok", "래치·정지 없음"] : ["neutral", "로봇 미수신"];
    // 진행: 경고 빨강 → 로봇별 판정을 모은다(하나라도 bad 면 빨강, 응답 없음이면 회색, 대기가 있으면 노랑, 전부 주행·도착일 때만 초록)
    let progress;
    const tones = robots.map(r => [r, robotProgressTone(r)]);
    const pick = t => tones.filter(x => x[1] === t).map(x => x[0]);
    if (m.warning) progress = ["bad", String(m.warning).split(" | ")[0]];
    else if (m.mission === "RUNNING") {
      const bad = pick("bad"), none = pick("neutral"), warn = pick("warn");
      progress = bad.length ? ["bad", bad.map(r => `${r.name} ${r.held ? "정지" : r.driveCode === 8 ? "비상정지" : r.driveCode === 9 ? "연결 끊김" : r.startGaveUp ? "START 실패" : r.localized === false ? "위치 모름" : "해제 보류"}`).join(", ")]
        : none.length ? ["neutral", none.map(r => `${r.name} 응답 없음`).join(", ")]
        : warn.length ? ["warn", warn.map(r => `${r.name} ${r.waitingFor ? "대기" : ko("drive", r.driveCode)}`).join(", ")]
        : robots.length ? ["ok", "전부 주행·도착"] : ["neutral", "로봇 미수신"];
    }
    else if (m.mission === "DONE") progress = ["ok", "완료"];
    else if (m.mission === "ESTOP" || m.mission === "STOPPED") progress = ["warn", missionKo];
    else progress = ["neutral", missionKo];
    const data = m.demo ? ["pending", "예시 값(DEMO)"]
      : (typeof m.ageSec === "number" && m.ageSec <= 3) ? ["ok", `${ageText} 수신`] : ["warn", `${ageText} 수신`];
    return {sentence, safety, progress, data};
  }

  function profileLabel() {
    const p = state.profiles;
    const cur = (p?.available || []).find(a => a.name === p?.active);
    return cur?.label || p?.label || state.fleet?.profile?.label || null;
  }

  function setLight(id, pair) {
    const el = $(id);
    if (!el || !pair) return;
    el.className = "light " + pair[0];
    const em = el.querySelector("em");
    if (em) em.textContent = pair[1];
  }

  function renderSummaryBand() {
    const el = $("summary-sentence");
    if (!el) return;
    const link = fleetLink();
    const s = buildSummary({
      link, demo: DEMO,
      ageSec: state.lastUpdate ? (Date.now() - state.lastUpdate) / 1000 : null,
      ageText: fleetAgeText(),
      mission: state.fleet?.mission_state || null,
      label: profileLabel(),
      warning: link === "ok" ? (state.fleet?.warning || null) : null,   // 끊겼으면 옛 경고다(renderLink 와 같은 규칙)
      estopLatched: Boolean(state.fleet?.estop_latched),
      robots: [1, 2].map(deriveRobot)
    });
    // 검토 P2: 바뀐 것만 쓴다 — 같은 글을 초마다 다시 쓰면 aria-live 가 계속 읽고, 줄 수가 바뀌면 아래 정지 버튼이 움직인다(OPS-4 류)
    if (el.textContent !== s.sentence) { el.textContent = s.sentence; el.title = s.sentence; }
    setLight("light-safety", s.safety);
    setLight("light-progress", s.progress);
    setLight("light-data", s.data);
  }

  // U-6: 부제 = 좌표 프로파일 label · 경과 = 화면이 RUNNING 을 처음 본 뒤(코디네이터 시작 시각은 API 에 없다 — 그래서 "화면 기준")
  function renderSubtitle() {
    const el = $("subtitle");
    if (!el) return;
    const label = profileLabel();
    const link = fleetLink();
    const text = label ? "좌표 프로파일 · " + label : "좌표 프로파일 미수신 · MINI PROJECT 2";
    const out = label && link !== "ok" && !DEMO ? text + " (옛 값)" : text;   // 검토 P3: 끊기거나 코디네이터가 없으면 옛 label 이다
    if (el.textContent !== out) el.textContent = out;
  }

  function trackRunning(mission) {
    if (mission === "RUNNING") { if (!state.runningSince) state.runningSince = Date.now(); state.runningFrozenMs = null; }
    else if (mission === "DONE" || mission === "STOPPED" || mission === "ESTOP") {
      // 검토 P2: RUNNING 을 벗어나면 시계를 세운다 — "완료" 옆에서 경과가 계속 늘지 않게
      if (state.runningSince && state.runningFrozenMs === null) state.runningFrozenMs = Date.now() - state.runningSince;
    }
    else if (mission === "ASSIGNED" || mission === "IDLE") { state.runningSince = null; state.runningFrozenMs = null; }   // 새 배정이면 처음부터
    // mission 이 없는 응답(코디네이터 없음 등)은 시계를 건드리지 않는다
  }

  function elapsedText() {
    if (!state.runningSince) return "경과 —";
    const ms = state.runningFrozenMs !== null ? state.runningFrozenMs : Date.now() - state.runningSince;
    const s = Math.floor(ms / 1000);
    const mmss = `${String(Math.floor(s / 60)).padStart(2, "0")}:${String(s % 60).padStart(2, "0")}`;
    const stale = fleetLink() === "down" && state.runningFrozenMs === null ? " · 옛 값" : "";
    return (state.runningFrozenMs !== null ? "소요 " : "경과 ") + mmss + " (화면 기준)" + stale;
  }

  // U-5: 스트림이 없으면 검은 상자가 아니라 글로 말한다 — <img> 를 문구로 바꾼다(요소가 남지 않는다).
  // 검토 P1: 실물 게이트웨이의 MJPEG 경로는 프레임이 없어도 200 으로 열어 두므로 error 가 영영 안 온다 → 첫 프레임 기한(5 s)도 둔다.
  const MEDIA_FIRST_FRAME_MS = 5000;
  function initMedia() {
    document.querySelectorAll("img[data-media]").forEach(img => {
      const fail = () => {
        if (!img.isConnected) return;
        const box = document.createElement("div");
        box.className = "media-missing";
        box.textContent = `영상 없음 — ${img.dataset.media || ""} · ${img.dataset.missingWhy || "스트림 없음"}`;
        const host = img.parentElement;
        img.replaceWith(box);
        if (host) host.classList.add("media-missing-host");
      };
      img.addEventListener("error", fail, {once: true});
      // 스크립트는 문서 끝에서 돈다 — 그 전에 이미 실패한 <img>(complete 인데 크기 0)는 error 가 다시 오지 않는다
      if (img.complete && img.naturalWidth === 0) fail();
      else setTimeout(() => { if (img.isConnected && img.naturalWidth === 0) fail(); }, MEDIA_FIRST_FRAME_MS);
    });
  }

  // 로봇별 정지·재개 (D7 · REVIEW_20260926 G): 게이트웨이를 재기동하면 로봇이 링크유실로 래치되고 코디네이터가 그 로봇을
  // "링크유실 래치 — 로봇 재개 필요" 로 세운다. 화면에 로봇 재개가 없으면 월요일 ② 로봇 지도 전환이 REFUSED 로 막힌다.
  async function robotAction(index, cmd) {
    if (DEMO) return;
    const r = deriveRobot(index);
    const what = cmd === "resume" ? "재개" : "정지";
    // 통합 검토 OPS-4: 멈추는 명령은 묻지 않는다(플릿 비상정지 버튼도 안 묻는다). 움직일 수 있는 재개만 확인을 받는다.
    if (cmd === "resume" && !window.confirm(`${r.name} 을(를) 재개합니다.\n로봇 재개는 이 로봇의 정지·ESTOP·링크유실 래치를 풉니다(Nav2 취소 확인 뒤). 플릿이 달리는 중이면 이 로봇도 다시 달립니다.\n계속할까요?`)) return;
    try {
      const body = await postJson(`/api/${robotKey(index)}/${cmd}`, {});
      // 202 = 보냈지만 적용 여부를 모른다 — 성공이라 하지 않는다
      alert((body.success ? "✅ " : "⚠️ ") + `${r.name} ${what}: ` + (body.message || "") + (body.applied === null || body.success === false ? " (적용 여부 모름)" : ""));
    } catch (e) {
      alert(`⛔ ${r.name} ${what} 실패: ` + e.message);
    }
    refresh();
  }

  const defaultSteps = [
    ["START 확인","MARKER"],["Nav2 출발","NAV2"],["시나리오 구간 접근","NAV2"],
    ["Marker 이벤트 확인","MARKER"],["공유구간 통행 허가","FLEET"],
    ["Nav2 최종 접근","NAV2"],["GOAL 확인","MARKER"]
  ];

  function renderScenario() {
    const host = $("scenario-robots");
    if (!host) return;
    host.innerHTML = [1,2].map(i => {
      const mr = missionRobot(i);
      const current = mr?.current_step ?? -1;
      const steps = Array.isArray(mr?.steps) && mr.steps.length ? mr.steps : defaultSteps.map((x,idx)=>({name:x[0],mode:x[1],status: current<0 ? "pending" : idx<current?"done":idx===current?"current":"pending"}));
      return `
        <article class="card scenario-card">
          <div class="robot-head">
            <div class="robot-title"><span class="robot-index r${i}">${i}</span><div><strong>Pinky ${i}</strong><p>${mr?.scenario || "표준 하이브리드 시나리오"}</p></div></div>
            <span class="status-pill ${mr ? "ok":"pending"}">${mr?.state || "BACKEND PENDING"}</span>
          </div>
          <div class="scenario-steps">
            ${steps.map((s,idx)=>{
              const st=s.status || "pending";
              const symbol=st==="done"?"✓":st==="current"?"●":st==="wait"?"!":st==="na"?"–":String(idx+1);   // na = 해당 없음(미구현 단계)
              return `<div class="scenario-step ${st}"><span class="step-dot">${symbol}</span><div class="step-copy"><strong>${esc(s.name)}</strong><span>${esc(s.detail || "")}</span></div><span class="step-mode">${esc(s.mode || "")}</span></div>`;
            }).join("")}
          </div>
        </article>`;
    }).join("");
  }

  function renderNavigation() {
    const host = $("navigation-robots");
    if (!host) return;
    host.innerHTML = [1,2].map(i => {
      const r = deriveRobot(i);
      const n = navRobot(i) || {};
      return `
        <article class="card robot-card">
          <div class="robot-head">
            <div class="robot-title"><span class="robot-index r${i}">${i}</span><div><strong>Pinky ${i} Navigation</strong><p>Nav2 기본 이동 + Marker/Fleet 보조</p></div></div>
            <span class="status-pill ${toneForRobot(r)}">${esc(r.navStatus)}</span>
          </div>
          <div class="robot-kpis">
            <div class="kpi"><span>Goal</span><strong>${n.goal ? `${fmt(n.goal.x)}, ${fmt(n.goal.y)}` : "—"}</strong></div>
            <div class="kpi"><span>Global Path</span><strong>${n.path_points ?? "—"} pts</strong></div>
            <div class="kpi"><span>Obstacle</span><strong>${n.obstacle_state || "—"}</strong></div>
            <div class="kpi"><span>Replan</span><strong>${n.replan_count ?? "—"}</strong></div>
          </div>
          <div class="robot-detail-grid">
            <div class="data-row"><span>RobotState</span><strong>x ${fmt(r.x)} / y ${fmt(r.y)}</strong></div>
            <div class="data-row"><span>External Vision</span><strong>${n.external_vision?.state || "—"}</strong></div>
            <div class="data-row"><span>Nav2 Action</span><strong>${n.action_state || "—"}</strong></div>
            <div class="data-row"><span>Scenario Mode</span><strong>${n.scenario_mode || "—"}</strong></div>
          </div>
        </article>`;
    }).join("");
  }

  function metric(title, value, note="") {
    return `<article class="card metric-card"><span>${esc(title)}</span><strong>${esc(value ?? "—")}</strong><small>${esc(note)}</small></article>`;
  }

  function renderVision() {
    const v = state.ops.vision || {};
    $("vision-metrics").innerHTML = [
      metric("Tracking", v.tracking || "백엔드 대기", "Pinky 1 / 2"),
      metric("Reference Markers", v.reference_seen ?? "—", "ArUco 40~43"),
      metric("Reprojection", typeof v.reproj_error==="number"?fmt(v.reproj_error,2)+" px":"—", "Homography quality"),
      metric("Pipeline Latency", typeof v.pipeline_latency==="number"?fmt(v.pipeline_latency*1000,0)+" ms":"—", "Tablet processing")
    ].join("");
    const events = v.zone_events || [];
    $("vision-event-body").innerHTML = events.length ? events.slice(0,20).map(e=>`<tr><td>${esc(e.time||e.timestamp||"—")}</td><td>${esc(e.robot_name)}</td><td>${esc(e.camera_id)}</td><td>${esc(e.zone_id)}</td><td>${esc(e.event_type)}</td><td>${esc(e.confidence)}</td><td>${esc(e.sequence)}</td></tr>`).join("") : emptyRow(7,"ZoneEvent 백엔드 연동 대기");
  }

  function renderCommunications() {
    const c = state.ops.communications || {};
    $("communication-summary").innerHTML = [
      metric("Tablet → Relay", c.tablet_to_relay?.state || "미측정", c.tablet_to_relay?.latency || ""),
      metric("Domain Bridge", c.bridge?.state || "미측정", c.bridge?.detail || ""),
      metric("Pinky 1 Link", c.pinky1?.state || "미측정", c.pinky1?.latency || ""),
      metric("Pinky 2 Link", c.pinky2?.state || "미측정", c.pinky2?.latency || "")
    ].join("");
    const links = c.links || [];
    $("link-health-body").innerHTML = links.length ? links.map(x=>`<tr><td>${esc(x.segment)}</td><td>${esc(x.message)}</td><td>${esc(x.rate ?? "미측정")}</td><td>${esc(x.last_seen ?? "미측정")}</td><td>${esc(x.latency ?? "미측정")}</td><td class="${statusClass(x.state)}">${esc(x.state ?? "미측정")}</td></tr>`).join("") : emptyRow(6,"통신 계측 API 구현 대기");
    const trace = c.traces || [];
    $("trace-body").innerHTML = trace.length ? trace.slice(0,50).map(x=>`<tr><td>${esc(x.time)}</td><td>${esc(x.correlation_id)}</td><td>${esc(x.source)}</td><td>${esc(x.stage)}</td><td class="${statusClass(x.result)}">${esc(x.result)}</td><td>${esc(x.detail||"")}</td></tr>`).join("") : emptyRow(6,"Message Trace 백엔드 구현 대기");
  }

  function renderDiagnostics() {
    const host = $("diagnostic-robots");
    host.innerHTML = [1,2].map(i => {
      const d = diagRobot(i) || {};
      const sections = [
        ["위치 / 상태",[["PoseFuser",d.pose_fuser],["TF map→odom",d.tf],["RobotState",d.robot_state]]],
        ["Navigation",[["Nav2",d.nav2],["Planner",d.planner],["Controller",d.controller],["Obstacle",d.obstacle]]],
        ["Safety",[["DriveCommandGate",d.drive_gate],["Motor Watchdog",d.motor_watchdog],["E-STOP",d.estop]]],
        ["Sensor",[["Camera",d.camera],["LiDAR",d.lidar],["Battery",d.battery]]]
      ];
      return `<article class="card diag-card"><div class="robot-head"><div class="robot-title"><span class="robot-index r${i}">${i}</span><strong>Pinky ${i}</strong></div><span class="status-pill ${d.overall?pillTone(d.overall):"pending"}">${esc(d.overall||"BACKEND PENDING")}</span></div>${sections.map(s=>`<div class="diag-section"><h4>${s[0]}</h4>${s[1].map(row=>`<div class="diag-row"><span>${row[0]}</span><strong>${esc(row[1] ?? "미측정")}</strong></div>`).join("")}</div>`).join("")}</article>`;
    }).join("");
  }

  function normalizeLogs() {
    const ops = state.ops.events?.events;
    if (Array.isArray(ops)) return ops;
    return (state.logs || []).map((line,idx)=>({time:"—",type:"SYSTEM",robot:"—",event:"Gateway Log",detail:typeof line==="string"?line:(line.message||JSON.stringify(line)),_id:idx}));
  }

  function renderEvents() {
    const filter = $("event-filter")?.value || "ALL";
    let rows = normalizeLogs();
    if (filter !== "ALL") rows = rows.filter(x => String(x.type||"").toUpperCase() === filter);
    $("events-body").innerHTML = rows.length ? rows.slice(0,100).map(x=>`<tr><td>${esc(x.time||x.timestamp||"—")}</td><td>${esc(x.type||"SYSTEM")}</td><td>${esc(x.robot||"—")}</td><td>${esc(x.event||x.name||"—")}</td><td>${esc(x.detail||x.message||"")}</td></tr>`).join("") : emptyRow(5,"표시할 이벤트가 없습니다.");
  }

  function renderConfig() {
    const c = state.ops.config || {};
    const cards = [
      ["운영 모드",[["기본 이동",c.motion_mode||"NAV2"],["시나리오 보조",c.scenario_mode||"MARKER / FLEET"],["현장 상태",c.field_status||"BACKEND PENDING"]]],
      ["좌표 / 지도",[["Map Frame",c.map_frame||"map"],["Arena",c.arena||"2.34m × 1.26m (계획값)"],["Route Source",c.route_source||"Relay / Nav2"]]],
      ["안전",[["E-STOP",c.estop_policy||"Robot safety layer"],["Command Owner",c.command_owner||"통합계획 확정 필요"],["Link Loss",c.link_loss_policy||"Fail-safe stop"]]]
    ];
    $("config-cards").innerHTML = cards.map(x=>`<article class="card config-card"><h3>${x[0]}</h3>${x[1].map(r=>`<div class="config-row"><span>${r[0]}</span><strong>${esc(r[1])}</strong></div>`).join("")}</article>`).join("");
  }

  // statusClass(good/warn/bad/muted) → status-pill 클래스(ok/warn/bad/neutral)
  function pillTone(v) {
    return {good:"ok", warn:"warn", bad:"bad"}[statusClass(v)] || "neutral";
  }

  function emptyRow(cols, text) { return `<tr><td colspan="${cols}" class="muted">${esc(text)}</td></tr>`; }
  function statusClass(v) {
    const s=String(v||"").toUpperCase();
    if (s.includes("OK")||s.includes("LIVE")||s.includes("HEALTH")||s.includes("ACCEPT")) return "good";
    if (s.includes("WARN")||s.includes("WAIT")||s.includes("PENDING")) return "warn";
    if (s.includes("FAIL")||s.includes("LOST")||s.includes("ERROR")||s.includes("DROP")) return "bad";
    return "muted";
  }

  function renderSummary() {
    const mission = state.fleet?.mission_state || state.ops.overview?.mission_state || "—";
    const down = fleetLink() === "down";
    // 통합 검토 OPS-2: 못 받는 동안 미션 상태는 옛 값이다 — 초록·빨강으로 현재처럼 칠하지 않는다
    setPill($("mission-status"), "미션 " + ko("mission", mission) + (down ? " · 옛 값" : ""),
            down ? "neutral" : mission==="RUNNING"?"ok":mission==="ESTOP"?"bad":"neutral");
    $("summary-fleet").textContent = ko("mission", mission);
    const vis = state.ops.vision?.tracking || "미측정";
    $("summary-vision").textContent = vis;
    setPill($("camera-overall"), state.ops.vision ? vis : "상태 API 대기", state.ops.vision ? pillTone(vis):"pending");
    renderLink();
    setPill($("scenario-contract"), state.ops.mission ? "CONNECTED":"BACKEND PENDING", state.ops.mission?"ok":"pending");
    setPill($("nav-contract"), state.ops.navigation ? "CONNECTED":"PARTIAL / PENDING", state.ops.navigation?"ok":"pending");
    setPill($("comms-contract"), state.ops.communications ? "CONNECTED":"BACKEND PENDING", state.ops.communications?"ok":"pending");
  }

  // 통합 검토 OPS-2·OPS-7: 연결 알약 · 마지막 수신 나이 · 알림 띠(끊김·코디네이터 없음·control_note). 1 s 마다도 부른다 —
  // 요청이 응답 없이 매달려 갱신이 안 끝나도 화면이 옛 값이라고 말하게.
  function renderLink() {
    const link = fleetLink();
    const age = fleetAgeText();
    if (link === "down") setPill($("backend-status"), "RELAY 끊김 · 마지막 수신 " + age, "bad");
    else if (link === "no_coordinator") setPill($("backend-status"), "COORDINATOR 없음", "bad");
    else if (link === "ok") setPill($("backend-status"), "RELAY CONNECTED", "ok");
    else setPill($("backend-status"), "BACKEND 확인 중", "neutral");
    const ageEl = $("fleet-age");
    if (ageEl) {
      ageEl.textContent = "갱신 " + age;
      ageEl.classList.toggle("stale", link === "down");
    }
    const banner = $("alert-banner");
    if (banner) {
      const lines = [];
      if (link === "down") {
        lines.push(["bad", `⛔ 게이트웨이 응답 없음 — 아래 값은 ${age} 것이다(갱신되지 않는다)` + (state.fleetError ? ` · ${state.fleetError}` : "")]);
      } else if (link === "no_coordinator") {
        lines.push(["bad", "⛔ 플릿 코디네이터 없음 — 플릿 명령·로봇 정지·재개가 받는 곳 없이 503 이다" +
                           (state.fleet?.detail ? ` · ${state.fleet.detail}` : "")]);
      }
      if (state.fleet?.control_note) lines.push(["warn", "⚠️ 제어 상태 알림 — " + state.fleet.control_note]);
      // 코디네이터 경고(STALL_NO_REQUEST · PARKED_BLOCK · ROBOT_IDLE_IN_RUNNING · ARRIVED_NOT_AT_GOAL · DEADLOCK) — 예전엔
      // /api/fleet/status 의 'warning' 에만 있어 화면에서는 로봇이 멀쩡해 보였다. 끊겼으면 옛 경고라 싣지 않는다.
      if (link === "ok" && state.fleet?.warning) {
        String(state.fleet.warning).split(" | ").filter(Boolean).forEach(w => lines.push(["warn", "⚠️ 플릿 경고 — " + w]));
      }
      banner.innerHTML = lines.map(([tone, text]) => `<div class="alert-line ${tone}">${esc(text)}</div>`).join("");
      banner.hidden = !lines.length;
    }
    return link;
  }

  function drawMaps() {
    drawMap($("fleet-map"), false);
    drawMap($("nav-map"), true);
  }

  function collectMapData() {
    const n = state.ops.navigation || {};
    // U-4: 지도 범위는 좌표 프로파일의 frame 을 따른다. 예전엔 옛 경기장 노드(START_A·START_B·GOAL_C)를 기본값으로 그려
    // 프로파일이 map4 여도 옛 노드가 보였다 — 프로파일이 안 말한 노드는 그리지 않는다.
    const frame = state.profiles?.frame || state.fleet?.profile?.frame || (DEMO ? "team11_map5" : null);
    // 지도 사각형은 프로파일이 싣는 지도 규격(state.profiles.map.bounds — map5: 원점 −0.01, 2.36 × 1.28)을 따른다. 없으면 map5 값.
    const pb = state.profiles?.map?.bounds || null;
    const arena = n.arena || (pb ? {x_min:pb.x_min,x_max:pb.x_max,y_min:pb.y_min,y_max:pb.y_max}
                            : frame === "team11_map5" ? {x_min:-0.01,x_max:2.35,y_min:-0.01,y_max:1.27}
                            : {x_min:-0.01,x_max:2.35,y_min:-0.01,y_max:1.27});
    return {
      arena,
      zones: n.zones || [],
      robots: [1,2].map(i=>({i, fleet:deriveRobot(i), nav:navRobot(i)||{}}))
    };
  }

  function drawMap(canvas, detailed) {
    if (!canvas || canvas.offsetParent === null) return;
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(300, Math.round(rect.width*dpr));
    canvas.height = Math.max(240, Math.round((detailed?560:390)*dpr));
    const ctx=canvas.getContext("2d");
    ctx.scale(dpr,dpr);
    const W=rect.width, H=detailed?560:390;
    ctx.clearRect(0,0,W,H);
    ctx.fillStyle="#0f141c"; ctx.fillRect(0,0,W,H);
    const m=28, data=collectMapData(), a=data.arena;
    const sx=x=>m+(x-a.x_min)/(a.x_max-a.x_min)*(W-2*m);
    const sy=y=>H-m-(y-a.y_min)/(a.y_max-a.y_min)*(H-2*m);

    // 1. 단일 정본 트랙 배경 맵 렌더링 (map5 고해상도)
    if (MAP_BG_IMG.complete && MAP_BG_IMG.naturalWidth > 0) {
      ctx.save();
      ctx.imageSmoothingEnabled = true;
      ctx.drawImage(MAP_BG_IMG, m, m, W - 2 * m, H - 2 * m);
      ctx.restore();
    } else {
      ctx.fillStyle="#f6f8fc"; ctx.fillRect(m,m,W-2*m,H-2*m);
      ctx.strokeStyle="#e4e9f1"; ctx.lineWidth=1;
      for(let i=0;i<=10;i++){const x=m+(W-2*m)*i/10;ctx.beginPath();ctx.moveTo(x,m);ctx.lineTo(x,H-m);ctx.stroke();}
      for(let i=0;i<=6;i++){const y=m+(H-2*m)*i/6;ctx.beginPath();ctx.moveTo(m,y);ctx.lineTo(W-m,y);ctx.stroke();}
    }
    ctx.strokeStyle="#4b5563";ctx.lineWidth=1.5;ctx.strokeRect(m,m,W-2*m,H-2*m);

    // 2. 미션 핵심 4개 노드 핀 마커 (BL, BR, TR, J)
    const CORE_NODES = [
      { id: "BL", x: 0.20, y: 0.20, color: "#ff9900" },
      { id: "BR", x: 2.15, y: 0.20, color: "#a371f7" },
      { id: "TR", x: 2.15, y: 1.08, color: "#2ea043" },
      { id: "J",  x: 1.18, y: 0.64, color: "#eab308" }
    ];
    CORE_NODES.forEach(n => {
      const nx = sx(n.x), ny = sy(n.y);
      ctx.beginPath(); ctx.arc(nx, ny, n.id === "J" ? 8 : 7, 0, Math.PI * 2);
      ctx.fillStyle = n.color; ctx.fill();
      ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 1.5; ctx.stroke();
      ctx.fillStyle = "#ffffff"; ctx.font = "bold 9px sans-serif";
      ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillText(n.id, nx, ny);
    });

    if(state.layers.zones) data.zones.forEach(z=>{
      ctx.beginPath();ctx.arc(sx(z.x),sy(z.y),9,0,Math.PI*2);ctx.fillStyle=z.id.includes("GOAL")?"#18a36f":"#9aa7b9";ctx.fill();
      ctx.fillStyle="#607089";ctx.font="11px sans-serif";ctx.fillText(z.id,sx(z.x)+12,sy(z.y)+4);
    });

    data.robots.forEach(({i,fleet,nav})=>{
      const path=nav.global_path || nav.route || [];
      if(state.layers.route && Array.isArray(path) && path.length>1){
        ctx.beginPath(); path.forEach((p,idx)=>{const x=sx(p.x??p[0]),y=sy(p.y??p[1]);idx?ctx.lineTo(x,y):ctx.moveTo(x,y);});
        ctx.strokeStyle=i===1?"rgba(230,59,80,.45)":"rgba(37,99,235,.45)";ctx.lineWidth=3;ctx.setLineDash([8,5]);ctx.stroke();ctx.setLineDash([]);
      }
      if(state.layers.robot && typeof fleet.x==="number" && typeof fleet.y==="number"){
        const x=sx(fleet.x),y=sy(fleet.y);
        // 위치 융합 신선(<0.6s) 리플 펄스 링
        const fixAge = fleet.poseFuser?.fix_age_s;
        if (typeof fixAge === "number" && fixAge < 0.6) {
          ctx.beginPath(); ctx.arc(x, y, 16, 0, Math.PI * 2);
          ctx.strokeStyle = i === 1 ? "rgba(230,59,80,.35)" : "rgba(37,99,235,.35)";
          ctx.lineWidth = 2.5; ctx.stroke();
        }
        ctx.beginPath();ctx.arc(x,y,9,0,Math.PI*2);ctx.fillStyle=i===1?"#e63b50":"#2563eb";ctx.fill();
        ctx.fillStyle="#14213d";ctx.font="700 11px sans-serif";ctx.fillText("P"+i,x+12,y+4);
        if(typeof fleet.yaw==="number"){ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+20*Math.cos(-fleet.yaw),y+20*Math.sin(-fleet.yaw));ctx.strokeStyle=i===1?"#e63b50":"#2563eb";ctx.lineWidth=2;ctx.stroke();}
      }
      const v = fleet.extVision || nav.external_vision;
      if(state.layers.vision && v && typeof v.x==="number" && typeof v.y==="number"){
        const vx=sx(v.x), vy=sy(v.y);
        // Golden Crosshair ⌖
        ctx.strokeStyle="#d97706"; ctx.lineWidth=2;
        ctx.beginPath(); ctx.arc(vx, vy, 11, 0, Math.PI*2); ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(vx - 15, vy); ctx.lineTo(vx + 15, vy);
        ctx.moveTo(vx, vy - 15); ctx.lineTo(vx, vy + 15);
        ctx.stroke();
        ctx.fillStyle="#b45309"; ctx.font="bold 10px monospace";
        ctx.fillText(`⌖ P${i} 관제Fix`, vx + 14, vy - 4);

        // 관제 위치와 로봇 위치 간 초록 점선 오차 벡터 & 거리 표시
        if(state.layers.robot && typeof fleet.x==="number" && typeof fleet.y==="number"){
          const rx=sx(fleet.x), ry=sy(fleet.y);
          ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(vx, vy);
          ctx.strokeStyle="#16a34a"; ctx.lineWidth=1.6; ctx.setLineDash([4,4]); ctx.stroke(); ctx.setLineDash([]);
          const distCm = (Math.hypot(fleet.x - v.x, fleet.y - v.y) * 100).toFixed(1);
          const mx = (rx + vx) / 2, my = (ry + vy) / 2;
          ctx.fillStyle="#15803d"; ctx.font="bold 9px sans-serif";
          ctx.fillText(`Δ ${distCm}cm`, mx + 4, my - 4);
        }
      }
    });
    const hasPath=data.robots.some(r=>Array.isArray(r.nav.global_path||r.nav.route) && (r.nav.global_path||r.nav.route).length);
    if($("map-empty")) $("map-empty").style.display = hasPath ? "none":"block";
  }

  // U-4: 데모는 좌표 정본(map5 임시 도로망 · pinky1 BL→TR, pinky2 BR→BL)으로. 예전 데모는 옛 경기장(START_A/GOAL_C · 2.34×1.26)이었고
  // 비전 값(ARUCO 2/2)은 없는 기능을 있는 것처럼 보였다 — 비전은 "미수신"(마커 ID 는 태블릿 설정 정본 30/31 · 40~43 그대로).
  const DEMO_NODES = [{id:"BL",x:.2,y:.2},{id:"BR",x:2.15,y:.2},{id:"TR",x:2.15,y:1.08},{id:"J",x:1.18,y:.64}];
  const DEMO_LABEL = "팀11 map5 — 임시 도로망 4노드(BL·BR·TR·J), 좌하단 원점 2.36 × 1.28 · pinky1 BL→TR, pinky2 BR→BL";
  function demoData() {
    const P = id => DEMO_NODES.find(n => n.id === id);
    const path = ids => ids.map(P).map(n => ({x:n.x, y:n.y}));
    const steps1 = [["배정","FLEET","done"],["출발(START 확인)","FLEET","done"],["구간 진행 BL→J","LANE","done"],["교차로 허가","FLEET","current"],["최종 접근 J→TR","LANE","pending"],["도착 확정","FLEET","pending"],["구역 이벤트(마커)","MARKER","na"]];
    const steps2 = [["배정","FLEET","done"],["출발(START 확인)","FLEET","done"],["구간 진행 BR→J","LANE","wait"],["교차로 허가","FLEET","pending"],["최종 접근 J→BL","LANE","pending"],["도착 확정","FLEET","pending"],["구역 이벤트(마커)","MARKER","na"]];
    return {
      profiles:{active:"team11_map5",frame:"team11_map5",robot_map_name:"map5",label:DEMO_LABEL,mission_state:"RUNNING",map:{bounds:{x_min:-0.01,x_max:2.35,y_min:-0.01,y_max:1.27}},available:[{name:"team11_map5",label:DEMO_LABEL,valid:true,warnings:[]}],robots:{pinky1:{state:"MATCH",robot_map_name:"map5"},pinky2:{state:"MATCH",robot_map_name:"map5"}}},
      fleet:{mission_state:"RUNNING",estop_latched:false,control_note:null,warning:null,profile:{active:"team11_map5",frame:"team11_map5",label:DEMO_LABEL},robots:{
        pinky1:{start_node:"BL",goal_node:"TR",start:{acknowledged:true},clear_until_idx:12,waiting_for:"",blocked_by:"",is_stale:false,is_unlocalized:false,arrival_status:"NOT_ARRIVED",state:{x:.75,y:.45,yaw:.42,localized:true},lane_status:{drive_state:1,state_reason:"주행 중 — J 까지 허가"},comparator:{route_progress:.28,cross_track_error:.028}},
        pinky2:{start_node:"BR",goal_node:"BL",start:{acknowledged:true},clear_until_idx:8,waiting_for:"BL_J",blocked_by:"pinky1",is_stale:false,is_unlocalized:false,arrival_status:"NOT_ARRIVED",state:{x:1.9,y:.31,yaw:2.71,localized:true},lane_status:{drive_state:2,state_reason:"허가 대기 — BL_J 를 pinky1 이 쥠"},comparator:{route_progress:.12,cross_track_error:.012}}
      }},
      ops:{
        mission:{robots:{
          pinky1:{state:"RUNNING",current_step:3,scenario:"임시 미션 · BL → TR",steps:steps1.map(x=>({name:x[0],mode:x[1],status:x[2]}))},
          pinky2:{state:"WAITING",current_step:2,scenario:"임시 미션 · BR → BL (J 앞 대기)",steps:steps2.map(x=>({name:x[0],mode:x[1],status:x[2]}))}
        }},
        navigation:{arena:{x_min:-0.01,x_max:2.35,y_min:-0.01,y_max:1.27},zones:DEMO_NODES,robots:{
          pinky1:{nav_status:"ACTIVE",action_state:"CRUISE",obstacle_state:"CLEAR",replan_count:0,path_points:22,scenario_mode:"FLEET_CLEARANCE",goal:{x:1.18,y:.64},route_progress:.28,cross_track_error:.028,global_path:path(["BL","J","TR"]),external_vision:{state:"미수신",marker_id:30,x:null,y:null}},
          pinky2:{nav_status:"HOLD",action_state:"WAIT_CLEARANCE",obstacle_state:"CLEAR",replan_count:0,path_points:22,scenario_mode:"WAIT_CLEARANCE",goal:null,route_progress:.12,cross_track_error:.012,global_path:path(["BR","J","BL"]),external_vision:{state:"미수신",marker_id:31,x:null,y:null}}
        }},
        vision:{tracking:"미수신 — 태블릿 T-11·READY 전",reference_seen:0,reference_ids:[40,41,42,43],robot_marker_ids:{pinky1:30,pinky2:31},reproj_error:null,pipeline_latency:null,zone_events:[]},
        // 검토 P2: 태블릿 PoseFix 사슬(Y700→Relay→D10)은 아직 없다 — 데모에도 LIVE 로 두지 않는다(미측정). D8→D11 레인 명령만 실재
        communications:{tablet_to_relay:{state:"미측정",latency:"태블릿 T-10 전"},bridge:{state:"미측정"},pinky1:{state:"미측정"},pinky2:{state:"미측정"},links:[
          {segment:"D8→D11",message:"LaneCommand",rate:"10.0 Hz",last_seen:"0.10 s",latency:"23 ms",state:"LIVE"}
        ],traces:[]},
        diagnostics:{robots:{
          pinky1:{overall:"HEALTHY",pose_source:"AMCL",pose_fuser:"미수신",tf:"LIVE",robot_state:"10 Hz",nav2:"ACTIVE",planner:"READY",controller:"ACTIVE",obstacle:"CLEAR",drive_gate:"READY",motor_watchdog:"ARMED",estop:"CLEAR",camera:"미수신",lidar:"LIVE",battery:"78%"},
          pinky2:{overall:"HEALTHY",pose_source:"AMCL",pose_fuser:"미수신",tf:"LIVE",robot_state:"10 Hz",nav2:"HOLD",planner:"READY",controller:"HOLD",obstacle:"CLEAR",drive_gate:"READY",motor_watchdog:"ARMED",estop:"CLEAR",camera:"미수신",lidar:"LIVE",battery:"74%"}
        }},
        events:{events:[
          {time:"20:18:41",type:"MISSION",robot:"Pinky 2",event:"통행 대기",detail:"BL_JS 를 pinky1 이 쥠"},
          {time:"20:18:39",type:"NAV2",robot:"Pinky 1",event:"목표 전송",detail:"웨이포인트 23 (LM 까지 허가)"},
          {time:"20:18:35",type:"MISSION",robot:"Pinky 1",event:"BL_JS 허가",detail:"공유 구간 예약 — pinky1 이 쥠, pinky2 대기"}
        ]},
        config:{motion_mode:"LANE",scenario_mode:"FLEET(예약) · 마커 미구현",field_status:"DEMO",map_frame:"team11_map5",arena:"2.36m × 1.28m (map5, 좌하단 원점)",route_source:"Relay 도로망 + lane_agent_node",estop_policy:"Robot safety layer",command_owner:"Fleet coordinator",link_loss_policy:"Fail-safe stop"}
      }
    };
  }

  async function refresh() {
    if (DEMO) {
      const d=demoData(); state.fleet=d.fleet; state.ops={...state.ops,...d.ops}; state.profiles=d.profiles; state.lastUpdate=Date.now(); renderAll(); return;
    }
    const requests = [
      ["fleet","/api/fleet/status"],["gateway","/api/status"],["logs","/api/logs"],["profiles","/api/fleet/profiles"],
      ["overview","/api/ops/overview"],["mission","/api/ops/mission"],["navigation","/api/ops/navigation"],
      ["vision","/api/ops/vision"],["communications","/api/ops/communications"],
      ["diagnostics","/api/ops/diagnostics"],["events","/api/ops/events"],["config","/api/ops/config"]
    ];
    const results = await Promise.allSettled(requests.map(x=>getJson(x[1])));
    results.forEach((r,idx)=>{
      const key=requests[idx][0];
      if(r.status!=="fulfilled") {
        // 통합 검토 OPS-2: 예전엔 실패를 건너뛰고 lastUpdate 만 새로 찍어, 죽은 게이트웨이의 마지막 값이 초록으로 얼었다
        if(key==="fleet") state.fleetError = String(r.reason?.message || r.reason || "응답 없음");
        return;
      }
      const val=r.value;
      if(key==="fleet") { state.fleet=val; state.fleetError=null; state.lastUpdate=Date.now(); }
      else if(key==="profiles") state.profiles=val;
      else if(key==="gateway") state.gateway=val;
      else if(key==="logs") state.logs=val.logs||[];
      else state.ops[key]=val;
    });
    renderAll();
  }

  // 관제 U-1~U-6 의 새 그리기 — 검토 P3: 예상 밖 응답으로 여기서 던져도 로봇 카드·정지 버튼(OPS-4)과 연결 판정(OPS-2)은 살아야 한다
  function renderControlExtras() {
    try {
      trackRunning(state.fleet?.mission_state);
      applyViewOnly();
      renderSubtitle();
      renderSummaryBand();

      // 인스턴스 런타임 배너 및 상단 상태 알약 (도커 vs 베어메탈 현장)
      const inst = state.gateway?.instance || {};
      const instBanner = $("inst-banner");
      if (instBanner) {
        if (inst.is_docker || (inst.label && !inst.label.includes("baremetal"))) {
          instBanner.style.display = "block";
          instBanner.textContent = `🐳 Docker Replica Container (${inst.label || "가상 HIL 환경"}) · ROS_DOMAIN_ID: ${inst.rosDomainId ?? 8} · Host: ${inst.host || "container"}`;
        } else {
          instBanner.style.display = "none";
        }
      }
      const instPill = $("runtime-instance-pill");
      if (instPill) {
        const domId = inst.rosDomainId ?? 8;
        if (inst.is_docker) {
          setPill(instPill, `🐳 Docker Replicas (D${domId})`, "warn");
          instPill.title = `도커 가상 컨테이너 인스턴스 (Host: ${inst.host || "docker"})`;
        } else {
          setPill(instPill, `🖥️ 현장 베어메탈 (D${domId})`, "ok");
          instPill.title = `현장 베어메탈 릴레이 PC (Host: ${inst.host || "baremetal"})`;
        }
      }

      // 관제 위치 전송 차단 (고장 주입 테스트) 버튼 상태
      const faultBlocked = Boolean(state.gateway?.fault_injection?.pose_fix_blocked);
      const btnFault = $("btn-fault-pose-block");
      if (btnFault) {
        if (faultBlocked) {
          btnFault.className = "btn bad";
          btnFault.textContent = "🚨 관제 위치 차단 중 (클릭 시 복구)";
          btnFault.title = "현재 관제 위치(PoseFix) 전송이 차단되어 로봇이 3초 후 데드맨 정지합니다. 클릭하면 전송을 복구합니다.";
        } else {
          btnFault.className = "btn warn";
          btnFault.textContent = "⚠️ 관제 위치 전송 차단 (주행 정지 증명)";
          btnFault.title = "관제 위치(overhead_pose) 전송을 인위적으로 차단하여 로봇이 3초 뒤 데드맨으로 정지하는지 검증합니다.";
        }
      }
    } catch (e) {
      console.error("summary/view-only render failed", e);
    }
  }

  function renderAll() {
    state.renderedLink = fleetLink();
    renderSummary();
    renderDashboardRobots();
    renderScenario();
    renderNavigation();
    renderVision();
    renderCommunications();
    renderDiagnostics();
    renderEvents();
    renderConfig();
    renderProfile();
    renderAssign();
    renderVision();
    renderControlExtras();
    drawMaps();
  }

  // ---- R-7 좌표 프로파일 (웹 전환) -------------------------------------------------------------
  // ⚠️ 서버가 말하지 않은 것은 초록으로 칠하지 않는다: 로봇 지도 대조는 로봇이 보고한 원점·크기로만 MATCH.
  function renderProfile() {
    const p = state.profiles;
    const pill = $("profile-pill"), body = $("profile-body"), sel = $("profile-select");
    if (!pill || !body || !sel) return;
    if (DEMO || !p) {
      setPill(pill, DEMO ? "DEMO" : "미수신", DEMO ? "pending" : "neutral");
      body.innerHTML = `<p class="muted">${DEMO ? "DEMO 에서는 전환하지 않습니다." : "/api/fleet/profiles 미수신"}</p>`;
      if ($("summary-profile")) $("summary-profile").textContent = "미수신";
      ["profile-switch","profile-robot-maps","profile-initial-poses"].forEach(id=>{ if($(id)) $(id).disabled = true; });
      return;
    }
    setPill(pill, p.active || "미수신", p.active ? "ok" : "neutral");
    if ($("summary-profile")) $("summary-profile").textContent = p.active ? `${p.active} (${p.frame || "?"})` : "미수신";
    const avail = p.available || [];
    const keyNow = avail.map(a=>a.name+":"+a.valid).join(",");
    if (sel.dataset.key !== keyNow) {              // 목록이 바뀔 때만 다시 만든다 — 고르는 중인 값을 지우지 않게
      const keep = sel.value;
      sel.innerHTML = avail.map(a=>`<option value="${esc(a.name)}" ${a.valid?"":"disabled"} title="${esc((a.problems||[]).concat(a.warnings||[]).join("; "))}">${esc(a.name)} — ${esc(a.label)}${a.valid?"":" (깨짐)"}${a.valid&&(a.warnings||[]).length?" (경고)":""}</option>`).join("");
      sel.value = keep && avail.some(a=>a.name===keep) ? keep : (p.active || "");
      sel.dataset.key = keyNow;
    }
    const blocker = p.switch_blocker;
    const robots = p.robots || {};
    const rows = Object.keys(robots).map(n=>{
      const r = robots[n] || {};
      const tone = r.state === "MATCH" ? "ok" : (r.state === "MISMATCH" ? "bad" : "neutral");
      return `<div class="config-row"><span>${esc(n)} 지도 대조</span><strong><span class="status-pill ${tone}">${esc(r.state||"미수신")}</span> ${esc(r.robot_map_name||"")} <span class="muted">${esc(r.detail||"")}</span></strong></div>`;
    }).join("");
    const cur = avail.find(a=>a.name===p.active) || {};
    body.innerHTML =
      `<div class="config-row"><span>현재</span><strong>${esc(cur.label||p.label||"미수신")}</strong></div>` +
      `<div class="config-row"><span>중계 좌표 frame</span><strong>${esc(p.frame||"미수신")}</strong></div>` +
      `<div class="config-row"><span>로봇 지도 이름</span><strong>${esc(p.robot_map_name||"—")}</strong></div>` +
      `<div class="config-row"><span>플릿 상태</span><strong>${esc(p.mission_state||"미수신")}</strong></div>` +
      `<div class="config-row"><span>지금 전환</span><strong><span class="status-pill ${blocker?"warn":"ok"}">${blocker?"불가":"가능"}</span> ${esc(blocker||"")}</strong></div>` +
      (p.note ? `<div class="config-row"><span>알림</span><strong class="muted">${esc(p.note)}</strong></div>` : "") +
      ((p.unassigned||[]).length ? `<div class="config-row"><span>경로 없음</span><strong><span class="status-pill warn">${esc(p.unassigned.join(", "))}</span> <span class="muted">배정 거절 — 출발하지 않는다</span></strong></div>` : "") +
      // REVIEW_20260926 G: 재기동 뒤 링크유실 래치 로봇은 화면이 정상이어도 지도 전환(SET_MAP)을 REFUSED 한다 — 이유를 보인다
      (Object.keys(p.held||{}).length ? `<div class="config-row"><span>로봇별 정지</span><strong>${Object.entries(p.held).map(([n,why])=>`<span class="status-pill warn">${esc(n)}</span> <span class="muted">${esc(why)}</span>`).join(" ")}</strong></div>` : "") +
      ((cur.warnings||[]).length ? `<div class="config-row"><span>미션 경고</span><strong class="muted">${esc(cur.warnings.join("; "))}</strong></div>` : "") + rows;
    ["profile-switch","profile-robot-maps","profile-initial-poses"].forEach(id=>{ if($(id)) $(id).disabled = !!blocker; });
  }

  // 비전 미션 (2026-09-30, 게이트웨이 --vision): 시나리오 버튼과 교차로 통행권. /api/fleet/status 의 mode·vision 만 본다.
  const VISION_STAGE = {"vision:approach": "교차로로 가는 중", "vision:junction": "교차로 동작 중",
                        "vision:after_junction": "교차로 통과 — 목적지로", "vision:arrived": "도착"};
  function renderVision() {
    const panel = $("vision-panel");
    if (!panel) return;
    const f = state.fleet || {};
    const v = f.mode === "vision" ? f.vision : null;
    panel.hidden = !v;
    if ($("summary-mode")) $("summary-mode").textContent = v ? "비전 차선 주행 + 교차로 통행권" : "Nav2 이동 + 도로망 예약";
    if (!v) return;
    const host = $("vision-scenarios");
    const key = (v.scenarios || []).map(s => s.name).join(",");
    if (host && host.dataset.key !== key) {                 // 버튼은 목록이 바뀔 때만 다시 만든다(OPS-4)
      host.innerHTML = (v.scenarios || []).map(s =>
        `<button class="btn primary" data-scenario="${esc(s.name)}" data-moving="1" title="${esc(s.label)}">${esc(s.label.split("—")[0].trim() || s.name)} 시작</button>`).join("");
      host.querySelectorAll("[data-scenario]").forEach(b => b.addEventListener("click", () => scenarioAction(b.dataset.scenario, b.title)));
      host.dataset.key = key;
      applyViewOnly();
    }
    const run = v.run;
    const st = $("vision-status");
    if (st) {
      if (!run) st.textContent = "시나리오 대기 — 버튼을 누르면 로봇에 교차로 계획을 보내고 출발한다";
      else st.textContent = `${run.label} · 통행권: ${run.holder || "비어 있음"} · 대기: ${(run.queue || []).join(", ") || "없음"}`
                            + ` · 통과 순서: ${(run.grant_order || []).join(" → ") || "—"}${run.done ? " · 완료" : ""}`;
    }
    const rows = $("vision-robots");
    if (rows) {
      rows.innerHTML = run ? Object.entries(run.robots || {}).map(([n, r]) => {
        const stage = VISION_STAGE[r.stage] || r.stage || "—";
        const depart = r.depart_in > 0 ? ` · 출발까지 ${r.depart_in}s` : "";
        const sl = r.stop_line_count ? `${r.stop_line_count}번째 정지선` : "정지선 수 미정(통행권 순서로)";
        return `<div class="config-row"><span>${esc(n)} ${esc(r.start)}→${esc(r.goal)}</span>`
             + `<strong>${esc(stage)} · ${esc(ko("drive", r.drive_state))} · 통과 ${r.clearance ? "허가" : "대기"} · ${esc(sl)}${esc(depart)}</strong></div>`;
      }).join("") : "";
    }
  }

  async function scenarioAction(name, label) {
    if (DEMO) return;
    if (!window.confirm(`${label || name}\n로봇을 출발 지점에 놓았나요? 교차로 계획을 보내고 출발합니다.`)) return;
    try {
      const r = await postJson("/api/fleet/scenario", {name});
      if (r.applied === null || r.success === false) alert(`⚠️ ${name}: ` + (r.message || "") + " (적용 여부 모름)");
    } catch (e) {
      alert("시나리오 시작 실패: " + e.message);
    }
    refresh();
  }

  // 웹 배정(2026-09-29): 시작·목적지를 관제 PC yaml 이 아니라 여기서 고른다. 드롭다운은 /api/fleet/profiles.graph.endpoints
  //   (지금 도로망의 endpoint 노드) 로 채운다 — 맵이 바뀌면 road_graph.yaml 만 고친다. 배정은 기존 /api/fleet/assign.
  function renderAssign() {
    const pill = $("assign-pill"), body = $("assign-body");
    if (!pill || !body) return;
    const p = state.profiles;
    const g = p?.graph;
    if (DEMO || !g || !Array.isArray(g.endpoints)) {
      setPill(pill, DEMO ? "DEMO" : "미수신", DEMO ? "pending" : "neutral");
      if (!body.querySelector("[data-assign-row]")) body.innerHTML = `<p class="muted">${DEMO ? "DEMO 에서는 배정하지 않습니다." : "/api/fleet/profiles 미수신 — 도로망 노드를 모른다"}</p>`;
      return;
    }
    setPill(pill, `${g.endpoints.length} 지점`, g.endpoints.length >= 2 ? "ok" : "warn");
    const nodeLabel = id => { const n = (g.nodes||[]).find(x=>x.id===id); return n && n.label ? `${id} — ${n.label}` : id; };
    const keyNow = g.endpoints.join(",");
    if (!body.querySelector("[data-assign-row]")) {
      // 버튼·select 는 한 번만 만든다(OPS-4) — 갱신은 옵션·현재값 칸만 바꾼다
      body.innerHTML = [1,2].map(i => `
        <div class="config-row" data-assign-row="${i}">
          <span>${robotKey(i)} <em class="muted" data-assign-now="${i}">—</em></span>
          <strong>
            <select data-assign-start="${i}" aria-label="${robotKey(i)} 시작" data-moving="1"></select> →
            <select data-assign-goal="${i}" aria-label="${robotKey(i)} 목적지" data-moving="1"></select>
            <button class="btn" data-assign-btn="${i}" data-moving="1">배정</button>
          </strong>
        </div>`).join("");
      body.querySelectorAll("[data-assign-btn]").forEach(b => b.addEventListener("click", () => assignAction(Number(b.dataset.assignBtn))));
      applyViewOnly();
    }
    [1,2].forEach(i => {
      const f = fleetRobot(i) || {};
      const now = body.querySelector(`[data-assign-now="${i}"]`);
      if (now) now.textContent = (f.start_node && f.goal_node) ? `현재 ${f.start_node}→${f.goal_node}` : "현재 경로 없음";
      [["start", f.start_node], ["goal", f.goal_node]].forEach(([kind, cur]) => {
        const sel = body.querySelector(`[data-assign-${kind}="${i}"]`);
        if (!sel) return;
        if (sel.dataset.key !== keyNow) {
          const keep = sel.value;
          sel.innerHTML = g.endpoints.map(id => `<option value="${esc(id)}">${esc(nodeLabel(id))}</option>`).join("");
          sel.value = keep && g.endpoints.includes(keep) ? keep : (cur && g.endpoints.includes(cur) ? cur : (g.endpoints[0] || ""));
          sel.dataset.key = keyNow;
        }
      });
    });
    const blocker = p.switch_blocker;
    body.querySelectorAll("[data-assign-btn]").forEach(b => { b.disabled = !!blocker; b.title = blocker || ""; });
  }

  async function assignAction(i) {
    if (DEMO) return;
    const body = $("assign-body"), msg = $("assign-msg");
    const start = body.querySelector(`[data-assign-start="${i}"]`)?.value || "";
    const goal = body.querySelector(`[data-assign-goal="${i}"]`)?.value || "";
    const robot = robotKey(i);
    if (!start || !goal) { msg.textContent = "⛔ 시작·목적지를 고른다"; return; }
    if (start === goal) { msg.textContent = "⛔ 시작과 목적지가 같다"; return; }
    if (!window.confirm(`${robot}: ${start} → ${goal} 로 배정합니다.\n경로가 새로 계산되고 출발은 하지 않습니다. 계속할까요?`)) return;
    try {
      const r = await postJson("/api/fleet/assign", {robot, start, goal});
      msg.textContent = (r.success ? "✅ " : "⚠️ ") + `${robot} ${start}→${goal} ` + (r.message || "") + (r.applied === null ? " (적용 여부 모름)" : "");
    } catch (e) {
      msg.textContent = "⛔ " + e.message;
    }
    refresh();
  }

  async function profileAction(url, payload, confirmText) {
    if (DEMO) return;
    if (!window.confirm(confirmText)) return;
    const msg = $("profile-msg");
    try {
      const body = await postJson(url, payload);
      // 202 = 보냈지만 적용 여부를 모른다 — 성공이라 하지 않는다
      msg.textContent = (body.success ? "✅ " : "⚠️ ") + (body.message || "") + (body.applied === null ? " (적용 여부 모름)" : "");
    } catch (e) {
      msg.textContent = "⛔ " + e.message;
    }
    refresh();
  }

  function initProfileControls() {
    const sel = $("profile-select");
    if ($("profile-switch")) $("profile-switch").addEventListener("click", ()=>{
      const name = sel ? sel.value : "";
      profileAction("/api/fleet/profile", {name},
        `중계 좌표를 '${name}' 로 바꿉니다.\n도로망·미션·화면 지도가 바뀌고 경로가 새로 배정됩니다(출발은 하지 않습니다).\n태블릿(T-11)과 같은 순간에 바꾸세요. 계속할까요?`);
    });
    if ($("profile-robot-maps")) $("profile-robot-maps").addEventListener("click", ()=>{
      const p = state.profiles || {};
      profileAction("/api/fleet/robot_maps", {},
        `모든 로봇의 Nav2 지도를 '${p.robot_map_name}' 로 바꾸라고 보냅니다(CMD_SET_MAP).\n로봇에 그 지도 파일이 있어야 합니다. 계속할까요?`);
    });
    if ($("profile-initial-poses")) $("profile-initial-poses").addEventListener("click", ()=>{
      profileAction("/api/fleet/initial_poses", {},
        "경로가 있는 로봇에 '출발 노드' 를 초기 위치로 보냅니다(CMD_SET_INITIAL_POSE).\n로봇이 실제로 출발 노드에 놓여 있어야 맞습니다. 계속할까요?");
    });
  }

  // R-5: ?demo=1 은 실측이 아니다 — 화면 맨 위에 크게 밝힌다 (작은 글씨는 사진에서 사라진다)
  if (DEMO) {
    const badge = document.createElement("div");
    badge.id = "demo-badge";
    badge.textContent = "DEMO DATA — 실측이 아닌 예시 값";
    badge.style.cssText = "position:sticky;top:0;z-index:9999;padding:10px 16px;text-align:center;" +
      "font:800 22px/1.2 sans-serif;letter-spacing:.08em;color:#fff;background:#c81e3a;";
    document.body.prepend(badge);
    document.title = "[DEMO] " + document.title;
  }

  initTabs();
  initControls();
  initProfileControls();
  initMedia();
  applyViewOnly();
  window.__v2 = {buildSummary, robotPhrase, robotProgressTone, clearanceText, driveText, ko, PHRASES, viewOnly, renderControl,
                 _test: {setGateway: (g) => { state.gateway = g; applyViewOnly(); }}};   // 시험이 순수 함수·화면 훅을 직접 부른다
  setInterval(()=>{
    if($("clock")) $("clock").textContent=nowText();
    // 통합 검토 OPS-2 의 연결 판정이 먼저다(검토 P3) — 아래 새 그리기가 던져도 이 줄은 이미 돌았다
    const linkNow = renderLink();
    try {
      if($("mission-elapsed")) $("mission-elapsed").textContent=elapsedText();
      if($("summary-sentence")) renderSummaryBand();                // 데이터 신호등의 "N s 전" 은 초마다 움직인다
    } catch (e) { console.error("summary tick failed", e); }
    // 통합 검토 OPS-2: 갱신이 끝나지 않아도(응답이 매달림) 연결 상태가 바뀌면 카드까지 다시 그린다 — 버튼은 그대로다(OPS-4)
    if (linkNow !== state.renderedLink) renderAll();
  },1000);
  window.addEventListener("resize",()=>requestAnimationFrame(drawMaps));
  refresh();
  setInterval(refresh, 2000);
})();