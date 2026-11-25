// JAX 팔 대시보드 — 상태는 WS(/ws, 0.25초마다 전체 스냅샷), 안 되면 /api/state 폴링.
// 탭 3개: 연결 / 각도 제어 / 역기구학. 위 칩·모드 스위치·전부 정지·경고 띠는 모든 탭 공통.
// 모터를 움직이는 길은 각도 제어 탭의 '보내기' 하나(→ POST /api/joint/send → /arm_control/joint_command).
// 3D 뷰는 viewer.js(ES 모듈, three.js)가 window.JaxViewer 로 준다. 못 읽어도 나머지 화면은 돈다.
"use strict";
const $ = (s, r = document) => r.querySelector(s);
const el = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (k === "class") e.className = v;
    else if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
    else if (v !== undefined && v !== null && v !== false) e.setAttribute(k, v);
  }
  for (const k of kids.flat()) if (k !== null && k !== undefined && k !== false) e.append(k.nodeType ? k : document.createTextNode(String(k)));
  return e;
};
let S = null, sock = null, pollTimer = null;
const VIRT = () => !!(S && S.mode && S.mode.virtual);

function toast(text, level = "info") {
  const t = el("div", { class: "toast " + level }, text);
  $("#toasts").append(t);
  setTimeout(() => t.remove(), level === "error" ? 9000 : 4500);
}
function setChip(c, text, cls) { c.className = "chip " + (cls || ""); c.innerHTML = '<span class="dot"></span>'; c.append(text); }
function chip(id, text, cls) { setChip(document.getElementById(id), text, cls); }
async function post(url, body, quiet) {
  try {
    const r = await fetch(url, { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify(body || {}), keepalive: true });
    const j = await r.json();
    if (!j.ok) toast(j.error || "실패", "error"); else if (j.msg && !quiet) toast(j.msg);
    return j;
  } catch (e) { toast("서버 연결 실패", "error"); return { ok: false }; }
}
function stopAll() { post("/api/stop"); }
$("#btn-stop").onclick = stopAll;
document.addEventListener("keydown", (e) => { if (e.key === "Escape") stopAll(); });
// 슬라이더 파란 채움(--p): 사용자가 움직일 때와 코드가 값을 바꿀 때 모두
function rangeFill(e) { const lo = parseFloat(e.min) || 0, hi = parseFloat(e.max) || 0, v = parseFloat(e.value) || 0; e.style.setProperty("--p", (hi > lo ? (v - lo) / (hi - lo) * 100 : 0) + "%"); }
document.addEventListener("input", (ev) => { if (ev.target && ev.target.type === "range") rangeFill(ev.target); });
const vb = () => VIRT() ? el("span", { class: "vbadge" }, "가상") : null;

// ---------- 모드 (실물 / 가상) ----------
async function setMode(v) {
  if (VIRT() === v) return;
  if (v && !confirm("가상 모드로 바꿀까요?\n\n화면의 노드·모터·원점이 전부 가상 팔이 된다(실물과 상관없음). 실물 노드가 켜져 있으면 거절된다.")) return;
  if (!v && !confirm("실물(ROS) 모드로 바꿀까요?\n\n가상 노드는 전부 꺼지고, 이제부터 버튼이 실제 로봇에 간다.")) return;
  await post("/api/mode", { virtual: v });
}
$("#mode-real").onclick = () => setMode(false);
$("#mode-virtual").onclick = () => setMode(true);
let rosBusy = false;
async function rosConnect() {
  if (rosBusy) return;
  rosBusy = true;
  try { await post("/api/ros/connect"); } finally { rosBusy = false; if (S) render(); }
}
const rosLabel = () => (S && S.ros.conn && S.ros.conn.state === "connected") ? "ROS 다시 연결" : "ROS 연결(source)";
$("#btn-ros").onclick = rosConnect;

// ---------- 탭 ----------
const TABS = [["conn", "연결", "켜기"], ["angle", "각도 제어", "관절 움직이기"], ["ik", "역기구학", "집기 계획"], ["rl", "강화학습", "정책으로 집기"]];
const OLD_TABS = { ak: "conn", servo: "angle", finger: "angle", arm: "angle", vision: "ik" };   // 예전 주소
const TAB_IDS = new Set(TABS.map((t) => t[0]));
const TAB_KEY = "jaxdash.tab";
const tabBtns = {};
let curTab = null;
function lsGet(k) { try { return localStorage.getItem(k); } catch (e) { return null; } }
function lsSet(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* 저장 못 해도 화면은 그대로 */ } }
function showTab(id) {
  id = OLD_TABS[id] || id;
  if (!TAB_IDS.has(id)) id = "conn";
  curTab = id;
  for (const [t] of TABS) {
    document.getElementById("page-" + t).hidden = t !== id;
    tabBtns[t].classList.toggle("active", t === id);
    tabBtns[t].setAttribute("aria-selected", t === id ? "true" : "false");
  }
  lsSet(TAB_KEY, id);
  if (id === "rl" && window.JaxRL) window.JaxRL.mount(document.getElementById("page-rl"));   // 강화학습 탭은 처음 열 때만 만든다 (정책 정보·WebGL)
  if (S) { initViewers(); renderViewers(); updateIkView(); }
  for (const v of Object.values(VW)) if (v && v.resize) { v.resize(); }
  streamSyncAll();
}
// ---------- MJPEG 스트림 (<img>) — 보이는 탭·보이는 창에서만 연결 ----------
// 브라우저는 호스트당 HTTP 연결 6개: 숨은 탭에 스트림이 열려 있으면 긴 요청과 겹쳐 STOP POST 가 줄 설 수 있다.
// 각 탭은 streamSet(img, 켜기?) 만 부르고, 실제 연결은 탭이 보일 때만 (떠나면 끊고 돌아오면 다시 건다).
const STREAM_URL = "/api/live/stream.mjpg";
const BLANK_IMG = "data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7";   // src 지우기만으로는 안 끊는 브라우저가 있어 빈 그림으로 바꿔 끊음
function streamSync(img) {
  const page = img.closest(".page");
  const want = img.dataset.stream === "1" && !document.hidden && !(page && page.hidden);
  if (want && img.dataset.streamOn !== "1") { img.dataset.streamOn = "1"; img.src = STREAM_URL + "?t=" + Date.now(); }
  else if (!want && img.dataset.streamOn === "1") { img.dataset.streamOn = ""; img.src = BLANK_IMG; }
}
function streamSet(img, on) { img.dataset.stream = on ? "1" : ""; streamSync(img); }
function streamSyncAll() { document.querySelectorAll("img[data-stream]").forEach(streamSync); }
document.addEventListener("visibilitychange", streamSyncAll);
// 폰 영상 보기 180° (폰이 거치대에 뒤집혀 끼워짐 — 보기만 돌림, 인식 계산은 안 바뀜). 영상을 누르면 켜고 끔, 기본 켬, 브라우저에 기억
(function camFlip() {
  let on = true;
  try { on = localStorage.getItem("jaxdash.camflip") !== "0"; } catch (e) { /* 없어도 됨 */ }
  document.head.append(el("style", {}, "body.camflip img[data-stream] { transform: rotate(180deg); } img[data-stream] { cursor: pointer; }"));
  document.body.classList.toggle("camflip", on);
  document.addEventListener("click", (ev) => {
    if (!(ev.target instanceof HTMLImageElement) || !("stream" in ev.target.dataset)) return;
    on = !on; document.body.classList.toggle("camflip", on);
    try { localStorage.setItem("jaxdash.camflip", on ? "1" : "0"); } catch (e) { /* 없어도 됨 */ }
  });
  document.addEventListener("mouseover", (ev) => { const t = ev.target; if (t instanceof HTMLImageElement && "stream" in t.dataset) t.title = "누르면 영상 180° 돌리기 (보기만)"; });
})();
(function buildTabs() {
  const nav = $("#tabs");
  for (const [id, label, sub] of TABS) {
    const b = el("a", { class: "tab", href: "#" + id, role: "tab" }, label, el("small", { class: "tsub" }, sub), el("span", { class: "tdot" }));
    tabBtns[id] = b;
    nav.append(b);
  }
  window.addEventListener("hashchange", () => showTab(location.hash.slice(1)));
  const topH = () => document.documentElement.style.setProperty("--topbar-h", $(".topbar").offsetHeight + "px");
  topH(); window.addEventListener("resize", topH);
})();
function renderTabDots() {
  for (const [id] of TABS) {
    const svcs = S.services.filter((s) => s.tab === id);
    const on = svcs.filter((s) => s.alive).length;
    const dead = svcs.some((s) => s.kind !== "once" && !s.alive && s.exit_code != null && s.exit_code !== 0);   // 한 번짜리(계획 거절 = 코드 1)는 빨간 점 아님
    const d = tabBtns[id].querySelector(".tdot");
    d.className = "tdot " + (on ? "on" : dead ? "bad" : "");
    d.textContent = on ? String(on) : "";
  }
}

// ---------- 서버 연결 ----------
function connect() {
  sock = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  sock.onopen = () => { chip("chip-ws", "연결됨", "on"); if (pollTimer) { clearInterval(pollTimer); pollTimer = null; } };
  sock.onmessage = (ev) => { S = JSON.parse(ev.data); render(); };
  sock.onclose = () => {
    chip("chip-ws", "WS 끊김 — 폴링", "warn");
    if (!pollTimer) pollTimer = setInterval(poll, 1000);
    setTimeout(connect, 3000);
  };
}
async function poll() {
  try { S = await (await fetch("/api/state")).json(); render(); if (!sock || sock.readyState !== WebSocket.OPEN) chip("chip-ws", "폴링 연결", "warn"); }
  catch (e) { chip("chip-ws", "서버 끊김", "bad"); }
}

// ---------- 그리기 ----------
const fmtDur = (s) => s == null ? "" : s < 60 ? `${s.toFixed(0)}초` : s < 3600 ? `${Math.floor(s / 60)}분 ${Math.floor(s % 60)}초` : `${Math.floor(s / 3600)}시간 ${Math.floor(s % 3600 / 60)}분`;
const fmtAge = (a) => a == null ? "—" : a < 60 ? `${a.toFixed(1)}초 전` : `${fmtDur(a)} 전`;
const LV = ["on", "warn", "bad", "warn"];
const LVT = ["정상", "주의", "에러", "오래됨"];
const kv = (k, v) => [el("span", { class: "k" }, k), el("span", { class: "v" }, v ?? "—")];
const f1 = (x, u = "°") => x == null || !Number.isFinite(x) ? "—" : `${Number(x).toFixed(1)}${u}`;
const f2 = (x, u = "°") => x == null ? "—" : `${Number(x).toFixed(2)}${u}`;
const svcById = (id) => (S.services.find((x) => x.id === id) || {});

// ---------- 재생 중 '지금 무슨 동작' (서버 play_phase: playback.phase·phase_k·phase_n·left_s) ----------
const PHASE_SHOW_S = 6;   // 끝난 뒤 이만큼(초) 더 보인다
function playbackNow() {   // 지금 모드의 팔 재생 상태 (가상 = virtual_state, 실물 직접 CAN = direct_state)
  const r = (S && S.ros) || {};
  return ((VIRT() ? r.virtual_state : r.direct_state) || {}).playback || {};
}
function phaseText(pb) {   // "큐브 위로 가는 중 · 3/12단계 · 남은 21초" (단계표 없는 재생이면 "")
  if (!pb || !pb.phase) return "";
  const k = pb.phase_k && pb.phase_n ? ` · ${pb.phase_k}/${pb.phase_n}단계` : "";
  return pb.running ? `${pb.phase}${k} · 남은 ${Math.ceil(pb.left_s || 0)}초` : pb.phase;
}
function renderPhaseLine() {
  const n = $("#phaseline"), pb = playbackNow();
  const recent = !pb.running && pb.ended && S.t - pb.ended < PHASE_SHOW_S;
  const show = !!pb.phase && (pb.running || recent);
  if (n.hidden !== !show) {
    n.hidden = !show;
    document.documentElement.style.setProperty("--topbar-h", $(".topbar").offsetHeight + "px");
  }
  if (!show) return;
  const stop = !pb.running && pb.reason && pb.reason !== "끝까지 재생";
  n.className = "phaseline" + (pb.running ? "" : stop ? " stop" : " done");
  const k = pb.phase_k && pb.phase_n ? `${pb.phase_k}/${pb.phase_n}단계` : "";
  n.replaceChildren(el("span", {}, (VIRT() ? "(가상) " : "") + (pb.running ? "▶ " : "") + pb.phase),
    el("small", {}, pb.running ? [k, `남은 ${Math.ceil(pb.left_s || 0)}초`, `프레임 ${pb.frame}/${pb.frames - 1}`].filter(Boolean).join(" · ") : k));
}

// 모드(가상 ↔ 실물 ROS ↔ 직접 CAN)가 바뀌면 '한 번만 묻는' 확인과 손 위치 따라가기를 되돌린다 (2026-11-25 검토:
// 가상에서 한 번 '예' 를 누르면 실물로 바꾼 뒤 첫 보내기가 경고 없이 실제 팔을 움직였고, 따라가기가 켜진 채 남아 0.2초마다 보냈다)
let _modeKindSeen = null;
function resetConfirmsOnModeChange() {
  const kind = (S.mode || {}).kind || (VIRT() ? "virtual" : "ros");
  if (_modeKindSeen !== null && kind !== _modeKindSeen) {
    if (typeof A !== "undefined") A.confirmed = false;
    if (typeof HK !== "undefined") {
      HK.warned = false;
      if (HK.follow) {
        HK.follow = false; HK.followSent = null; HK.followSol = null;
        if (HK.f && HK.f.followBox) HK.f.followBox.checked = false;
        toast("모드가 바뀌어 '팔도 바로 따라가기' 를 껐다");
      }
    }
  }
  _modeKindSeen = kind;
}

function render() {
  if (!S) return;
  resetConfirmsOnModeChange();
  if (!curTab) { const h = location.hash.slice(1); showTab(OLD_TABS[h] || (TAB_IDS.has(h) ? h : (lsGet(TAB_KEY) || "conn"))); }
  const v = VIRT(), r = S.ros, cs = (r.conn || {}).state;
  document.body.classList.toggle("virtual", v);
  $("#brand-virtual").hidden = !v;
  $("#virtbar").hidden = !v;
  $("#mode-real").className = v ? "" : "on";
  $("#mode-virtual").className = v ? "on v" : "";
  const wb = $("#warnbar");
  wb.innerHTML = v ? "⚠ 전부 정지 = (가상) 노드 종료 → 가상 팔이 떨어진다. 실물이면 <b>팔이 떨어질 수 있다.</b>"
    : "⚠ 전부 정지 = 노드 종료 → 실물은 <b>팔이 떨어질 수 있다.</b> 먼저 팔을 받친다.";
  wb.title = "ak45_node 가 끝나면 AK45 에 브레이크 0A → 홀드가 풀린다.";
  const sg = $("#suggestbar");
  if (!v && S.mode && !S.mode.ros_possible) {
    if (sg.hidden) { sg.hidden = false; sg.replaceChildren(`이 PC엔 ROS 없음 (${S.mode.ros_setup} 없음) — 실물 모드는 못 쓴다. `,
      el("button", { class: "btn small", onclick: () => setMode(true) }, "가상 모드로 바꾸기")); }
  } else sg.hidden = true;
  if (v) chip("chip-ros", "가상 팔", "on");
  else chip("chip-ros", r.available ? "ROS 구독 중" : cs === "starting" ? "ROS 연결 중" : cs === "dead" ? "ROS 자식 죽음" : cs === "stale" ? "ROS 응답 없음" : "ROS 없음",
    r.available ? "on" : (cs === "dead" || cs === "stale") ? "bad" : cs === "starting" ? "warn" : "");
  const rb = $("#btn-ros"); rb.hidden = v; rb.textContent = rosBusy ? "연결 중…" : rosLabel(); rb.disabled = rosBusy;
  const c = S.can;
  chip("chip-can", c.text || "can0", !c.available ? "" : c.up ? (c.state && c.state !== "ERROR-ACTIVE" ? "warn" : "on") : "bad");
  const running = S.services.filter((s) => s.alive).length;
  chip("chip-run", `실행 ${running}${v ? " (가상)" : ""}`, running ? "on" : "");
  const sb = $("#straybar");
  if (S.strays.length) {
    sb.hidden = false;
    sb.textContent = "대시보드 밖에서 떠 있는 노드(전부 정지로 안 꺼짐): " + S.strays.map((x) => `${x.name} pid ${x.pid}`).join(", ") +
      " — 그 터미널에서 끄거나 pkill -INT -f <이름>";
  } else sb.hidden = true;
  renderTabDots();
  renderConn();
  renderServices();
  renderSteps();
  renderMotors();
  renderCalib();
  renderHoming();
  renderGoHome();
  renderAngles();
  renderPose();
  renderTraj();
  renderHandIk();
  renderPhaseLine();
  renderViewers();
  $("#foot").textContent = `${v ? "가상 모드 · " : ""}워크스페이스 ${S.info.ws} · ROS ${S.info.distro} · 레포 ${S.info.repo} · 로그 ${S.info.logs} · ${S.info.platform}` +
    ` · 메시 ${S.robot.mesh_ok ? S.robot.mesh_dir : "없음"}`;
}

// ---------- 연결 탭: ROS / 가상 카드 ----------
const C = {};
function renderConn() {
  const box = $("#conn-ros"), v = VIRT();
  const key = v ? "v" : "r";
  if (C.key !== key) {
    C.key = key;
    if (v) {
      C.vnodes = el("div", { class: "kv" }); C.vpower = el("span", { class: "chip" }); C.vmsg = el("div", { class: "note" });
      box.replaceChildren(el("div", { class: "card wide" },
        el("h3", {}, el("span", { class: "vbadge" }, "가상"), "가상 로봇팔", C.vpower),
        el("p", { class: "help" }, "실물 없이 같은 규칙으로 도는 가상 노드(virtual_arm.py): ak45_node(첫 명령 전 안 잡음, 마지막 목표 홀딩, 60 °/s 모터), " +
          "stm_bridge(보낸 각 그대로), arm_control_node(calibration 변환 + max_speed 램프, arm_control 모듈 그대로), homing_guard(homing.py 그대로). " +
          "아래 '노드 켜기/끄기' 로 가상 노드를 켠다(arm_homed.launch.py 하나면 전부)."),
        el("div", { class: "row" },
          el("button", { class: "btn red", onclick: () => { if (confirm("가상 AK45 전원을 끈다: 피드백이 끊기고 가상 팔이 떨어진다.")) post("/api/virtual/power", { on: false }); } }, "가상 전원 끄기"),
          el("button", { class: "btn line", onclick: () => post("/api/virtual/power", { on: true }) }, "가상 전원 켜기 (영점 틀어짐)")),
        el("p", { class: "help" }, "전원을 다시 켜면 엔코더 영점이 15~40° 틀어진다(단일 엔코더 흉내): homing_guard 가 원점을 지우고 그 관절 명령을 막는지 본다."),
        C.vmsg, C.vnodes));
    } else {
      C.rchip = el("span", { class: "chip" }); C.rnote = el("div", { class: "note" }); C.rhelp = el("div", { class: "help" });
      C.rbtn = el("button", { class: "btn", onclick: rosConnect }, "ROS 연결(source)");
      box.replaceChildren(el("div", { class: "card wide" }, el("h3", {}, "ROS 연결", C.rchip),
        el("p", { class: "help" }, "ROS 구독·발행은 ROS 를 source 한 자식 프로세스(ros_monitor_proc.py)에서 돈다. 노드 켜기/끄기는 ROS 연결 없이도 된다."),
        C.rnote, el("div", { class: "row" }, C.rbtn), C.rhelp));
    }
  }
  if (v) {
    const vs = S.ros.virtual_state || {};
    setChip(C.vpower, vs.power ? "AK45 전원 켜짐" : "AK45 전원 꺼짐", vs.power ? "on" : "bad");
    $("#conn-note").textContent = "· 가상";
    C.vmsg.textContent = `켜진 가상 노드: ${(vs.nodes || []).join(", ") || "없음"}`;
    const rows = Object.entries(vs.phys_deg || {}).map(([n, p]) => kv(`${n} 실제 축 / 엔코더 영점 어긋남`, `${p.toFixed(1)}° / ${(vs.enc_off_deg || {})[n]?.toFixed(1)}°`));
    C.vnodes.replaceChildren(...rows.flat());
  } else {
    const r = S.ros, c = r.conn || {};
    setChip(C.rchip, r.available ? "연결됨" : c.state === "starting" ? "연결 중" : "없음", r.available ? "on" : c.state === "starting" ? "warn" : "bad");
    $("#conn-note").textContent = "";
    C.rnote.textContent = r.available ? `ROS ${c.distro} · 자식 pid ${c.pid}` : (r.reason || "ROS 없음");
    C.rnote.className = "note " + (r.available ? "" : ["dead", "stale"].includes(c.state) ? "bad" : "warn");
    C.rbtn.textContent = rosBusy ? "연결 중…" : rosLabel(); C.rbtn.disabled = rosBusy;
    C.rhelp.textContent = `source ${c.setup || "?"} (+ ${c.ws_setup || "?"} 있으면) · 로그 ${c.log || ""}`;
  }
}
function rosMissing(box) {
  const c = S.ros.conn || {}, key = [S.ros.reason, c.state, c.pid, rosBusy].join("|");
  if (box.dataset.rosKey === key) return;
  box.dataset.rosKey = key;
  box.replaceChildren(el("div", { class: "card muted" },
    el("div", { class: "note " + (["dead", "stale"].includes(c.state) ? "bad" : "") }, S.ros.reason || "ROS 없음"),
    el("div", { class: "help" }, "위 'ROS 연결' 카드 또는 화면 위 '가상' 스위치.")));
}

// ---------- 연결 탭: 순서대로 켜기 (① can0 ② ROS ③ 팔) ----------
const ST = {};
function svcState(s) {
  if (!s) return ["없음", ""];
  if (!s.available) return ["못 씀", ""];
  if (s.stopping) return ["끄는 중", "warn"];
  if (s.alive) return [s.kind === "once" ? "실행 중" : "켜짐", "on"];
  if (s.exit_code != null && s.exit_code !== 0) return [`죽음 (코드 ${s.exit_code})`, "bad"];
  return ["꺼짐", ""];
}
function svcParams(id) {
  const c = cards[id];
  return c ? Object.fromEntries(Object.entries(c.fields).map(([k, i]) => [k, i.type === "checkbox" ? (i.checked ? "1" : "") : i.value])) : {};
}
function renderSteps() {
  const box = $("#conn-steps"), v = VIRT();
  if (!ST.built) {
    ST.built = true;
    const step = (n, title) => {
      const o = { chip: el("span", { class: "chip" }), note: el("div", { class: "note" }), row: el("div", { class: "row" }) };
      o.root = el("div", { class: "card step" }, el("h3", {}, el("span", { class: "num" }, n), title, o.chip), o.note, o.row);
      return o;
    };
    ST.can = step("1", "CAN 켜기");
    ST.can.b = el("button", { class: "btn", onclick: () => post("/api/svc/can_up/start", { params: {} }) }, "can0 올리기");
    ST.can.row.append(ST.can.b);
    ST.ros = step("2", "연결");
    ST.ros.b = el("button", { class: "btn", onclick: rosConnect }, "연결");
    ST.ros.row.append(ST.ros.b);
    ST.arm = step("3", "팔 켜기");
    ST.arm.on = el("button", { class: "btn", onclick: () => post("/api/svc/arm_homed/start", { params: svcParams("arm_homed") }) }, "▶ 켜기");
    ST.arm.off = el("button", { class: "btn red", onclick: () => {
      const s = S.services.find((x) => x.id === "arm_homed");
      if (s && s.stop_warn && !VIRT() && !confirm(s.stop_warn)) return;
      post("/api/svc/arm_homed/stop");
    } }, "■ 끄기");
    ST.arm.row.append(ST.arm.on, ST.arm.off);
    box.append(ST.can.root, ST.ros.root, ST.arm.root);
  }
  const can = S.can, canSvc = S.services.find((x) => x.id === "can_up");
  if (v) { setChip(ST.can.chip, "가상", "on"); ST.can.note.textContent = "가상 모드에선 필요 없음"; ST.can.b.disabled = true; }
  else {
    setChip(ST.can.chip, !can.available ? "못 봄" : can.up ? "UP" : "꺼짐", !can.available ? "" : can.up ? "on" : "bad");
    ST.can.note.textContent = !can.available ? (canSvc && !canSvc.available ? canSvc.why : "can0 상태를 못 읽음") : can.up ? (can.state || "") : "재부팅마다 1번";
    ST.can.b.disabled = !canSvc || !canSvc.available || canSvc.alive || can.up;
  }
  const r = S.ros, cs = (r.conn || {}).state;
  if (v) { setChip(ST.ros.chip, "가상 팔", "on"); ST.ros.note.textContent = "가상 모드에선 필요 없음"; ST.ros.b.disabled = true; }
  else {
    setChip(ST.ros.chip, r.available ? "연결됨" : cs === "starting" ? "연결 중" : "안 됨", r.available ? "on" : cs === "starting" ? "warn" : "bad");
    ST.ros.note.textContent = r.available ? `ROS ${(r.conn || {}).distro || ""}` : (r.reason || "ROS 없음");
    ST.ros.b.textContent = rosBusy ? "연결 중…" : r.available ? "다시 연결" : "연결"; ST.ros.b.disabled = rosBusy;
  }
  const arm = S.services.find((x) => x.id === "arm_homed"), [t, cls] = svcState(arm);
  setChip(ST.arm.chip, (arm && arm.virtual ? "가상 · " : "") + t, cls);
  ST.arm.note.textContent = !arm ? "" : !arm.available ? arm.why : arm.error || "AK45 4개 · 서보 · 원점 검사 · 각도 변환을 한 번에";
  ST.arm.on.disabled = !arm || !arm.available || arm.alive || arm.stopping;
  ST.arm.off.disabled = !arm || !arm.alive || arm.stopping;
}

// 서보 핀(STM32 채널) 고르기: 바꾸면 그 핀을 쓰던 관절과 맞바꿈 (직접 CAN, POST /api/direct/calib pin). 저장은 보정 저장으로
function pinSelect(c) {
  const direct = S.mode && S.mode.kind === "direct";
  if (!direct) return el("span", {}, String(c.id));
  const sel = el("select", { class: "pin", title: "서보가 꽂힌 STM32 핀. 바꾸면 그 핀을 쓰던 관절과 맞바꾼다 (둘 다 '안 보냄' — 다음 명령은 바로 튐). 저장해야 남는다" },
    ...Array.from({ length: 10 }, (_, i) => el("option", { value: i }, String(i))));
  sel.value = String(c.id);
  sel.addEventListener("change", async () => {
    const r = await post("/api/direct/calib", { action: "pin", joint: c.joint, value: +sel.value });
    if (r && r.ok) toast(r.msg || "핀 바꿈"); else { toast((r && r.error) || "못 바꿈", "error"); sel.value = String(c.id); }
  });
  return sel;
}

// ---------- AK45 (한 줄씩) ----------
function renderMotors() {
  const r = S.ros, box = $("#motors");
  const ae = document.activeElement;
  if (ae && ae.tagName === "SELECT" && box.contains(ae)) return;   // 핀 고르는 중엔 다시 그리지 않음 (목록이 닫히지 않게)
  if (!r.available) { $("#ros-note").textContent = ""; rosMissing(box); return; }
  delete box.dataset.rosKey;
  box.replaceChildren();
  $("#ros-note").textContent = (VIRT() ? "(가상) " : "") + (r.diag_age == null ? "ak45_node 응답 없음 (팔 꺼짐?)" : `${r.diag_age.toFixed(1)}초 전`);
  if (!r.motors.length) { box.append(el("div", { class: "card muted" }, "팔을 켜면 AK45 4개가 여기 나온다")); return; }
  const rows = r.motors.map((m) => {
    const v = m.values || {}, stale = m.age > 3, lv = stale ? 3 : m.level;
    return el("tr", {},
      el("td", {}, el("b", {}, m.name.replace("ak45/", ""))),
      el("td", {}, el("span", { class: "chip " + LV[lv] }, el("span", { class: "dot" }), LVT[lv])),
      el("td", { class: "num" }, v.position_deg != null ? `${v.position_deg}°` : "—"),
      el("td", { class: "num" }, `${v.temperature_c ?? "—"} °C`),
      el("td", {}, v.command_active === "1" ? "홀드 중" : "힘 없음"),
      el("td", { class: "note " + (lv === 2 ? "bad" : lv ? "warn" : "") }, stale ? "응답 끊김" : lv ? m.message : ""));
  });
  let bus = null;
  if (r.bus) {
    const lv = r.bus.age > 3 ? 3 : r.bus.level;
    bus = el("div", { class: "note" }, "CAN 버스 ", el("span", { class: "chip " + LV[lv] }, el("span", { class: "dot" }), LVT[lv]), " ", lv ? r.bus.message : "");
  }
  box.append(el("div", { class: "card mtable" }, el("table", {},
    el("thead", {}, el("tr", {}, ...["모터", "상태", "위치", "온도", "힘", ""].map((h) => el("th", {}, h)))),
    el("tbody", {}, ...rows)), bus || ""));
  // 서보: 위치 피드백이 없어 '마지막으로 보낸 값' (sim ° = 관절 각, 모터 ° = 서보에 보낸 0~180)
  const dcal = (r.direct_state || {}).calibration || r.calibration;
  const live = dcal && dcal.length ? dcal : null;   // 직접 CAN: 메모리 보정(핀 바꾼 것 포함, 저장 전도)
  const sv = r.servos || {}, cal = (live || (S.calibration || {}).joints || []).filter((c) => c.motor === "servo");
  if (cal.length) {
    const srow = cal.map((c) => {
      const s = sv[c.name], sim = s ? (s.deg - c.zero_deg) / ((c.sign || 1) * (c.scale || 1)) : null;
      return el("tr", {},
        el("td", {}, el("b", {}, c.joint)), el("td", { class: "muted" }, c.name),
        el("td", {}, pinSelect(c)),
        el("td", { class: "num" }, s ? `${sim.toFixed(1)}°` : "—"),
        el("td", { class: "num muted" }, s ? `${s.deg.toFixed(1)}°` : ""),
        el("td", { class: "note" }, s ? `${s.age < 60 ? s.age.toFixed(0) + "초" : (s.age / 60).toFixed(0) + "분"} 전 보냄` : "안 보냄 (첫 명령은 바로 튐)"));
    });
    box.append(el("div", { class: "card mtable" }, el("h3", {}, "서보 (마지막으로 보낸 값 — 피드백 없음)"), el("table", {},
      el("thead", {}, el("tr", {}, ...["관절", "서보", "핀", "각", "서보 각", ""].map((h) => el("th", {}, h)))),
      el("tbody", {}, ...srow))));
  }
}

// ---------- AK45 진단 (예전 카드, 안 씀) ----------
function renderMotorsFull() {
  const r = S.ros, box = $("#motors");
  if (!r.available) { $("#ros-note").textContent = ""; rosMissing(box); return; }
  delete box.dataset.rosKey;
  box.replaceChildren();
  $("#ros-note").textContent = (VIRT() ? "(가상) " : "") + (r.diag_age == null ? "/diagnostics 아직 없음 (ak45_node 꺼짐?)" :
    `/diagnostics ${r.diag_age.toFixed(1)}초 전 · /joint_states ${r.joint_age == null ? "없음" : r.joint_age.toFixed(1) + "초 전"}`);
  if (!r.motors.length) { box.append(el("div", { class: "card muted" }, "ak45_node 진단을 기다리는 중")); return; }
  for (const m of r.motors) {
    const v = m.values || {}, stale = m.age > 3;
    const lv = stale ? 3 : m.level;
    box.append(el("div", { class: "card" },
      el("h3", {}, vb(), m.name.replace("ak45/", ""), el("span", { class: "chip " + LV[lv] }, el("span", { class: "dot" }), LVT[lv]),
        el("span", { class: "help", style: "margin:0" }, m.hardware_id)),
      el("div", { class: "big" }, v.position_deg != null ? `${v.position_deg}°` : "—"),
      el("div", { class: "note " + (lv === 2 ? "bad" : lv ? "warn" : "") }, stale ? `진단 ${m.age.toFixed(0)}초 전 (ak45_node 멈춤?)` : m.message),
      el("div", { class: "kv" },
        kv("온도", `${v.temperature_c ?? "—"} °C`), kv("에러 코드", v.error_code ?? "—"),
        kv("워치독", v.watchdog_ok === "1" ? "OK" : "끊김"), kv("target", v.target_deg === "none" ? "없음" : `${v.target_deg}°`),
        kv("command_active", v.command_active === "1" ? "예 (홀드 중)" : "아니오"), kv("오차", v.position_error_deg === "none" ? "—" : `${v.position_error_deg}°`),
        kv("전류", `${v.current_a ?? "—"} A`), kv("속도", `${v.speed_erpm ?? "—"} erpm`),
        kv("/joint_states", m.joint_deg == null ? "없음" : `${m.joint_deg.toFixed(2)}°`))));
  }
  if (r.bus) {
    const v = r.bus.values || {}, stale = r.bus.age > 3, lv = stale ? 3 : r.bus.level;
    box.append(el("div", { class: "card" },
      el("h3", {}, vb(), "CAN 버스", el("span", { class: "chip " + LV[lv] }, el("span", { class: "dot" }), LVT[lv])),
      el("div", { class: "big" }, v.state || "—"),
      el("div", { class: "note " + (lv === 2 ? "bad" : lv ? "warn" : "") }, r.bus.message),
      el("div", { class: "kv" }, kv("에러 프레임", v.err_frames), kv("bit-stuff", v.bit_stuff_errors), kv("form", v.form_errors),
        kv("no-ack", v.ack_errors), kv("버스오프", v.busoff_count), kv("재시작", v.restart_count))));
  }
}

// ---------- 노드 카드 (한 번 만들고 바뀌는 부분만) ----------
const cards = {};
function renderServices() {
  for (const s of S.services) {
    let c = cards[s.id];
    if (!c) {
      c = cards[s.id] = s.id === "pick_dry" ? buildPick(s) : buildService(s);
      (document.getElementById("svc-" + s.tab) || $("#svc-conn")).append(c.root);
    }
    updateService(c, s);
    if (c.updateExtra) c.updateExtra(s);
  }
}
function buildService(s) {
  const c = { fields: {} };
  c.chip = el("span", { class: "chip" });
  c.vb = el("span", { class: "vbadge", hidden: true }, "가상");
  c.status = el("div", { class: "note" });
  c.note = el("div", { class: "note" });
  c.cmd = el("div", { class: "cmd", hidden: true });
  c.log = el("pre", { class: "logbox" + (s.tab === "ik" ? " tall" : "") });
  c.logbox = el("details", { class: "log" },
    el("summary", {}, s.kind === "once" ? `출력 (마지막 실행, 최대 ${s.tail}줄)` : `로그 (마지막 ${s.tail}줄)`), c.log,
    el("div", { class: "row" }, el("a", { href: `/api/log/${s.id}?n=1000`, target: "_blank", class: "help" }, "로그 1000줄 보기")));
  const fields = s.params.map((p) => {
    const inp = el("input", { value: p.value, placeholder: p.hint || "" });
    c.fields[p.key] = inp;
    return el("label", { class: "field" }, `${p.label}${p.hint ? " (" + p.hint + ")" : ""}`, inp);
  });
  const params = () => Object.fromEntries(Object.entries(c.fields).map(([k, i]) => [k, i.value]));
  const once = s.kind === "once";
  const warn = () => s.stop_warn && !VIRT() && !confirm(s.stop_warn);
  c.bStart = el("button", { class: "btn", onclick: () => post(`/api/svc/${s.id}/start`, { params: params() }) }, once ? "▶ 실행" : "▶ 켜기");
  c.bStop = el("button", { class: "btn red", onclick: () => { if (warn()) return; post(`/api/svc/${s.id}/stop`); } }, once ? "■ 중단" : "■ 끄기");
  c.bRestart = once ? null : el("button", { class: "btn line", onclick: () => { if (warn()) return; post(`/api/svc/${s.id}/restart`, { params: params() }); } }, "↻ 재시작");
  c.root = el("div", { class: "card" + (s.tab === "ik" ? " wide" : "") },
    el("h3", {}, c.vb, s.label, c.chip), el("p", { class: "help" }, `[${s.group}] ${s.help}`),
    fields, el("div", { class: "row" }, c.bStart, c.bStop, c.bRestart), c.status, c.note, c.cmd, c.logbox);
  return c;
}
function updateService(c, s) {
  let cls = "", txt = "꺼짐";
  if (!s.available) { txt = "못 씀"; }
  else if (s.stopping) { cls = "warn"; txt = "끄는 중"; }
  else if (s.alive) { cls = "on"; txt = s.kind === "once" ? "실행 중" : "켜짐"; }
  else if (s.exit_code != null) { cls = s.exit_code === 0 ? "" : "bad"; txt = s.exit_code === 0 ? "끝남 (0)" : `죽음 (코드 ${s.exit_code})`; }
  setChip(c.chip, txt, cls);
  c.vb.hidden = !s.virtual;
  c.bStart.disabled = !s.available || s.alive || s.stopping;
  c.bStop.disabled = !s.alive || s.stopping;
  if (c.bRestart) c.bRestart.disabled = !s.available || s.stopping;
  for (const inp of Object.values(c.fields)) inp.disabled = s.alive;
  let st = "";
  if (!s.available) st = s.why;
  else if (s.alive) st = s.virtual ? `가상 노드 · 가동 ${fmtDur(s.uptime)}` : `pid ${s.pid} · 가동 ${fmtDur(s.uptime)}${s.adopted ? " · 이전 대시보드에서 켠 것(다시 붙잡음)" : ""}`;
  else if (s.ended) st = `끝난 시각 ${new Date(s.ended * 1000).toLocaleTimeString()}`;
  c.status.textContent = st;
  const note = s.error || s.stop_note;
  c.note.textContent = note;
  c.note.className = "note " + (s.error || /SIGKILL|살아 있음/.test(s.stop_note) ? "bad" : /SIGTERM/.test(s.stop_note) ? "warn" : "");
  c.cmd.hidden = !s.cmd; c.cmd.textContent = s.cmd ? "$ " + (s.cmd.length > 400 ? s.cmd.slice(0, 400) + " …" : s.cmd) : "";
  const atBottom = c.log.scrollTop + c.log.clientHeight >= c.log.scrollHeight - 4;
  let lines = s.log;
  if (s.kind === "once") {
    let k = -1;
    for (let i = lines.length - 1; i >= 0; i--) if (/^===== .* 켜기: /.test(lines[i])) { k = i; break; }
    if (k >= 0) lines = lines.slice(k);
  }
  const text = lines.join("\n");
  if (c.log.textContent !== text) { c.log.textContent = text; if (atBottom) c.log.scrollTop = c.log.scrollHeight; }
  if (!c.keepClosed && (s.alive || (s.tab === "ik" && text)) && !c.opened) { c.logbox.open = true; c.opened = true; }
}

// ---------- 역기구학 탭: pick_place --dry-run (큐브 칸 → 계획 돌려 보기 → 결과 한 줄) ----------
// updateService 가 쓰는 칸(chip·status·note·cmd·log·logbox·bStart·bStop·fields)은 buildService 와 같은 이름으로 만든다.
function buildPick(s) {
  const c = { fields: {}, keepClosed: true };
  const P0 = Object.fromEntries(s.params.map((p) => [p.key, p]));
  const fld = (k, lab, unit, w) => {
    const p = P0[k] || { value: "", hint: "", label: k };
    const inp = el("input", { value: p.value, placeholder: k === "d" ? "0.5575" : k === "ik" ? "비우면 기본" : "", inputmode: k === "ik" ? null : "decimal" });
    c.fields[k] = inp;
    return el("label", { class: "field", title: `${p.label}${p.hint ? " (" + p.hint + ")" : ""}`, style: w ? `width:${w}` : null },
      el("span", {}, lab, unit ? el("small", {}, " " + unit) : null), inp);
  };
  c.chip = el("span", { class: "chip" });
  c.vb = el("span", { class: "vbadge", hidden: true }, "가상");
  c.status = el("div", { class: "note" });
  c.note = el("div", { class: "note" });
  c.cmd = el("div", { class: "cmd", hidden: true });
  c.log = el("pre", { class: "logbox tall" });
  c.logbox = el("details", { class: "log" }, el("summary", {}, "출력 보기"), c.log,
    el("div", { class: "row" }, el("a", { href: `/api/log/${s.id}?n=1000`, target: "_blank", class: "help" }, "로그 1000줄 보기")));
  const params = () => Object.fromEntries(Object.entries(c.fields).map(([k, i]) => [k, i.type === "checkbox" ? (i.checked ? "1" : "") : i.value]));
  // 계획 방식: 기본 = 대시보드 해석 IK (지도와 같은 IK, 물리 없음). 체크 = 팀 계획 (pick_place, MuJoCo 물리 시뮬)
  const pTeam = P0.team_sim || { value: "" };
  c.fields.team_sim = el("input", { type: "checkbox" });
  c.fields.team_sim.checked = /^(1|true|on|yes)$/i.test(String(pTeam.value || ""));
  const teamBox = el("label", { class: "help", style: "display:inline-flex;gap:6px;align-items:center;margin:8px 0 0",
    title: "끄면(기본) 지도와 같은 해석 IK 로 경유점을 풀어 계획 — 물리·쥐기 시뮬 없음. 켜면 팀 계획(team-jax/IK motion1 MuJoCo 물리 시뮬, 자기 IK)" },
    c.fields.team_sim, "팀 시뮬 계획 (물리)");
  c.fields.grasp = el("select", {}, el("option", { value: "auto" }, "자동"), el("option", { value: "vertical" }, "수직"), el("option", { value: "horizontal" }, "수평"), el("option", { value: "tilted" }, "기울임 (어제 방식, 손목 ±90)"));
  c.fields.grasp.value = /^(vertical|horizontal)$/.test((P0.grasp || {}).value || "") ? P0.grasp.value : "auto";
  c.bStart = el("button", { class: "btn", onclick: () => post(`/api/svc/${s.id}/start`, { params: params() }) }, "▶ 계획 돌려 보기");
  c.bStop = el("button", { class: "btn red", onclick: () => post(`/api/svc/${s.id}/stop`) }, "■ 중단");
  c.result = el("div", { class: "result" });
  c.root = el("div", {},
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "1"), "큐브와 놓을 자리"),
    el("div", { class: "card" },
      buildMap(c),
      el("details", { class: "log" }, el("summary", {}, "숫자로 입력"),
        el("div", { class: "frow" }, el("b", {}, "큐브"), fld("cube_x", "x", "m"), fld("cube_y", "y", "m"), fld("cube_yaw", "yaw", "°")),
        el("div", { class: "frow" }, el("b", {}, "놓을 자리"), fld("target_x", "x", "m"), fld("target_y", "y", "m")),
        el("div", { class: "help", style: "margin:8px 0 0" }, "로봇 좌표: 원점 = 어깨, +x = 로봇 왼쪽, 앞 = −y, yaw = 위에서 반시계")),
      el("div", { id: "objpose" }),
      // 책상 높이 (2026-11-23): 어깨 축에서 책상 면까지 아래로 몇 m. 비우면 기본 0.5575 (실물, 2026-11-25 프로파일 8 cm 내림). 지도·계획·HOME·손 위치('책상 위 cm')가 이 높이를 쓴다
      el("div", { class: "frow", title: "어깨 축(shoulder_roll)에서 책상 윗면까지 아래로 잰 거리. 비우면 기본 0.5575 m (실물 책상). 0.44 ~ 0.60 m 만 (책상이 높을수록 = 값이 작을수록 집을 수 있는 곳이 좁다). " +
        "바꾸면 집을 수 있는 곳 지도를 다시 계산하고(약 1분), 계획·HOME 으로 가기·손 위치의 '책상 위 cm' 가 이 높이를 쓴다" },
        el("b", {}, "책상 높이"), fld("d", "어깨 축 아래", "m"),
        el("span", { class: "help", style: "padding-bottom:10px" }, "비우면 0.5575 · 0.44~0.60")),
      // 집는 방향 (2026-11-24): 자동 = 수직 → 수평 → 기울임 순서로 되는 것. 지도(집을 수 있는 곳)·확인·계획이 이 방향을 쓴다
      el("div", { class: "frow", title: "자동: 수직(위에서)이 되면 수직, 안 되는 자리만 수평(옆에서), 그것도 안 되면 손목이 되는 만큼 기울여서. " +
        "수직: 손 축이 수직에서 5° 안, 위에서 곧게. 수평: 손을 눕혀 큐브 옆 4~6 cm 에서 밀어 넣음 — 큐브 면이 팔(어깨) 쪽을 볼 때만 되고, 지도는 지금 큐브 yaw 기준" },
        el("b", {}, "집는 방향"), el("label", { class: "field" }, el("span", {}, "방향"), c.fields.grasp),
        el("span", { class: "help", style: "padding-bottom:10px" }, "자동 = 수직 먼저")),
      el("details", { class: "log" }, el("summary", {}, "고급"),
        el("div", { class: "frow" }, fld("max_speed", "max_speed", "°/s")),
        fld("ik", "IK 경로 (team-jax/IK)", ""),
        el("p", { class: "help", style: "margin-top:8px" }, `[${s.group}] ${s.help}`), c.cmd)),
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "2"), "계획 돌려 보기"),
    el("div", { class: "card" },
      el("div", { class: "row", style: "align-items:center;margin-top:0" }, c.bStart, c.bStop, c.vb, c.chip),
      teamBox,
      el("div", { class: "help", style: "margin:8px 0 0" }, "계획·검사만 한다 — 모터는 안 움직인다. 기본 계획은 지도와 같은 IK (물리·쥐기 시뮬 없음)."),
      c.result, c.status, c.note, c.logbox));
  for (const k of ["cube_x", "cube_y", "cube_yaw", "target_x", "target_y", "d"]) c.fields[k].addEventListener("input", () => mapUpdate());
  // 책상 높이를 바꾸면 지도·비전 큐브 판정·손 위치를 그 높이로 다시 (치는 중엔 기다렸다가)
  c.fields.d.addEventListener("input", () => { clearTimeout(c.tD); c.tD = setTimeout(() => { G.visKey = null; graspLoad(); if (RX.on) reachLoad(); }, 700); });
  // 집는 방향을 바꾸면 지도·비전 큐브 판정을 그 방향으로 다시. 수평 지도는 큐브 yaw 기준이라 yaw 를 바꿔도 다시
  const regrasp = (ms) => { clearTimeout(c.tG); c.tG = setTimeout(() => { G.visKey = null; graspLoad(); mapUpdate(); }, ms); };
  c.fields.grasp.addEventListener("change", () => regrasp(0));
  c.fields.cube_yaw.addEventListener("input", () => { if (graspMode() === "horizontal") regrasp(700); });
  setTimeout(() => mapUpdate(), 0);
  c.updateExtra = (sv) => {
    c.bStop.hidden = !sv.alive;
    c.chip.hidden = !sv.alive && sv.available;   // 끝난 뒤엔 결과 줄이 대신 말한다 (코드 1 = 거절이지 고장이 아님)
    for (const inp of Object.values(c.fields)) inp.disabled = sv.alive;
    const [txt, cls] = pickSummary(sv);
    c.result.textContent = txt; c.result.className = "result " + cls; c.result.hidden = !txt;
    c.status.hidden = !sv.alive;
  };
  return c;
}
const JNAME = (n) => (JLABEL[n] || /^finger/.test(n)) ? `${jLabel(n)}(${n})` : n;
function lastRun(s) {
  const lines = s.log || [];
  let k = -1;
  for (let i = lines.length - 1; i >= 0; i--) if (/^===== .* 켜기: /.test(lines[i])) { k = i; break; }
  return k >= 0 ? lines.slice(k) : (s.exit_code == null ? [] : lines);
}
// 칸 값이 지금 계획(T.traj.real = 계획에 쓴 값)과 다른가 — 다르면 재생을 막는다 (화면에 보이는 값 ≠ 재생될 계획)
function planStale() {
  const r = T.traj && T.traj.real, c = M.c;
  if (!r || !c || !Array.isArray(r.cube) || !Array.isArray(r.target)) return false;
  const far = (k, v, tol) => v != null && !(Math.abs(parseFloat(c.fields[k].value) - v) <= tol);
  return far("cube_x", r.cube[0], 0.0005) || far("cube_y", r.cube[1], 0.0005) || far("cube_yaw", r.cube[2], 0.05) ||
    far("target_x", r.target[0], 0.0005) || far("target_y", r.target[1], 0.0005) || (r.d != null && Math.abs(tableDNum() - r.d) > 0.0005) ||
    (r.grasp != null && r.grasp !== graspMode());
}
function pickSummary(s) {
  if (s.alive) return ["⏳ 계획 중… (보통 10~30초)", "run"];
  // (새 계획이 막 끝나 궤적을 아직 다시 읽는 중이면 판단하지 않는다 — 옛 궤적과 비교하게 됨)
  if (planStale() && (T.info || {}).mtime === S.traj_mtime) return ["값이 바뀜 — '계획 돌려 보기'를 다시", "warn"];
  const lines = lastRun(s);
  if (!lines.length || s.exit_code == null) return [s.available ? "아직 안 돌림 — 위 칸을 확인하고 '계획 돌려 보기'" : (s.why || ""), ""];
  const t = T.traj, an = t && t.planner === "analytic";
  const extra = t ? ` (${an ? "IK 계획" : "시뮬"} ${(t.positions.length * t.dt).toFixed(1)}초${t.final_error_m != null ? ` · 놓은 오차 ${(t.final_error_m * 1000).toFixed(1)} mm` : ""}` +
    `${an && t.checks ? ` · 잡을 때 집는 점 오차 ${t.checks.grasp_grip_error_mm} mm · 물리 시뮬 없음` : ""})` : "";
  const fmt = (x) => `${Math.round(x)}°`.replace("-", "−");
  if (s.exit_code === 0) return ["✅ 통과: 계획·검사 모두 OK" + (an && t.grasp && t.grasp.text ? ` · ${t.grasp.text}` : "") + extra, "ok"];
  if (s.exit_code === 3) {
    const r = lines.find((l) => /^refused: /.test(l));
    return ["⛔ 팔이 못 닿음" + (r ? ": " + r.replace(/^refused: /, "") : ""), "bad"];
  }
  if (s.exit_code === 1) {
    const reasons = [];
    for (const l of lines) {
      let m = /^check: (\w+) goes ([-\d.]+)\.\.([-\d.]+) deg but the calibration allows ([-\d.]+)\.\.([-\d.]+)/.exec(l);
      if (m) {
        const [lo, hi, alo, ahi] = [m[2], m[3], m[4], m[5]].map(parseFloat);
        const out = lo < alo - 0.05 ? lo : hi;
        reasons.push(`${JNAME(m[1])} 이 ${fmt(out)}까지 가는데 허용 범위는 ${fmt(alo)}~${fmt(ahi)}`);
        continue;
      }
      m = /^refused: (?!fix the above)(.*)/.exec(l);
      if (m) { reasons.push((t && t.planner === "analytic" ? "계획 검사 실패: " : "시뮬 실패: ") + m[1]); continue; }
      m = /^check: (?!calibration not verified)(.*)/.exec(l);
      if (m) reasons.push(m[1]);
    }
    const unv = lines.find((l) => /^check: calibration not verified/.test(l));
    const nUnv = unv ? unv.split(":").pop().split(",").filter((x) => x.trim()).length : 0;
    // 계획 자체는 통과, 남은 건 AK45 보정뿐 → 가상 재생은 되므로 통과로 보여 줌 (실물 재생 때만 보정 필요, 2026-11-21)
    if (!reasons.length && nUnv) return [`✅ 계획 OK${extra}` + (VIRT() ? "" : ` — 실물 재생 전 AK45 보정 확인 필요 (${nUnv}개)`), "ok"];
    if (nUnv) reasons.push(`AK45 보정 미확인 ${nUnv}개`);
    return ["⛔ 거절: " + (reasons.join(" · ") || "출력 보기 참고") + (t ? " — 궤적은 오른쪽 3D 에서 볼 수 있음" : ""), "bad"];
  }
  if (s.exit_code === 5) return ["■ 중간에 멈춤", "warn"];
  const err = [...lines].reverse().find((l) => /Error|error|Traceback/.test(l));
  return [`⚠ 실행 실패 (코드 ${s.exit_code})${err ? ": " + err.slice(0, 160) : ""} — 출력 보기`, "bad"];
}

// ---------- 역기구학 탭: 2D 지도 (위에서 본 책상: 로봇 = 위, 앞(책상) = 아래, 로봇 왼쪽 = 화면 오른쪽) ----------
// 판정 = POST /api/ik/pair (실제 IK: 같은 파지로 집기·놓기 둘 다 풀리나 — 도달 맵 안 씀, 2026-11-20 사용자 "도달 지도 만들지 마라").
// 격자 = GET /api/reachmap?yaw= 의 칸(칸 중심에 붙이기용). 옛 도달 맵 색칠·판정(realcoords.plan)은 '옛 도달 지도 보기'(기본 끔)로만.
// 지도는 cube_x/y/yaw·target_x/y 칸을 쓰고 읽는다(양방향). '계획 돌려 보기'는 그 칸 값 그대로.
const M = { mode: "cube", snap: true, data: null, yawKey: null, seq: 0, chkSeq: 0, c: null, old: false };
const MS = 1000, MX0 = -0.28, MX1 = 0.38, MY0 = -0.45, MY1 = 0.075, ML = 36, MT = 8, MR = 10, MB = 52;   // 1 m = 1000 단위
const MW = ML + (MX1 - MX0) * MS + MR, MH = MT + (MY1 - MY0) * MS + MB;
const msx = (x) => ML + (x - MX0) * MS, msy = (y) => MT + (MY1 - y) * MS;
const SVGNS = "http://www.w3.org/2000/svg";
function sv(tag, attrs = {}, ...kids) {
  const e = document.createElementNS(SVGNS, tag);
  for (const [k, v] of Object.entries(attrs)) if (v !== undefined && v !== null) e.setAttribute(k, v);
  for (const k of kids) e.append(k.nodeType ? k : document.createTextNode(String(k)));
  return e;
}
const foldYaw = (d) => ((((d + 45) % 90) + 90) % 90) - 45;
const cm = (m) => { const v = Math.round(m * 1000) / 10; return (v > 0 ? "+" : v < 0 ? "−" : "") + Math.abs(v).toFixed(1); };
const degS = (d) => `${d > 0 ? "+" : d < 0 ? "−" : ""}${Math.abs(Math.round(d))}°`;
// 책상 높이 칸 (어깨 축 아래 m): tableD() = 서버에 보낼 글자 그대로 (빈 값 = 기본, 범위 검사는 서버 hand_ik.table_d),
// tableDNum() = 화면 계산용 숫자 (비었거나 숫자가 아니면 기본 0.5575 = 서버 hand_ik.TABLE_D_DEFAULT)
function tableD() { const f = M.c && M.c.fields.d; return f ? f.value.trim() : ""; }
function tableDNum() { const v = parseFloat(tableD()); return Number.isFinite(v) ? v : 0.5575; }
// 집는 방향 칸: auto / vertical / horizontal. graspQ() = 지도·확인 요청에 붙일 값 (수평은 지금 큐브 yaw 기준 — 큐브 면이 팔 쪽을 볼 때만 되므로)
function graspMode() { const f = M.c && M.c.fields.grasp; return f ? f.value : "auto"; }
function graspQ() {
  const g = graspMode(), y = M.c ? parseFloat(M.c.fields.cube_yaw.value) : NaN;
  return g === "horizontal" && Number.isFinite(y) ? { grasp: g, yaw: y } : { grasp: g };
}
const GRASP_KO = { auto: "자동", vertical: "수직", horizontal: "수평", tilted: "기울임" };
function mapVals() {
  const c = M.c; if (!c) return {};
  const n = (k) => { const v = parseFloat(c.fields[k] && c.fields[k].value); return Number.isFinite(v) ? v : null; };
  const cube = n("cube_x") != null && n("cube_y") != null ? { x: n("cube_x"), y: n("cube_y"), yaw: n("cube_yaw") ?? 0 } : null;
  const target = n("target_x") != null && n("target_y") != null ? { x: n("target_x"), y: n("target_y") } : null;
  return { cube, target, d: n("d") };
}
function buildMap(c) {
  M.c = c;
  const seg = el("div", { class: "modesw seg", role: "group", "aria-label": "고르는 것" });
  M.bCube = el("button", { onclick: () => mapMode("cube") }, el("i", { class: "sw cube" }), "큐브 놓기");
  M.bTarget = el("button", { onclick: () => mapMode("target") }, el("i", { class: "sw tgt" }), "놓을 자리 고르기");
  M.bHand = el("button", { onclick: () => mapMode("hand"), title: "아래 '손 위치로 보내기' 의 x, y 를 지도에서 고르기" }, el("i", { class: "sw hand" }), "손 위치");
  seg.append(M.bCube, M.bTarget, M.bHand);
  const snap = el("input", { type: "checkbox", checked: "" });
  snap.addEventListener("change", () => { M.snap = snap.checked; });
  const old = el("input", { type: "checkbox" });
  old.addEventListener("change", () => { M.old = old.checked; mapPaint(); mapUpdate(); });
  M.yaw = el("input", { type: "range", min: -45, max: 45, step: 1, value: 0 });
  M.yawNum = el("span", { class: "num" });
  M.yaw.addEventListener("input", () => {
    if (c.fields.cube_yaw.disabled) return;
    c.fields.cube_yaw.value = M.yaw.value; mapUpdate();
  });
  // 정적 그림: 눈금, 로봇
  const g0 = sv("g");
  for (let x = -0.2; x <= 0.301; x += 0.1) {
    const X = msx(x).toFixed(1);
    g0.append(sv("line", { x1: X, y1: MT, x2: X, y2: MH - MB, class: "mg" }), sv("text", { x: X, y: MH - MB + 22, "text-anchor": "middle", class: "mt" }, cm(x).replace(".0", "")));
  }
  for (let y = -0.4; y <= 0.001; y += 0.1) {
    const Y = msy(y).toFixed(1);
    g0.append(sv("line", { x1: ML, y1: Y, x2: MW - MR, y2: Y, class: "mg" }), sv("text", { x: ML - 5, y: (+Y + 4).toFixed(1), "text-anchor": "end", class: "mt" }, cm(y).replace(".0", "")));
  }
  g0.append(sv("text", { x: ML, y: MH - 6, class: "mt" }, "cm  ·  ← 로봇 오른쪽"), sv("text", { x: MW - MR, y: MH - 6, "text-anchor": "end", class: "mt" }, "로봇 왼쪽 →"));
  const ox = msx(0), oy = msy(0);
  const gR = sv("g", { class: "mrobot" },
    sv("rect", { x: ox - 70, y: MT, width: 140, height: oy - MT + 6, rx: 10, class: "mbody" }),
    sv("circle", { cx: ox, cy: oy, r: 8, class: "msh" }),
    sv("text", { x: ox, y: MT + 22, "text-anchor": "middle", class: "mlab" }, "로봇"),
    sv("text", { x: ox + 14, y: oy + 4, class: "mt" }, "어깨 (0,0)"),
    sv("path", { d: `M${ox - 92},${oy - 22} v30 m-6,-8 l6,9 l6,-9`, class: "mfront" }),
    sv("text", { x: ox - 104, y: oy - 2, "text-anchor": "end", class: "mt" }, "앞 (책상)"));
  M.gCells = sv("g");
  M.gReach = sv("g");   // 갈 수 있는 곳 (손 패널 높이·기준점에서 위치 IK 가 풀림, GET /api/ik/reachxyz): 초록
  M.gGrasp = sv("g");   // 집을 수 있는 곳 (해석 IK, GET /api/ik/graspmap): 파랑 = 됨, 회색 = 안 됨
  M.gHover = sv("rect", { class: "mhl", visibility: "hidden", rx: 3 });
  M.gVis = sv("rect", { class: "mvis", width: 50, height: 50, x: -25, y: -25, rx: 4, visibility: "hidden" });
  M.gArrow = sv("line", { class: "marrow", "marker-end": "url(#marr)", visibility: "hidden" });
  M.gTgt = sv("rect", { class: "mtgt", width: 50, height: 50, x: -25, y: -25, rx: 4, visibility: "hidden" });
  M.gHand = sv("g", { visibility: "hidden", class: "mhand" }, sv("circle", { r: 14 }), sv("path", { d: "M-22,0 H22 M0,-22 V22" }));
  M.gCube = sv("g", { visibility: "hidden" }, sv("rect", { class: "mcube", width: 50, height: 50, x: -25, y: -25, rx: 5 }),
    sv("line", { x1: 0, y1: 0, x2: 25, y2: 0, class: "mcubeax" }));
  const defs = sv("defs", {}, sv("marker", { id: "marr", viewBox: "0 0 10 10", refX: 8, refY: 5, markerWidth: 7, markerHeight: 7, orient: "auto-start-reverse" },
    sv("path", { d: "M0,0 L10,5 L0,10 z", fill: "#4e5968" })));
  M.svg = sv("svg", { viewBox: `0 0 ${MW.toFixed(0)} ${MH.toFixed(0)}`, class: "rmap", role: "img", "aria-label": "책상 위에서 본 도달 지도" },
    defs, g0, M.gGrasp, M.gReach, M.gCells, gR, M.gHover, M.gVis, M.gArrow, M.gTgt, M.gCube, M.gHand);
  M.svg.addEventListener("mousemove", (e) => mapHover(mapPt(e)));
  M.svg.addEventListener("mouseleave", () => mapHover(null));
  M.svg.addEventListener("click", (e) => mapClick(mapPt(e)));
  // [손 위치] 모드: 지도에서 끌면 손 x, y 가 따라가며 빠른 풀이, 놓으면 전체 풀이
  M.svg.addEventListener("pointerdown", (e) => { if (M.mode !== "hand") return; M.drag = true; M.dragMoved = false; M.svg.setPointerCapture(e.pointerId); });
  M.svg.addEventListener("pointermove", (e) => { if (!M.drag) return; const p = mapPt(e); M.dragMoved = true; hkSetXY(p.x, p.y, true); });
  M.svg.addEventListener("pointerup", () => { if (!M.drag) return; M.drag = false; if (M.dragMoved) hkEdit(false); });
  M.hover = el("div", { class: "mhover" }, " ");
  M.guide = el("div", { class: "mguide", hidden: true });
  M.res = el("div", { class: "result" }, "지도 읽는 중…");
  M.err = el("div", { class: "note warn", hidden: true });
  G.box3d = el("input", { type: "checkbox", checked: "" });
  G.box3d.addEventListener("change", () => { if (VW.ik && VW.ik.setGraspVisible) VW.ik.setGraspVisible(G.box3d.checked); });
  G.note = el("span", { class: "help", style: "margin:0" }, "집을 수 있는 곳 계산 중…");
  G.wide = el("input", { type: "checkbox", checked: "" });
  G.wide.addEventListener("change", () => { if (G.wide.checked && !Object.keys(G.other).length) graspLoad(); else graspPaint(); });
  RX.chk = el("input", { type: "checkbox" });
  RX.chk.addEventListener("change", () => { RX.on = RX.chk.checked; M.gGrasp.style.display = RX.on ? "none" : ""; reachLoad(); });
  RX.note = el("span", { class: "help", style: "margin:0" }, "");
  mapMode("cube");
  setTimeout(graspLoad, 0);
  return el("div", { class: "mapbox" },
    el("div", { class: "lead", style: "margin:0 0 10px" }, "지도에서 ", el("b", {}, "큐브"), " → ", el("b", {}, "놓을 자리"), " 순서로 누르세요."),
    el("div", { class: "mctl" }, seg, el("label", { class: "chk" }, snap, "칸 중심에 붙이기"),
      el("label", { class: "chk", title: "−40° 고정 파지로 만든 옛 reachmap_real_yaw.npz 색칠·판정 (다시 만들지 않음)" }, old, "옛 도달 지도 보기")),
    el("div", { class: "mctl" }, el("span", { class: "mlabel" }, "큐브 yaw"), M.yaw, M.yawNum),
    el("div", { class: "mlegend" }, el("span", {}, "집을 수 있음 · 손 기울기"), el("span", {}, "0° 수직", el("i", { class: "sw tiltbar" }), "90° 수평"),
      el("span", {}, el("i", { class: "sw ok gother" }), "다른 방향이면 됨"), el("label", { class: "chk", title: "지금 고른 집는 방향으로는 안 되지만 자동·기울임·수평 중 하나로 되는 칸을 옅게 (방향마다 지도를 한 번씩 더 계산, 서버가 기억)" }, G.wide, "다른 방향도"),
      el("span", {}, el("i", { class: "sw no" }), "못 집음"),
      el("label", { class: "chk", title: "3D 뷰 책상에 집을 수 있는 곳을 반투명 초록으로" }, G.box3d, "3D 에 초록으로"), G.note),
    el("div", { class: "mlegend" }, el("label", { class: "chk", title: "집기 조건 없이: 아래 '손 위치로 보내기'의 높이·기준점으로 x, y 마다 같은 역기구학(위치만)이 풀리면 초록" },
      RX.chk, "갈 수 있는 곳 보기 (손 위치 높이에서 IK 풀림)"), el("span", {}, el("i", { class: "sw reach" }), "갈 수 있음"), RX.note),
    el("div", { class: "mlegend" }, el("span", {}, "판정은 실제 IK (아래 결과 줄) · 옛 지도 보기를 켜면 "), el("span", {}, el("i", { class: "sw ok" }), "옛 맵 됨"),
      el("span", {}, el("i", { class: "sw no" }), "옛 맵 안 됨"), el("span", {}, el("i", { class: "sw cube" }), "큐브"), el("span", {}, el("i", { class: "sw tgt" }), "놓을 자리"),
      el("span", {}, el("i", { class: "sw vis" }), "비전")),
    M.guide, el("div", { class: "mwrap" }, M.svg), M.hover, M.res, M.err);
}
function mapMode(m) {
  M.mode = m;
  M.bCube.className = m === "cube" ? "on" : ""; M.bTarget.className = m === "target" ? "on" : ""; M.bHand.className = m === "hand" ? "on" : "";
  mapPaint();
}
// 파지 = {hand_yaw, flip}: flip = 손 반 바퀴(HAND_FLIP). 한 배달은 한 파지로 끝까지 가므로(env._servable)
// 큐브 칸과 놓을 자리 칸에 같은 파지가 있어야 됨(realcoords.plan).
const gKey = (g) => `${g.hand_yaw}|${g.flip}`;
const gHand = (g) => (g.flip === true ? "손 반 바퀴" : g.flip === false ? "손 그대로" : "손");
const gName = (g) => `${gHand(g)}, ${degS(g.hand_yaw)}`;
function gKinds(gs) {   // 파지 목록 -> "반 바퀴 손으로만" 처럼
  const p = gs.some((g) => g.flip === false), f = gs.some((g) => g.flip === true);
  return p && f ? "손 그대로·반 바퀴 둘 다로" : f ? "반 바퀴 손으로만" : p ? "그대로 손으로만" : "어떤 손으로든";
}
function cubeCell() { const v = mapVals(); return v.cube ? mapCell(v.cube) : null; }
// 큐브를 찍은 뒤 '놓을 자리 고르기'면 그 큐브를 잡을 수 있는 파지로 놓기가 되는 칸만 파랗게, 나머지는 흐리게.
function mapPaint() {
  const d = M.data; if (!d || !M.rects || !M.guide) return;
  if (!M.old) {   // 옛 맵 끔: 격자만
    for (const r of M.rects) r.setAttribute("class", "rc grid");
    M.guide.hidden = true; return;
  }
  const v = mapVals(), cc = v.cube ? mapCell(v.cube) : null;
  const pick = M.mode === "target" && v.cube;
  const keys = pick && cc ? new Set(cc.grasps.map(gKey)) : null;
  d.cells.forEach((c, n) => {
    let cls;
    if (pick) cls = keys && c.grasps.some((g) => keys.has(gKey(g))) ? "ok" : "dim";
    else cls = c.feasible ? "ok" : "no";   // 파랑 = 큐브를 놓아도 되는 칸 하나만 (연파랑·진파랑 구분은 헷갈려서 뺌, 2026-11-20)
    M.rects[n].setAttribute("class", "rc " + cls);
  });
  let txt = "", bad = false;
  if (pick) {
    if (!cc) { txt = "큐브가 지도 밖이라 놓을 자리를 칠할 수 없음"; bad = true; }
    else if (!cc.grasps.length) { txt = "이 큐브 자리는 어느 손 모양으로도 집고 놓기가 안 됨 — 큐브를 파란 칸으로 옮기세요"; bad = true; }
    else {
      const k = gKinds(cc.grasps), n = d.cells.filter((c) => c.grasps.some((g) => keys.has(gKey(g)))).length;
      const head = k === "반 바퀴 손으로만" ? "이 큐브는 손을 반 바퀴 돌려 잡아야 해서" : k === "그대로 손으로만" ? "이 큐브는 손을 그대로 두고 잡아야 해서" :
        k === "손 그대로·반 바퀴 둘 다로" ? "이 큐브는 손 그대로·반 바퀴 둘 다로 잡을 수 있어" : "이 큐브를 잡는 손 모양으로";
      txt = `${head}, 놓을 수 있는 곳은 파란 칸만 (${n}칸 · ${cc.grasps.map(gName).join(" / ")})`;
    }
  }
  M.guide.hidden = !txt; M.guide.textContent = txt; M.guide.className = "mguide" + (bad ? " bad" : "");
}
function mapPt(e) {
  const p = M.svg.createSVGPoint(); p.x = e.clientX; p.y = e.clientY;
  const q = p.matrixTransform(M.svg.getScreenCTM().inverse());
  return { x: MX0 + (q.x - ML) / MS, y: MY1 - (q.y - MT) / MS };
}
function mapCell(p) {
  const d = M.data; if (!d || !p) return null;
  const hx = d.cell.dx / 2 + 1e-9, hy = d.cell.dy / 2 + 1e-9;
  return d.cells.find((c) => Math.abs(c.x - p.x) <= hx && Math.abs(c.y - p.y) <= hy) || null;
}
function cellText(c) {
  if (!c) return "지도 밖 (판정 칸 없음)";
  const t = c.grasps.length ? ` (${c.grasps.map(gName).join(" 또는 ")})` : "";
  return c.feasible ? "집고 놓기 됨" + t : c.grasp && c.place ? "가장자리 — 집기·놓기는 되지만 옆 칸이 안 돼 여유 없음" :
    c.grasp ? "집기만 됨 (놓기 안 됨)" : c.place ? "놓기만 됨 (집기 안 됨)" : "안 됨 — 팔이 못 감";
}
function mapHover(p) {
  if (!p) { M.gHover.setAttribute("visibility", "hidden"); M.hover.textContent = " "; return; }
  const c = mapCell(p);
  if (c) {
    const w = M.data.cell.dx * MS, h = M.data.cell.dy * MS;
    M.gHover.setAttribute("x", msx(c.x) - w / 2); M.gHover.setAttribute("y", msy(c.y) - h / 2);
    M.gHover.setAttribute("width", w); M.gHover.setAttribute("height", h); M.gHover.setAttribute("visibility", "visible");
  } else M.gHover.setAttribute("visibility", "hidden");
  M.hover.textContent = `x ${cm(p.x)} cm · y ${cm(p.y)} cm — ${p.y < 0 ? `앞으로 ${Math.abs(p.y * 100).toFixed(0)} cm` : "어깨 뒤"}, ` +
    `${p.x >= 0 ? "왼쪽" : "오른쪽"} ${Math.abs(p.x * 100).toFixed(0)} cm` + (RX.on ? reachHover(p) : graspHover(p)) + (M.old ? ` · 옛 맵: ${cellText(c)}` : "");
}
function mapClick(p) {
  const c = M.c;
  if (!p || !c) return;
  if (M.mode === "hand") {
    if (M.dragMoved) { M.dragMoved = false; return; }   // 끌기가 끝난 click: 끈 자리 그대로 (칸 중심에 안 붙임)
    const cell = mapCell(p), at = M.snap && cell ? { x: cell.x, y: cell.y } : p; hkSetXY(at.x, at.y, false); return;
  }
  if (c.fields.cube_x.disabled) { toast("계획 중에는 못 바꿈", "error"); return; }
  const cell = mapCell(p);
  const at = M.snap && cell ? { x: cell.x, y: cell.y } : p;
  const k = M.mode === "cube" ? ["cube_x", "cube_y"] : ["target_x", "target_y"];
  c.fields[k[0]].value = at.x.toFixed(3); c.fields[k[1]].value = at.y.toFixed(3);
  const cc = M.mode === "target" ? cubeCell() : null;
  if (!M.old) {   // 어디든 고를 수 있음 — 판정은 실제 IK (결과 줄)
    mapNext();
    mapUpdate(); return;
  }
  if (M.mode === "cube" && (!cell || !cell.feasible)) {
    toast(`⚠ 그 칸은 ${cell ? "집을 수 없는 칸" : "지도 밖"} — 파란 칸을 누르세요 (그래도 계획 돌려 보기는 됨)`, "warn");
    mapUpdate(); return;   // 놓을 자리 고르기로 안 넘어감: 이 큐브로는 칠할 놓을 자리가 없다
  }
  if (!cell || !cell.feasible) toast(`⚠ 그 자리는 지도상 ${cell ? "안 되는 칸" : "지도 밖"} — 그래도 계획 돌려 보기는 됨`, "warn");
  else if (cc && !cell.grasps.some((g) => cc.grasps.some((h) => gKey(g) === gKey(h))))
    toast("⚠ 그 칸은 이 큐브를 잡는 손 모양으로는 못 놓음 — 그래도 계획 돌려 보기는 됨", "warn");
  mapNext();
  mapUpdate();
}
// 한 번 고르면 다음 단계로: 큐브 놓기 → 놓을 자리 고르기 → 손 위치 (버튼 순서대로. 다시 고르려면 그 버튼을 누른다. 모드만 바뀌고 팔은 안 움직인다)
function mapNext() {
  if (M.mode === "cube") mapMode("target");
  else if (M.mode === "target") { mapMode("hand"); toast("놓을 자리 골랐음 → 이제 '손 위치' 모드 (다시 고르려면 '놓을 자리 고르기')"); }
}
function mapVision(o) {
  if (!M.gVis) return;
  if (!o) { M.gVis.setAttribute("visibility", "hidden"); return; }
  M.gVis.setAttribute("transform", `translate(${msx(o.x).toFixed(1)},${msy(o.y).toFixed(1)}) rotate(${(-o.yaw_deg).toFixed(1)})`);
  M.gVis.setAttribute("class", "mvis" + (G.vis ? (G.vis.graspable ? " gy" : " gn") : ""));
  M.gVis.setAttribute("visibility", "visible");
}
// ---------- 집을 수 있는 곳 (해석 IK) ----------
// 정의(서버 hand_ik.grasp_eval → pick_plan.Planner.point_check, 2026-11-21): '대시보드 계획이 여기서 집기·놓기 가능' —
// 손끝을 큐브 중심에 + 손가락이 책상 위 8 mm 이상 + 큐브 위 3 cm 에서 곧게 내려가는 길(손가락이 큐브에 안 닿음), 큐브 yaw 모두.
// 지도 = 1 cm 격자, 비전 큐브 = 그 점 그대로.
const G = { data: null, seq: 0, vis: null, visKey: null, visBusy: false, other: {} };
// 칸 색 = 그 자리에서 고른 집기 자세의 손 기울기 (서버 d.tilts, 수직 0° → 수평 90°): 진한 파랑 → 보라 → 주황 (2026-11-24 사용자)
const TILT_STOPS = [[0, [29, 78, 216]], [45, [147, 51, 234]], [90, [234, 88, 12]]];
function tiltColor(t) {
  t = Math.max(0, Math.min(90, t));
  const k = t <= 45 ? 0 : 1, [a, ca] = TILT_STOPS[k], [b, cb] = TILT_STOPS[k + 1], u = (t - a) / (b - a);
  return `rgb(${ca.map((v, i) => Math.round(v + (cb[i] - v) * u)).join(",")})`;
}
const TILT_BIN = 5;   // 색 단계 (°) — 같은 단계 칸은 한 path 로
// 다른 집는 방향이면 되는 칸 (옅게): 지금 방향 지도 다음에 하나씩 받는다 (서버가 기억). 수직은 자동에 들어 있어 안 받음
const graspOthers = (g) => ["auto", "tilted", "horizontal"].filter((m) => m !== g);
const graspSy = () => (HK.f && HK.f.fix.checked ? parseFloat(HK.f.fixv.value) || 0 : 0);
async function graspLoad() {
  const my = ++G.seq, sy = graspSy();
  if (G.note) G.note.textContent = "집을 수 있는 곳 계산 중… (처음 한 번 약 1분)";
  try {
    const gq = graspQ();
    const d = await (await fetch(`/api/ik/graspmap?shoulder_yaw=${encodeURIComponent(sy)}&d=${encodeURIComponent(tableD())}&grasp=${gq.grasp}` +
      (gq.yaw != null ? `&yaw=${encodeURIComponent(gq.yaw)}` : ""))).json();
    if (my !== G.seq) return;
    if (!d.ok) { G.data = null; if (G.note) G.note.textContent = "⚠ 집을 수 있는 곳 계산 실패: " + (d.reason || "?"); graspPaint(); return; }
    G.data = d; G.other = {};
    if (G.note) G.note.textContent = `${d.counts.graspable}칸 (1 cm) · ` + (d.grasp && d.grasp !== "auto" ? `집는 방향 ${GRASP_KO[d.grasp]}${d.yaw != null ? ` (큐브 yaw ${d.yaw}°)` : ""} · ` : "") +
      `손끝(집는 점)을 책상 위 ${d.grip_above_table_m != null ? +(d.grip_above_table_m * 100).toFixed(1) : 3.5} cm 에 · 어깨 옆 ${d.shoulder_yaw}°` + (d.d != null && Math.abs(d.d - 0.5575) > 1e-6 ? ` · 책상 높이 ${d.d} m` : "");
    graspPaint();
    if (!G.wide || !G.wide.checked) return;
    const base = G.note.textContent;
    for (const m of graspOthers(d.grasp)) {
      G.note.textContent = base + ` · 다른 방향(${GRASP_KO[m]}) 계산 중…`;
      const o = await (await fetch(`/api/ik/graspmap?shoulder_yaw=${encodeURIComponent(sy)}&d=${encodeURIComponent(tableD())}&grasp=${m}`)).json();
      if (my !== G.seq) return;
      if (o.ok && o.xs.length === d.xs.length && o.ys.length === d.ys.length) { G.other[m] = o; graspPaint(); }
    }
    const n = graspCells().filter((c) => c.other).length;
    G.note.textContent = base + ` · 다른 방향이면 +${n}칸 (옅게)`;
  } catch (e) { if (my === G.seq && G.note) G.note.textContent = "⚠ 집을 수 있는 곳을 못 받음 (서버)"; }
}
function graspCell(i, j) {   // {tilt, mode, other} 또는 null: 지금 방향으로 되면 그 기울기, 아니면 다른 방향 중 가장 수직에 가까운 것
  const d = G.data;
  if (d.rows[j][i] === "2") return { tilt: d.tilts ? d.tilts[j][i] : null, mode: d.grasp, other: false };
  let best = null;
  if (G.wide && G.wide.checked) for (const [m, o] of Object.entries(G.other)) {
    if (o.rows[j][i] !== "2") continue;
    const t = o.tilts ? o.tilts[j][i] : null;
    if (!best || (t != null && (best.tilt == null || t < best.tilt))) best = { tilt: t, mode: m, other: true };
  }
  return best;
}
function graspCells() {
  const d = G.data, out = [];
  if (d) d.rows.forEach((row, j) => { for (let i = 0; i < row.length; i++) { const c = graspCell(i, j); if (c) out.push({ ...c, i, j }); } });
  return out;
}
function graspPaint() {
  const d = G.data;
  try { if (VW.ik && VW.ik.setGraspArea) { VW.ik.setGraspArea(d); VW.ik.setGraspVisible(!G.box3d || G.box3d.checked); } } catch (e) { console.error("3D 지도", e); }   // 3D 가 안 돼도 2D 지도는 칠한다
  if (!M.gGrasp) return;
  if (!d) { M.gGrasp.replaceChildren(); return; }
  const h = d.step * MS, paths = {};   // "b|other" -> path d (기울기 단계·옅음별로 한 path)
  for (const c of graspCells()) {
    const b = c.tilt == null ? -1 : Math.round(c.tilt / TILT_BIN) * TILT_BIN, k = `${b}|${c.other ? 1 : 0}`;
    paths[k] = (paths[k] || "") + `M${(msx(d.xs[c.i]) - h / 2).toFixed(1)},${(msy(d.ys[c.j]) - h / 2).toFixed(1)}h${h.toFixed(1)}v${h.toFixed(1)}h${(-h).toFixed(1)}z`;
  }
  const x0 = msx(d.xs[0]) - h / 2, y0 = msy(d.ys[d.ys.length - 1]) - h / 2;
  M.gGrasp.replaceChildren(
    sv("rect", { x: x0.toFixed(1), y: y0.toFixed(1), width: (d.xs.length * h).toFixed(1), height: (d.ys.length * h).toFixed(1), class: "gno" }),
    ...Object.entries(paths).map(([k, p]) => { const [b, o] = k.split("|").map(Number);
      return sv("path", { d: p, class: o ? "gok gother" : "gok", style: b < 0 ? "" : `fill:${tiltColor(b)}` }); }));
}
function graspAt(p) {   // 격자 칸 코드: 2 됨, 1 손목만 닿음, 0 안 닿음, null 지도 밖
  const d = G.data; if (!d || !p) return null;
  const i = Math.round((p.x - d.xs[0]) / d.step), j = Math.round((p.y - d.ys[0]) / d.step);
  return i >= 0 && j >= 0 && j < d.rows.length && i < d.rows[j].length ? +d.rows[j][i] : null;
}
function graspHover(p) {
  const c = graspAt(p);
  const d = G.data, i = c == null ? 0 : Math.round((p.x - d.xs[0]) / d.step), j = c == null ? 0 : Math.round((p.y - d.ys[0]) / d.step);
  const g = c == null ? null : graspCell(i, j), tt = g && g.tilt != null ? ` · 손 기울기 ${g.tilt}° (${g.tilt <= 15 ? "수직에 가까움" : g.tilt >= 75 ? "수평에 가까움" : "기울여"})` : "";
  if (g && g.other) return ` · ⚪ 지금 방향으론 못 집음 — ${GRASP_KO[g.mode]}(으)로는 됨` + tt;
  return c == null ? "" : c === 2 ? " · ✅ 집을 수 있음" + tt : c === 1 ? " · ❌ 못 집음 (손이 기울어 손가락이 책상에 닿거나, 곧게 내려가는 길 없음)" : " · ❌ 못 집음 (팔이 안 닿음)";
}
async function graspVision(o) {   // 비전 큐브 자리 그대로 판정 (격자 칸 아님). 1 mm·어깨 옆 값이 바뀌면 다시.
  if (!o) { G.vis = null; G.visKey = null; return; }
  const gq = graspQ();
  const key = `${o.x.toFixed(3)},${o.y.toFixed(3)},${graspSy()},${tableD()},${gq.grasp},${gq.yaw}`;
  if (key === G.visKey || G.visBusy) return;
  G.visBusy = true;
  try {
    const r = await (await fetch("/api/ik/graspable", { method: "POST", headers: { "content-type": "application/json" },
      body: JSON.stringify({ x: o.x, y: o.y, shoulder_yaw: graspSy(), d: tableD(), ...gq }) })).json();
    G.visKey = key; G.vis = r.ok ? r : null;
  } catch (e) { G.vis = null; }
  G.visBusy = false;
  renderPose(); updateIkView();
}
async function mapLoad(yaw) {
  const my = ++M.seq;
  try {
    const r = await fetch(`/api/reachmap?yaw=${encodeURIComponent(yaw)}`);
    const d = await r.json();
    if (my !== M.seq) return;
    if (!d.ok) { M.data = null; M.gCells.replaceChildren(); M.err.hidden = false; M.err.textContent = "⚠ " + (d.error || "도달 맵 없음") + " — 지도 색 없이 고를 수만 있음"; return; }
    M.err.hidden = true;
    M.data = d;
    const w = d.cell.dx * MS - 2, h = d.cell.dy * MS - 2;
    for (const c of d.cells) c.grasps = c.grasps || c.turns.map((t) => ({ hand_yaw: t, flip: null }));   // 옛 서버
    M.rects = d.cells.map((c) => sv("rect", {
      x: (msx(c.x) - w / 2).toFixed(1), y: (msy(c.y) - h / 2).toFixed(1), width: w.toFixed(1), height: h.toFixed(1), rx: 3, class: "rc no" }));
    M.gCells.replaceChildren(...M.rects);
    mapPaint();
  } catch (e) { if (my === M.seq) { M.err.hidden = false; M.err.textContent = "⚠ 도달 맵을 못 받음 (서버)"; } }
}
function mapUpdate() {
  const v = mapVals();
  if (!M.svg) return;
  const place = (g, p, yaw) => {
    if (!p) { g.setAttribute("visibility", "hidden"); return; }
    g.setAttribute("transform", `translate(${msx(p.x).toFixed(1)},${msy(p.y).toFixed(1)})` + (yaw ? ` rotate(${(-yaw).toFixed(1)})` : ""));
    g.setAttribute("visibility", "visible");
  };
  place(M.gCube, v.cube, v.cube && v.cube.yaw);
  place(M.gTgt, v.target, v.cube && v.cube.yaw);
  if (v.cube && v.target) {
    const x1 = msx(v.cube.x), y1 = msy(v.cube.y), x2 = msx(v.target.x), y2 = msy(v.target.y), L = Math.hypot(x2 - x1, y2 - y1);
    if (L > 40) {
      const ux = (x2 - x1) / L, uy = (y2 - y1) / L, a = 30, b = 30;
      for (const [k, val] of [["x1", x1 + ux * a], ["y1", y1 + uy * a], ["x2", x2 - ux * b], ["y2", y2 - uy * b]]) M.gArrow.setAttribute(k, val.toFixed(1));
      M.gArrow.setAttribute("visibility", "visible");
    } else M.gArrow.setAttribute("visibility", "hidden");
  } else M.gArrow.setAttribute("visibility", "hidden");
  const yaw = v.cube ? v.cube.yaw : 0, fy = foldYaw(yaw);
  if (document.activeElement !== M.yaw) { M.yaw.value = Math.round(fy); rangeFill(M.yaw); }
  M.yawNum.textContent = `${degS(yaw)}${Math.abs(yaw - fy) > 0.05 ? ` (= ${degS(fy)}, 큐브는 90° 마다 같음)` : ""}`;
  M.yaw.disabled = M.c.fields.cube_yaw.disabled;
  const key = (Math.round(yaw * 2) / 2).toFixed(1);
  if (key !== M.yawKey) {
    M.yawKey = key;
    clearTimeout(M.tLoad); M.tLoad = setTimeout(() => mapLoad(yaw), 80);
  }
  mapPaint();
  clearTimeout(M.tChk); M.tChk = setTimeout(mapCheck, 150);
}
async function mapCheck() {
  const v = mapVals();
  if (!v.cube || !v.target) { M.res.textContent = "큐브와 놓을 자리를 지도에서 누르세요"; M.res.className = "result"; return; }
  if (!M.old) return mapCheckIk(v);
  const my = ++M.chkSeq;
  let r;
  try {
    r = await (await fetch("/api/reachmap/check", { method: "POST", headers: { "content-type": "application/json" },
      body: JSON.stringify({ cube: [v.cube.x, v.cube.y, v.cube.yaw], target: [v.target.x, v.target.y], d: v.d }) })).json();
  } catch (e) { r = { ok: false, error: "서버 연결 실패" }; }
  if (my !== M.chkSeq) return;
  const [txt, cls] = checkText(r);
  M.res.textContent = txt; M.res.className = "result " + cls;
  M.res.title = r.ok ? `realcoords.plan: ${r.reason} · sim 큐브 (${r.cube.slice(0, 2).map((x) => x.toFixed(3)).join(", ")}) yaw ${degS(r.yaw)} → 놓을 자리 (${r.target.map((x) => x.toFixed(3)).join(", ")})` : "";
}
// 실제 IK 판정: /api/ik/pair. 손 위치 카드의 어깨 옆 고정·기울기 상한을 그대로 쓴다.
async function mapCheckIk(v) {
  const my = ++M.chkSeq;
  M.res.textContent = "⏳ IK 로 판정 중…"; M.res.className = "result run";
  const hv = typeof hkVals === "function" ? hkVals() : null;
  const t0 = performance.now();
  let r;
  try {
    r = await (await fetch("/api/ik/pair", { method: "POST", headers: { "content-type": "application/json" },
      body: JSON.stringify({ cube: [v.cube.x, v.cube.y, v.cube.yaw], target: [v.target.x, v.target.y],
        fix_deg: hv ? hv.fix_deg : {}, tilt_max_deg: hv ? hv.tilt_max_deg : 60, d: tableD(), grasp: graspMode() }) })).json();
  } catch (e) { r = { ok: false, reason: "서버 연결 실패" }; }
  if (my !== M.chkSeq) return;
  const ms = Math.round(performance.now() - t0);
  const fx = r.fixed && Object.keys(r.fixed).length ? ` · 어깨 옆 ${Object.values(r.fixed).map((x) => degS(x)).join(",")} 고정` : "";
  if (!r.ok) { M.res.textContent = "⚠ IK 판정 못 함: " + (r.reason || "?"); M.res.className = "result warn"; return; }
  if (r.grasp_mode) {   // 집는 방향을 골랐을 때 (수직·수평): 계획과 같은 판정 (pick_plan.point_check), 그 방향으로
    const ko = GRASP_KO[r.grasp_mode] || r.grasp_mode;
    M.res.textContent = r.both ? `✅ IK: ${ko}으로 집기·놓기 둘 다 됨${fx} · ${ms} ms — 이제 ② 계획 돌려 보기` : `⛔ IK (${ko}): ${r.why || "안 됨"}${fx} · ${ms} ms`;
    M.res.className = "result " + (r.both ? "ok" : "bad");
    M.res.title = "집는 방향을 고르면 계획과 같은 판정 (pick_plan.point_check) 을 큐브 자리·놓을 자리에 그 큐브 yaw 로";
    return;
  }
  if (r.both) {
    const g = r.grasp;
    M.res.textContent = `✅ IK: 집기·놓기 둘 다 됨 (${g.flip ? "손 반 바퀴" : "손 그대로"}, 손 돌림 ${degS(g.hand_turn)}, 기울기 ${degS(g.tilt_deg)})${fx} · ${ms} ms — 이제 ② 계획 돌려 보기`;
    M.res.className = "result ok";
  } else {
    const why = !r.pick_any ? (r.place_alone ? "큐브를 집는 자세가 안 풀림 (놓을 자리는 됨)" : "큐브 집기·놓을 자리 둘 다 안 풀림")
      : "집기는 되지만 같은 손 모양·기울기로 놓을 자리가 안 풀림";
    M.res.textContent = `⛔ IK: ${why}${fx} · ${ms} ms — 그래도 계획 돌려 보기는 됨 (시뮬이 다시 확인)`;
    M.res.className = "result bad";
  }
  M.res.title = "실제 IK (hand_ik.pair): 파지마다 집기 = 기울기 자유 풀이, 놓기 = 같은 기울기 6D. 집는 점 = 큐브 중심 +1 cm = 책상 위 3.5 cm";
}
function checkText(r) {
  if (!r.ok) return ["⚠ 판정 못 함: " + (r.error || "?"), "warn"];
  if (r.reachable) return [`✅ 됨 (${gHand(r)}, ${degS(r.hand_yaw)}) — 이제 ② 계획 돌려 보기`, "ok"];
  const tail = " — 그래도 계획 돌려 보기는 됨 (시뮬이 다시 확인)";
  if (/no reach map for D/.test(r.reason)) return [`⚠ d(어깨~책상) ${r.d} m 가 지도(${r.map_d} m)와 달라 판정 못 함` + tail, "warn"];
  const a = r.alone || {}, cu = a.cube || {}, tg = a.target || {};
  const out = (x) => /outside/.test(x.reason || "");
  let why;
  if (!cu.reachable && !tg.reachable) why = out(cu) && out(tg) ? "둘 다 지도 밖" : "팔이 큐브 자리·놓을 자리 둘 다 못 감";
  else if (!cu.reachable) why = out(cu) ? "큐브가 지도 밖" : "팔이 큐브 자리에 못 감";
  else if (!tg.reachable) why = out(tg) ? "놓을 자리가 지도 밖" : "팔이 놓을 자리에 못 감";
  else {   // 둘 다 따로는 되지만 같은 파지(손 모양·손 돌림)가 없음
    const cg = r.cube_grasps || cu.grasps || [], tgg = r.target_grasps || tg.grasps || [];
    why = gKinds(cg) !== gKinds(tgg) ? `큐브는 ${gKinds(cg)}, 놓을 자리는 ${gKinds(tgg)} 됨 — 같은 손으로 둘 다 안 됨`
      : `큐브는 ${cg.map(gName).join(" / ")}, 놓을 자리는 ${tgg.map(gName).join(" / ")} 로만 됨 — 같은 손으로 둘 다 안 됨`;
  }
  return [`⛔ 안 됨: ${why} (큐브 yaw ${degS(foldYaw(r.input.cube[2]))} 기준)` + tail, "bad"];
}

// ---------- 역기구학 탭: 손 위치로 보내기 (POST /api/ik/hand → 유령 → 각도 보내기와 같은 /api/joint/send) ----------
// 점 = 손이 쥐는 큐브의 중심(실좌표, m). z 는 '책상 위 cm' 로 받는다(책상 = 어깨 아래 D 0.4775 m, realcoords.MAP_D).
// 손 방향: auto(기본) = 손가락 아래, 손가락 사이 축 둘레 기울기를 ±상한(기본 60°) 안에서 IK 가 고름(지금 자세에서 최소 회전 먼저, 2° 안 동률이면 수직에 가까운 것),
// vertical = 손끝 수직(기울기 0), grasp = 팀 IK 원래 값(−40° 고정).
// 풀기는 자동: 슬라이더를 끄는 동안 = 빠른 풀이(직전 해에서 한 번, fast: true, 최신 것만), 손을 떼거나 숫자를 바꾸면 300 ms 뒤 전체 풀이.
const HK = { last: null, good: null, fullSeq: 0, fullAt: 0, inflight: false, pending: false, timer: null, busy: false,
  warned: false, check: null, sentAt: 0, mapD: 0.5575, ghostBad: false, fastT: [], fastLat: [], fastSrv: [],
  follow: false, followSent: null, followNote: "", followN: 0 };
const ARM7 = ["shoulder_roll", "shoulder_yaw", "upper_arm_roll", "elbow", "wrist_pitch", "wrist_yaw", "hand_roll"];
const hkSame = (a, b) => JSON.stringify(a) === JSON.stringify(b);
// 따라가기 진단 로그: 켜져 있는 동안 틱·풀이 응답마다 한 줄 → 1초마다 POST /api/log/follow → dashboard/logs/follow.log
const hkR1 = (o) => o ? Object.fromEntries(ARM7.filter((n) => o[n] != null).map((n) => [n, Math.round(o[n] * 10) / 10])) : null;
const hkArm = () => { const p = (S && S.pose) || {}; return Object.fromEntries(ARM7.filter((n) => p[n]).map((n) => [n, Math.round(p[n].deg * 10) / 10])); };
function hkDiff(a, b) {   // 관절별 차이 최대값과 그 관절 (각도는 ±180 감김 없이 그냥 차이: 관절 범위가 ±180 안)
  let d = 0, j = null;
  if (!a || !b) return { d: null, j: null };
  for (const n of ARM7) { if (a[n] == null || b[n] == null) return { d: 999, j: n }; const x = Math.abs(a[n] - b[n]); if (x > d) { d = x; j = n; } }
  return { d: Math.round(d * 10) / 10, j };
}
HK.flogQ = [];
function hkFlog(ev, o) {
  if (!HK.follow && !/^follow_/.test(ev)) return;
  const v = hkVals();
  HK.flogQ.push({ t: Math.round(performance.now()), ev, tgt: v && { x: v.x, y: v.y, zc: v.z_table_cm, yaw: v.yaw_deg, mode: v.mode, fix: v.fix_deg.shoulder_yaw ?? null },
    arm: hkArm(), sent_ref: hkR1(HK.followSent), n_sent: HK.followN, ...o });
  if (HK.flogQ.length > 400) HK.flogQ.splice(0, HK.flogQ.length - 400);
}
function hkSolLog(r) {   // 풀이 응답 요약
  return r ? { kind: r.fast ? "fast" : "full", ok: !!r.ok, pos_err: r.pos_err_mm, ang_err: r.ang_err_deg, flip: r.flip, turn: r.hand_turn, tilt: r.tilt_deg,
    q: hkR1(r.joints), ms: r.ms, reason: r.ok ? undefined : r.reason, meth: r.method, nsol: r.n_solutions, req: r.req && { x: r.req.x, y: r.req.y, zc: r.req.z_table_cm, yaw: r.req.yaw_deg } } : null;
}
async function hkFlogFlush() {
  if (!HK.flogQ.length) return;
  const rows = HK.flogQ.splice(0);
  try { await fetch("/api/log/follow", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ rows }) }); } catch (e) { /* 로그는 버림 */ }
}
function hkVals() {
  if (!HK.f) return null;
  const n = (e) => { const v = parseFloat(e.value); return Number.isFinite(v) ? v : null; };
  const x = n(HK.f.x.num), y = n(HK.f.y.num), zc = n(HK.f.z.num), yaw = n(HK.f.yaw.num), fv = n(HK.f.fixv), tm = n(HK.f.tmax);
  if (x == null || y == null || zc == null) return null;
  return { x, y, z_table_cm: zc, z: zc / 100 - tableDNum(), d: tableD(), yaw_deg: yaw ?? 0, mode: HK.f.mode.value, tilt_max_deg: tm ?? 60, ref: HK.f.ref.value,
    fix_deg: HK.f.fix.checked ? { shoulder_yaw: fv ?? 0 } : {} };
}
function hkSetXY(x, y, live) {
  if (!HK.f) return;
  HK.f.x.set(x); HK.f.y.set(y);
  hkEdit(!!live);
}
function hkMapMark() {
  if (!M.gHand) return;
  const v = hkVals();
  if (!v) { M.gHand.setAttribute("visibility", "hidden"); return; }
  M.gHand.setAttribute("transform", `translate(${msx(v.x).toFixed(1)},${msy(v.y).toFixed(1)})`);
  M.gHand.setAttribute("visibility", "visible");
}
// 입력이 바뀜: live = 끄는 중(빠른 풀이), 아니면 300 ms 뒤 전체 풀이
function hkEdit(live) {
  hkMapMark();
  if (!live && RX.on) { clearTimeout(RX.t); RX.t = setTimeout(reachLoad, 500); }
  clearTimeout(HK.timer);
  if (live && HK.good) hkFast();
  else HK.timer = setTimeout(hkSolve, live ? 400 : 300);
  hkRender(); updateIkView();
}
async function hkFast() {
  if (HK.inflight) { HK.pending = true; return; }
  const v = hkVals();
  let g = HK.good;
  if (!v || !g) return;
  // 따라가기 중: 마지막 좋은 해가 보낸 해에서 20° 넘게 떨어진 다른 가지면(전체 풀이가 건너뜀) 보낸 해에서 이어 푼다
  // (그 시작점이 한 번 못 따라가면 다음엔 마지막 좋은 해에서)
  let fromSent = false;
  if (HK.follow && HK.followSol && HK.followSent && !HK.sentSeedFail && hkDiff(g.joints, HK.followSent).d > 20) { g = HK.followSol; fromSent = true; }
  HK.inflight = true; HK.pending = false;
  const t0 = performance.now();
  let r;
  try {
    r = await (await fetch("/api/ik/hand", { method: "POST", headers: { "content-type": "application/json" },
      body: JSON.stringify({ ...v, fast: true, seed_deg: g.joints, flip: g.flip, hand_turn: g.hand_turn }) })).json();
  } catch (e) { r = { ok: false, reason: "서버 연결 실패" }; }
  const t1 = performance.now();
  HK.inflight = false;
  if (HK.fullAt >= t0) hkFlog("solve_drop", { kind: "fast", why: "그사이 전체 풀이가 끝나 버림" });
  if (HK.fullAt < t0) {   // 그사이 전체 풀이가 끝났으면 이 빠른 결과는 버림
    HK.fastT.push(t1); HK.fastLat.push(t1 - t0); if (r.ms != null) HK.fastSrv.push(r.ms);
    for (const a of [HK.fastLat, HK.fastSrv]) if (a.length > 30) a.shift();
    while (HK.fastT.length && t1 - HK.fastT[0] > 2000) HK.fastT.shift();
    r.req = v; HK.last = r;
    hkFlog("solve", { ...hkSolLog(r), seed_from: fromSent ? "보낸 해" : "마지막 좋은 해", seed: hkR1(g.joints), d_seed: hkDiff(r.joints, g.joints), d_sent: hkDiff(r.joints, HK.followSent) });
    if (fromSent) HK.sentSeedFail = !r.ok;
    if (r.ok) { HK.good = r; HK.ghostBad = false; }
    else { HK.ghostBad = true; if (!HK.busy) hkSolve(); }   // 빠른 풀이가 못 따라가면(다른 팔 모양이 필요) 끄는 중에도 전체 풀이 한 번
    hkRender(); updateIkView();
  }
  if (HK.pending) hkFast();
}
async function hkSolve() {
  const v = hkVals();
  if (!v) return;
  const my = ++HK.fullSeq;
  HK.busy = true; hkRender();
  let r;
  try {
    // 따라가기 중이면 직전 보낸 해 근처(같은 가지) 해를 먼저 고르게 near_deg 를 준다
    // 기준 자세는 서버가 지금 팔(피드백)로 잡고, 빠진 관절(서보 안 보냄)은 직전 보낸 해로 채운다 + 이어 풀기 시작점
    const ns = (HK.follow && HK.followSent) || HK.sentJ, near = ns ? Object.fromEntries(ARM7.filter((n) => ns[n] != null).map((n) => [n, ns[n]])) : null;
    r = await (await fetch("/api/ik/hand", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify(near ? { ...v, near_deg: near } : v) })).json();
  } catch (e) { r = { ok: false, reason: "서버 연결 실패" }; }
  if (my !== HK.fullSeq) { hkFlog("solve_drop", { kind: "full", why: "더 새 전체 풀이 요청이 있음" }); return; }   // 더 새 요청이 있음
  HK.busy = false; HK.fullAt = performance.now();
  r.req = v; HK.last = r; HK.check = null;
  hkFlog("solve", { ...hkSolLog(r), d_prev: hkDiff(r.joints, HK.good && HK.good.joints), d_sent: hkDiff(r.joints, HK.followSent), n_good: r.n_good, max_move: r.max_move_deg });
  if (r.ok) { HK.good = r; HK.ghostBad = false; } else HK.ghostBad = true;
  hkRender(); updateIkView();
}
function hkSlider(lab, unit, min, max, step, val, digits) {
  const range = el("input", { type: "range", min, max, step, value: val });
  const num = el("input", { type: "number", min, max, step, value: val, inputmode: "decimal" });
  range.addEventListener("input", () => { num.value = (+range.value).toFixed(digits); hkEdit(true); });
  range.addEventListener("change", () => hkEdit(false));
  num.addEventListener("input", () => { range.value = num.value; rangeFill(range); hkEdit(false); });
  const set = (v) => { num.value = (+v).toFixed(digits); range.value = v; rangeFill(range); };
  return { range, num, set, row: el("div", { class: "mctl" }, el("span", { class: "mlabel" }, lab), range, num, el("span", {}, unit)) };
}
const HK_ABOVE = 0.2435;   // 손이 수직일 때 손목 중앙이 집는 점보다 위 (m) — 서버 ref_info 가 오면 그 값
const HK_GRIP_CM = 3.5;   // 큐브 잡는 높이: 집는 점 = 큐브 중심 + 1 cm = 책상 위 3.5 cm (hand_ik.GRIP_ABOVE_CENTER) — 서버 ref_info 가 오면 그 값
const HK_WRIST_Z0 = Math.round(HK_GRIP_CM + HK_ABOVE * 100);   // 손목 기준 기본 높이 = 집는 점 3.5 cm 에 해당 (28 cm)
const hkAbove = () => (HK.good && HK.good.ref_info && HK.good.ref_info.wrist_above_grip_vertical_m) || HK_ABOVE;
const hkGripCm = () => { const r = HK.good && HK.good.ref_info; return r && r.grip_above_table_m != null ? +(r.grip_above_table_m * 100).toFixed(1) : HK_GRIP_CM; };
const hkCubeZ = () => HK.f.ref.value === "wrist" ? Math.round((hkGripCm() + hkAbove() * 100) * 2) / 2 : hkGripCm();
function hkRefUi() {   // 큐브 잡는 높이 버튼: 손목 기준은 수직 손에서만 계산이 맞음
  HK.preset.hidden = HK.f.ref.value === "wrist" && HK.f.mode.value !== "vertical";   // 위치만이면 손목 각에 따라 달라 숨김
  HK.preset.textContent = `큐브 잡는 높이 (${hkCubeZ()} cm)`;
}
function hkRefChanged() {   // 기준점 바꾸면 높이도 그 기준에 맞게 (수직 손 기준 거리만큼) + 슬라이더 범위
  const f = HK.f, w = f.ref.value === "wrist", z = parseFloat(f.z.num.value) || 0, d = hkAbove() * 100;
  f.z.range.min = w ? 25 : 0; f.z.range.max = w ? 65 : 40;
  f.z.row.querySelector(".mlabel").textContent = w ? "높이 (손목 중앙, 책상 위)" : "높이 (집는 점, 책상 위)";
  f.z.set(Math.round((w ? z + d : Math.max(0, z - d)) * 2) / 2);
  HK.good = null; HK.followSent = null; HK.followSol = null;   // 다른 기준의 해는 이어 쓰지 않음
  hkRefUi(); hkEdit(false);
}
function buildHandIk() {
  const f = HK.f = {};
  f.x = hkSlider("x (왼쪽 +)", "m", -0.25, 0.35, 0.005, 0.1, 3);
  f.y = hkSlider("y (앞 −)", "m", -0.45, 0.05, 0.005, -0.3, 3);
  // 기준점: 손목 중앙(기본, 손목 굽힘·비틀기 축의 교점) / 손가락 사이(집는 점). 높이 = 그 기준점의 책상 위 높이
  // 사용자 2026-11-20 밤: 기준점 = 손끝(집는 점) 기본, 역기구학은 손목 중앙으로 풀고 손이 기운 만큼 손목을 옮겨 맞춤
  f.z = hkSlider("높이 (집는 점, 책상 위)", "cm", 0, 40, 0.5, HK_GRIP_CM, 1);
  f.ref = el("select", {}, el("option", { value: "grip" }, "손끝 사이 (집는 점, 기본)"), el("option", { value: "wrist" }, "손목 중앙"));
  f.ref.addEventListener("change", () => hkRefChanged());
  f.yaw = hkSlider("손 방향 (잡을 큐브 yaw)", "°", -45, 45, 1, 0, 0);
  f.mode = el("select", {}, el("option", { value: "position" }, "위치만 (손은 손목으로 최대한 아래, 기본)"),
    el("option", { value: "vertical" }, "수직 (기울기 0°)"),
    el("option", { value: "auto" }, "손가락 아래 (기울기 자동, 0~상한)"),
    el("option", { value: "grasp" }, "−40° 고정 (팀 IK 원래 값)"));
  f.tmax = el("input", { type: "number", min: 0, max: 90, step: 5, value: 60, style: "width:64px", title: "기울기 자동일 때 손가락 사이 축 둘레 기울기 상한 (양쪽)" });
  f.tmax.addEventListener("input", () => hkEdit(false));
  f.fix = el("input", { type: "checkbox", checked: "" });   // 기본 켬, 0° (사용자 2026-11-20 저녁: 위치만 모드에서 답을 몇 개로)
  f.fixv = el("input", { type: "number", min: 0, max: 45, step: 1, value: 0, style: "width:70px" });
  f.open = el("input", { type: "checkbox" });
  f.followBox = el("input", { type: "checkbox" });
  f.mode.addEventListener("change", () => { hkRefUi(); hkEdit(false); });
  f.fix.addEventListener("change", () => { hkEdit(false); graspLoad(); });
  f.fixv.addEventListener("input", () => { hkEdit(false); clearTimeout(G.tFix); G.tFix = setTimeout(graspLoad, 400); });
  f.followBox.addEventListener("change", () => {
    if (f.followBox.checked && !confirm((VIRT() ? "(가상 팔) " : "⚠ 실제 모터가 움직인다.\n\n") +
      "팔도 바로 따라가기: 풀린 자세를 0.2초마다 각도 보내기 길(같은 막힘·원점 규칙, arm_control 램프)로 보낸다.\n" +
      "한 번에 20° 넘게 바뀌는 해는 안 보낸다. 중간 경로 충돌 검사는 없다. 켤까요?")) { f.followBox.checked = false; return; }
    hkFlog(f.followBox.checked ? "follow_on" : "follow_off", { why: f.followBox.checked ? "사용자 켬" : "사용자 끔" });
    HK.follow = f.followBox.checked; HK.followSent = null; HK.followSol = null; HK.followNote = ""; HK.followLastWhy = "";
    hkFlogFlush();
    // 켤 때 팔이 유령 자세에서 20° 넘게 떨어져 있으면(예: HOME) 따라가기는 한 번도 못 보낸다 → 먼저 [보내기] 로 옮길지 묻기
    const r = HK.last, pz = S.pose || {};
    if (HK.follow && r && r.ok && hkSame(hkVals(), r.req)) {
      const d = hkDiff(r.joints, Object.fromEntries(ARM7.map((n) => [n, pz[n] ? pz[n].deg : null])));
      if (d.d > 20 && confirm(`팔이 유령 자세에서 ${d.d > 900 ? "?" : d.d.toFixed(0) + "°"} (${d.j}) 떨어져 있어 따라가기가 못 보낸다 (한 번에 20° 까지).
먼저 [보내기] 로 유령 자세까지 옮길까요?`)) {
        hkFlog("follow_on", { why: "켤 때 먼저 [보내기]", d: d.d, dj: d.j }); hkSend();
      }
    }
  });
  // 큐브 잡는 높이: 집는 점이 큐브 중심 +1 cm(책상 위 3.5 cm)에 오는 높이 — 손목 기준이면 수직 손에서 + 손목↔집는 점 거리(24.35 cm)
  HK.preset = el("button", { class: "btn line small", onclick: () => { f.z.set(hkCubeZ()); hkEdit(false); }, title: "집는 점 = 큐브(5 cm) 중심 + 1 cm = 책상 위 3.5 cm (패드가 큐브 옆면 위쪽을 쥠)" }, "큐브 잡는 높이");
  f.z.row.append(HK.preset);
  const fromCube = el("button", { class: "btn line small", title: "위 지도의 큐브 x, y, yaw 를 가져오기", onclick: () => {
    const v = mapVals(); if (!v.cube) { toast("지도에 큐브가 없음", "error"); return; }
    f.yaw.set(Math.round(foldYaw(v.cube.yaw))); hkSetXY(v.cube.x, v.cube.y, false);
  } }, "지도의 큐브 자리 쓰기");
  const pickMap = el("button", { class: "btn line small", onclick: () => { mapMode("hand"); if (M.svg) M.svg.scrollIntoView({ block: "center", behavior: "smooth" }); },
    title: "지도 위 [손 위치] 모드: 누르거나 끌면 x, y 가 바뀐다" }, "지도에서 고르기·끌기");
  HK.bSolve = el("button", { class: "btn line small", onclick: hkSolve, title: "자동으로 풀지만, 다시 풀고 싶을 때" }, "다시 풀기");
  HK.bSend = el("button", { class: "btn", onclick: hkSend }, "보내기");
  HK.bCheck = el("button", { class: "btn line", onclick: () => hkFk(true), title: "지금 팔 자세(AK45 피드백·서보 보낸 값)로 FK → 손이 쥐는 점이 목표에서 몇 mm" }, "지금 손 위치 확인");
  HK.vb = el("span", { class: "vbadge", hidden: true }, "가상");
  HK.res = el("div", { class: "result hkres" }, "풀는 중…");
  HK.meta = el("div", { class: "hkmeta" });
  HK.rate = el("div", { class: "hkmeta" });
  HK.gate = el("div", { class: "hkmeta" });
  HK.chk = el("div", { class: "hkmeta" });
  // 해석 IK(위치만 + 어깨 옆 고정): 답 목록에서 다른 해 골라 보내기 (고급)
  HK.solsBody = el("div", { class: "hkmeta" });
  HK.sols = el("details", { class: "log", hidden: true }, el("summary", {}, "고급: 다른 해 고르기 (해석 IK 의 답 목록)"), HK.solsBody);
  const root = el("div", { class: "hk" },
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "3"), "손 위치로 보내기"),
    el("div", { class: "card" },
      el("p", { class: "help", style: "margin-top:0" }, "손이 쥐는 점(큐브 중심)을 움직이면 바로 역기구학으로 풀어 3D 에 ",
        el("b", { style: "color:#2f66d0" }, "파란 유령"), "으로 보인다 (못 풀면 ", el("b", { style: "color:#dc2626" }, "빨강"), " + 마지막 좋은 해). [보내기] 로 팔을 그 자세로. 손가락은 그대로."),
      el("div", { class: "row", style: "margin-top:0" }, pickMap, fromCube),
      f.x.row, f.y.row, f.z.row, f.yaw.row,
      el("div", { class: "mctl" }, el("span", { class: "mlabel" }, "기준점"), f.ref),
      el("details", { class: "log" }, el("summary", {}, "고급: 손 자세 (기본 = 손 각도 무시, 손목 관절 그대로)"),
        el("div", { class: "mctl" }, el("span", { class: "mlabel" }, "손 자세"), f.mode, el("span", {}, " 기울기 상한 ±"), f.tmax, el("span", {}, "°"))),
      el("label", { class: "chk", style: "display:flex;gap:6px;align-items:center;margin:4px 0 0" }, f.fix, "어깨 옆으로 벌리기 고정 (shoulder_yaw)", f.fixv, "°"),
      el("p", { class: "help", style: "margin:4px 0 0" }, "손이 바닥을 봐도 답이 하나가 아니다: 손 위치·방향은 6개 값인데 관절이 7개라 같은 손 자세를 만드는 팔 모양이 여러 개(팔꿈치 위치가 남음)이고, 손도 반 바퀴 돌린 두 가지가 있어 계산이 하나를 고른다. " +
        "고정을 켜면 관절이 6개가 되어 거의 하나로 정해진다 (대신 닿는 곳이 줄어든다 — 안 풀리면 이유에 '고정 때문' 이 뜬다)."),
      el("label", { class: "chk", style: "display:flex;gap:6px;align-items:center;margin:4px 0 0" }, f.open, "보낼 때 손가락도 벌리기 (집기 전 자세)"),
      el("label", { class: "chk", style: "display:flex;gap:6px;align-items:center;margin:4px 0 0" }, f.followBox, "팔도 바로 따라가기 (0.2초마다 보냄, 20° 넘게 바뀌면 안 보냄)"),
      el("div", { class: "row", style: "align-items:center" }, HK.bSend, HK.bCheck, HK.bSolve, HK.vb),
      HK.res, HK.meta, HK.rate, HK.gate, HK.chk, HK.sols,
      el("details", { class: "log" }, el("summary", {}, "자세히"),
        el("p", { class: "help" }, "점 = 손이 쥐는 큐브의 중심. 파지 중심(end_effector)은 손 축 따라 21.5 mm 위 (env.pick_pose 와 같음, '수직만' 이면 0). " +
          "기울기 자동: 손가락이 닫히는 축(손가락 1·2 ↔ 3) 둘레로 기울여도 패드는 같은 두 면에 닿으므로 그 축 둘레 회전은 자유로 두고(위치 3 + 손가락 축 방향 2 만 맞춤), " +
          "기울기는 ±상한 안에서, 남는 자유도로 수직 쪽으로 당긴다. 후보 고르기: 벌린 손가락이 책상에 닿는 해는 탈락 → 지금 자세(피드백, 서보는 보낸 값, 없으면 직전 보낸 해·HOME)에서 최소 회전(관절 최대 이동각) → 그 최소에서 2° 안이면 수직에 가까운 것. " +
          "손 방향 = 이 손이 쥘 큐브의 yaw (지도와 같은 기준, 위에서 반시계). 전체 풀이: 큐브의 세 면 × 손 그대로/반 바퀴 × 시작점 5개, 관절 한계(calibration 의 보낼 수 있는 범위, 손 비틀기 ±88°) 안에서 " +
          "위치 5 mm · 방향 1.5° 안이면 풀림, 그중 지금 자세에서 가장 가까운 해. 끄는 중엔 빠른 풀이(직전 해에서 한 번, 같은 손 모양). team-jax/IK env.py·ik.py 를 그대로 쓴다. " +
          "[보내기] = 각도 제어 탭과 같은 /api/joint/send (같은 막힘 규칙·확인창). 지금 자세 → 목표까지 관절마다 직선으로 가며, 중간 경로의 충돌은 검사하지 않는다."))));
  $("#handik").append(root);
  hkRefUi();
  setInterval(hkFollowTick, 200);
  setInterval(hkFlogFlush, 1000);
  hkEdit(false);
}
function hkRender() {
  if (!HK.res || !S) return;
  const r = HK.last, v = hkVals();
  const cur = r && hkSame(v, r.req);
  HK.vb.hidden = !VIRT();
  if (r) {
    const hand = (r.flip ? "손 반 바퀴" : "손 그대로") + (r.tilt_deg != null ? ` · 기울기 ${degS(r.tilt_deg)}` : "");
    const sy = r.joints ? ` · 어깨 옆 ${f1(r.joints.shoulder_yaw)}` : "";
    const an = r.method === "analytic" && r.n_solutions ? ` · 해석 IK: 답 ${r.n_solutions}개 중 ${r.picked != null ? `${r.picked + 1}번(직접 고름)` : "최소 회전"}` : "";
    if (r.ok) {
      // 지금 자세(피드백, 빠진 관절은 직전 보낸 해)에서 가장 많이 도는 관절
      const pz = S.pose || {}, sj = HK.followSent || HK.sentJ || {};
      const now7 = Object.fromEntries(ARM7.map((n) => [n, pz[n] ? pz[n].deg : sj[n]]));
      const mv = hkDiff(r.joints, now7);
      HK.res.textContent = `✅ 풀림${r.fast ? " (빠른)" : ""} — 지금 자세에서 최대 ${mv.d == null || mv.d > 900 ? "?" : mv.d.toFixed(0)}° 회전${mv.j && mv.d < 900 ? ` (${jLabel(mv.j).replace(/ \(.*\)$/, "")})` : ""}\n오차 ${r.pos_err_mm} mm · 방향 ${r.ang_err_deg}° · ${hand}, 손 돌림 ${degS(r.hand_turn)}${sy}${an}` +
        (r.warn && r.warn.length ? "\n⚠ " + r.warn.join(" · ") : "") + (!cur ? (HK.busy || HK.inflight ? "\n⏳ 다시 푸는 중…" : "\n(값이 바뀜)") : "");
      HK.res.className = "result hkres " + (!cur || (r.warn && r.warn.length) ? "warn" : "ok");
    } else {
      HK.res.textContent = "⛔ " + (r.reason || "못 풂") + (r.unfixed && r.unfixed.ok ? "" : "") + (!cur ? "\n⏳ 다시 푸는 중…" : "");
      HK.res.className = "result hkres bad";
    }
    const j = r.joints;
    HK.meta.textContent = j ? `${r.ok ? "" : "가장 가까운 해: "}` + ARM7.map((n) => `${jLabel(n).replace(/ \(.*\)$/, "")} ${f1(j[n])}`).join(" · ") +
      (r.max_move_deg != null ? ` · 지금에서 최대 ${r.max_move_deg}° 이동` : "") + (r.at_limit && r.at_limit.length ? ` · 한계에 닿은 관절: ${r.at_limit.join(", ")}` : "") +
      (r.finger_low_mm != null ? ` · 벌린 손가락 끝 책상 위 ${r.finger_low_mm} mm` : "") + (r.load_s ? ` · 모델 읽기 ${r.load_s}초` : "") + (r.ms != null ? ` · 풀이 ${r.ms} ms` : "") : "";
  } else if (HK.busy) { HK.res.textContent = "⏳ 푸는 중… (처음엔 MuJoCo 모델을 읽느라 몇 초 더)"; HK.res.className = "result hkres run"; }
  hkSolsRender(r);
  const avg = (a) => a.length ? a.reduce((s, x) => s + x, 0) / a.length : null;
  const now = performance.now(), recent = HK.fastT.filter((t) => now - t <= 2000).length / 2;
  HK.rate.textContent = HK.fastLat.length ? `실시간 풀이: ${recent.toFixed(1)} 회/초 (최근 2초) · 왕복 ${avg(HK.fastLat).toFixed(0)} ms · 서버 풀이 ${avg(HK.fastSrv).toFixed(1)} ms` : "";
  const g = S.gate || { joints: {} };
  const blocked = ARM7.filter((n) => !((g.joints || {})[n] || {}).ok);
  HK.bSend.disabled = !(r && r.ok && cur) || blocked.length > 0;
  HK.gate.textContent = (blocked.length ? `보내기 막힘: ${blocked.length === ARM7.length ? shortGate(g) : blocked.map((n) => `${n} (${(g.joints[n] || {}).reason || "?"})`).join(", ")}`
    : `${VIRT() ? "(가상) " : ""}보낼 수 있음 — 각도 제어 탭과 같은 길`) + (HK.follow ? ` · 따라가기 켬 (보낸 ${HK.followN}번)${HK.followNote ? " — " + HK.followNote : ""}` : "");
  HK.gate.className = "hkmeta" + (blocked.length || HK.followNote ? " bad" : "");
  // 보낸 뒤 팔이 목표에 닿으면 FK 로 손 위치 한 번 확인
  if (HK.sentAt && HK.sentJ) {
    const p = S.pose || {}, done = ARM7.every((n) => p[n] && Math.abs(p[n].deg - HK.sentJ[n]) < 0.3);
    if (done) { HK.sentAt = 0; hkFk(false); }
    else if (performance.now() - HK.sentAt > 120000) HK.sentAt = 0;
  }
  if (HK.check) {
    const c = HK.check;
    const w = c.wref && c.wrist_real, pt = w ? c.wrist_real : c.grip_real;
    HK.chk.textContent = c.ok ? `지금 ${w ? "손목 중앙" : "손이 쥐는 점"} (FK): x ${cm(pt[0])} · y ${cm(pt[1])} cm · 책상 위 ${((pt[2] + tableDNum()) * 100).toFixed(1)} cm` +
      (c.diff_mm != null ? ` → 목표와 ${c.diff_mm.toFixed(1)} mm 차이` : "") + ` · 손 축 기울기 ${c.axis_tilt_deg}°` : "FK 못 함: " + (c.reason || "?");
  } else HK.chk.textContent = "";
}
// 해석 IK 답 목록 (전체 풀이 결과에만 있음). 고른 해로 HK.last 를 바꾸고 [보내기] 와 같은 길로 보낸다
function hkSolsRender(r) {
  if (!HK.sols) return;
  const list = r && r.ok && r.solutions && r.solutions.length > 1 ? r.solutions : null;
  HK.sols.hidden = !list;
  if (!list || HK.solsFor === r) return;
  HK.solsFor = r;
  HK.solsBody.replaceChildren(...list.map((sol, k) => el("div", { class: "row", style: "align-items:center;margin:2px 0" },
    el("button", { class: "btn line small", disabled: !sol.finger_ok || (r.picked == null ? sol.chosen : r.picked === k),
      title: sol.finger_ok ? "이 해를 유령 자세로 두고 [보내기] (확인창 그대로)" : "벌린 손가락이 책상에 닿음 — 못 고름",
      onclick: () => {
        const base = r.base || r;
        const r2 = { ...base, base, joints: sol.joints, max_move_deg: sol.max_move_deg, finger_low_mm: sol.finger_low_mm, picked: k };
        HK.last = r2; HK.good = r2; hkRender(); updateIkView(); hkSend();
      } }, (r.picked == null ? sol.chosen : r.picked === k) ? "지금 해" : "이 해 보내기"),
    el("span", {}, `${k + 1}. 최대 ${sol.max_move_deg}° 회전 · 어깨 앞뒤 ${f1(sol.joints.shoulder_roll)} · 위팔 비틀기 ${f1(sol.joints.upper_arm_roll)} · 팔꿈치 ${f1(sol.joints.elbow)}` +
      (sol.finger_ok ? "" : ` · 손가락 책상에 닿음(${sol.finger_low_mm} mm)`)))));
}
async function hkFk(manual) {
  const t = HK.sentReq || hkVals();
  const mode = (t && t.mode) || "auto";
  let r;
  try { r = await (await fetch("/api/ik/fk", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ mode }) })).json(); }
  catch (e) { r = { ok: false, reason: "서버 연결 실패" }; }
  if (r.ok && t) {
    r.wref = (t.ref || "grip") === "wrist" && !!r.wrist_real;
    const pt = r.wref ? r.wrist_real : r.grip_real;
    r.diff_mm = Math.hypot(pt[0] - t.x, pt[1] - t.y, pt[2] - t.z) * 1000;
  }
  HK.check = r; hkRender();
  if (manual && !r.ok) toast("FK 못 함: " + (r.reason || "?"), "error");
}
function hkJoints(r) {
  const g = S.gate || { joints: {} }, cal = calJ(), joints = {};
  for (const n of ARM7) joints[n] = Math.round(clampR(cal[n], r.joints[n]) * 100) / 100;
  if (HK.f.open.checked) for (const [n, v] of Object.entries(r.open_fingers_deg || {})) if (cal[n] && (g.joints[n] || {}).ok) joints[n] = Math.round(clampR(cal[n], v) * 100) / 100;
  return joints;
}
async function hkSend() {
  const r = HK.last;
  if (!r || !r.ok || !hkSame(hkVals(), r.req)) return;
  if (!HK.warned) {
    const mv = r.max_move_deg != null ? r.max_move_deg : null;
    const ok = confirm((VIRT() ? "(가상 팔) " : "⚠ 실제 모터가 움직인다.\n\n") +
      "지금 자세 → 목표 자세까지 관절마다 직선으로 간다 (arm_control_node 의 max_speed 램프).\n" +
      "중간 경로에서 팔·손이 책상이나 몸에 부딪히는지는 검사하지 않는다 — 길을 비우고 지켜본다.\n\n" +
      `가장 많이 도는 관절: ${mv != null ? mv + "°" : "?"}\n\n계속할까요?`);
    if (!ok) return;
    HK.warned = true;
  }
  if (!confirmFirst()) return;
  const joints = hkJoints(r);
  const res = await post("/api/joint/send", { joints });
  if (res.ok) {
    HK.sentAt = performance.now(); HK.sentJ = joints; HK.sentReq = r.req; HK.check = null; HK.followSent = joints; HK.followSol = r;
    hkFlog("send_manual", { sol: hkSolLog(r), q_sent: hkR1(joints) });
    for (const [n, v] of Object.entries(joints)) { A.sent[n] = v; if (A.built) setVal(n, v, true); }
  }
  hkRender();
}
let hkFollowBusy = false;
async function hkFollowTick() {
  if (!HK.follow || !S) return;
  const skip = (why, o) => {   // 같은 '변화 없음' 이 이어지면 한 번만
    if (why === HK.followLastWhy && why === "변화 없음(<0.2°)") return;
    HK.followLastWhy = why; hkFlog("tick", { sent: false, why, ...o });
  };
  if (hkFollowBusy) return skip("보내는 중(직전 POST 응답 대기)");
  const r = HK.last, g = S.gate || { joints: {} };
  if (!r) return skip("풀이 없음");
  if (!r.ok) return skip("풀이 실패", { sol: hkSolLog(r) });
  if (!hkSame(hkVals(), r.req)) return skip("오래된 풀이(입력이 바뀜, 새 풀이 대기)", { sol: hkSolLog(r) });
  const blk = ARM7.filter((n) => !((g.joints || {})[n] || {}).ok);
  if (blk.length) return skip("보내기 막힘: " + blk.map((n) => `${n} ${(g.joints[n] || {}).reason || "?"}`).join(", "));
  const joints = hkJoints(r);
  const p = S.pose || {}, ref = HK.followSent || Object.fromEntries(ARM7.map((n) => [n, p[n] ? p[n].deg : null]));
  const dd = hkDiff(joints, ref), big = dd.d;
  if (big < 0.2) return skip("변화 없음(<0.2°)");
  if (big > 20) {
    HK.followNote = big > 900 ? `${dd.j} 지금 위치를 모름(서보는 보낸 적이 없음) — 안 보냄 ([보내기] 로 먼저 옮기기)`
      : HK.followSent ? `이 점은 다른 팔 모양이라 ${big.toFixed(0)}° (${dd.j}) 바뀜 — 건너뜀 (이어지는 점에서 다시 따라감, 여기로 가려면 [보내기])`
      : `지금 팔에서 ${big.toFixed(0)}° (${dd.j}) — 안 보냄 ([보내기] 로 먼저 옮기기)`;
    return skip("20° 초과", { sol: hkSolLog(r), d: big, dj: dd.j, ref_from: HK.followSent ? "직전 보낸 해" : "지금 팔" });
  }
  HK.followNote = "";
  hkFollowBusy = true;
  let res;
  try {
    res = await (await fetch("/api/joint/send", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ joints }) })).json();
    if (res.ok) { HK.followSent = joints; HK.followSol = r; HK.sentSeedFail = false; HK.followN++; HK.sentAt = performance.now(); HK.sentJ = joints; HK.sentReq = r.req; for (const [n, v] of Object.entries(joints)) A.sent[n] = v; }
    else { HK.followNote = res.error || "보내기 실패"; }
  } catch (e) { res = { ok: false, error: "서버 연결 실패" }; HK.followNote = "서버 연결 실패"; }
  HK.followLastWhy = "";
  hkFlog("tick", { sent: !!res.ok, why: res.ok ? "보냄" : "보내기 거절: " + (res.error || "?"), sol: hkSolLog(r), d: big, dj: dd.j, q_sent: hkR1(joints) });
  hkFollowBusy = false;
}
function renderHandIk() {
  if (!$("#handik") || !S) return;
  if (!HK.f) { if (!((S.calibration || {}).joints || []).length) return; buildHandIk(); }
  hkRender();
}

// ---------- 보정 안내 ----------
let calibBuilt = false;
function renderCalib() {
  const g = S.calibrate, box = $("#calib");
  const on = (id) => svcById(id).alive;
  if (!calibBuilt) {
    calibBuilt = true;
    box.append(el("div", { class: "card" },
      el("h3", {}, "보정은 터미널에서"),
      el("p", { class: "help" }, g.note),
      g.commands.map((x) => el("div", { class: "cmd" }, x)),
      el("div", { class: "row" }, el("button", { class: "btn line", onclick: async () => {
        for (const id of g.needs) if (!on(id)) await post(`/api/svc/${id}/start`, {});
      } }, "보정 준비: ak45_node + stm_bridge_node 켜기")),
      el("div", { class: "note", id: "calib-check" })));
  }
  const bad = g.forbid.filter(on).map((id) => svcById(id).label);
  const miss = g.needs.filter((id) => !on(id)).map((id) => svcById(id).label || id);
  const n = $("#calib-check");
  n.className = "note " + (bad.length ? "bad" : miss.length ? "warn" : "");
  n.textContent = (VIRT() ? "(가상 모드: calibrate 는 실물 전용) " : "") + (bad.length ? `끄세요: ${bad.join(", ")} (calibrate 와 같은 모터에 명령)` :
    miss.length ? `아직 안 켬: ${miss.join(", ")}` : "준비됨: 터미널에서 calibrate --ros 실행");
}

// ---------- 원점 (homing_guard) ----------
const H = { rows: {} };
function homeAsk(joint) {
  const poses = S.home_poses || {};
  const pose = joint === "all" ? Object.entries(poses).map(([k, v]) => `${k} ${v}°`).join(", ") || "전부 sim 0°" : `${joint} sim ${poses[joint] ?? 0}°`;
  return `${VIRT() ? "(가상) " : ""}${joint === "all" ? "원점 없는 관절 전부" : joint}: 관절을 기준 자세(homing.json home_sim_deg: ${pose})에 두었나요?\n` +
    "지금 위치를 그 자세로 삼는다. 다른 자세에서 누르면 그만큼 틀어진 원점이 된다.";
}
const RELEASE_HELP = "쉬게 하기: guard 가 이 관절 명령 전달만 끊고 원점을 지운다. ak45_node 는 마지막 target 을 계속 잡고 있다 " +
  "(실제로 힘을 빼려면 ak45_node 종료 — 팔이 떨어질 수 있음).";
async function homingDo(action, joint) {
  if (action === "home" && !confirm(homeAsk(joint))) return;
  if (action === "release" && !confirm(`${joint === "all" ? "전부" : joint} 쉬게 하기?\n\n${RELEASE_HELP}`)) return;
  await post(`/api/homing/${action}`, { joint });
}
function buildHoming() {
  H.msg = el("div", { class: "note" });
  H.bHomeAll = el("button", { class: "btn", onclick: () => homingDo("home", "all"), title: "원점 없는 관절 전부: 지금 위치를 기준 자세로 삼는다" }, "전부 원점 잡기");
  H.bRelAll = el("button", { class: "btn line", onclick: () => homingDo("release", "all"), title: RELEASE_HELP }, "전부 쉬게");
  H.list = el("div", { class: "hlist" });
  H.vb = el("span", { class: "vbadge", hidden: true }, "가상");
  H.guide = el("div", { class: "lead" });
  H.card = el("div", { class: "card" },
    el("h3", { title: "homing_guard(arm_homed.launch.py)가 원점 잡기 전·전원 재투입 뒤·홀드가 끝난 뒤에는 AK45 명령을 막는다." },
      H.vb, "AK45 원점", H.chip = el("span", { class: "chip" })),
    H.guide, el("div", { class: "row" }, H.bHomeAll, H.bRelAll), H.msg,
    el("details", { class: "log" }, el("summary", {}, "관절별 원점"), el("p", { class: "help" }, RELEASE_HELP), H.list));
  $("#homing").append(H.card);
}
function homingRow(name) {
  const r = {};
  r.badge = el("span", { class: "chip" });
  r.reason = el("div", { class: "note" });
  r.kv = el("div", { class: "kv" });
  r.more = el("details", { class: "log" }, el("summary", {}, "자세히"), r.kv);
  r.bHome = el("button", { class: "btn", onclick: () => homingDo("home", name) }, "원점 잡기");
  r.bRel = el("button", { class: "btn line", onclick: () => homingDo("release", name) }, "쉬게 하기");
  r.root = el("div", { class: "hrow" }, el("div", { class: "hhead" }, el("b", {}, name), r.badge, el("span", { class: "spacer" }), r.bHome, r.bRel), r.reason, r.more);
  return r;
}
function renderHoming() {
  if (!H.card) buildHoming();
  H.vb.hidden = !VIRT();
  const h = (S.ros && S.ros.homing) || { available: false };
  const guardOn = h.available && h.seen && h.age <= 3;
  const poses = Object.values(S.home_poses || {});
  const zero = poses.every((x) => !x);
  H.guide.textContent = `팔을 기준 자세(${zero ? "시뮬 0°" : "homing.json"})에 두고 누르세요.`;
  let chipCls = "", chipTxt = "—", msg = "", msgCls = "warn";
  const armSvc = svcById("arm_launch").alive || svcById("arm_control").alive;
  if (!h.available) { msg = VIRT() ? "연결 탭 ③ 팔 켜기 먼저" : "연결 탭 ② ROS 연결 먼저"; msgCls = ""; chipTxt = "ROS 없음"; }
  else if (!h.seen) chipTxt = "검사 꺼짐", msg = armSvc ? "원점 검사 없이 켜짐 (arm.launch) — 원점 잡기는 연결 탭 ③ 팔 켜기(arm_homed)로" : "연결 탭 ③ 팔 켜기 먼저";
  else if (h.age > 3) { msg = `원점 정보가 ${h.age.toFixed(0)}초 전 것 — 연결 탭 ③ 팔이 켜져 있는지 보세요.`; chipTxt = "오래됨"; chipCls = "warn"; }
  else {
    const n = h.joints.filter((j) => j.homed).length;
    chipTxt = `원점 ${n}/${h.joints.length}`; chipCls = h.all_homed ? "on" : "bad";
    if (!h.all_homed) { msg = "원점 없는 AK45 는 움직이지 않는다."; msgCls = ""; }
  }
  setChip(H.chip, chipTxt, chipCls);
  H.chip.title = guardOn ? `/arm_control/homing ${h.age.toFixed(1)}초 전` : "";
  H.msg.textContent = msg; H.msg.className = "note " + (msg ? msgCls : "");
  $("#homing-note").textContent = "";
  H.bHomeAll.disabled = H.bRelAll.disabled = !guardOn;
  const joints = h.seen ? h.joints : Object.keys(S.home_poses || {}).map((j) => ({ joint: j }));
  const seen = new Set(joints.map((j) => j.joint));
  for (const [n, r] of Object.entries(H.rows)) if (!seen.has(n)) { r.root.remove(); delete H.rows[n]; }
  for (const j of joints) {
    let r = H.rows[j.joint];
    if (!r) { r = H.rows[j.joint] = homingRow(j.joint); H.list.append(r.root); }
    r.bHome.disabled = r.bRel.disabled = !guardOn;
    if (!guardOn || j.homed == null) { setChip(r.badge, "—", ""); r.reason.textContent = ""; r.kv.replaceChildren(); continue; }
    setChip(r.badge, j.homed ? "✅ 원점" : "⛔ 원점 없음", j.homed ? "on" : "bad");
    r.reason.textContent = j.reason || "";
    r.reason.className = "note " + (j.homed ? "" : "bad");
    const age = j.feedback_lost ? "끊김" : j.feedback_age_s == null ? "없음" : `${j.feedback_age_s.toFixed(2)}초`;
    r.kv.replaceChildren(...[
      kv("모터", j.motor), kv("기준 자세", `sim ${j.home_sim_deg ?? 0}°`),
      kv("오프셋", j.homed ? f2(j.offset_deg) : "—"), kv("세션 zero", f2(j.session_zero_deg)),
      kv("피드백", `${f2(j.raw_deg)} · ${age}`), kv("command_active", j.command_active == null ? "—" : j.command_active ? "예" : "아니오"),
    ].flat());
  }
}

// ---------- 각도 제어: 관절 14개 슬라이더 ----------
// 값 = 시뮬 관절각(°). 범위 = calibration.json sim_range_deg ∩ 모터 범위(서버 cmd_range_deg). 보내기 버튼으로만 나간다.
const A = { rows: {}, val: {}, base: {}, sent: {}, confirmed: false, link: false };
const WRIST = ["wrist_pitch", "wrist_yaw", "hand_roll"];
const FINGER_SETS = [["손가락 1", ["finger1_joint1", "finger1_joint2"]], ["손가락 2", ["finger2_joint1", "finger2_joint2"]],
  ["손가락 3", ["finger3_joint1", "finger3_joint2"]], ["손가락 회전", ["finger_rotation"]]];
const calJ = () => Object.fromEntries(((S.calibration || {}).joints || []).map((j) => [j.joint, j]));
const clampR = (j, v) => { const r = j.cmd_range_deg; return r ? Math.min(r[1], Math.max(r[0], v)) : v; };
function confirmFirst() {
  if (A.confirmed) return true;
  const unver = ((S.calibration || {}).joints || []).filter((j) => !j.verified).map((j) => j.joint);
  const sp = (svcById("arm_homed").alive ? svcById("arm_homed") : svcById("arm_launch").alive ? svcById("arm_launch") : svcById("arm_control"));
  const speed = ((sp.params || []).find((p) => p.key === "max_speed_deg_s") || {}).value || "?";
  const ok = confirm((VIRT() ? "(가상 팔) " : "⚠ 실제 모터가 움직인다.\n\n") +
    `calibration verified=false (${unver.length}개 관절: ${unver.join(", ") || "없음"}) — 방향·영점이 아직 가정이다.\n` +
    "팔 주변을 비우고, 팔을 받칠 준비를 한다.\n\n" +
    `arm_control_node 가 max_speed_deg_s ${speed} °/s 로 천천히 보낸다. 서보는 첫 명령에 램프 없이 바로 간다.\n` +
    "멈추려면 위 ■ 전부 정지(Esc) — 노드가 꺼져 팔이 떨어질 수 있다.\n\n보낼까요?");
  if (ok) A.confirmed = true;
  return ok;
}
async function sendJoints(names) {
  const g = S.gate || {}, cal = calJ();
  const ok = names.filter((n) => (g.joints[n] || {}).ok);
  const skip = names.filter((n) => !(g.joints[n] || {}).ok);
  if (!ok.length) { toast("보낼 수 있는 관절이 없음: " + (skip.map((n) => `${n} (${(g.joints[n] || {}).reason || g.reason})`).join(", ") || "바뀐 것 없음"), "error"); return; }
  if (!confirmFirst()) return;
  const joints = Object.fromEntries(ok.map((n) => [n, Math.round(clampR(cal[n], A.val[n]) * 100) / 100]));
  const r = await post("/api/joint/send", { joints });
  if (r.ok) for (const n of ok) { A.base[n] = A.val[n]; A.sent[n] = joints[n]; }
  if (skip.length) toast("안 보낸 관절: " + skip.map((n) => `${n} (${(g.joints[n] || {}).reason})`).join(", "), "error");
}
function syncToActual(quiet) {
  let k = 0;
  for (const [n, j] of Object.entries(calJ())) {
    const p = (S.pose || {})[n];
    const v = p ? p.deg : A.sent[n];
    if (v == null) continue;
    setVal(n, clampR(j, v), true);
    k++;
  }
  if (!quiet) toast(k ? `현재 자세로 ${k}개 관절 맞춤 (AK45 = 피드백, 서보 = 마지막으로 보낸 값)` : "현재 자세 값이 없음 (피드백·보낸 값 없음)");
}
function setVal(n, v, asBase) {
  A.val[n] = v;
  if (asBase) A.base[n] = v;
  const r = A.rows[n];
  if (r) { r.range.value = v; rangeFill(r.range); r.num.value = (Math.round(v * 10) / 10).toString(); }
}
function onUser(n, v) {
  const cal = calJ(), j = cal[n];
  if (!Number.isFinite(v)) return;
  v = clampR(j, v);
  setVal(n, v);
  if (A.link && /^finger\d_joint[12]$/.test(n)) {   // 손가락 1·2·3 같이: 같은 번호 관절을 같은 값으로
    const idx = n.slice(-1);
    for (const f of [1, 2, 3]) { const m = `finger${f}_joint${idx}`; if (m !== n && cal[m]) setVal(m, clampR(cal[m], v)); }
  }
  renderAngles();
}
// 이름 = 실제 움직임 (시뮬 이름은 CAD 에서 와서 roll/pitch/yaw 가 안 맞는다 — arm.xml 0° 자세에서 손이 움직이는 방향으로 확인, 2026-11-20)
const JLABEL = { shoulder_roll: "어깨 앞뒤 (pitch)", shoulder_yaw: "어깨 옆으로 (roll)", upper_arm_roll: "위팔 비틀기 (yaw)", elbow: "팔꿈치 (pitch)",
  wrist_pitch: "손목 비틀기 (W_YAW)", wrist_yaw: "손목 굽힘 (W_PITCH)", hand_roll: "손 비틀기 (W_X)", finger_rotation: "손가락 회전" };
const jLabel = (n) => JLABEL[n] || n.replace(/^finger(\d)_joint(\d)$/, "손가락$1 마디$2");
function angleRow(j) {
  const r = {}, rg = j.cmd_range_deg || [0, 0];
  const motor = j.motor === "ak45" ? `AK45 ID ${j.id}` : `서보 ${j.name} · ID ${j.id}`;
  r.range = el("input", { type: "range", min: rg[0], max: rg[1], step: 0.5, value: 0, "aria-label": jLabel(j.joint) });
  r.num = el("input", { type: "number", min: rg[0], max: rg[1], step: 0.5, value: 0, "aria-label": jLabel(j.joint) + " °" });
  r.range.addEventListener("input", () => onUser(j.joint, parseFloat(r.range.value)));
  r.num.addEventListener("change", () => onUser(j.joint, parseFloat(r.num.value)));
  r.send = el("button", { class: "btn line small", onclick: () => sendJoints([j.joint]) }, "보내기");
  r.cur = el("span", { class: "jcur" }); r.why = el("div", { class: "jwhy" });
  r.root = el("div", { class: "jrow", title: `${j.joint} · ${motor} · ${rg[0]}~${rg[1]}°${j.verified ? "" : " · 보정 미확인"}${j.note ? "\n" + j.note : ""}` },
    el("div", { class: "jn" }, jLabel(j.joint)), r.range, r.num, r.cur, r.send, r.why);
  return r;
}
// ---------- 빠른 손가락 제어 (각도 제어 탭 손가락 카드 맨 위, 2026-11-25 사용자) ----------
// 손가락 첫 마디 3개를 한 번에: 자세 버튼(펴기·벌리기·쥐기), '쥐는 정도' 슬라이더 하나, ±5° 버튼. 마디 2 는 0° (곧게) 로 같이 보낸다.
// 보내는 길은 위 슬라이더의 '보내기' 와 똑같다 (sendJoints: 보낼 수 있는지 검사 + 첫 확인창 + 램프). 값은 아래 슬라이더에도 그대로 보인다.
const FQ_J1 = ["finger1_joint1", "finger2_joint1", "finger3_joint1"], FQ_J2 = ["finger1_joint2", "finger2_joint2", "finger3_joint2"];
const FQ_POSES = [["펴기 0°", 0, "손가락을 원점(곧게)으로"], ["조금 벌리기 −15°", -15, "계획의 HOME 벌림과 같음"],
                  ["크게 벌리기 −60°", -60, "큐브 위에서 내려갈 때와 같음 (서보 끝에서 멈출 수 있다)"], ["쥐기 +5°", 5, "곧게를 지나 눌러 쥠 — 역기구학 실물 재생의 더 쥐기와 비슷"]];
function fingerQuick(cal) {
  const has = FQ_J1.filter((n) => cal[n]);
  if (!has.length) return el("div");
  const lo = Math.max(...has.map((n) => (cal[n].cmd_range_deg || [-90, 90])[0])), hi = Math.min(...has.map((n) => (cal[n].cmd_range_deg || [-90, 90])[1]));
  const send = (v1) => {      // 첫 마디 3개 = v1 (각자 범위로 자름), 마디 2 = 0
    const names = [];
    for (const n of has) { setVal(n, clampR(cal[n], v1)); names.push(n); }
    for (const n of FQ_J2) if (cal[n]) { setVal(n, clampR(cal[n], 0)); names.push(n); }
    renderAngles();
    sendJoints(names);
    num.value = String(Math.round(v1 * 10) / 10); rng.value = v1; rangeFill(rng);
  };
  const cur = () => { const v = has.map((n) => A.val[n] ?? 0); return v.reduce((a, b) => a + b, 0) / v.length; };
  const rng = el("input", { type: "range", min: Math.min(lo, -60), max: Math.max(hi, 10), step: 1, value: 0, "aria-label": "쥐는 정도 (첫 마디 3개)" });
  const num = el("input", { type: "number", min: rng.min, max: rng.max, step: 1, value: 0, style: "width:80px" });
  const live = el("input", { type: "checkbox", checked: "" });
  rng.addEventListener("input", () => { num.value = rng.value; for (const n of has) setVal(n, clampR(cal[n], parseFloat(rng.value))); renderAngles(); });
  rng.addEventListener("change", () => { if (live.checked) send(parseFloat(rng.value)); });     // 손을 놓을 때 한 번 보냄
  num.addEventListener("change", () => { const v = parseFloat(num.value); if (Number.isFinite(v)) { rng.value = v; rangeFill(rng); if (live.checked) send(v); } });
  const poses = el("div", { class: "row", style: "flex-wrap:wrap;gap:6px;margin:6px 0" },
    ...FQ_POSES.map(([lab, v, tip]) => el("button", { class: "btn line small", title: tip, onclick: () => send(v) }, lab)),
    el("button", { class: "btn line small", title: "첫 마디 3개를 5° 더 벌림", onclick: () => send(cur() - 5) }, "−5° 벌림"),
    el("button", { class: "btn line small", title: "첫 마디 3개를 5° 더 오므림", onclick: () => send(cur() + 5) }, "+5° 오므림"));
  const rotRow = cal.finger_rotation ? el("div", { class: "row", style: "flex-wrap:wrap;gap:6px;margin:4px 0" },
    el("span", { class: "help", style: "margin:0" }, "손가락 회전"),
    el("button", { class: "btn line small", onclick: () => { setVal("finger_rotation", clampR(cal.finger_rotation, 0)); renderAngles(); sendJoints(["finger_rotation"]); } }, "0° (HOME·역기구학)"),
    el("button", { class: "btn line small", onclick: () => { setVal("finger_rotation", clampR(cal.finger_rotation, 180)); renderAngles(); sendJoints(["finger_rotation"]); } }, "180° (강화학습 시작)")) : null;
  rangeFill(rng);
  return el("div", { class: "fquick", style: "border:1px solid var(--line, #e5e8eb);border-radius:12px;padding:10px 12px;margin:6px 0 10px" },
    el("div", { style: "font-weight:700;margin-bottom:4px" }, "빠른 손가락 제어 (첫 마디 3개 같이 · 마디 2 는 곧게)"),
    poses,
    el("div", { class: "row", style: "gap:8px;align-items:center;margin:4px 0" }, el("span", { class: "help", style: "margin:0;min-width:64px" }, "쥐는 정도"), rng, num, el("span", { class: "help", style: "margin:0" }, "°")),
    el("label", { class: "help", style: "display:flex;gap:6px;align-items:center;margin:2px 0" }, live, "슬라이더를 놓으면 바로 보내기 (끄면 아래 '보내기'·'바뀐 것 보내기' 로)"),
    rotRow,
    el("div", { class: "help", style: "margin:4px 0 0" }, "− = 벌림, + = 오므림 (0° = 원점, 곧게). 서보 범위 밖은 끝에서 멈춘다. 보내기 전 검사·확인창은 위와 같다."));
}

function buildAngles() {
  const cal = calJ(), box = $("#angles");
  if (!Object.keys(cal).length) return false;
  A.gate = el("div", { class: "gate" });
  A.vb = el("span", { class: "vbadge", hidden: true }, "가상");
  A.bAll = el("button", { class: "btn", onclick: () => sendJoints(Object.keys(cal).filter((n) => Math.abs((A.val[n] ?? 0) - (A.base[n] ?? 0)) > 1e-9)) }, "바뀐 것 보내기");
  A.bSync = el("button", { class: "btn line", onclick: () => syncToActual(), title: "슬라이더를 지금 팔 자세로 (AK45 = 피드백, 서보 = 마지막으로 보낸 값)" }, "지금 자세로 맞추기");
  A.bReset = el("button", { class: "btn line", onclick: () => { for (const n of Object.keys(cal)) setVal(n, A.base[n] ?? 0); renderAngles(); }, title: "아직 안 보낸 슬라이더 변경을 취소" }, "바꾼 것 취소");
  const linkBox = el("input", { type: "checkbox" });
  linkBox.addEventListener("change", () => { A.link = linkBox.checked; });
  A.servoNotes = [];
  const servoNote = () => { const n = el("div", { class: "note warn", hidden: true }); A.servoNotes.push(n); return n; };
  const grp = (title, names, extra) => {
    const card = el("div", { class: "card jgroup" }, el("h3", {}, title), extra || null);
    for (const n of names) if (cal[n]) { const r = A.rows[n] = angleRow(cal[n]); card.append(r.root); }
    return card;
  };
  const ak = Object.values(cal).filter((j) => j.motor === "ak45").map((j) => j.joint);
  const fbody = el("div", {},
    el("label", { class: "help", style: "display:flex;gap:6px;align-items:center;margin:6px 0" }, linkBox, "손가락 1·2·3 같이 움직이기"));
  fbody.append(fingerQuick(cal));   // 2026-11-25: 빠른 손가락 제어 (아래 슬라이더·보내기는 그대로)
  for (const [label, names] of FINGER_SETS) {
    fbody.append(el("div", { class: "fset" }, label));
    for (const n of names) if (cal[n]) { const r = A.rows[n] = angleRow(cal[n]); fbody.append(r.root); }
  }
  const fingers = el("details", { class: "card jgroup fold" }, el("summary", {}, "손가락 (서보 7개)"), servoNote(), fbody);
  box.append(el("div", { class: "card jgroup" },
    el("h3", {}, A.vb, "보내기"),
    A.gate, el("div", { class: "row" }, A.bAll, A.bSync, A.bReset),
    el("details", { class: "log" }, el("summary", {}, "자세히"),
      el("p", { class: "help" }, "슬라이더는 3D 의 파란 목표만 바꾼다. 모터로 가는 것은 '보내기' 뿐: /arm_control/joint_command (sim 관절명, rad) → " +
        "arm_control_node(calibration 변환·클립, max_speed 램프) → arm_homed 면 homing_guard(원점 검사) → ak45_node / stm_bridge_node. " +
        "각도는 시뮬 관절각(°). 줄에 마우스를 올리면 모터 ID·범위가 보인다."))),
    grp("어깨·팔꿈치 (AK45)", ak), grp("손목 (서보)", WRIST, servoNote()), fingers);
  for (const [n, j] of Object.entries(cal)) setVal(n, clampR(j, (S.pose || {})[n] ? S.pose[n].deg : 0), true);
  return true;
}
function shortGate(g) {
  if (g.ok) return `${VIRT() ? "(가상) " : ""}보낼 수 있음${g.homed_mode ? " · 원점 없는 AK45 는 막힘" : " · 원점 검사 없음"}`;
  const r = g.reason || "";
  if (/arm_control_node 가 안 켜짐/.test(r)) return "못 보냄: 팔이 안 켜짐 — 연결 탭 ③ 팔 켜기";
  if (/ROS/.test(r) && !(S.ros || {}).available) return "못 보냄: ROS 연결 안 됨 — 연결 탭 ② (또는 위 '가상')";
  return "못 보냄: " + r;
}
function renderAngles() {
  if (!A.built) A.built = buildAngles();
  if (!A.built) return;
  const g = S.gate || { joints: {} }, cal = calJ();
  A.vb.hidden = !VIRT();
  A.gate.className = "gate " + (g.ok ? "on" : "bad");
  A.gate.textContent = shortGate(g);
  A.gate.title = g.ok ? (g.homed_mode ? "homing_guard 사용 중(원점 없는 AK45 는 막힘)" : "homing_guard 없음(arm.launch / arm_control: AK45 원점 검사 없음)") : g.reason || "";
  let dirty = 0, servoNote = "";
  for (const [n, r] of Object.entries(A.rows)) {
    const jg = g.joints[n] || {}, p = (S.pose || {})[n];
    const isDirty = Math.abs((A.val[n] ?? 0) - (A.base[n] ?? 0)) > 1e-9;
    if (isDirty) dirty++;
    r.root.classList.toggle("dirty", isDirty);
    r.send.disabled = !jg.ok;
    r.send.title = jg.ok ? "이 관절만 보내기" : jg.reason || "";
    r.cur.textContent = p ? f1(p.deg) : "—";
    r.cur.className = "jcur" + (p && p.age > 3 && cal[n] && cal[n].motor === "ak45" ? " old" : "");   // 서보 = 보낸 값이라 오래돼도 정상
    r.cur.title = "지금 값" + (p ? ` (${p.src}${p.age > 3 ? ", " + fmtAge(p.age) : ""})` : " 없음") + (A.sent[n] != null ? ` · 보낸 값 ${f1(A.sent[n])}` : "");
    const isServo = cal[n] && cal[n].motor === "servo";
    if (isServo && jg.ok && jg.note && !servoNote) servoNote = jg.note;
    const why = !jg.ok ? (jg.reason || "") : (!isServo && jg.note) || "";
    r.why.textContent = why; r.why.hidden = !why;
    r.why.className = "jwhy " + (!jg.ok ? "bad" : "warn");
  }
  for (const e of A.servoNotes) { e.hidden = !servoNote; e.textContent = servoNote ? "⚠ " + servoNote.replace(/^첫 명령: /, "서보 첫 명령: ") : ""; }
  A.bAll.disabled = !g.ok || !dirty;
  A.bAll.textContent = dirty ? `바뀐 것 ${dirty}개 보내기` : "바뀐 것 보내기";
}

// ---------- 역기구학 탭: 비전 / 가상 큐브 ----------
const P = {};
function renderPose() {
  const r = S.ros, box = $("#objpose");
  if (!box) return;
  if (!P.root) {
    P.chip = el("span", { class: "chip" }); P.val = el("span", { class: "help", style: "margin:0" });
    P.gb = el("span", { class: "chip", hidden: true });
    P.bUse = el("button", { class: "btn line small", onclick: usePose, title: "/object_pose (frame shoulder_roll_axis) 값을 큐브 칸에 넣는다. frame camera 는 못 넣음. vision_node 켜기/끄기는 연결 탭 '고급'." }, "비전 값 넣기");
    P.vin = {};
    const fld = (k, v, lab) => { const i = el("input", { value: v }); P.vin[k] = i; return el("label", { class: "field" }, lab, i); };
    P.virt = el("details", { class: "log", hidden: true }, el("summary", {}, "가상 큐브 놓기"),
      el("div", { class: "help", style: "margin-top:6px" }, el("span", { class: "vbadge" }, "가상"), " 가상 vision_node 가 이 값을 /object_pose 로 1 Hz 발행"),
      el("div", { class: "minirow" }, fld("x", "0.10", "x m"), fld("y", "-0.30", "y m"), fld("z", "-0.4525", "z m"), fld("yaw", "20", "yaw °"),
        el("button", { class: "btn line", onclick: () => post("/api/virtual/cube", Object.fromEntries(Object.entries(P.vin).map(([k, i]) => [k, parseFloat(i.value)]))) }, "놓기"),
        el("button", { class: "btn line", title: "지도의 주황 큐브 자리·yaw 로 가상 큐브를 놓는다 (z 는 위 칸)", onclick: () => {
          const v = mapVals();
          if (!v.cube) { toast("지도에 큐브가 없음", "error"); return; }
          P.vin.x.value = v.cube.x.toFixed(3); P.vin.y.value = v.cube.y.toFixed(3); P.vin.yaw.value = v.cube.yaw.toFixed(1);
          post("/api/virtual/cube", Object.fromEntries(Object.entries(P.vin).map(([k, i]) => [k, parseFloat(i.value)])));
        } }, "지도 큐브 자리에 놓기")));
    P.root = el("div", { class: "vline" }, el("b", {}, "비전"), P.chip, P.val, P.gb, el("span", { class: "spacer" }), P.bUse);
    box.replaceChildren(P.root, P.virt);
  }
  const vs = svcById("vision");
  P.virt.hidden = !VIRT();
  const o = r.available ? r.object_pose : null;
  const stale = o && o.age > 3;
  setChip(P.chip, !r.available ? "ROS 없음" : o ? (stale ? "오래됨" : "받는 중") : vs.alive ? "기다리는 중" : "꺼짐", !o ? "" : stale ? "warn" : "on");
  P.chip.title = `vision_node ${vs.alive ? "켜짐" : "꺼짐"} · /object_pose ${!r.available ? (r.reason || "ROS 없음") : o ? fmtAge(o.age) + " 받음" : "아직 없음"}`;
  P.val.textContent = o ? `x ${o.x.toFixed(3)} · y ${o.y.toFixed(3)} · yaw ${o.yaw_deg.toFixed(0)}°${o.frame_id !== "shoulder_roll_axis" ? ` (${o.frame_id || "빈 frame"})` : ""}` : "";
  P.val.title = o ? `z ${o.z.toFixed(3)} · frame ${o.frame_id || "(빈)"}` : "";
  P.bUse.disabled = !o || o.frame_id !== "shoulder_roll_axis";
  const vo = o && o.frame_id === "shoulder_roll_axis" && !stale ? o : null;
  graspVision(vo);
  const gv = graspVis(vo);
  P.gb.hidden = !vo;
  P.gb.textContent = !gv ? "집기 판정 중…" : gv.graspable ? "집을 수 있음 ✅" : "못 집음 ❌";
  P.gb.className = "chip" + (gv ? (gv.graspable ? " on" : " bad") : "");
  P.gb.title = gv ? `${gv.reason} · 해석 IK: 손끝을 큐브 중심에, 해 ${gv.n_solutions}개` +
    (gv.tilt_min_deg != null ? `, 손 기울기 ${gv.tilt_min_deg}°` : "") + ` · 어깨 옆 ${gv.shoulder_yaw}°` : "";
  mapVision(o && o.frame_id === "shoulder_roll_axis" && !stale ? o : null);
}
function graspVis(o) {   // 지금 비전 큐브 자리의 판정 (자리가 1 mm 넘게 바뀌었으면 아직 없음)
  return o && G.vis && Math.abs(G.vis.x - o.x) < 1e-3 && Math.abs(G.vis.y - o.y) < 1e-3 ? G.vis : null;
}
function usePose() {
  const o = S.ros.object_pose, c = cards.pick_dry;
  if (!o || !c) return;
  const set = (k, v) => { if (c.fields[k] && !c.fields[k].disabled) c.fields[k].value = v; };
  set("cube_x", o.x.toFixed(3)); set("cube_y", o.y.toFixed(3)); set("cube_yaw", o.yaw_deg.toFixed(1));
  mapUpdate();
  toast("비전 값을 큐브 자리로 넣었다 (지도 주황). 이제 ▶ 계획 돌려 보기");
}

// ---------- 역기구학 탭: 궤적 재생 ----------
const T = { traj: null, mtime: undefined, frame: 0, playing: false, speed: 1, view: "traj" };
async function loadTraj() {
  try {
    const d = await (await fetch("/api/ik/trajectory")).json();
    T.info = d;
    T.traj = d.ok ? d.traj : null;
    T.frame = 0; T.playing = false;
    if (T.slider) { T.slider.max = T.traj ? T.traj.positions.length - 1 : 0; T.slider.value = 0; rangeFill(T.slider); }
  } catch (e) { T.info = { ok: false, error: "궤적을 못 받음" }; }
}
function trajPose(k) {
  const t = T.traj; if (!t) return null;
  const f = t.positions[Math.max(0, Math.min(t.positions.length - 1, Math.round(k)))];
  return Object.fromEntries(t.joint_names.map((n, i) => [n, f[i]]));
}
function buildTraj() {
  const box = $("#traj-ctl");
  T.status = el("div", { class: "note" });
  T.bPlay = el("button", { class: "btn", onclick: () => { if (!T.traj) return; if (T.frame >= T.traj.positions.length - 1) T.frame = 0; T.playing = !T.playing; T.last = performance.now(); } }, "▶ 재생");
  T.slider = el("input", { type: "range", min: 0, max: 0, step: 1, value: 0 });
  T.slider.addEventListener("input", () => { T.frame = parseFloat(T.slider.value); T.playing = false; });
  T.num = el("span", { class: "num" });
  const sp = el("select", {}, ...[1, 2, 5, 10].map((x) => el("option", { value: x }, `${x}×`)));
  sp.addEventListener("change", () => { T.speed = parseFloat(sp.value); });
  const vsel = el("select", {}, el("option", { value: "traj" }, "궤적 프레임 보기"), el("option", { value: "actual" }, "실제 현재 자세 보기"));
  vsel.addEventListener("change", () => { T.view = vsel.value; });
  T.vplay = el("button", { class: "btn", onclick: async () => {
    if (!confirm("가상 팔로 이 궤적을 재생할까요? (가상 arm_control_node 로 프레임을 보낸다. 실물과 상관없음)")) return;
    vsel.value = T.view = "actual"; await post("/api/ik/play_virtual");
  } }, "가상 팔로 재생");
  T.vstop = el("button", { class: "btn line", onclick: () => post("/api/ik/stop_virtual") }, "재생 멈춤");
  T.vrow = el("div", { class: "tbar", hidden: true }, el("span", { class: "vbadge" }, "가상"), T.vplay, T.vstop, T.vinfo = el("span", { class: "num" }));
  sp.title = "재생 속도";
  box.append(el("div", { class: "tbar" }, T.bPlay, T.slider, sp), el("div", { class: "tbar" }, T.status, el("span", { class: "spacer" }), T.num), T.vrow,
    el("details", { class: "log" }, el("summary", {}, "보기 바꾸기"), el("div", { class: "tbar" }, vsel)));
  setInterval(stepTraj, 50);
}
function stepTraj() {
  if (!T.playing || !T.traj) return;
  const now = performance.now(), dt = (now - (T.last || now)) / 1000; T.last = now;
  T.frame = Math.min(T.traj.positions.length - 1, T.frame + dt * T.speed / T.traj.dt);
  if (T.frame >= T.traj.positions.length - 1) T.playing = false;
  T.slider.value = Math.round(T.frame); rangeFill(T.slider);
  updateIkView();
}
function trajScene() {
  // 큐브·놓을 자리: 지금 칸 값, 없으면 궤적의 real(계획에 쓴 값), 그것도 없으면 마지막 실행 칸
  const t = T.traj, p = (T.info && T.info.params) || {};
  const num = (x) => { const v = parseFloat(x); return Number.isFinite(v) ? v : null; };
  let cube = null, target = null, d = num(p.d) ?? 0.5575;
  const v = mapVals();   // 지금 지도·숫자칸 값이 먼저 (고르는 중에도 3D 가 따라오게)
  if (v.cube && v.target) { cube = v.cube; target = v.target; d = v.d ?? (t && t.real && t.real.d) ?? d; }
  else if (t && t.real) { cube = { x: t.real.cube[0], y: t.real.cube[1], yaw: t.real.cube[2] }; target = { x: t.real.target[0], y: t.real.target[1] }; d = t.real.d ?? d; }
  else if (num(p.cube_x) != null) { cube = { x: num(p.cube_x), y: num(p.cube_y), yaw: num(p.cube_yaw) || 0 }; target = { x: num(p.target_x), y: num(p.target_y) }; }
  return { cube, target, d };
}
function renderTraj() {
  if (!T.status) buildTraj();
  if (S.traj_mtime !== T.mtime) { T.mtime = S.traj_mtime; loadTraj().then(updateIkView); }
  const t = T.traj, i = T.info || {};
  if (t) {
    const n = t.positions.length;
    T.status.textContent = `${i.mtime ? new Date(i.mtime * 1000).toLocaleTimeString() + " 계획" : "궤적"}${t.status === "playable" ? "" : " · " + t.status}`;
    T.status.title = `궤적${t.planner === "analytic" ? "(해석 IK 계획, 물리 없음)" : ""}: ${t.status} (${t.reason || ""}) · ${n} 프레임 · ${t.planner === "analytic" ? "" : "sim "}${(n * t.dt).toFixed(1)}초` +
      (t.final_error_m != null ? ` · 놓은 오차 ${(t.final_error_m * 1000).toFixed(1)} mm` : "");
    T.status.className = "note " + (t.status === "playable" ? "" : "warn");
    T.num.textContent = `${(T.frame * t.dt).toFixed(1)} / ${((n - 1) * t.dt).toFixed(1)}초`;
  } else {
    T.status.textContent = "궤적 없음 — ② 계획 돌려 보기";
    T.status.title = i.error || "";
    T.status.className = "note warn";
    T.num.textContent = "";
  }
  T.bPlay.disabled = !t; T.bPlay.textContent = T.playing ? "⏸ 일시정지" : "▶ 재생";
  const v = VIRT(), pb = ((S.ros || {}).virtual_state || {}).playback || {};
  T.vrow.hidden = !v;
  const stale = planStale();
  T.vplay.disabled = !t || pb.running || stale; T.vstop.disabled = !pb.running;
  T.vplay.title = stale ? "값이 바뀜 — 계획 다시" : "";
  T.vinfo.textContent = pb.running ? `재생 중 ${pb.frame}/${pb.frames - 1} (${pb.slow.toFixed(1)}배 느리게, 약 ${pb.seconds.toFixed(0)}초)` + (pb.phase ? " — " + phaseText(pb) : "") : pb.reason ? `마지막: ${pb.reason}` : "";
  updateIkView();
}

// ---------- 3D 뷰 ----------
const VW = {};
const toRad = (degMap) => Object.fromEntries(Object.entries(degMap).map(([k, v]) => [k, v * Math.PI / 180]));
function actualRad() { return toRad(Object.fromEntries(Object.entries(S.pose || {}).map(([k, p]) => [k, p.deg]))); }
function initViewers() {
  // 탭을 처음 열 때 그 탭의 3D 만 만든다(숨은 탭에 WebGL 을 미리 만들면 headless 등에서 안 그려지는 경우가 있었고, 메시 읽기도 줄인다)
  if (!window.JaxViewer || !S) return;
  if (curTab === "angle" && !VW.angle) {
    VW.angle = window.JaxViewer.create($("#v3d-angle"), { ghost: true });
    VW.angle.load(S.robot.mesh_ok).then(renderViewers);
  }
  if (curTab === "ik" && !VW.ik) {
    VW.ik = window.JaxViewer.create($("#v3d-ik"), { ghost: true });   // 유령 = 손 위치로 보내기의 목표 자세
    VW.ik.load(S.robot.mesh_ok).then(() => { graspPaint(); updateIkView(); });
  }
}
window.addEventListener("jaxviewer-ready", initViewers);
function renderViewers() {
  initViewers();
  if (!VW.angle || !S) return;
  const act = actualRad();
  VW.angle.setActual(act);
  // 목표 유령: 슬라이더가 실제와 0.5° 넘게 다르거나 바꾼 것이 있으면
  let show = false;
  for (const [n, v] of Object.entries(A.val)) {
    const p = (S.pose || {})[n];
    // 바꿨는데 안 보냄, 또는 이 화면에서 보냈고 아직 그 자리에 안 감(램프 중)
    if (Math.abs(v - (A.base[n] ?? v)) > 1e-9 || (A.sent[n] != null && p && Math.abs(p.deg - A.sent[n]) > 0.5)) { show = true; break; }
  }
  VW.angle.setGhost(show ? toRad(A.val) : null);
  const o = S.ros.available ? S.ros.object_pose : null;
  VW.angle.setObject(o && o.frame_id === "shoulder_roll_axis" ? { x: o.x, y: o.y, z: o.z, yaw: o.yaw_deg, grasp: (graspVis(o) || {}).graspable } : null);
  const nPose = Object.keys(S.pose || {}).length;
  const vn = $("#v3d-angle-note");
  vn.textContent = nPose ? "" : "지금 자세 값 없음 — 0° 로 그림";
  vn.title = `${VIRT() ? "(가상) " : ""}현재 자세: ${nPose}개 관절 값 (AK45 = 피드백, 서보 = 마지막으로 보낸 값). 값 없는 관절은 0°.`;
}
function updateIkView() {
  if (!VW.ik || !S) return;
  const sc = trajScene();
  VW.ik.setTableD(sc.d);
  VW.ik.setCube(sc.cube);
  VW.ik.setTarget(sc.target);
  const o = S.ros.available ? S.ros.object_pose : null;
  VW.ik.setObject(o && o.frame_id === "shoulder_roll_axis" ? { x: o.x, y: o.y, z: o.z, yaw: o.yaw_deg, grasp: (graspVis(o) || {}).graspable } : null);
  const pbRun = VIRT() && ((((S.ros || {}).virtual_state || {}).playback) || {}).running;   // 가상 재생 중엔 가상 팔(실제 자세)을 보인다
  // 손 위치로 보내기: 풀린 자세를 유령으로(손가락 등 나머지 관절은 지금 값), 목표점은 분홍 표시
  // 유령 = 마지막 좋은 해(HK.good). 지금 입력에서 못 풀면(HK.ghostBad) 빨강.
  const hs = HK.good, hv = hkVals(), hshow = !!(hs && hs.joints);
  VW.ik.setActual(T.view === "actual" || pbRun || !T.traj || hshow || HK.sentAt ? actualRad() : trajPose(T.frame));
  if (hshow) {
    const p = S.pose || {}, reached = !HK.ghostBad && ARM7.every((n) => p[n] && Math.abs(p[n].deg - hs.joints[n]) < 0.5);
    VW.ik.setGhost(reached ? null : Object.assign(actualRad(), toRad(hs.joints)));
    if (VW.ik.setGhostTint) VW.ik.setGhostTint(HK.ghostBad);
  } else VW.ik.setGhost(null);
  if (VW.ik.setHandMark) VW.ik.setHandMark(hv ? { x: hv.x, y: hv.y, z: hv.z } : null);
}

connect();
poll();

// ---------- HOME(모든 관절 0)으로 가기 ----------
const GH = {};
function buildGoHome() {
  GH.btn = el("button", { class: "btn", onclick: goHome }, "⌂ HOME(원점 자세)으로 가기");
  GH.stop = el("button", { class: "btn line", onclick: stopAll }, "멈춤");
  GH.servo = el("button", { class: "btn line", onclick: sendServosHome,
    title: "서보는 위치 피드백이 없어 한 번도 안 보낸 서보는 재생·HOME 이 거절한다. 서보 10개를 모두 원점(0° = 보정의 원점 각)으로 한 번에 보낸다 — 램프 없이 바로 그 각으로 간다" },
    "서보 한 번에 보내기");
  GH.vb = el("span", { class: "vbadge", hidden: true }, "가상");
  GH.note = el("div", { class: "note" });
  GH.card = el("div", { class: "card", style: "margin-bottom:16px" },
    el("h3", {}, GH.vb, "HOME 자세로 가기"),
    el("p", { class: "help" }, "지금 자세에서 HOME(모든 관절 0, 팔이 책상 옆에 곧게 늘어진 자세)으로 간다. " +
      "계획이 HOME 으로 돌아올 때와 같은 길(책상 위 → 팔꿈치 펴며 어깨 젖힘 → 팔 돌림)을 거꾸로 따라가고, 손가락은 벌린다. " +
      "책상 충돌 검사는 꺼져 있다 (책상 없다고 봄)."),
    el("div", { class: "row" }, GH.servo, GH.btn, GH.stop), GH.note);
  $("#gohome").append(GH.card);
}
function renderGoHome() {
  if (!GH.card) buildGoHome();
  const kind = (S.mode || {}).kind, v = VIRT();
  const pb = v ? (((S.ros || {}).virtual_state || {}).playback || {}) : ((S.ros || {}).direct_state || {}).playback || {};
  GH.vb.hidden = !v;
  const why = kind === "ros" ? "ROS 모드에선 아직 없음" : kind === "direct" && !DC() ? "연결 탭 ② 팔 연결 먼저" : pb.running ? "재생 중 — 끝나거나 멈춘 뒤" : "";
  GH.btn.disabled = !!why; GH.btn.title = why;
  GH.stop.disabled = !pb.running;
  GH.note.textContent = pb.running ? (pb.phase ? phaseText(pb) : `가는 중 ${pb.frame}/${pb.frames - 1}`) : pb.reason ? "마지막: " + pb.reason : "";
  // HOME·재생이 끝난 틱: 슬라이더를 지금 자세로 (옛 값으로 '보내기'를 누르면 팔이 되돌아가던 것)
  if (GH.wasRun && !pb.running) syncToActual(true);
  GH.wasRun = !!pb.running;
}
// 계획·HOME 시작 서보 값 (sim °): 손목 3개 0, 손가락 = IK env FINGER_OPEN (첫 마디 −0.26 rad, 둘째 0) = pick_plan open_exit
// finger_rotation 0° = 손가락 3 이 손가락 1·2 와 마주 보는 자세 (역기구학 계획이 가정하는 방향). 강화학습 재생은 이 서보를 180° 로 돌려 두므로 여기서 0 으로 되돌린다.
// 서보 10개 모두 원점 0° (calibration 의 zero_deg 그 각) — 2026-11-25 사용자: 각자 원점으로. 역기구학 계획은 손가락 벌림 −15° 로 시작하지만
// 실물 재생은 손가락 서보가 30° 안이면 그 각까지 램프로 간다 (real_arm.FINGER_START_TOL_DEG)
const SERVO_HOME_DEG = { wrist_pitch: 0, wrist_yaw: 0, hand_roll: 0,
  finger1_joint1: 0, finger1_joint2: 0, finger2_joint1: 0, finger2_joint2: 0, finger3_joint1: 0, finger3_joint2: 0,
  finger_rotation: 0 };
async function sendServosHome() {
  if (!confirm((VIRT() ? "(가상) " : "⚠ 실물 서보가 움직인다.\n\n") + "서보 10개를 모두 원점(0°)으로 한 번에 보낼까요? (각 서보의 보정 원점 각으로 간다)\n" +
    "램프 없이 그 각으로 바로 간다 (빠르게 튄다) — 손 주변을 비우세요.")) return;
  const r = await post("/api/joint/send", { joints: SERVO_HOME_DEG, instant: true });   // 서보만 램프 없이 바로 (2026-11-25 사용자)
  if (r.ok) toast(r.msg || "서보 보냄"); else toast(r.error || "못 보냄", "error");
}
async function goHome() {
  const real = !VIRT();
  if (!confirm((real ? "⚠ 실물 팔이 움직인다.\n\n" : "(가상) ") + "지금 자세에서 HOME(모든 관절 0)으로 갈까요?\n\n" +
    "책상 위로 손을 들고 → 어깨를 젖히며 팔꿈치를 펴고 → 팔을 돌려 내린다. 멈춤 = ■ STOP / Esc" + (real ? "\n\n팔 주변을 비웠나요?" : ""))) return;
  T.view = "actual";
  let r = await post("/api/arm/go_home", { force: false, d: tableD() }, true);
  if (!r.ok && r.need_force) {
    if (!confirm("검사에서 문제가 나왔다:\n\n- " + r.problems.join("\n- ") + "\n\n그래도 HOME 으로 갈까요?")) return;
    r = await post("/api/arm/go_home", { force: true, d: tableD() }, true);
  }
  if (r.ok) toast(r.msg || "HOME 으로");
}

// ---------- 갈 수 있는 곳 (2026-11-21): 손 패널 높이·기준점에서 x, y 마다 위치 역기구학이 풀리나 (집기 조건 없음) ----------
var RX = { on: false, data: null, seq: 0, t: null };
function reachKey() {
  const v = hkVals(); if (!v) return null;
  return { z_cm: v.z_table_cm, ref: v.ref, shoulder_yaw: (v.fix_deg && v.fix_deg.shoulder_yaw) || 0 };
}
async function reachLoad() {
  if (!RX.on) { RX.data = null; reachPaint(); if (RX.note) RX.note.textContent = ""; return; }
  const k = reachKey(); if (!k) return;
  const my = ++RX.seq;
  RX.note.textContent = `계산 중… (높이 ${k.z_cm} cm, ${k.ref === "wrist" ? "손목 중앙" : "손끝 사이"} · 높이마다 몇 초, 한 번 한 높이는 바로)`;
  try {
    const d = await (await fetch(`/api/ik/reachxyz?z_cm=${encodeURIComponent(k.z_cm)}&ref=${k.ref}&shoulder_yaw=${encodeURIComponent(k.shoulder_yaw)}&d=${encodeURIComponent(tableD())}`)).json();
    if (my !== RX.seq) return;
    if (!d.ok) { RX.data = null; RX.note.textContent = "⚠ 계산 실패: " + (d.reason || "?"); reachPaint(); return; }
    RX.data = d;
    RX.note.textContent = `${d.counts.reachable}칸 (${Math.round(d.step * 100)} cm 격자) · ${d.ref === "wrist" ? "손목 중앙" : "손끝 사이"} 책상 위 ${(d.z_table_m * 100).toFixed(1)} cm · 어깨 옆 ${d.shoulder_yaw}° · 높이는 아래 손 위치 칸에서`;
    reachPaint();
  } catch (e) { if (my === RX.seq) RX.note.textContent = "⚠ 서버에서 못 받음"; }
}
function reachPaint() {
  if (!M.gReach) return;
  const d = RX.on ? RX.data : null;
  if (!d) { M.gReach.replaceChildren(); return; }
  const h = d.step * MS;
  let ok = "";
  d.rows.forEach((row, j) => {
    const y = msy(d.ys[j]) - h / 2;
    for (let i = 0; i < row.length;) {
      if (row[i] !== "2") { i++; continue; }
      let k = i; while (k < row.length && row[k] === "2") k++;
      ok += `M${(msx(d.xs[i]) - h / 2).toFixed(1)},${y.toFixed(1)}h${((k - i) * h).toFixed(1)}v${h.toFixed(1)}h${(-(k - i) * h).toFixed(1)}z`;
      i = k;
    }
  });
  const x0 = msx(d.xs[0]) - h / 2, y0 = msy(d.ys[d.ys.length - 1]) - h / 2;
  M.gReach.replaceChildren(
    sv("rect", { x: x0.toFixed(1), y: y0.toFixed(1), width: (d.xs.length * h).toFixed(1), height: (d.ys.length * h).toFixed(1), class: "rno" }),
    sv("path", { d: ok, class: "rok" }));
}
function reachHover(p) {
  const d = RX.data; if (!d || !p) return "";
  const i = Math.round((p.x - d.xs[0]) / d.step), j = Math.round((p.y - d.ys[0]) / d.step);
  if (!(i >= 0 && j >= 0 && j < d.rows.length && i < d.rows[j].length)) return "";
  return d.rows[j][i] === "2" ? ` · ✅ 갈 수 있음 (높이 ${(d.z_table_m * 100).toFixed(1)} cm)` : ` · ❌ 못 감 (높이 ${(d.z_table_m * 100).toFixed(1)} cm)`;
}
