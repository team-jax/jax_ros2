// 탑 쌓기 탭: 실시간 인식 (live.js → GET /api/live/cubes: 거치한 폰의 영상, 가상 모드면 가상 큐브) → 큐브 여러 개 → 한 번에 한 큐브씩 탑 위로.
// 옮기기 = 역기구학 탭과 같은 우리 IK 계획 (강화학습 아님): 계획 = POST /api/svc/pick_dry/start (역기구학 탭 칸 + stack 칸 = 놓는 층·다른 큐브),
// 재생 = 역기구학 탭의 원래 버튼(가상 팔로 재생 / 실물로 재생)을 그대로 누른다 → 확인창·검사·force 확인이 전부 원래 것.
// 걸음 목록 (2026-11-24): 'N층: N번 옮기기' 마다 버튼. 상태가 없다 — 서버(stack_api.steps)가 지금 인식 + 순서·탑 자리만으로 완료·다음·막음을 정한다
// (탑 속 큐브의 어긋남 2 cm · 기울기 15° · 층). 그래서 한 걸음 하고 다른 일을 하다 와도, 페이지·서버를 다시 켜도 이어서 누른다.
// 순서·탑 자리는 서버 logs/stack_plan.json 에 둔다 (GET/POST /api/stack/plan). 버튼은 팔 HOME · 영상 있음 · 집을 수 있음일 때만.
// app.js·direct.js·phone.js 는 안 고친다: 탭 버튼도 여기서 단다 (phone.js 와 같은 방식).
"use strict";
(function () {
  if (!window.JAX_STACK_ON) return;   // 2026-11-25 사용자: 탑 쌓기 탭 없앰 — 예전 페이지가 이 파일을 읽어도 탭을 안 단다 (되살리려면 window.JAX_STACK_ON = true)
  const page = document.getElementById("page-stack");
  if (!page) return;
  const ST = { det: null, grasp: {}, busy: false, going: false, mapMode: "tower", played: false, sawRun: false, view: null, extra: [], att: null, rows: null, endT: 0 };

  TABS.push(["stack", "탑 쌓기", "큐브 여러 개"]);
  TAB_IDS.add("stack");
  const tb = el("a", { class: "tab", href: "#stack", role: "tab" }, "탑 쌓기", el("small", { class: "tsub" }, "큐브 여러 개"), el("span", { class: "tdot" }));
  tabBtns.stack = tb;
  $("#tabs").append(tb);
  const _showTab = showTab;
  showTab = function (id) { _showTab(id); if (curTab === "stack") onShow(); };

  // ---------- 화면 ----------
  const show = (node, text, cls, small) => { node.hidden = false; node.className = "result " + (cls || ""); node.replaceChildren(text, small ? el("small", {}, small) : ""); };
  ST.img = el("img", { class: "st-img", hidden: true, alt: "지금 프레임" });
  ST.src = el("div", { class: "lv-src" });
  ST.res = el("div", { class: "result", hidden: true });
  ST.list = el("div", { class: "st-list", hidden: true });
  ST.order = el("input", { placeholder: "비우면 보이는 큐브 아무거나", inputmode: "numeric", onchange: () => { save(); mapDraw(); spot(); plan(); } });
  const xy = () => { save(); mapDraw(); spot(); plan(); };
  ST.sx = el("input", { inputmode: "decimal", value: lsGet("jaxdash.stack.x") || "-0.16", onchange: xy });
  ST.sy = el("input", { inputmode: "decimal", value: lsGet("jaxdash.stack.y") || "-0.28", onchange: xy });
  ST.syaw = el("input", { inputmode: "decimal", value: "0", onchange: xy });
  ST.map = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  ST.map.setAttribute("class", "st-map");
  ST.bmTower = el("button", { class: "on", onclick: () => setMapMode("tower") }, "탑 자리");
  ST.bmCube = el("button", { onclick: () => setMapMode("cube") }, "가상 큐브");
  ST.mapSw = el("div", { class: "modesw seg", role: "group", hidden: true, style: "margin-top:12px" }, ST.bmTower, ST.bmCube);
  ST.mapNote = el("div", { class: "help", style: "margin:6px 0 0" }, "지도를 눌러 탑 자리를 고른다 (위 = 로봇, 아래 = 앞, 오른쪽 = 로봇 왼쪽 +x).");
  ST.spotNote = el("div", { class: "result", hidden: true });
  ST.steps = el("div", { class: "st-steps" });
  ST.bStop = el("button", { class: "btn red", onclick: () => post(VIRT() ? "/api/ik/stop_virtual" : "/api/ik/stop_real") }, "■ 멈춤");
  ST.note = el("div", { class: "result" }, "큐브를 알아보면 목록이 나온다 (실시간 인식 준비 중)");
  ST.bPlay = el("button", { class: "btn", onclick: () => { if (!T.traj) return; if (T.frame >= T.traj.positions.length - 1) T.frame = 0; T.playing = !T.playing; T.last = performance.now(); } }, "▶ 재생");
  ST.slider = el("input", { type: "range", min: 0, max: 0, step: 1, value: 0 });
  ST.slider.addEventListener("input", () => { T.frame = parseFloat(ST.slider.value); T.playing = false; draw(); });
  ST.num = el("span", { class: "num" });
  document.head.append(el("style", {}, `
#page-stack .st-big { width: 100%; min-height: 60px; font-size: 18px; border-radius: 14px; }
#page-stack .st-row { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 12px; }
#page-stack .st-row > .btn { flex: 1 1 130px; min-height: 48px; }
#page-stack .st-img { display: block; width: 100%; max-height: 46vh; object-fit: contain; border-radius: 12px; background: var(--fill); margin-top: 14px; }
#page-stack .result { margin-top: 12px; }
#page-stack .result small { display: block; font-weight: 500; margin-top: 4px; }
#page-stack .st-list { margin-top: 12px; overflow-x: auto; }
#page-stack .st-list table.jt td, #page-stack .st-list table.jt th { padding: 8px 6px; }
#page-stack .st-list tr.top td { font-weight: 700; }
#page-stack .frow .field { width: 92px; }
#page-stack .frow .field.wide { width: 170px; }
#page-stack .st-map { display: block; width: 100%; max-width: 460px; margin: 12px auto 0; border-radius: 12px; background: var(--fill); touch-action: none; cursor: crosshair; }
#page-stack .frow .field input, #page-stack .frow .field select { font-size: 16px; }
#page-stack ol.help { padding-left: 20px; margin: 10px 0 0; line-height: 1.6; }
#page-stack .st-steps { margin-top: 14px; display: flex; flex-direction: column; gap: 8px; }
#page-stack .st-step { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
#page-stack .st-step > .btn { flex: 1 1 220px; min-height: 52px; font-size: 17px; border-radius: 12px; }
#page-stack .st-step .st-done { flex: 1 1 220px; min-height: 52px; display: flex; align-items: center; justify-content: center; border-radius: 12px; font-weight: 700; background: var(--fill); color: #15803d; }
#page-stack .st-step .st-done.st-bad { color: #b91c1c; }
#page-stack .st-step small { flex: 1 1 100%; color: var(--sub); }
@media (max-width: 980px) { #page-stack .split-r { order: 1; } }
@media (max-width: 640px) { #page-stack .frow .field.wide { width: 100%; } }`));
  $("#stack-left").append(
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "1"), "큐브 인식"),
    el("div", { class: "card" },
      ST.src, ST.img, ST.res, ST.list),
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "2"), "탑"),
    el("div", { class: "card" },
      el("div", { class: "frow" }, el("b", {}, "순서"), el("label", { class: "field wide" }, el("span", {}, "옮길 큐브 번호", el("small", {}, " 차례대로, 비워도 됨")), ST.order)),
      el("div", { class: "frow" }, el("b", {}, "탑 자리"),
        el("label", { class: "field" }, el("span", {}, "x", el("small", {}, " m")), ST.sx), el("label", { class: "field" }, el("span", {}, "y", el("small", {}, " m")), ST.sy),
        el("label", { class: "field" }, el("span", {}, "각", el("small", {}, " °")), ST.syaw)),
      el("div", { class: "help", style: "margin:8px 0 0" }, "역기구학 탭과 같은 좌표: 어깨 축 바로 아래가 (0, 0), +x = 로봇 왼쪽, 앞(책상 쪽) = −y. 순서의 첫 큐브를 이 자리 바닥에 놓고 그 위로 쌓는다. 책상 위에는 큐브를 하나씩 (탑에서 8 cm 넘게, 15 cm 이상 권장) 두면 된다 — 순서의 다음 번호가 없으면 보이는 큐브를 옮긴다."),
      ST.mapSw, ST.map, lvTools(), ST.mapNote, ST.spotNote,
      ST.steps,
      el("div", { class: "st-row" }, ST.bStop),
      ST.note,
      el("div", { class: "help", style: "margin:8px 0 0" }, "버튼 하나 = 큐브 하나: 계획(역기구학 탭과 같은 IK, 손은 수직) → 지금 모드의 팔로 재생할지 확인창. 다음 층 버튼은 카메라가 아래 층을 탑 자리에서 알아봐야 켜진다 (놓인 큐브 어긋남 2 cm · 기울기 15° · 층 확인). 언제든 이어서 누르면 된다.")),
    el("details", { class: "adv" }, el("summary", {}, "준비 — 인쇄물 · 놓는 법"),
      el("div", { class: "card" },
        el("a", { class: "btn line", href: "/api/stack/sheet", target: "_blank" }, "큐브 마커 PDF 열기 (4x4 ID 0~4, 40 mm)"),
        el("ol", { class: "help" },
          el("li", {}, "실제 크기 100% 로 인쇄해 큐브마다 다른 번호를 윗면에 붙인다. 기준 종이는 없어도 된다 (쓰면 휴대폰 카메라 탭의 것·자리 설정 그대로)."),
          el("li", {}, "큐브끼리 15 cm 이상 띄운다 — 벌린 손가락이 닫히며 큐브 중심에서 12 cm 쯤까지 나온다."),
          el("li", {}, "탑 자리는 로봇 오른쪽 앞 (x −0.16, y −0.28 근처, 지도의 진한 초록) 이 좋다: 3층째를 놓을 수 있는 곳이 좁고, 팔이 드나드는 자리 (x 0, y −0.34) 에서 15 cm 넘게 떨어져야 손가락이 탑에 안 닿는다."),
          el("li", {}, "이미 놓여 있는 큐브 위에 쌓으려면 탑 자리를 그 큐브 자리로 (지도에서 그 큐브를 누름) — 그 큐브는 순서에서 빼도 된다."),
          el("li", {}, "기본 책상 높이에서는 3층까지 (4층째는 팔이 못 닿는다).")))));
  $("#stack-ctl").append(el("div", { class: "tbar" }, ST.bPlay, ST.slider), el("div", { class: "tbar" }, el("span", { class: "spacer" }), ST.num));

  // ---------- 실시간 인식 결과 → 큐브 목록 (live.js 가 0.5초마다. 사진 올리기는 없다) ----------
  LQ.on((all) => { if (curTab === "stack") setDet(all); });
  function setDet(all) {
    const virt = all.mode === "virtual";
    const cubes = (all.cubes || []).filter((c) => c.id != null);   // 번호 없는 뎁스 큐브는 탑 쌓기에 안 씀
    const r = { ok: !all.stale && (cubes.length > 0 || virt), stale: !!all.stale, cubes, refs: all.refs || [], warn: all.warn || [], reason: all.reason, t: all.t_frame, virt };
    const sig = JSON.stringify([r.ok, cubes.map((c) => [c.id, Math.round(c.x * 1000), Math.round(c.y * 1000), Math.round(c.yaw_deg), c.level, Math.round(c.tilt_deg || 0)])]);
    ST.mapSw.hidden = !virt; if (!virt && ST.mapMode !== "tower") setMapMode("tower");
    ST.src.textContent = lvSrcText(all);
    const live = !virt && !all.stale;   // 그림 = MJPEG 스트림 (한 번 걸고, 영상 있음/없음이 바뀔 때만 다시)
    if (live !== ST.imgLive) { ST.imgLive = live; ST.img.hidden = !live; streamSet(ST.img, live); }
    ST.det = r; ST.all = all;
    if (sig === ST.sig) { mapDraw(); return; }
    ST.sig = sig;
    const old = ST.grasp; ST.grasp = {};
    for (const c of cubes) { const g = old[c.id]; if (g && g.at && Math.hypot(g.at[0] - c.x, g.at[1] - c.y) < 0.003) ST.grasp[c.id] = g; }
    const warn = (r.warn || []).join(" · ");
    if (!r.ok) { show(ST.res, r.stale ? "영상 없음 — 탑 쌓기 막음" : r.reason || "큐브를 못 찾음", "bad", r.stale ? (all.reason || "") : warn); ST.list.hidden = true; baseUI(); plan(); return; }
    show(ST.res, virt ? `가상 큐브 ${cubes.length}개` : `큐브 ${cubes.length}개 보임`, warn ? "warn" : "ok", virt ? "지도에서 '가상 큐브' 를 골라 놓기·끌기·돌리기" : `기준 마커 ${r.refs.length}개` + (warn ? " · " + warn : ""));
    baseUI();
    if (ST.spotOk == null) spot();
    for (const c of cubes) if (c.level === 0 && !ST.grasp[c.id]) graspable(c);
    table();
    plan();
  }
  function setMapMode(m) {
    ST.mapMode = m; ST.bmTower.classList.toggle("on", m === "tower"); ST.bmCube.classList.toggle("on", m === "cube");
    ST.mapNote.textContent = m === "cube" ? "가상 큐브: 빈 곳을 누르면 놓기, 끌면 옮기기 (다른 큐브 위에 놓으면 쌓임), 누르면 골라서 각·지우기." : "지도를 눌러 탑 자리를 고른다.";
  }
  async function graspable(c) {
    const det = ST.det;
    let g;
    try {
      g = await (await fetch("/api/ik/graspable", { method: "POST", headers: { "content-type": "application/json" },
        body: JSON.stringify({ x: c.x, y: c.y, d: tableD(), grasp: "vertical" }) })).json();
    } catch (e) { g = { ok: false }; }
    if (ST.det !== det) return;
    g.at = [c.x, c.y];
    ST.grasp[c.id] = g;
    table();
  }
  function table() {
    const r = ST.det;
    if (!r || !r.ok) { ST.list.hidden = true; return; }
    const top = ST.step && ST.step.top, nxt = ST.step && ST.step.cube;
    const rows = r.cubes.slice().sort((a, b) => a.id - b.id).map((c) => {
      const g = ST.grasp[c.id];
      const what = top && top.id === c.id ? "탑 맨 위" : nxt && nxt.id === c.id ? "다음" : "";
      const gt = c.level > 0 ? "—" : !g ? "…" : !g.ok ? "?" : g.graspable ? "✓" : "✗";
      return el("tr", { class: what ? "top" : "", title: g && g.ok && !g.graspable ? g.reason || "" : "" },
        el("td", {}, String(c.id), what ? el("small", {}, " " + what) : ""), el("td", { class: "n" }, c.x.toFixed(3)), el("td", { class: "n" }, c.y.toFixed(3)),
        el("td", { class: "n" }, c.yaw_deg.toFixed(0) + "°"), el("td", { class: "n" }, `${c.level + 1}층`),
        el("td", { class: c.level > 0 || !g || !g.ok ? "" : g.graspable ? "ok" : "no" }, gt));
    });
    ST.list.hidden = false;
    ST.list.replaceChildren(el("table", { class: "jt" },
      el("thead", {}, el("tr", {}, ...["큐브", "x (m)", "y (m)", "각", "높이", "집기"].map((h, i) => el("th", { class: i > 0 && i < 5 ? "n" : "", style: i > 0 && i < 5 ? "text-align:right" : i === 5 ? "text-align:center" : "" }, h)))), el("tbody", {}, ...rows)));
  }

  // ---------- 탑 자리·순서 ----------
  const ids = () => (ST.det && ST.det.ok ? ST.det.cubes.map((c) => c.id).sort((a, b) => a - b) : []);
  function baseUI() {
    mapDraw();
  }
  function chosen() {   // 지금 고른 탑 자리·순서. 틀리면 { error }
    const seq = ST.order.value.split(/[\s,]+/).filter(Boolean).map(Number);
    if (seq.some((i) => !Number.isInteger(i)) || new Set(seq).size !== seq.length) return { error: "순서는 큐브 번호를 쉼표로 (예: 2, 0, 1), 같은 번호는 한 번만 — 비우면 보이는 큐브 아무거나" };
    const x = parseFloat(ST.sx.value), y = parseFloat(ST.sy.value), yaw = parseFloat(ST.syaw.value) || 0;
    if (!Number.isFinite(x) || !Number.isFinite(y)) return { error: "탑 자리 x, y 숫자 (m)" };
    return { base: { x, y, yaw_deg: yaw }, order: seq };
  }
  function save() {   // 순서·탑 자리 → 서버 (logs/stack_plan.json: 페이지를 다시 열어도, 다른 기기에서도 같은 값)
    const ch = chosen();
    if (!ch.error) post("/api/stack/plan", ch, true);
  }
  async function load() {
    try {
      const r = await (await fetch("/api/stack/plan")).json();
      if (r.order) ST.order.value = r.order.join(", ");
      if (r.base) { ST.sx.value = (+r.base.x).toFixed(3); ST.sy.value = (+r.base.y).toFixed(3); ST.syaw.value = String(r.base.yaw_deg || 0); }
    } catch (e) { /* 없으면 기본값 */ }
    ST.loaded = true; ST.sig = null;
    baseUI(); spot(); plan();
  }

  // ---------- 탑 자리: 지도 (놓을 수 있는 곳) + 그 자리 확인 ----------
  const MX0 = -0.27, MX1 = 0.23, MY0 = -0.45, MY1 = -0.13, K = 1000;   // 실좌표 m → 그림: x 오른쪽 = +x, 아래 = −y (앞)
  const sv = (tag, at, ...kids) => { const e = document.createElementNS("http://www.w3.org/2000/svg", tag); for (const [k, v] of Object.entries(at)) e.setAttribute(k, v); e.append(...kids); return e; };
  const px = (x) => (x - MX0) * K, py = (y) => (MY1 - y) * K;
  ST.map.setAttribute("viewBox", `0 0 ${(MX1 - MX0) * K} ${(MY1 - MY0) * K}`);
  function mapLoad() {
    const d = tableD();
    if (ST.mapD === d || ST.mapBusy) return;
    ST.mapBusy = true; ST.mapNote.textContent = "놓을 수 있는 곳 계산 중… (처음 한 번 10~40초)";
    fetch("/api/stack/map?d=" + encodeURIComponent(d)).then((r) => r.json()).then((r) => {
      ST.mapBusy = false; ST.mapD = d; ST.mapData = r.ok ? r : null;
      ST.mapNote.textContent = r.ok ? "지도를 눌러 탑 자리를 고른다. 초록 = 놓을 수 있는 곳 (진할수록 높이: 1·2·3층), 위 = 로봇 · 아래 = 앞 · 오른쪽 = 로봇 왼쪽(+x)." : "지도를 못 만듦: " + (r.error || "");
      mapDraw();
    }).catch(() => { ST.mapBusy = false; ST.mapD = d; });
  }
  function mapDraw() {
    const g = [], m = ST.mapData, need = (chosen().order || []).length || 3;
    if (m) m.rows.forEach((row, j) => { for (let i = 0; i < row.length; i++) { const n = +row[i]; if (n) g.push(sv("rect", { x: px(m.xs[i] - m.step / 2), y: py(m.ys[j] + m.step / 2), width: m.step * K, height: m.step * K,
      fill: n >= Math.min(need, 3) ? "#16a34a" : "#86efac", "fill-opacity": 0.25 + 0.2 * Math.min(n, 3) })); } });
    g.push(sv("circle", { cx: px(0), cy: py(-0.145), r: 7, fill: "#64748b" }), sv("text", { x: px(0) + 12, y: py(-0.145) + 5, "font-size": 15, fill: "#64748b" }, "로봇 쪽 (어깨 = x 0, y 0)"));
    g.push(...lvDrawCubes(px, py));
    const x = parseFloat(ST.sx.value), y = parseFloat(ST.sy.value);
    if (Number.isFinite(x) && Number.isFinite(y)) g.push(sv("rect", { x: px(x) - 29, y: py(y) - 29, width: 58, height: 58, fill: "none", stroke: "#1d4ed8", "stroke-width": 4, "stroke-dasharray": "9 5", rx: 4,
      transform: `rotate(${-(parseFloat(ST.syaw.value) || 0)} ${px(x)} ${py(y)})` }), sv("text", { x: px(x), y: py(y) - 36, "font-size": 15, "font-weight": 700, fill: "#1d4ed8", "text-anchor": "middle" }, "탑"));
    ST.map.replaceChildren(...g);
  }
  function towerAt(x, y, near) {   // 지도에서 탑 자리 고르기. 큐브를 누르면 그 큐브 자리 (그 위에 쌓기)
    if (near) { x = near.x; y = near.y; ST.syaw.value = near.yaw_deg.toFixed(0); } else { x = Math.round(x * 100) / 100; y = Math.round(y * 100) / 100; }
    ST.sx.value = x.toFixed(3); ST.sy.value = y.toFixed(3);
    xy();
  }
  lvMapAttach(ST.map, { box: [MX0, MX1, MY0, MY1], redraw: mapDraw, edit: () => LQ.virtual() && ST.mapMode === "cube",
    onCube: (c) => { if (ST.mapMode === "tower") towerAt(c.x, c.y, c); },
    onEmpty: (x, y) => { if (ST.mapMode === "cube" && LQ.virtual()) return false; towerAt(x, y, null); return true; } });
  let spotSeq = 0;
  async function spot() {   // 그 자리에 순서의 큐브 수만큼 층을 놓을 수 있나 (계획기와 같은 판정)
    const seq = ++spotSeq, ch = chosen();
    ST.spotOk = null;
    if (ch.error) { ST.spotNote.hidden = true; return; }
    lsSet("jaxdash.stack.x", ST.sx.value); lsSet("jaxdash.stack.y", ST.sy.value);
    let r;
    try {
      r = await (await fetch("/api/stack/spot", { method: "POST", headers: { "content-type": "application/json" },
        body: JSON.stringify({ x: ch.base.x, y: ch.base.y, yaw_deg: ch.base.yaw_deg, levels: Math.min(8, ch.order.length || 3), d: tableD() }) })).json();
    } catch (e) { r = { ok: false, text: "탑 자리 확인 못 함 (서버 연결)" }; }
    if (seq !== spotSeq) return;
    ST.spotOk = !!r.ok;
    show(ST.spotNote, `탑 자리 (${ch.base.x.toFixed(3)}, ${ch.base.y.toFixed(3)}): ` + (r.text || ""), r.ok ? "ok" : "bad");
  }
  // ---------- 걸음 목록 (서버가 지금 인식으로 판단, 상태 없음) ----------
  let planSeq = 0;
  async function plan() {
    const seq = ++planSeq;
    if (!ST.loaded) return;
    if (!ST.det || !ST.det.ok) { ST.rows = null; ST.step = null; ST.stepText = ST.det && ST.det.stale ? "영상 없음" : "큐브를 못 찾음"; table(); return; }
    const ch = chosen();
    if (ch.error) { ST.rows = null; ST.step = null; ST.stepText = ch.error; table(); return; }
    let r;
    try {
      r = await (await fetch("/api/stack/steps", { method: "POST", headers: { "content-type": "application/json" },
        body: JSON.stringify({ cubes: ST.det.cubes, base: ch.base, order: ch.order, level_unsure: !!(ST.all && ST.all.level_unsure) }) })).json();
    } catch (e) { r = { ok: false, text: "서버 연결 실패" }; }
    if (seq !== planSeq) return;
    ST.rows = r.ok ? r.steps : null;
    ST.step = r.next ? { ok: true, done: false, ...r.next } : { ok: false, done: !!r.done };
    ST.stepText = r.text || ""; ST.stepBad = !r.ok || !!r.bad; ST.stepWarn = (r.warn || []).join(" · ");
    table(); scene(); draw();
  }
  const AK4 = ["shoulder_roll", "shoulder_yaw", "upper_arm_roll", "elbow"];
  function gate() {   // 다음 걸음 버튼을 막는 화면 쪽 조건 (서버가 본 것 = 탑·큐브 상태, 여기 = 팔·영상·집기). 막으면 이유, 아니면 ""
    const kind = S.mode && S.mode.kind, st = ST.step;
    const pb = (kind === "virtual" ? (S.ros.virtual_state || {}) : (S.ros.direct_state || {})).playback || {};
    if (pb.running) return `재생 중 ${pb.frame}/${pb.frames - 1}`;
    if (ST.going) return "계획 중…";
    if (!LQ.r || LQ.r.stale) return "영상 없음";
    if (LQ.r.td_mismatch) return "책상 높이가 이 화면 d 칸과 다르게 계산된 결과 — 다시 계산 중";
    if (LQ.r.level_unsure) return "큐브 층이 확실하지 않음 (기준 종이를 같이 보이게 하거나 큐브를 폰 가까이)";
    const us = LQ.cubes().filter((c) => c.unsure);   // 집을 큐브·탑·피할 큐브 중 하나라도 층·자리가 확실하지 않으면 (지금 프레임)
    if (us.length) return `큐브 ${us.map((c) => c.id).join(", ")} 의 층·자리가 확실하지 않음 (마커 한 변이 안 맞음)`;
    const p = S.pose || {};
    if (!AK4.every((n) => p[n]) || ARM7.some((n) => p[n] && Math.abs(p[n].deg) >= 5)) return "팔이 HOME 이 아님 (AK45 4개 피드백 + 보낸 서보 값이 0° ±5°)";   // 서보는 보낸 값만 안다 (아직 안 보냈으면 없음)
    if (LQ.r.mode !== "virtual" && (LQ.r.t_frame || 0) <= ST.endT + 0.3) return "재생이 끝난 뒤의 새 프레임을 기다림";
    const g = st && st.cube && ST.grasp[st.cube.id];
    if (!g) return "집을 수 있나 확인 중…";
    if (g.ok && !g.graspable) return `큐브 ${st.cube.id} 는 못 집는 자리: ${g.reason || ""}`;
    if (ST.spotOk === false) return "탑 자리에 그 층까지 못 놓음 (위 탑 자리 확인)";
    if (!cards.pick_dry) return "역기구학 계획 항목이 없다";
    return "";
  }
  function renderSteps() {   // 0.3초마다: 걸음마다 한 줄 (완료 / 버튼 / 막힌 이유)
    const rows = ST.rows, kind = S.mode && S.mode.kind, why = gate(), sig = JSON.stringify([rows, why, kind]);
    if (sig === ST.rowSig) return;
    ST.rowSig = sig;
    if (!rows) { ST.steps.replaceChildren(el("div", { class: "help" }, ST.stepText || "")); }
    else ST.steps.replaceChildren(...rows.map((q) => {
      if (q.state === "bad") return el("div", { class: "st-step" }, el("div", { class: "st-done st-bad" }, `✗ ${q.text} — 문제`), el("small", {}, q.why));
      if (q.state === "done") return el("div", { class: "st-step" }, el("div", { class: "st-done" }, `✓ ${q.text} — 완료`), el("small", {}, q.why));
      const on = q.state === "next" && !why;
      const b = el("button", { class: "btn" + (q.state === "next" ? "" : " line"), disabled: !on, onclick: () => go(q.n) }, q.text + (on ? ` (${kind === "virtual" ? "가상 팔" : "실물 팔"})` : ""));
      return el("div", { class: "st-step" }, b, el("small", {}, q.state === "next" ? why : q.why));
    }));
  }
  function line() {   // 상태 한 줄
    if (ST.going || ST.played || Date.now() < (ST.holdUntil || 0)) return;   // 방금 띄운 계획·재생 글은 잠시 둔다
    const r = ST.step, why = r && r.ok ? gate() : "";
    show(ST.note, ST.stepText || "", ST.stepBad ? "bad" : r && r.done ? "ok" : why || ST.stepWarn ? "warn" : "", [why, ST.stepWarn].filter(Boolean).join(" · "));
  }

  // ---------- 탑 쌓기 = 한 큐브: 역기구학 탭의 칸을 채우고 ▶ 계획 → (통과하면) 그 탭의 재생 버튼 ----------
  const sleep = (ms) => new Promise((ok) => setTimeout(ok, ms));
  async function wait(cond, ms) {
    for (const t0 = Date.now(); Date.now() - t0 < ms; await sleep(200)) if (cond()) return true;
    return false;
  }
  async function go(n) {
    const c = cards.pick_dry, st = ST.step;
    if (!st || !st.ok || st.done || st.n !== n || gate()) return;
    ST.going = true;
    try {
      save();   // 누른 순서·탑 자리를 남긴다 (기본값 그대로 눌러도)
      if (!c.fields.stack) c.fields.stack = el("input", { type: "hidden" });   // 역기구학 탭·휴대폰 탭의 계획은 이 칸을 비워 보낸다
      const g0 = c.fields.grasp ? c.fields.grasp.value : null;   // 탑 쌓기는 수직으로 — 역기구학 탭의 집는 방향은 되돌린다 (2026-11-25 검토)
      for (const [k, v] of Object.entries(st.params)) c.fields[k].value = v;
      mapUpdate();
      const params = Object.fromEntries(Object.entries(c.fields).map(([k, i]) => [k, i.type === "checkbox" ? (i.checked ? "1" : "") : i.value]));
      c.fields.stack.value = "";
      if (c.fields.grasp && g0 !== null) c.fields.grasp.value = g0;
      const m0 = S.traj_mtime, st0 = svcById("pick_dry").started;
      const head = `${st.level + 1}층: ${st.cube.id}번 옮기기`;
      show(ST.note, head + ": 계획 중…", "run");
      const r = await post("/api/svc/pick_dry/start", { params });
      if (!r.ok) { show(ST.note, head + ": 계획을 못 시작함", "bad", r.error || ""); return; }
      const done = await wait(() => { const v = svcById("pick_dry"); return v.started !== st0 && !v.alive && v.exit_code != null; }, 180000);
      const sv = svcById("pick_dry");
      if (!done) { show(ST.note, head + ": 계획이 3분 넘게 안 끝남", "bad", "역기구학 탭에서 확인"); return; }
      const [txt, cls] = pickSummary(sv);
      const got = sv.exit_code !== 3 && await wait(() => S.traj_mtime != null && S.traj_mtime !== m0 && T.mtime === S.traj_mtime && T.info && T.info.mtime === S.traj_mtime, 4000);
      if (!got || !T.traj) { show(ST.note, head + ": 계획 실패 — 팔은 안 움직임", "bad", txt || `종료코드 ${sv.exit_code}`); return; }
      // 이 계획이 정말 이 걸음(놓는 층·다른 큐브)으로 돌았나 — 아니면 탑 속으로 놓으러 가는 계획일 수 있다
      const real = T.traj.real || {};
      if ((real.place_level || 0) !== st.level || (real.obstacles || []).length !== st.obstacles.length) {
        show(ST.note, head + ": 계획이 탑 쌓기 값 없이 돌았다 — 재생 안 함", "bad", "다시 누르기 (역기구학 탭에서 그 사이 계획을 돌렸는지 확인)"); return;
      }
      scene(); T.frame = 0; draw();
      if (T.traj.status !== "playable") { show(ST.note, head + ": 계획이 검사에 걸림 — 재생 안 함", "bad", txt || T.traj.reason || ""); return; }
      const pass = head + ": " + (txt || "계획 나옴");
      await sleep(600);
      const kind = S.mode && S.mode.kind;
      const btn = kind === "virtual" ? T.vplay : kind === "direct" ? (typeof D !== "undefined" && D.play && D.play.go) : null;
      if (!btn) { show(ST.note, pass, cls, "이 모드의 재생은 역기구학 탭에서"); return; }
      if (btn.disabled) { show(ST.note, pass, "warn", "재생은 아직 못 함: " + (btn.title || (kind === "direct" ? "연결 탭에서 팔 연결·원점 먼저" : "이미 재생 중"))); return; }
      show(ST.note, pass, cls, "확인창에서 재생을 고르면 팔이 움직인다");
      ST.played = true; ST.sawRun = false; ST.head = head; ST.cur = { level: st.level, id: st.cube.id };
      btn.click();   // 원래 버튼 = 원래 확인창 (가상: 1번, 실물: 2번 + 검사 문제면 한 번 더)
      setTimeout(() => {   // 확인창을 취소했거나 원래 검사가 거절하면 (팔 꺼짐·원점 없음 …) 재생이 안 시작된다 — 이유는 화면 위 알림
        const k = S.mode && S.mode.kind, p = (k === "virtual" ? (S.ros.virtual_state || {}) : (S.ros.direct_state || {})).playback || {};
        if (ST.played && !ST.sawRun && !p.running) { ST.played = false; ST.holdUntil = Date.now() + 15000; show(ST.note, pass, "warn", "재생이 시작되지 않았다 — 확인창을 취소했거나 거절됨 (화면 아래 알림). 고친 뒤 다시 누르기"); }
      }, 4000);
    } finally { ST.going = false; if (!ST.played) ST.holdUntil = Date.now() + 15000; }
  }

  // ---------- 3D: 사진의 큐브들 + 지금 계획 다시 보기 ----------
  function onShow() {
    if (!ST.view && window.JaxViewer && S && S.robot) {
      ST.view = window.JaxViewer.create($("#v3d-stack"), {});
      ST.view.load(S.robot.mesh_ok).then(() => { scene(); draw(); });
    }
    if (ST.view && ST.view.resize) ST.view.resize();
    lvPoll();
    scene(); draw(); mapLoad(); if (ST.spotOk == null) spot();
  }
  window.addEventListener("jaxviewer-ready", () => { if (curTab === "stack") onShow(); });
  const CUBE = 0.05, COL = { tower: 0x2563eb, loose: 0xf97316, next: 0xf97316, other: 0x94a3b8 };
  function scene() {   // 큐브 상자들: 사진에서 본 것 + 탑 속에 가려진 것. 다음에 옮길 큐브는 v.cube (주황), 놓을 자리는 v.target (초록)
    const v = ST.view;
    if (!v || v.dead || !v.cube) return;
    const d = tableDNum();
    v.setTableD(d);
    for (const m of ST.extra) { v.scene.remove(m); m.material.dispose(); }
    ST.extra = []; ST.att = null;
    const st = ST.step, det = ST.det;
    const box = (x, y, yaw, level, color, opacity) => {
      const m = v.cube.clone();
      m.material = v.cube.material.clone(); m.material.color.setHex(color);
      if (opacity < 1) { m.material.transparent = true; m.material.opacity = opacity; }
      m.position.set(x, y, -d + CUBE * (level + 0.5)); m.rotation.set(0, 0, yaw * Math.PI / 180); m.visible = true;
      v.scene.add(m); ST.extra.push(m);
    };
    const nid = st && st.cube ? st.cube.id : null;
    if (st && st.ok && !st.done) for (const o of st.obstacles) box(o[0], o[1], o[2], o[3], st.target && Math.hypot(o[0] - st.target.x, o[1] - st.target.y) < 1e-3 ? COL.tower : COL.other, 1);
    else if (det && det.ok) for (const c of det.cubes) for (let k = 0; k <= c.level; k++) box(c.x, c.y, c.yaw_deg, k, k === c.level ? COL.tower : COL.other, 1);
    v.cube.visible = nid != null; v.target.visible = nid != null;
    v.dirty();
  }
  function trajIsStep() {   // 지금 궤적이 화면의 걸음(다음 큐브)의 계획인가
    const st = ST.step, r = T.traj && T.traj.real;
    return !!(st && st.ok && !st.done && r && Math.abs(r.cube[0] - st.cube.x) < 1e-3 && Math.abs(r.cube[1] - st.cube.y) < 1e-3 && (r.place_level || 0) === st.level);
  }
  function draw() {
    const v = ST.view, st = ST.step;
    if (curTab !== "stack") return;
    const t = trajIsStep() ? T.traj : null;
    ST.bPlay.disabled = !t; ST.bPlay.textContent = T.playing && t ? "⏸ 일시정지" : "▶ 재생";
    ST.slider.max = t ? t.positions.length - 1 : 0; ST.slider.disabled = !t;
    if (t) { ST.slider.value = Math.round(T.frame); rangeFill(ST.slider); }
    ST.num.textContent = t ? `${(T.frame * t.dt).toFixed(1)} / ${((t.positions.length - 1) * t.dt).toFixed(1)}초` : "계획을 돌리면 여기서 다시 본다";
    if (!v || v.dead || !v.ready) return;
    const kind = S.mode && S.mode.kind;
    const pb = (kind === "virtual" ? (S.ros.virtual_state || {}) : (S.ros.direct_state || {})).playback || {};
    const k = t ? Math.max(0, Math.min(t.positions.length - 1, Math.round(T.frame))) : 0;
    v.setActual(pb.running || !t ? actualRad() : trajPose(k));
    if (!st || !st.ok || st.done || !st.cube) { v.dirty(); return; }
    const d = tableDNum(), c = st.cube, tg = st.target;
    v.target.position.set(tg.x, tg.y, -d + CUBE * (st.level + 0.5)); v.target.rotation.set(0, 0, tg.yaw_deg * Math.PI / 180);
    const home = () => { v.cube.position.set(c.x, c.y, -d + CUBE / 2); v.cube.quaternion.set(0, 0, Math.sin(c.yaw_deg * Math.PI / 360), Math.cos(c.yaw_deg * Math.PI / 360)); v.cube.scale.set(1, 1, 1); };
    home();
    if (t && !pb.running) {   // 손가락을 닫은 프레임에 큐브를 손에 붙여 따라가게 (그림만 — 물리 아님), 놓은 뒤엔 놓을 자리에
      const f = (lab) => { const w = (t.waypoints || []).find((x) => x.label === lab); return w ? w.frame : null; };
      const f0 = f("손가락 닫기"), f1 = f("내려감: 놓는 자세");
      const hand = v.actual && v.actual.joints.get("hand_roll");
      if (f0 != null && f1 != null && hand && hand.obj) {
        if (!ST.att || ST.att.m !== T.mtime) {
          v.actual.setPose(trajPose(f0)); v.actual.group.updateMatrixWorld(true);
          v.cube.updateMatrix();
          ST.att = { m: T.mtime, rel: hand.obj.matrixWorld.clone().invert().multiply(v.cube.matrix) };
          v.actual.setPose(trajPose(k));
        }
        if (k > f1) { v.cube.position.set(tg.x, tg.y, -d + CUBE * (st.level + 0.5)); v.cube.rotation.set(0, 0, tg.yaw_deg * Math.PI / 180); }
        else if (k >= f0) { v.actual.group.updateMatrixWorld(true); hand.obj.matrixWorld.clone().multiply(ST.att.rel).decompose(v.cube.position, v.cube.quaternion, v.cube.scale); }
      }
    }
    v.dirty();
  }

  // ---------- 0.3초마다: 버튼·재생 상태 ----------
  function tick() {
    if (!S || curTab !== "stack") return;
    const kind = S.mode && S.mode.kind;
    const pb = (kind === "virtual" ? (S.ros.virtual_state || {}) : (S.ros.direct_state || {})).playback || {};
    if (ST.played && pb.running) ST.sawRun = true;
    if (ST.played && ST.sawRun && !pb.running) {   // 우리가 누른 재생이 끝남 → 다음 걸음은 지금 인식이 정한다 (실물은 이 뒤의 새 프레임부터)
      ST.played = false; ST.sawRun = false; ST.endT = S.t || Date.now() / 1000;
      if (pb.reason === "끝까지 재생" && ST.cur) post("/api/stack/placed", ST.cur, true);   // 힌트 기록 (가려진 층 번호 · 다른 층에 보이면 막기)
      show(ST.note, `${ST.head}: 재생 끝 (${pb.reason || "?"})`, "run", "놓인 큐브는 실시간 인식으로 확인 — 맞으면 다음 층 버튼이 켜진다");
      ST.sig = null; if (LQ.r) setDet(LQ.r);
      ST.holdUntil = Date.now() + 3000;
    }
    line();
    renderSteps();
    if (ST.mapD !== tableD()) mapLoad();
    ST.bStop.disabled = !pb.running;
    draw();
  }
  setInterval(tick, 300);
  setInterval(() => { if (curTab === "stack" && T.playing) draw(); }, 60);
  load();
  if (location.hash === "#stack" && curTab && curTab !== "stack") showTab("stack");
  else if (curTab === "stack") onShow();
})();
