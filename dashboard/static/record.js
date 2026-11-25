// 실행 기록 (run_record.py): 위 칩 '기록 중 ● runs/…' + 연결 탭 맨 아래 접힌 '기록' 칸 (최근 기록 · zip 받기 · 손으로 시작/끝 · 디스크).
// 실물 재생(역기구학·HOME·탑 쌓기·강화학습)은 서버가 자동으로 기록한다. 이 파일은 보여 주기만 (app.js 뒤에 읽힘: $, el, post, toast).
"use strict";
(function () {
  const css = `.rec-chip.rec { background: var(--red-soft); color: var(--red-d); font-weight: 700; }
.rec-chip.rec .dot { background: var(--red); animation: recblink 1s steps(2) infinite; }
@keyframes recblink { 50% { opacity: .25; } }
.rec-tbl { width: 100%; border-collapse: collapse; font-size: 13px; }
.rec-tbl td, .rec-tbl th { padding: 6px 4px; border-bottom: 1px solid var(--line, #eef0f3); text-align: left; vertical-align: top; }
.rec-tbl th { color: var(--muted); font-weight: 600; }
.rec-tbl .nm { font-family: ui-monospace, monospace; font-size: 12px; word-break: break-all; }
.rec-row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; margin: 8px 0; }
.rec-wrap { overflow-x: auto; }`;
  document.head.append(el("style", {}, css));
  const chipEl = el("span", { class: "chip rec-chip", id: "chip-rec", title: "실행 기록 (연결 탭 맨 아래)" });
  const chips = document.querySelector(".topbar .chips");
  if (chips) chips.append(chipEl);
  chipEl.addEventListener("click", () => { location.hash = "conn"; box.open = true; box.scrollIntoView({ behavior: "smooth" }); });

  const box = el("details", { class: "adv", id: "rec-box" }, el("summary", {}, "기록 — 실물 재생마다 영상·관절·인식·계획 저장"));
  const body = el("div", { class: "card" });
  box.append(body);
  const page = document.getElementById("page-conn");
  if (page) page.append(box);

  const mb = (n) => n == null ? "—" : n >= 1 << 30 ? (n / (1 << 30)).toFixed(2) + " GB" : (n / (1 << 20)).toFixed(1) + " MB";
  const KIND = { ik: "역기구학", home: "HOME", stack: "탑 쌓기", rl: "강화학습", manual: "손으로" };
  let R = null, busy = false;

  function setChipRec(r) {
    const low = r && !r.on && r.low_disk;   // 남은 디스크가 모자라 기록을 안 엶
    chipEl.className = "chip rec-chip" + (r && r.on ? " rec" : low ? " bad" : "");
    chipEl.innerHTML = '<span class="dot"></span>';
    chipEl.append(r && r.on ? `기록 중 runs/${r.run}` : low ? `기록 못 함: 디스크 ${mb(low.free)} 남음` : "기록");
    chipEl.title = low ? `남은 디스크가 ${low.min_gb} GB 아래라 실행 기록을 안 연다` : "실행 기록 (연결 탭 맨 아래)";
  }

  function render() {
    const r = R;
    if (!r || !box.open) return;
    const s = r.settings || {};
    const top = el("div", { class: "rec-row" },
      r.on ? el("b", { style: "color:var(--red-d)" }, `● 기록 중 runs/${r.run} · ${r.seconds}초 · 관절 ${r.rows}줄 · 영상 ${r.frames}장` + (r.ending ? " · 끝내는 중" : ""))
           : el("span", { class: "muted" }, "기록 안 하는 중 — 실물 재생을 하면 자동으로 시작"),
      el("span", { class: "spacer", style: "flex:1" }),
      r.on ? el("button", { class: "btn line small", onclick: () => act("/api/record/stop") }, "기록 끝")
           : el("button", { class: "btn line small", onclick: () => act("/api/record/start", { kind: "manual" }) }, "기록 시작 (손으로)"));
    const opts = el("div", { class: "rec-row" },
      el("label", {}, el("input", { type: "checkbox", ...(s.auto !== false ? { checked: "" } : {}), onchange: (e) => act("/api/record/settings", { auto: e.target.checked }, true) }), " 재생마다 자동 기록"),
      el("label", {}, el("input", { type: "checkbox", ...(s.virtual !== false ? { checked: "" } : {}), onchange: (e) => act("/api/record/settings", { virtual: e.target.checked }, true) }), " 가상 모드도"));
    const L = r.list || { runs: [] };
    const disk = el("div", { class: "muted", style: "font-size:13px" },
      `기록 ${L.count}개 · ${mb(L.bytes)} (한도 ${L.keep_n}개 / ${L.keep_gb} GB, 넘으면 오래된 것부터 지움) · 디스크 남은 곳 ${mb(L.free)}`);
    const rows = (L.runs || []).map((x) => el("tr", {},
      el("td", { class: "nm" }, x.name),
      el("td", {}, (KIND[x.kind] || x.kind) + (x.mode === "virtual" ? " (가상)" : "")),
      el("td", {}, x.recording ? "기록 중" : (x.seconds != null ? `${Math.round(x.seconds)}초` : "—") + (x.hz ? ` · ${x.hz} Hz` : "")),
      el("td", {}, x.reason || ""),
      el("td", {}, mb(x.bytes) + (x.video ? " · 영상" : "")),
      el("td", {}, x.recording ? "" : el("a", { href: `/api/record/zip/${encodeURIComponent(x.name)}`, download: `${x.name}.zip` }, "zip"))));
    const tbl = rows.length ? el("div", { class: "rec-wrap" }, el("table", { class: "rec-tbl" },
      el("tr", {}, el("th", {}, "폴더"), el("th", {}, "종류"), el("th", {}, "길이"), el("th", {}, "끝난 이유"), el("th", {}, "크기"), el("th", {}, "")), rows))
      : el("div", { class: "muted" }, "아직 기록 없음");
    const help = el("div", { class: "muted", style: "font-size:12px;margin-top:8px" },
      "폴더 안: video_live.mp4(폰 영상 10 fps, 실제 시간) · video_infer.mp4(뎁스·세그) · joints.csv(50 Hz 관절) · cubes.csv(인식) · plan.json(재생한 궤적) · events.log(로그·누른 버튼·STOP·고장) · meta.json · config/(보정 사본)");
    body.replaceChildren(top, opts, disk, tbl, help);
  }

  async function act(url, b, quiet) {
    await post(url, b || {}, quiet);
    await poll(true);
  }

  async function poll(force) {
    if (busy || (document.hidden && !force)) return;
    busy = true;
    try { R = await (await fetch("/api/record/status")).json(); } catch (e) { R = null; }
    busy = false;
    setChipRec(R);
    render();
  }
  if (new URLSearchParams(location.search).has("rec")) box.open = true;   // 주소에 ?rec=1 이면 기록 칸을 펼친 채로
  box.addEventListener("toggle", () => poll(true));
  setInterval(poll, 1500);
  poll(true);
})();
