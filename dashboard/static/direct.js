// 실물(직접 CAN, ROS 없음) 모드 화면. app.js 다음에 읽혀 app.js 의 그리기 함수를 감싼다(app.js 는 거의 그대로 → 팀 버전과 합치기 쉽게).
// 서버: direct_api.py (/api/direct/*, /api/ik/play_real) → real_arm.py 가 can0 을 직접 다룬다.
// 직접 CAN 모드가 아니면(가상 / --ros) 감싼 함수는 원래 것만 부른다.
"use strict";
const DIRECT = () => !!(S && S.mode && S.mode.kind === "direct");
const DS = () => ((S && S.ros && S.ros.direct_state) || {});
const DC = () => !!DS().connected;
const D = {};

// ---------- 모드 스위치: 실물 / 가상 ----------
setMode = async function (v) {
  if (VIRT() === v) return;
  const ros = S && S.mode && S.mode.ros_enabled;
  if (v && !confirm("가상 모드로 바꿀까요?\n\n화면의 모터·원점이 전부 가상 팔이 된다(실물과 상관없음). 실물 팔이 연결돼 있으면 거절된다.")) return;
  if (!v && !confirm(ros ? "실물(ROS) 모드로 바꿀까요?\n\n가상 노드는 전부 꺼지고, 이제부터 버튼이 실제 로봇에 간다."
    : "실물 모드로 바꿀까요? (직접 CAN, ROS 없음)\n\n가상 팔은 꺼진다. 연결 탭 ① CAN 켜기 → ② 팔 연결 순서로.")) return;
  await post("/api/mode", { virtual: v });
};

// ---------- 위 띠 ----------
const _render = render;
render = function () {
  _render();
  if (!S) return;
  const d = DIRECT();
  document.body.classList.toggle("direct", d);
  const ros = S.mode && S.mode.ros_enabled;
  $("#mode-real").textContent = ros ? "실물(ROS)" : "실물";
  $("#mode-real").title = ros ? "실물 로봇 (ROS 자식 프로세스)" : "실물 로봇: 이 프로그램이 can0 을 직접 (ROS 없음)";
  const sb = $("#btn-stop");
  if (d) {
    const ds = DS();
    chip("chip-ros", ds.connected ? "직접 CAN · 팔 연결됨" : "직접 CAN · 팔 연결 안 됨", ds.connected ? "on" : "");
    $("#btn-ros").hidden = true;
    $("#suggestbar").hidden = true;
    const wb = $("#warnbar");
    wb.innerHTML = "■ STOP(Esc) = 모든 관절 <b>지금 자리 홀드</b> (팔 안 떨어짐). 힘 빼기는 연결 탭 ② 에서 따로 — <b>팔이 떨어진다</b>.";
    wb.title = "직접 CAN: STOP 은 노드를 끄지 않고 AK45 를 지금 측정 위치에 다시 잡는다. 대시보드를 끄면 브레이크 0 A (힘 빠짐).";
    if (sb.dataset.d !== "1") { sb.dataset.d = "1"; sb.innerHTML = "■ STOP<small>지금 자리 홀드 · Esc</small>"; sb.title = "모든 관절 지금 자리 홀드, 재생 멈춤 (Esc)"; }
    const fg = Object.keys(ds.foreign || {});
    if (fg.length) { const x = $("#straybar"); x.hidden = false; x.textContent = "⚠ 다른 프로그램이 같은 버스에 명령 중: " + fg.join(", ") + " — ak45_node·stm_bridge_node·cansend 를 끄세요"; }
  } else if (sb.dataset.d === "1") {
    sb.dataset.d = ""; sb.innerHTML = "■ 전부 정지<small>모든 노드 끄기 · Esc</small>"; sb.title = "켜진 항목 전부 끄기 (Esc)";
  }
};

// ---------- 연결 탭 ① CAN ② 팔 연결 ----------
const _renderSteps = renderSteps;
renderSteps = function () {
  _renderSteps();
  const d = DIRECT();
  ST.ros.root.hidden = d; ST.arm.root.hidden = d;
  if (!D.arm) buildArmStep();
  D.arm.root.hidden = !d;
  if (d) updateArmStep();
};
function buildArmStep() {
  const o = D.arm = { chip: el("span", { class: "chip" }), note: el("div", { class: "note" }), row: el("div", { class: "row" }) };
  o.speed = el("input", { value: "30", inputmode: "decimal", style: "width:5em" });
  o.on = el("button", { class: "btn", onclick: () => post("/api/direct/connect", { max_speed: o.speed.value }) }, "▶ 팔 연결");
  o.limp = el("button", { class: "btn red", onclick: () => {
    if (!confirm("힘 빼기: AK45 4개에 브레이크 0 A — 홀드가 풀려 팔이 떨어진다.\n\n팔을 받쳤나요?")) return;
    post("/api/direct/limp", { joint: "all" });
  } }, "힘 빼기");
  o.off = el("button", { class: "btn line", onclick: () => {
    if (!confirm("팔 연결 끊기: AK45 에 브레이크 0 A 를 보내고 can0 을 닫는다 — 팔이 떨어진다.\n\n팔을 받쳤나요?")) return;
    post("/api/direct/disconnect");
  } }, "■ 연결 끊기");
  o.row.append(el("label", { class: "field", style: "display:inline-flex;gap:6px;align-items:center;margin:0" }, "속도 °/s", o.speed), o.on, o.limp, o.off);
  o.root = el("div", { class: "card step" }, el("h3", {}, el("span", { class: "num" }, "2"), "팔 연결", o.chip), o.note, o.row);
  $("#conn-steps").append(o.root);
}
function updateArmStep() {
  const o = D.arm, ds = DS(), c = DC();
  const nf = (S.ros.motors || []).filter((m) => m.values && m.values.watchdog_ok === "1").length;
  setChip(o.chip, c ? `연결됨 · 피드백 ${nf}/4` : "안 됨", c ? (nf === 4 ? "on" : "warn") : "");
  o.note.textContent = c ? `${ds.interface} ${ds.channel}${ds.lock ? " · 잠금 " + ds.lock : ""} · 속도 ${ds.params.max_speed} °/s · ` +
      "AK45 는 원점을 잡기 전엔 안 잡는다 (각도 제어 탭)" : "can0 을 이 프로그램이 직접 연다 (ROS 없음). ak45_node 등이 떠 있으면 거절된다.";
  o.on.disabled = c; o.speed.disabled = c; o.off.disabled = !c; o.limp.disabled = !c;
}

// ---------- 연결 탭: AK45 표 (연결 전 문구) ----------
const _renderMotors = renderMotors;
renderMotors = function () {
  if (DIRECT() && !S.ros.available) {
    const box = $("#motors"), key = "d|" + S.ros.reason;
    $("#ros-note").textContent = "";
    if (box.dataset.rosKey !== key) { box.dataset.rosKey = key; box.replaceChildren(el("div", { class: "card muted" }, el("div", { class: "note" }, "② 팔 연결을 누르면 AK45 4개가 여기 나온다"))); }
    return;
  }
  _renderMotors();
  if (DIRECT()) $("#ros-note").textContent = "직접 CAN · 피드백 50 Hz";
};

// ---------- 연결 탭 고급: ROS 카드·노드 카드·보정 안내를 숨기고 직접 CAN 정보 + 보정 ----------
const _renderConn = renderConn;
renderConn = function () {
  const d = DIRECT();
  $("#conn-ros").hidden = d;
  const sc = $("#svc-conn"); sc.hidden = d; if (sc.previousElementSibling) sc.previousElementSibling.hidden = d;
  const sm = $("#conn-adv > summary"); if (sm) sm.textContent = d ? "고급 — 버스 상태 · 로그 · 보정" : "고급 — 노드 따로 켜기 · 로그 · 보정 · 시험";
  if (!D.info) {
    D.info = { kv: el("div", { class: "kv" }), log: el("pre", { class: "logbox" }) };
    D.info.root = el("div", { class: "grid", id: "dinfo" }, el("div", { class: "card wide" }, el("h3", {}, "직접 CAN"),
      el("p", { class: "help" }, "real_arm.py: rx 스레드(피드백·에러 프레임) + 50 Hz 제어(램프·carrot·홀드 20 Hz). 안전 기본값은 아래 표. " +
        "끊김 ≤ gap_keep_s 는 홀드 유지, 넘으면 멈춤·원점 지움. 튐 jump_deg → 지금 자리 홀드. 막힘·온도·에러 → 그 관절 힘 뺌."),
      D.info.kv, el("details", { class: "log", open: true }, el("summary", {}, "로그 (마지막 40줄)"), D.info.log)));
    $("#conn-ros").after(D.info.root);
  }
  D.info.root.hidden = !d;
  if (!d) { _renderConn(); return; }
  const ds = DS(), b = (S.ros.bus || {}).values || {}, p = ds.params || {};
  D.info.kv.replaceChildren(...[
    kv("버스", S.ros.bus ? `${b.state} · ${S.ros.bus.message}` : "—"),
    kv("에러 프레임 / bit-stuff / no-ack / bus-off", S.ros.bus ? `${b.err_frames} / ${b.bit_stuff_errors} / ${b.ack_errors} / ${b.busoff_count}` : "—"),
    kv("송신 실패", S.ros.bus ? `${b.send_errors}${b.last_send_error ? " · " + b.last_send_error : ""}` : "—"),
    kv("속도 · 가속 · 한 스텝", `${p.max_speed} °/s · ${p.max_acc} °/s² · ${p.max_step}°`),
    kv("carrot · 홀드 재송신", `±${p.carrot}° · ${p.hold_hz} Hz`),
    kv("끊김 홀드 유지 · 튐", `${p.gap_keep_s} s · ${p.jump_deg}°`),
    kv("막힘", `${p.stall_deg}° 가 ${p.stall_s} s`), kv("온도 경고 · 힘 뺌", `${p.temp_warn} · ${p.temp_stop} ℃`),
    kv("서보 재전송", `${p.servo_resend_hz} Hz`),
  ].flat());
  const t = (ds.log || []).join("\n");
  if (D.info.log.textContent !== t) { D.info.log.textContent = t; D.info.log.scrollTop = D.info.log.scrollHeight; }
};

// ---------- 보정 (calibrate.py 의 m ±N / z / f / k / v / w 를 화면에서) ----------
const _renderCalib = renderCalib;
renderCalib = function () {
  const d = DIRECT();
  const h = $("#calib").previousElementSibling;
  if (!D.cal) buildCalib();
  D.cal.root.hidden = !d;
  $("#calib").hidden = d;
  if (h) h.textContent = d ? "보정 (화면에서)" : "보정 (calibrate)";
  if (!d) { _renderCalib(); return; }
  updateCalib();
};
function buildCalib() {
  const c = D.cal = {};
  c.sel = el("select", {});
  c.sel.addEventListener("change", updateCalib);
  c.kv = el("div", { class: "kv" });
  c.n = el("input", { value: "2", inputmode: "decimal", style: "width:4em" });
  const mv = (sgn) => async () => {
    const j = c.sel.value, v = sgn * parseFloat(c.n.value);
    if (!c.ok && !confirm("⚠ 보정: 모터를 지금 자리에서 조금씩 움직인다 (원점 없이 모터 각 그대로, 한 번에 ±10° 까지).\n" +
      "AK45 는 그 순간부터 홀드한다. 서보 첫 명령은 위치를 몰라 바로 그 각으로 간다.\n\n팔 주변을 비우고 받칠 준비를 했나요?")) return;
    c.ok = true;
    await post("/api/direct/calib", { action: "move", joint: j, value: v });
  };
  c.scale = el("input", { value: "1", inputmode: "decimal", style: "width:5em" });
  c.ver = el("input", { type: "checkbox" });
  c.ver.addEventListener("change", () => post("/api/direct/calib", { action: "verify", joint: c.sel.value, value: c.ver.checked }));
  c.dirty = el("div", { class: "note" });
  c.save = el("button", { class: "btn", onclick: () => post("/api/direct/calib", { action: "save" }) }, "calibration.json 저장");
  c.root = el("div", { class: "card wide" },
    el("h3", {}, "관절 하나씩 보정"),
    el("p", { class: "help" }, "관절을 고르고 −/+ 로 모터를 조금씩 움직여 시뮬 0° 자세를 찾는다 → '지금 = sim 0' (zero_deg). " +
      "+ 방향이 시뮬과 반대면 '방향 뒤집기' (sign). 맞으면 '확인됨' (verified) → 저장. 팔 연결(②) 후에만 된다."),
    el("div", { class: "row" }, el("label", { class: "field", style: "margin:0" }, "관절", c.sel)),
    c.kv,
    el("div", { class: "row" }, el("button", { class: "btn line", onclick: mv(-1) }, "− 움직이기"), c.n, el("span", { class: "help", style: "margin:0" }, "°"),
      el("button", { class: "btn line", onclick: mv(1) }, "+ 움직이기")),
    el("div", { class: "row" },
      el("button", { class: "btn line", onclick: () => { if (confirm(`${c.sel.value}: 지금 모터 각을 sim 0° 로 삼을까요? (AK45 는 원점도 여기로)`)) post("/api/direct/calib", { action: "zero", joint: c.sel.value }); } }, "지금 = sim 0 (영점)"),
      el("button", { class: "btn line", onclick: () => post("/api/direct/calib", { action: "flip", joint: c.sel.value }) }, "방향 뒤집기"),
      el("label", { class: "field", style: "display:inline-flex;gap:6px;align-items:center;margin:0" }, "scale", c.scale,
        el("button", { class: "btn line small", onclick: () => post("/api/direct/calib", { action: "scale", joint: c.sel.value, value: parseFloat(c.scale.value) }) }, "적용")),
      el("label", { class: "help", style: "display:inline-flex;gap:6px;align-items:center;margin:0" }, c.ver, "확인됨 (verified)")),
    el("div", { class: "row" }, c.save), c.dirty);
  $("#calib").after(c.root);
}
function updateCalib() {
  const c = D.cal, ds = DS(), rows = ds.calibration || (S.calibration || {}).joints || [];
  if (c.sel.options.length !== rows.length) {
    const cur = c.sel.value;
    c.sel.replaceChildren(...rows.map((j) => el("option", { value: j.joint }, `${jLabel(j.joint)} (${j.joint}) — ${j.motor === "ak45" ? "AK45 " + j.id : "서보 " + j.name}`)));
    if (cur) c.sel.value = cur;
  }
  const j = rows.find((x) => x.joint === c.sel.value) || rows[0];
  const conn = DC();
  for (const b of c.root.querySelectorAll("button, input, select")) if (b !== c.sel) b.disabled = !conn || (b === c.save && !ds.calib_dirty);
  if (!j) return;
  let now = "—";
  if (j.motor === "ak45") { const r = (S.ros.joints || {})[`ak45_${j.id}`]; now = r ? `${r.deg.toFixed(2)}° (피드백 ${r.age.toFixed(1)}초 전)` : "피드백 없음"; }
  else { const r = (S.ros.servos || {})[j.name]; now = r ? `${r.deg.toFixed(1)}° (보낸 값)` : "아직 안 보냄 (첫 명령은 바로 감)"; }
  const key = [j.joint, j.sign, j.scale, j.zero_deg, j.verified, now].join("|");
  if (c.key !== key) {
    c.key = key;
    c.kv.replaceChildren(...[kv("모터", j.motor === "ak45" ? `AK45 ID ${j.id}` : `서보 ${j.name} (ID ${j.id})`), kv("지금 모터 각", now),
      kv("zero_deg", `${j.zero_deg}°`), kv("sign · scale", `${j.sign > 0 ? "+1" : "−1"} · ${j.scale}`), kv("verified", j.verified ? "예" : "아니오 (가정)")].flat());
    c.ver.checked = !!j.verified;
    if (document.activeElement !== c.scale) c.scale.value = j.scale;
  }
  c.dirty.textContent = ds.calib_dirty ? "저장 안 된 변경 있음 — 저장해야 다음 연결·역기구학 검사에 쓰인다" : (conn ? `파일: ${ds.calibration_file || ""}` : "② 팔 연결 후에 쓸 수 있다");
  c.dirty.className = "note " + (ds.calib_dirty ? "warn" : "");
}

// ---------- 원점 ----------
const _renderHoming = renderHoming;
renderHoming = function () {
  _renderHoming();
  if (DIRECT() && !DC()) { setChip(H.chip, "연결 안 됨", ""); H.msg.textContent = "연결 탭 ② 팔 연결 먼저"; H.msg.className = "note"; }
};
const _homingDo = homingDo;
homingDo = async function (action, joint) {
  if (!DIRECT()) return _homingDo(action, joint);
  if (action === "home" && !confirm(homeAsk(joint) + "\n\n원점을 잡는 순간 그 AK45 가 지금 자리에서 힘을 준다(홀드 시작).")) return;
  if (action === "release" && !confirm(`${joint === "all" ? "전부" : joint} 쉬게 하기?\n\n원점을 지우고 명령을 막는다. 팔은 지금 자리에서 계속 홀드한다 ` +
    "(힘을 빼려면 연결 탭 ② '힘 빼기' — 팔이 떨어짐).")) return;
  await post(`/api/homing/${action}`, { joint });
};

// ---------- 각도 제어: 첫 보내기 확인창 ----------
const _confirmFirst = confirmFirst;
confirmFirst = function () {
  if (!DIRECT()) return _confirmFirst();
  if (A.confirmed) return true;
  const unver = ((S.calibration || {}).joints || []).filter((j) => !j.verified).map((j) => j.joint);
  const ok = confirm("⚠ 실제 모터가 움직인다 (직접 CAN).\n\n" +
    `calibration verified=false (${unver.length}개 관절: ${unver.join(", ") || "없음"}) — 방향·영점이 아직 가정이다.\n` +
    "처음엔 2~5° 만. 팔 주변을 비우고, 팔을 받칠 준비를 한다.\n\n" +
    `AK45 는 ${DS().params ? DS().params.max_speed : "?"} °/s 램프. 서보는 첫 명령에 램프 없이 바로 간다(손목 확인).\n` +
    "멈추려면 위 ■ STOP (Esc) = 지금 자리 홀드.\n\n보낼까요?");
  if (ok) A.confirmed = true;
  return ok;
};

// ---------- 역기구학: 실물 재생 (직접 CAN 에서만, 확인창 2단계) ----------
const _renderTraj = renderTraj;
renderTraj = function () {
  _renderTraj();
  if (!D.play) {
    const o = D.play = {};
    o.go = el("button", { class: "btn red", onclick: playReal }, "실물로 재생");
    o.stop = el("button", { class: "btn line", onclick: () => post("/api/ik/stop_real") }, "재생 멈춤 (그 자리 홀드)");
    o.info = el("span", { class: "num" });
    o.root = el("div", { class: "tbar", hidden: true }, el("b", {}, "실물"), o.go, o.stop, o.info);
    $("#traj-ctl").append(o.root);
  }
  const o = D.play, pb = DS().playback || {};
  o.root.hidden = !DIRECT();
  const stale = planStale();
  o.go.disabled = !T.traj || !DC() || pb.running || stale;
  o.go.title = !DC() ? "연결 탭 ② 팔 연결 먼저" : !T.traj ? "계획(궤적) 먼저" : stale ? "값이 바뀜 — 계획 다시" : "";
  o.stop.disabled = !pb.running;
  o.info.textContent = pb.running && pb.holding ? `첫 자세 유지 중 — ${pb.hold}초 뒤에 움직이기 시작 (전체 약 ${pb.seconds.toFixed(0)}초)` : pb.running ? `재생 중 ${pb.frame}/${pb.frames - 1} (${pb.slow.toFixed(1)}배 느리게, 약 ${pb.seconds.toFixed(0)}초)` + (pb.phase ? " — " + phaseText(pb) : "") : pb.reason ? endText(pb) : "";
  const bad = !pb.running && pb.reason && pb.reason !== "끝까지 재생";
  o.info.style.color = bad ? "var(--red-d)" : ""; o.info.style.fontWeight = bad ? "700" : "";   // 중간에 멈추면 빨갛고 굵게 (작은 글씨로 놓치지 않게)
};
// 재생이 끝난 이유: 어디(프레임·초)에서, 왜. 실물에서만 생기는 중단(막힘·온도·에러·피드백 끊김·추종 15° 초과)이 여기에 나온다.
function endText(pb) {
  if (pb.reason === "끝까지 재생") return "끝까지 재생함";
  const at = Number.isFinite(pb.frame) && pb.frames ? `${pb.frame}/${pb.frames - 1} 프레임 (약 ${(pb.frame * (pb.period || 0)).toFixed(0)}초)` : "";
  return pb.reason === "stopped" ? `멈춤 버튼으로 멈춤 — ${at}` : `⚠ ${at}에서 멈춤: ${pb.reason} (연결 탭 로그에도 남음)`;
}
async function playReal() {
  const mtime = T.info && T.info.mtime;     // 지금 화면이 보여 주는 계획. 확인창 사이에 계획이 바뀌면 서버가 거절
  if (!confirm("⚠ 실물 팔로 이 궤적을 재생한다.\n\n원점 4개가 잡혀 있어야 하고, 첫 프레임까지 천천히 간 뒤 재생한다. " +
    "AK45 가 명령에서 15° 넘게 0.5초 벌어지면 멈추고 그 자리 홀드.\n\n팔 주변을 비웠나요?")) return;
  if (!confirm("정말 실물로 재생할까요? (멈춤 = ■ STOP / Esc)")) return;
  T.view = "actual";
  let r = await post("/api/ik/play_real", { force: false, mtime }, true);
  if (!r.ok && /첫 프레임에서/.test(r.error || "")) {   // 지금 자세가 계획 시작(HOME)과 다름 — 예: 손 위치로 보낸 자세 그대로. HOME 으로 먼저 가는 길을 바로 묻는다
    if (confirm("지금 자세가 계획의 시작 자세(HOME)와 달라서 재생이 거절됐다.\n\n" + r.error + "\n\nHOME 으로 먼저 갈까요? (도착한 뒤 '실물로 재생'을 다시 누른다)")) goHome();
    return;
  }
  if (!r.ok && r.need_force) {
    if (!confirm("검사에서 문제가 나왔다 (play_trajectory 면 --force 가 필요):\n\n- " + r.problems.join("\n- ") + "\n\n그래도 재생할까요?")) return;
    r = await post("/api/ik/play_real", { force: true, mtime });
  } else if (r.ok && r.msg) toast(r.msg);
}


// ---------- 서보 원점 맞추기 (3D 로 0° 모양과 + 방향을 보고 실물을 따라 맞춤 → /api/direct/calib) ----------
// 모양 설명은 IK 모델(arm.xml)에서 잰 것 (2026-11-23): 로봇 틀 x = 왼쪽, y = 뒤, z = 위. HOME(모든 관절 0)에서 손가락은 곧게 아래.
const SZ_JOINTS = [
  { j: "wrist_yaw", t: "손목 굽힘", ref: {}, plus: 30,
    zero: "0° = 손이 아래팔과 일직선 (팔을 곧게 늘어뜨리면 손가락도 곧게 아래).", dir: "+ = 손이 앞(책상 쪽)으로 굽음." },
  { j: "wrist_pitch", t: "손목 비틀기", ref: { wrist_yaw: 30 }, plus: 30,
    zero: "0° = 손목을 + 로 굽혔을 때(참고 자세: 손목 굽힘 +30°) 손이 정확히 앞쪽으로 굽음. 먼저 손목 굽힘을 맞춘 뒤.",
    dir: "+ = 위에서 볼 때 반시계로 비틂." },
  { j: "hand_roll", t: "손 비틀기", ref: {}, plus: 30,
    zero: "0° = 위에서 볼 때 엄지(손가락 3)가 왼쪽(3D 의 'x 왼쪽' 쪽) 약간 뒤, 두 손가락(1·2)이 오른쪽. 손목 굽힘·비틀기를 먼저 맞춘 뒤.",
    dir: "+ = 위에서 볼 때 반시계." },
  ...[1, 2, 3].flatMap((k) => [
    { j: `finger${k}_joint1`, t: `손가락 ${k} 첫 마디`, ref: {}, plus: 30,
      zero: "0° = 첫 마디가 손 축과 나란히 곧게 (HOME 에서 곧게 아래).", dir: "+ = 안쪽(가운데)으로 오므림." },
    { j: `finger${k}_joint2`, t: `손가락 ${k} 끝 마디`, ref: {}, plus: 30,
      zero: "0° = 끝 마디가 첫 마디와 일직선.", dir: "+ = 안쪽으로 굽힘." },
  ]),
  { j: "finger_rotation", t: "손가락 회전 (엄지)", ref: {}, plus: 0,
    zero: "0° = 엄지(손가락 3)가 두 손가락(1·2)을 정면으로 마주봄. 계획은 이 자세로 고정 (시뮬에서 안 도는 관절이라 3D 유령은 안 움직임, 각도 제어 슬라이더도 0 고정).",
    dir: "± 로 실물을 마주보게 맞춘 뒤 '지금이 0°'." },
];
const SZ = {};
const toR = (o) => Object.fromEntries(Object.entries(o).map(([k, v]) => [k, v * Math.PI / 180]));
function buildServoZero() {
  SZ.sel = el("select", {}, ...SZ_JOINTS.map((x, i) => el("option", { value: i }, `${x.t} (${x.j})`)));
  SZ.sel.addEventListener("change", () => { SZ.shown = null; updateServoZero(); });
  SZ.desc = el("div", {});
  SZ.now = el("div", { class: "kv" });
  SZ.v3 = el("div", { class: "v3d", style: "height:360px;min-height:0" });
  const mv = (d) => el("button", { class: "btn line", onclick: () => szMove(d) }, (d > 0 ? "+" : "−") + Math.abs(d) + "°");
  SZ.dirty = el("div", { class: "note" });
  SZ.root = el("div", { class: "card wide", id: "servo-zero" },
    el("h3", {}, "서보 원점 맞추기"),
    el("p", { class: "help" }, "① 서보를 고른다 → ② 3D 의 진한 팔(0° 모양)과 실물이 같아질 때까지 ± 로 움직인다 → ③ '지금이 0°' → " +
      "④ +5° 를 눌러 실물이 3D 의 파란 유령(+ 방향) 쪽으로 가는지 본다 (반대면 '방향 뒤집기') → ⑤ 확인됨 → 저장. 순서: 손목 굽힘 → 손목 비틀기 → 손 비틀기 → 손가락."),
    el("div", { class: "row" }, el("label", { class: "field", style: "margin:0" }, "서보", SZ.sel),
      el("button", { class: "btn line", onclick: szSendRef, title: "다른 서보를 이 서보의 참고 자세(예: 손목 비틀기면 손목 굽힘 +30°)로, 나머지는 0° 로 보낸다" }, "참고 자세 보내기")),
    SZ.desc,
    el("div", { class: "grid", style: "grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:16px;align-items:start" },
      el("div", {}, SZ.v3, el("div", { class: "note" }, "진한 팔 = 맞춰야 할 0° 모양 · 파란 유령 = + 방향")),
      el("div", {}, SZ.now,
        el("div", { class: "row" }, mv(-10), mv(-5), mv(-1), mv(1), mv(5), mv(10)),
        el("div", { class: "row" },
          el("button", { class: "btn", onclick: () => szCal("zero", "지금 보낸 서보 각을 이 관절의 0° 로 삼을까요?") }, "지금이 0°"),
          el("button", { class: "btn line", onclick: () => szCal("flip", "이 관절의 + 방향을 뒤집을까요?") }, "방향 뒤집기"),
          el("button", { class: "btn line", onclick: () => szCal("verify", null, true) }, "확인됨")),
        el("div", { class: "row" }, el("button", { class: "btn", onclick: () => post("/api/direct/calib", { action: "save" }) }, "calibration.json 저장")),
        SZ.dirty)));
  $("#motors").after(SZ.root);
}
async function szMove(d) {
  const x = SZ_JOINTS[+SZ.sel.value];
  if (!SZ.ok && !confirm("⚠ 서보가 움직인다. 한 번도 안 보낸 서보는 첫 명령에 바로 그 각으로 튄다.\n손 주변을 비웠나요?")) return;
  SZ.ok = true;
  const r = await post("/api/direct/calib", { action: "move", joint: x.j, value: d });
  if (r && r.ok === false) toast(r.error || "못 움직임", "error");
}
async function szCal(action, ask, value) {
  const x = SZ_JOINTS[+SZ.sel.value];
  if (ask && !confirm(`${x.t} (${x.j}): ${ask}`)) return;
  const r = await post("/api/direct/calib", { action, joint: x.j, value });
  if (r && r.ok) toast(r.msg || "됨"); else toast((r && r.error) || "실패", "error");
}
async function szSendRef() {
  const x = SZ_JOINTS[+SZ.sel.value];
  const joints = {};
  for (const y of SZ_JOINTS) if (y.j !== x.j) joints[y.j] = 0;
  Object.assign(joints, x.ref);
  if (!confirm("다른 서보를 보낼까요?\n" + Object.entries(joints).map(([k, v]) => `${k} ${v}°`).join(", ") +
    "\n\n처음 보내는 서보는 바로 튄다. 손 주변을 비우세요.")) return;
  const r = await post("/api/joint/send", { joints });
  if (r && r.ok) toast(r.msg || "보냄"); else toast((r && r.error) || "못 보냄", "error");
}
function updateServoZero() {
  const d = DIRECT();
  if (!SZ.root) { if (!d) return; buildServoZero(); }
  SZ.root.hidden = !d;
  if (!d) return;
  const x = SZ_JOINTS[+SZ.sel.value];
  const rows = DS().calibration || (S.calibration || {}).joints || [];
  const c = rows.find((r) => r.joint === x.j) || {};
  const sent = ((S.ros || {}).servos || {})[c.name];
  const sim = sent ? (sent.deg - c.zero_deg) / ((c.sign || 1) * (c.scale || 1)) : null;
  if (SZ.shown !== x.j) {
    SZ.desc.replaceChildren(el("p", {}, el("b", {}, x.zero), " ", x.dir));
  }
  SZ.now.replaceChildren(...[
    ["서보", `${c.name || "?"} (id ${c.id ?? "?"})`],
    ["지금 보낸 각", sent ? `${sim.toFixed(1)}° (서보 ${sent.deg.toFixed(1)}°)` : "안 보냄 — ± 를 누르면 첫 명령은 0° 자리(서보 " + (c.zero_deg ?? "?") + "°)에서 시작"],
    ["0° 자리 (zero_deg)", `서보 ${c.zero_deg ?? "?"}°`], ["방향 (sign)", c.sign > 0 ? "+1" : "−1"],
    ["확인됨", c.verified ? "예" : "아니오"],
  ].flatMap(([k, v]) => [el("span", { class: "k" }, k), el("span", {}, v)]));
  const dirty = DS().calib_dirty;
  SZ.dirty.textContent = dirty ? "⚠ 바꾼 보정이 아직 저장 안 됨 (저장 전엔 재생·HOME 이 거절된다)" : "";
  if (window.JaxViewer && !SZ.vw && !SZ.root.hidden && SZ.v3.clientWidth) {
    SZ.vw = window.JaxViewer.create(SZ.v3, { ghost: true });
    SZ.vw.load(S.robot.mesh_ok).then(() => { SZ.shown = null; updateServoZero(); });
  }
  if (SZ.vw && SZ.vw.ready && SZ.shown !== x.j) {
    SZ.vw.setActual(toR(x.ref));
    SZ.vw.setGhost(toR(Object.assign({}, x.ref, { [x.j]: (x.ref[x.j] || 0) + x.plus })));
    if (SZ.vw.focus) SZ.vw.focus([-0.05, -0.03, -0.6], 0.45);
    SZ.shown = x.j;
  }
}
const _renderCalibSZ = renderCalib;
renderCalib = function () { _renderCalibSZ(); try { updateServoZero(); } catch (e) { console.error(e); } };
