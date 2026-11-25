// 강화학습 돌리기 (시뮬 전용). 대시보드 '강화학습' 탭(#page-rl)과 별도 페이지(/rl, rl.html)가 같은 코드를 쓴다: JaxRL.mount(컨테이너).
// 서버: GET /api/rl/info, POST /api/rl/run → 궤적을 viewer.js 로 재생. 실물 팔·CAN 에는 아무것도 보내지 않는다.
// 좌표: 지도는 3D 와 같은 로봇 틀 (가로 = 로봇 왼쪽 +, 세로 = 앞이 아래). 큐브 (x, y) 입력은 정책 사양의 위치 오프셋(cm)이고
// 기준 큐브가 로봇 틀 (−0.048, −0.285) 에 있다 (sim2sim 기준 큐브 월드 (−0.285, 0.26) 를 rl_worker 의 변환으로):
//   로봇 틀 x = −0.048 − y/100,  로봇 틀 y = −0.285 + x/100.
// 요소 id 는 전부 'rl-' 로 시작하고 컨테이너 안에서만 찾는다 (대시보드 본 화면의 id 와 안 겹치게).
(function () {
  "use strict";
  const TEMPLATE = `
<div class="warnbar rl-banner">시뮬레이션만 돌립니다. 실물 팔·CAN·모터에는 아무것도 보내지 않습니다. 역기구학 탭(수식으로 푸는 집기)과는 따로입니다.</div>
<div class="split">
  <div class="split-l">
    <h2 class="sec"><span class="snum">1</span>큐브 놓을 곳 <small>정책이 학습한 범위 안(파란 칸)에서 고른다</small>
      <span class="chip" id="rl-chip-pol"><span class="dot"></span>정책 확인 중</span></h2>
    <div class="card">
      <div id="rl-mapbox"></div>
      <div class="frow2">
        <label class="field">x (cm)<input id="rl-in-x" type="number" step="0.5" value="0"><small>+ 면 책상 앞 끝에서 로봇 쪽(뒤)으로</small></label>
        <label class="field">y (cm)<input id="rl-in-y" type="number" step="0.5" value="0"><small>+ 면 로봇 오른쪽으로</small></label>
        <label class="field">큐브 각도 (°)<input id="rl-in-yaw" type="number" step="5" value="0"></label>
        <label class="field">큐브 무게 (g)<input id="rl-in-mass" type="number" step="5" value="40" min="20" max="100"></label>
        <label class="field" style="display:none">놓을 자리 x (cm)<input id="rl-in-tx" type="number" step="0.5" placeholder="없음"><small>x, y 와 같은 기준 · 비우면 집고 들기만</small></label>
        <label class="field" style="display:none">놓을 자리 y (cm)<input id="rl-in-ty" type="number" step="0.5" placeholder="없음"></label>
      </div>
      <div class="row" id="rl-presets" style="display:none"><button class="btn line small" id="rl-clear-target">놓을 자리 지우기</button></div>
      <div class="note" id="rl-target-note"></div>
      <div class="note" id="rl-spot-note"></div>
      <div class="row" style="margin-top:8px;gap:6px;flex-wrap:wrap">
        <button class="btn" id="rl-live-run" title="휴대폰 카메라 실시간 인식(휴대폰 탭과 같은 보정)으로 큐브 자리를 가져와 바로 시뮬을 돌린다 — 실물 재생은 ③ 에서 따로">📷 카메라 큐브로 강화학습 돌리기</button>
        <button class="btn line" id="rl-live-get">카메라 큐브 자리만 가져오기</button>
      </div>
      <div class="note" id="rl-live-note">정책은 책상이 어깨 아래 47.75 cm 인 장면에서 배웠다 — 실물 책상(약 55.75 cm)이면 큐브를 <b>약 8 cm 받침대</b> 위에 올리고, 카메라도 그 높이로 잰다.</div>
    </div>

    <div class="card" id="rl-camcard">
      <h3>카메라 <span class="legend">3D 에 카메라 위치·시야를 그리고, 카메라 결과로 큐브 위치를 읽는다</span></h3>
      <p class="help">카메라를 어디에 붙였는지는 레포에 확정값이 없다(vision_node 의 임시 <b>camera_to_robot</b>, 폰 고정 방법 미정). 직접 잰 값을 넣어야 3D 와 큐브 위치가 맞는다. 로봇 틀: 원점 = 어깨 롤 축, x = 로봇 왼쪽(+), y = 뒤(+, 앞은 −), z = 위. 카메라 틀 = OpenCV (x 오른쪽, y 아래, z 앞).</p>
      <label class="chk"><input type="checkbox" id="rl-cam-on"> 3D 에 카메라 표시</label>
      <div class="frow2">
        <label class="field">x (m)<input id="rl-cam-x" type="number" step="0.01"></label>
        <label class="field">y (m)<input id="rl-cam-y" type="number" step="0.01"></label>
        <label class="field">z (m)<input id="rl-cam-z" type="number" step="0.01"></label>
        <label class="field">roll (°)<input id="rl-cam-roll" type="number" step="1"></label>
        <label class="field">pitch (°)<input id="rl-cam-pitch" type="number" step="1"></label>
        <label class="field">yaw (°)<input id="rl-cam-yaw" type="number" step="1"></label>
        <label class="field">가로 시야각 (°)<input id="rl-cam-fov" type="number" step="1" value="70"></label>
      </div>
      <div class="row"><button class="btn line small" id="rl-cam-example">예시 값 넣기 (실측 아님: 어깨 위 25 cm, 로봇 앞 35 cm 에서 아래를 봄)</button></div>
      <div class="note" id="rl-cam-vis"></div>
      <label class="field" style="margin-top:12px">카메라 프로그램 결과 파일 (inference.json)<input id="rl-cam-path" type="text" placeholder="예: ~/Desktop/StereoDepthSeg/runs/cube_test/inference.json"><small>SteroDepthSeg 가 1초마다 덮어쓰는 파일. 3초 넘게 안 바뀐 파일은 안 쓴다. 큐브 후보가 정확히 1개일 때만 읽는다.</small></label>
      <div class="row"><button class="btn line" id="rl-cam-read">카메라로 큐브 위치 읽기</button></div>
      <div class="note" id="rl-cam-note"></div>
    </div>

    <h2 class="sec"><span class="snum">2</span>돌려 보기</h2>
    <div class="card">
      <p class="help">정책 <b>grasp_lift_23025</b> (관측 45 → 256 → 256 → 11)과 조회표로 MuJoCo 에서 늘어뜨린 팔 → 들어올리기 → 밀어 넣기 → 쥐고 들기 → 복귀 → <b>그 자리에서 손가락을 펴 큐브를 놓기</b> → HOME 을 한 번 돌린다. 옮기거나 내려놓지 않는다 (드는 것만).</p>
      <div class="help" style="margin:6px 0 2px">끝 동작 (2026-11-25: 놓기는 기본에서 뺌)</div>
      <label class="chk"><input type="radio" name="rl-end" id="rl-end-hold" checked> 집어 들고 ㄴ자 자세로 복귀 — 쥔 채로 끝 (정책 그대로)</label>
      <label class="chk"><input type="radio" name="rl-end" id="rl-home"> ㄴ자 복귀 뒤 그 자리에서 손가락을 펴 큐브를 놓고 HOME(모든 관절 0°)으로</label>
      <div class="row" style="margin-top:6px"><button class="btn" id="rl-btn-run">▶ 강화학습으로 돌리기</button></div>
      <div class="result" id="rl-result" hidden></div>
      <div class="kv" id="rl-summary" style="margin-top:12px" hidden></div>
      <div class="phases" id="rl-phases"></div>
    </div>

    <h2 class="sec"><span class="snum">3</span>실물 팔로 재생 <small>시뮬 궤적을 그대로 따라 움직인다</small></h2>
    <div class="card" id="rl-realcard">
      <p class="help"><b>열린 고리</b>: 시뮬에서 나온 관절 궤적을 그대로 따라갈 뿐, 실물의 큐브 위치나 관절 피드백으로 정책을 다시 돌리지 않는다. 시작 자세는 <b>AK45 = HOME(모든 관절 0°)</b>, <b>서보 = 손목 0°, 손가락 마디 0°, finger_rotation 180°</b>이다. AK45 는 각도 제어 탭의 'HOME 으로 가기', 서보는 아래 '서보를 시작 자세로 보내기'. <b>역기구학 HOME 의 손가락(−15°)·finger_rotation 0° 와 다르다</b> — 역기구학으로 돌아갈 땐 각도 제어 탭 '서보 한 번에 보내기'.</p>
      <div class="rl-checks" id="rl-real-checks"></div>
      <label class="chk" style="margin-top:10px"><input type="checkbox" id="rl-ack"> 책상 앞 끝이 어깨에서 22.5 cm, 윗면이 어깨 아래 47.75 cm 이고 큐브를 지도 위치에 놓았다</label>
      <div class="frow2"><label class="field">최고 속도 (°/s)<input id="rl-real-speed" type="number" value="15" min="1" max="90" step="1"><small>처음엔 느리게 (대시보드 상한 30)</small></label>
        <label class="field">손가락 더 쥐기: 배율<input id="rl-grip-gain" type="number" value="1.4" min="1" max="2.5" step="0.1"><small>닫힌 각 × 이것 (1 = 시뮬 그대로)</small></label>
        <label class="field">손가락 더 쥐기: 더하기 (°)<input id="rl-grip-add" type="number" value="10" min="0" max="25" step="1"><small>쥐는 동안 첫 마디를 더 닫음 (0 = 안 더함)</small></label></div>
      <p class="help" style="margin:4px 0 0">시뮬은 큐브에 닿는 각(손가락 마디 약 8°)에서 힘이 나지만 실물 서보는 그 각까지만 가서 덜 쥔다 — <b>실물로 보낼 때만</b> 쥐는 동안 <b>첫 마디(joint1)</b>를 더 닫는다 — 마디 2 는 시뮬 그대로 (더 굽히면 바깥으로 꺾였다). 정책·시뮬·3D 는 그대로. <span id="rl-grip-note"></span></p>
      <label class="chk" style="margin-top:8px"><input type="checkbox" id="rl-real-autohome" checked> 끝까지 재생하면 HOME 으로 돌아가기 (확인창이 뜬다 · 손가락을 펴므로 쥔 큐브는 그 자리에 놓인다)</label>
      <div class="row">
        <button class="btn line" id="rl-real-start" title="이 궤적의 첫 프레임 서보 값을 한 번에 보낸다 (AK45 는 각도 제어 탭의 HOME 으로 가기)">서보를 시작 자세로 보내기</button>
        <button class="btn red" id="rl-real-go" disabled>실물로 재생</button>
        <button class="btn line" id="rl-real-stop">재생 멈춤 (그 자리 홀드)</button>
        <button class="btn line" id="rl-real-home" title="각도 제어 탭의 'HOME 으로 가기' 와 같다: 책상 위로 손을 들고 → 어깨를 젖히며 팔꿈치를 펴고 → 팔을 돌려 내린다">HOME 으로 돌아가기</button>
      </div>
      <div class="note" id="rl-real-note"></div>
    </div>

    <h2 class="sec"><span class="snum">4</span>손 위치로 보내기 <small>역기구학으로 풀어 팔을 그 자리에 둔다 (집지는 않음)</small></h2>
    <div class="card" id="rl-handcard">
      <p class="help">지도에서 <b>손 위치</b> 모드로 누르면 x, y 가 채워지고 바로 푼다. 손이 쥐는 점(큐브 중심 높이)을 역기구학으로 풀어 3D 에 <b>파란 유령</b>으로 보이고, [보내기]로 팔을 그 자세로 보낸다 (실물 모드면 실제로 움직인다).</p>
      <div class="frow2">
        <label class="field">x (cm, 로봇 왼쪽 +)<input id="rl-hand-x" type="number" step="0.5" value="0"></label>
        <label class="field">y (cm, 앞 −)<input id="rl-hand-y" type="number" step="0.5" value="-30"></label>
        <label class="field">높이 (cm, 책상 위)<input id="rl-hand-z" type="number" step="0.5" value="3.5"></label>
        <label class="field">손 방향 (큐브 yaw °)<input id="rl-hand-yaw" type="number" step="5" value="0"></label>
        <label class="field">어깨 옆으로 벌리기 고정 (°)<input id="rl-hand-sy" type="number" step="1" value="0"><small>shoulder_yaw — 비우면 고정 안 함</small></label>
      </div>
      <div class="row"><button class="btn line" id="rl-hand-solve">풀기</button><button class="btn red" id="rl-hand-send" disabled>보내기</button></div>
      <div class="result" id="rl-hand-result" hidden></div>
    </div>

    <details class="adv"><summary>정책 정보</summary><div class="card"><div class="kv" id="rl-policy-info"></div></div></details>
  </div>
  <div class="split-r">
    <div class="card v3d-card">
      <h3>3D 재생 <span class="legend"><span style="color:#ea580c;font-weight:700">주황</span> 큐브 · 책상은 정책이 본 장면(어깨에서 22.5 cm 앞)</span></h3>
      <div class="v3d" id="rl-v3d"></div>
      <div class="tbar">
        <button class="btn line small" id="rl-btn-play" disabled>▶ 재생</button>
        <input type="range" id="rl-seek" min="0" max="0" value="0" step="1" disabled>
        <span class="num" id="rl-tnum">0.0 / 0.0 s</span>
        <select id="rl-speed" title="재생 속도"><option value="0.5">0.5×</option><option value="1" selected>1×</option><option value="2">2×</option><option value="4">4×</option></select>
      </div>
      <div class="note" id="rl-phase-now">돌린 결과가 아직 없다.</div>
    </div>
  </div>
</div>`;

  function mount(root) {
    if (root.dataset.rlMounted) return;
    root.dataset.rlMounted = "1";
    root.innerHTML = TEMPLATE;
    const $ = (id) => root.querySelector("#rl-" + id);
    const REF_X = -0.048, REF_Y = -0.285;
    const PRESETS = [["기준 (0, 0)", 0, 0, 0], ["(2, 28) 45°", 2, 28, 45], ["(2, −28)", 2, -28, 0], ["(−8, 18)", -8, 18, 0],
                     ["(−8, −18)", -8, -18, 0], ["(−14, −8)", -14, -8, 0], ["(−14, 6)", -14, 6, 0]];
    const state = { info: null, run: null, viewer: null, t: 0, playing: false, last: 0, k: 0, busy: false, mode: "cube", hand: null, vision: null };

    function toast(msg, bad) {
      const t = document.createElement("div");
      t.className = "toast" + (bad ? " bad" : "");
      t.textContent = msg;
      document.getElementById("toasts").append(t);
      setTimeout(() => t.remove(), 5000);
    }
    async function api(path, body) {
      const r = await fetch(path, body === undefined ? {} : { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify(body) });
      if (!r.ok) throw new Error(path + " " + r.status);
      return r.json();
    }
    const num = (id) => parseFloat($(id).value);
    const spotOk = () => [num("in-x"), num("in-y"), num("in-yaw"), num("in-mass")].every(Number.isFinite);

    // ------------------------------------------------------------ 지도 (역기구학 탭 지도와 같은 SVG·CSS 클래스·좌표식: 1 m = 1000 단위, 로봇 = 위, 앞(책상) = 아래, 로봇 왼쪽 = 화면 오른쪽)
    const MS = 1000, MX0 = -0.38, MX1 = 0.38, MY0 = -0.56, MY1 = 0.075, ML = 36, MT = 8, MR = 10, MB = 52;
    const MW = ML + (MX1 - MX0) * MS + MR, MH = MT + (MY1 - MY0) * MS + MB;
    const msx = (x) => ML + (x - MX0) * MS, msy = (y) => MT + (MY1 - y) * MS;
    const SVGNS = "http://www.w3.org/2000/svg";
    function sv(tag, attrs = {}, ...kids) {
      const e = document.createElementNS(SVGNS, tag);
      for (const [k, v] of Object.entries(attrs)) if (v !== undefined && v !== null) e.setAttribute(k, v);
      for (const k of kids) e.append(k && k.nodeType ? k : document.createTextNode(String(k)));
      return e;
    }
    function h(tag, attrs = {}, ...kids) {
      const e = document.createElement(tag);
      for (const [k, v] of Object.entries(attrs)) { if (v === undefined || v === null) continue; if (k === "class") e.className = v; else e.setAttribute(k, v); }
      for (const k of kids) e.append(k && k.nodeType ? k : document.createTextNode(String(k)));
      return e;
    }
    const M = { svg: null, snap: true, reach: null, gGrasp: null, gReach: null, gCube: null, gTgt: null, gArrow: null, gHand: null, gVis: null };
    const foldYaw = (d) => ((((d + 45) % 90) + 90) % 90) - 45;
    const degS = (d) => `${d > 0 ? "+" : d < 0 ? "−" : ""}${Math.abs(Math.round(d))}°`;
    const cmS = (m) => { const v = Math.round(m * 1000) / 10; return (v > 0 ? "+" : v < 0 ? "−" : "") + Math.abs(v).toFixed(1); };
    function spotToRobot(x, y) { return [(REF_X - y / 100) * 100, (REF_Y + x / 100) * 100]; }      // 위치 오프셋 cm → 로봇 틀 cm
    function robotToSpot(rx, ry) { return [(ry / 100 - REF_Y) * 100, -(rx / 100 - REF_X) * 100]; }
    const targetOk = () => false;   // 2026-11-25 사용자: 강화학습은 드는 것만 — 놓을 자리로 옮기지 않는다 (서버 run_with_target 은 남겨 둠)

    function buildMap() {
      const seg = h("div", { class: "modesw seg", role: "group", "aria-label": "고르는 것" },
        h("button", { id: "rl-m-cube", class: "on" }, h("i", { class: "sw cube" }), "큐브 놓기"),
        h("button", { id: "rl-m-target", style: "display:none", title: "지도를 누르면 놓을 자리 (비우면 집고 들기만)" }, h("i", { class: "sw tgt" }), "놓을 자리 고르기"),
        h("button", { id: "rl-m-hand", title: "아래 '손 위치로 보내기' 의 x, y 를 지도에서 고르기" }, h("i", { class: "sw hand" }), "손 위치"));
      const snap = h("input", { type: "checkbox", id: "rl-snap", checked: "" });
      const yaw = h("input", { type: "range", id: "rl-yaw-range", min: -45, max: 45, step: 1, value: 0 });
      // 정적 그림: 눈금, 로봇, 책상 앞 끝
      const g0 = sv("g");
      for (let x = -0.3; x <= 0.301; x += 0.1) {
        const X = msx(x).toFixed(1);
        g0.append(sv("line", { x1: X, y1: MT, x2: X, y2: MH - MB, class: "mg" }), sv("text", { x: X, y: MH - MB + 22, "text-anchor": "middle", class: "mt" }, cmS(x).replace(".0", "")));
      }
      for (let y = -0.5; y <= 0.001; y += 0.1) {
        const Y = msy(y).toFixed(1);
        g0.append(sv("line", { x1: ML, y1: Y, x2: MW - MR, y2: Y, class: "mg" }), sv("text", { x: ML - 5, y: (+Y + 4).toFixed(1), "text-anchor": "end", class: "mt" }, cmS(y).replace(".0", "")));
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
      const ty = msy(-0.225);   // 정책 장면의 책상 앞 끝 (어깨에서 22.5 cm)
      const gT = sv("g", {}, sv("line", { x1: ML, y1: ty, x2: MW - MR, y2: ty, class: "rl-tedge" }), sv("text", { x: MW - MR - 6, y: ty - 8, "text-anchor": "end", class: "rl-tlab" }, "책상 앞 끝 (어깨에서 22.5 cm)"));
      M.gGrasp = sv("g");      // 정책이 학습한 범위: 파랑 = 집을 수 있음, 회색 = 못 집음
      M.gReach = sv("g");      // 갈 수 있는 곳 (손 높이에서 위치 IK 가 풀림, GET /api/ik/reachxyz): 초록
      M.gVis = sv("rect", { class: "mvis", width: 50, height: 50, x: -25, y: -25, rx: 4, visibility: "hidden" });
      M.gArrow = sv("line", { class: "marrow", "marker-end": "url(#rl-marr)", visibility: "hidden" });
      M.gTgt = sv("rect", { class: "mtgt", width: 50, height: 50, x: -25, y: -25, rx: 4, visibility: "hidden" });
      M.gHand = sv("g", { visibility: "hidden", class: "mhand" }, sv("circle", { r: 14 }), sv("path", { d: "M-22,0 H22 M0,-22 V22" }));
      M.gCube = sv("g", { visibility: "hidden" }, sv("rect", { class: "mcube", width: 50, height: 50, x: -25, y: -25, rx: 5 }), sv("line", { x1: 0, y1: 0, x2: 25, y2: 0, class: "mcubeax" }));
      const defs = sv("defs", {}, sv("marker", { id: "rl-marr", viewBox: "0 0 10 10", refX: 8, refY: 5, markerWidth: 7, markerHeight: 7, orient: "auto-start-reverse" }, sv("path", { d: "M0,0 L10,5 L0,10 z", fill: "#4e5968" })));
      M.svg = sv("svg", { id: "rl-map", viewBox: `0 0 ${MW.toFixed(0)} ${MH.toFixed(0)}`, class: "rmap", role: "img", "aria-label": "책상 위에서 본 지도" },
        defs, g0, M.gGrasp, M.gReach, gT, gR, M.gVis, M.gArrow, M.gTgt, M.gCube, M.gHand);
      $("mapbox").replaceChildren(h("div", { class: "mapbox" },
        h("div", { class: "lead", style: "margin:0 0 10px" }, "지도에서 ", h("b", {}, "큐브"), " 자리를 누르세요."),
        h("div", { class: "mctl" }, seg, h("label", { class: "chk", title: "정책 조회표 격자(2 cm)의 칸 중심에 맞춘다. 끄면 0.5 cm" }, snap, "칸 중심에 붙이기")),
        h("div", { class: "mctl" }, h("span", { class: "mlabel" }, "큐브 yaw"), yaw, h("span", { class: "num", id: "rl-yaw-num" })),
        h("div", { class: "mlegend" }, h("span", {}, h("i", { class: "sw ok" }), "집을 수 있음 (정책이 학습한 범위)"), h("span", {}, h("i", { class: "sw no" }), "못 집음"),
          h("label", { class: "chk", title: "3D 뷰 책상에 집을 수 있는 곳을 반투명 초록으로" }, h("input", { type: "checkbox", id: "rl-grasp3d", checked: "" }), "3D 에 초록으로"),
          h("span", { class: "help", id: "rl-grid-info", style: "margin:0" }, "정책 범위 읽는 중…")),
        h("div", { class: "mlegend" }, h("label", { class: "chk", title: "집기 조건 없이: 아래 '손 위치로 보내기'의 높이로 x, y 마다 같은 역기구학(위치만)이 풀리면 초록" },
          h("input", { type: "checkbox", id: "rl-reach-chk" }), "갈 수 있는 곳 보기 (손 위치 높이에서 IK 풀림)"), h("span", {}, h("i", { class: "sw reach" }), "갈 수 있음"),
          h("span", { class: "help", id: "rl-reach-note", style: "margin:0" })),
        h("div", { class: "mlegend" }, h("span", {}, h("i", { class: "sw cube" }), "큐브"),
          h("span", {}, h("i", { class: "sw hand" }), "손 위치"), h("span", {}, h("i", { class: "sw vis" }), "비전 (카메라가 본 큐브)")),
        h("div", { class: "mwrap" }, M.svg), h("div", { class: "mhover", id: "rl-hover" }, " ")));
    }
    function place(g, xm, ym, yaw, extra) {
      g.setAttribute("transform", `translate(${msx(xm).toFixed(1)},${msy(ym).toFixed(1)})` + (yaw ? ` rotate(${(-yaw).toFixed(1)})` : ""));
      g.setAttribute("visibility", "visible");
    }
    function mapUpdate() {
      const yaw = num("in-yaw"), has = spotOk();
      if (has) { const [rx, ry] = spotToRobot(num("in-x"), num("in-y")); place(M.gCube, rx / 100, ry / 100, yaw); } else M.gCube.setAttribute("visibility", "hidden");
      if (has && targetOk()) {
        const [cx0, cy0] = spotToRobot(num("in-x"), num("in-y")), [tx0, ty0] = spotToRobot(num("in-tx"), num("in-ty"));
        place(M.gTgt, tx0 / 100, ty0 / 100, yaw);
        const x1 = msx(cx0 / 100), y1 = msy(cy0 / 100), x2 = msx(tx0 / 100), y2 = msy(ty0 / 100), L = Math.hypot(x2 - x1, y2 - y1);
        if (L > 40) {
          const ux = (x2 - x1) / L, uy = (y2 - y1) / L, a = 30, b = 30;
          for (const [k, val] of [["x1", x1 + ux * a], ["y1", y1 + uy * a], ["x2", x2 - ux * b], ["y2", y2 - uy * b]]) M.gArrow.setAttribute(k, val.toFixed(1));
          M.gArrow.setAttribute("visibility", "visible");
        } else M.gArrow.setAttribute("visibility", "hidden");
      } else { M.gTgt.setAttribute("visibility", "hidden"); M.gArrow.setAttribute("visibility", "hidden"); }
      const hx = num("hand-x"), hy = num("hand-y");
      if (Number.isFinite(hx) && Number.isFinite(hy)) place(M.gHand, hx / 100, hy / 100, 0); else M.gHand.setAttribute("visibility", "hidden");
      if (state.vision) place(M.gVis, state.vision.x, state.vision.y, state.vision.yaw); else M.gVis.setAttribute("visibility", "hidden");
      const ys = $("yaw-range");
      if (ys && has && document.activeElement !== ys) ys.value = Math.round(foldYaw(yaw));
      const yn = $("yaw-num");
      if (yn) yn.textContent = has ? `${degS(yaw)}${Math.abs(yaw - foldYaw(yaw)) > 0.05 ? ` (= ${degS(foldYaw(yaw))}, 큐브는 90° 마다 같음)` : ""}` : "";
    }
    // 정책 격자(info) → 지도·3D 용 칸 데이터 (역기구학 graspmap 과 같은 모양 {xs, ys, rows, step}, 로봇 틀 m, 행 = y 오름차순, '2' = 됨)
    function graspData() {
      const I = state.info;
      if (!I || !I.valid) return null;
      const order = I.ys_cm.map((_, i) => i).sort((a, b) => I.ys_cm[b] - I.ys_cm[a]);   // 로봇 x = REF_X − dy/100: dy 큰 쪽이 x 작음 → x 오름차순 = dy 내림차순
      const xs = order.map((i) => REF_X - I.ys_cm[i] / 100), ys = I.xs_cm.map((dx) => REF_Y + dx / 100);
      const rows = ys.map((_, jx) => order.map((i) => (I.valid[i][jx] ? "2" : "0")).join(""));
      return { xs, ys, rows, step: 0.02 };
    }
    function paintCells(g, d, cls0, cls1) {
      if (!d) { g.replaceChildren(); return; }
      const hh = d.step * MS;
      let ok = "";
      d.rows.forEach((row, j) => {
        const y = msy(d.ys[j]) - hh / 2;
        for (let i = 0; i < row.length;) {
          if (row[i] !== "2") { i++; continue; }
          let k = i; while (k < row.length && row[k] === "2") k++;
          ok += `M${(msx(d.xs[i]) - hh / 2).toFixed(1)},${y.toFixed(1)}h${((k - i) * hh).toFixed(1)}v${hh.toFixed(1)}h${(-(k - i) * hh).toFixed(1)}z`;
          i = k;
        }
      });
      const x0 = msx(d.xs[0]) - hh / 2, y0 = msy(d.ys[d.ys.length - 1]) - hh / 2;
      g.replaceChildren(sv("rect", { x: x0.toFixed(1), y: y0.toFixed(1), width: (d.xs.length * hh).toFixed(1), height: (d.ys.length * hh).toFixed(1), class: cls0 }), sv("path", { d: ok, class: cls1 }));
    }
    function paintGrasp() {
      const d = graspData();
      paintCells(M.gGrasp, d, "gno", "gok");
      const n = d ? d.rows.reduce((a, r) => a + (r.match(/2/g) || []).length, 0) : 0;
      $("grid-info").textContent = d ? `${n}칸 (2 cm) · 정책이 학습한 범위 · 손끝(집는 점)을 큐브 중심에` : "정책 범위 없음";
      paint3D();
    }
    function paint3D() {   // 3D 책상에 반투명 초록 (viewer.setGraspArea 는 역기구학 탭과 같은 함수)
      const v = state.viewer;
      if (!v || !v.setGraspArea) return;
      v.setGraspArea(graspData());
      if (v.setGraspVisible) v.setGraspVisible($("grasp3d").checked);
    }
    function isValidSpot(x, y) {
      const I = state.info;
      if (!I || !I.valid) return null;
      // 가장 가까운 격자점 (2 cm 간격) 이 유효하면 안쪽으로 본다 — 최종 판정은 서버(summary.valid_spot)
      let bi = -1, bj = -1, bd = 1e9;
      I.xs_cm.forEach((vx, a) => I.ys_cm.forEach((vy, b) => { const d = Math.hypot(vx - x, vy - y); if (d < bd) { bd = d; bi = a; bj = b; } }));
      return bd <= 1.5 && I.valid[bj][bi];
    }
    function updateNote() {
      const n = $("spot-note");
      if (!spotOk()) { n.textContent = "숫자를 넣어 주세요."; n.className = "note bad"; return; }
      const tn = $("target-note");
      if (targetOk()) {
        const d = Math.hypot(num("in-tx") - num("in-x"), num("in-ty") - num("in-y"));
        tn.textContent = d > 15 ? `⚠ 놓을 자리가 큐브에서 ${d.toFixed(0)} cm — 쥔 손 방향 그대로 옮길 수 있는 거리(대략 15 cm 안)를 넘으면 못 옮긴다 (돌려 보면 알려 준다)` :
          `놓을 자리: 큐브에서 ${d.toFixed(0)} cm. 정책이 집고, 쥔 손 방향 그대로 옮겨 놓는다 (시뮬 물리로 검증).`;
        tn.className = "note" + (d > 15 ? " warn" : "");
      } else { tn.textContent = ""; tn.className = "note"; }
      const v = isValidSpot(num("in-x"), num("in-y"));
      if (v === null) { n.textContent = ""; n.className = "note"; }
      else if (v) { n.textContent = "학습한 범위 안입니다."; n.className = "note"; }
      else { n.textContent = "파란 칸 밖입니다 — 정책이 학습하지 않은 곳이라 실패할 가능성이 큽니다. 그래도 돌려 볼 수는 있습니다."; n.className = "note warn"; }
      mapUpdate();
    }
    function setMode(m) {
      state.mode = m;
      for (const [id, k] of [["m-cube", "cube"], ["m-target", "target"], ["m-hand", "hand"]]) $(id).className = k === m ? "on" : "";
    }
    // 한 번 고르면 다음 단계로: 큐브 놓기 → 놓을 자리 고르기 → 손 위치 (버튼 순서대로, 다시 고르려면 그 버튼)
    function advanceMode() { if (state.mode === "target") setMode("hand"); }   // 큐브를 고른 뒤엔 그대로 (놓을 자리 없음)
    function mapPt(e) {
      let q;
      try { const p = M.svg.createSVGPoint(); p.x = e.clientX; p.y = e.clientY; q = p.matrixTransform(M.svg.getScreenCTM().inverse()); }
      catch (err) { const r = M.svg.getBoundingClientRect(); q = { x: (e.clientX - r.left) / r.width * MW, y: (e.clientY - r.top) / r.height * MH }; }
      return { x: MX0 + (q.x - ML) / MS, y: MY1 - (q.y - MT) / MS };   // 로봇 틀 m
    }
    function nearest(arr, v) { return arr.reduce((b, a) => (Math.abs(a - v) < Math.abs(b - v) ? a : b), arr[0]); }
    function pick(ev, mode) {
      const p = mapPt(ev), rx = p.x * 100, ry = p.y * 100;
      const snap = $("snap").checked, I = state.info;
      if (mode === "hand") {
        const st = snap ? 1 : 0.5, q = (v) => (Math.round(v / st) * st).toString();
        $("hand-x").value = q(rx); $("hand-y").value = q(ry); handEdit(); updateNote(); return;
      }
      let [x, y] = robotToSpot(rx, ry);
      if (snap && I && I.xs_cm && I.ys_cm) { x = nearest(I.xs_cm, x); y = nearest(I.ys_cm, y); }   // 정책 조회표 격자(2 cm)의 칸 중심
      else { x = Math.round(x * 2) / 2; y = Math.round(y * 2) / 2; }
      const f = mode === "target" ? ["in-tx", "in-ty"] : ["in-x", "in-y"];
      $(f[0]).value = x.toString(); $(f[1]).value = y.toString();
      updateNote();
    }
    function hoverText(ev) {
      const p = mapPt(ev), rx = p.x * 100, ry = p.y * 100;
      const [sx, sy] = robotToSpot(rx, ry), v = isValidSpot(sx, sy);
      $("hover").textContent = `x ${cmS(p.x)} cm · y ${cmS(p.y)} cm — ${p.y < 0 ? `앞으로 ${Math.abs(p.y * 100).toFixed(0)} cm` : "어깨 뒤"}, ${p.x >= 0 ? "왼쪽" : "오른쪽"} ${Math.abs(p.x * 100).toFixed(0)} cm` +
        (v === null ? "" : v ? " · ✅ 집을 수 있음" : " · ⛔ 학습 범위 밖");
    }
    // 갈 수 있는 곳 (손 높이에서 위치 IK 가 풀림): GET /api/ik/reachxyz — 켜면 파랑(집을 수 있음) 대신 초록으로
    let reachTimer = null;
    async function reachLoad() {
      clearTimeout(reachTimer);
      const on = $("reach-chk").checked;
      M.gGrasp.style.display = on ? "none" : "";
      const note = $("reach-note");
      if (!on) { M.reach = null; paintCells(M.gReach, null); note.textContent = ""; return; }
      const zc = num("hand-z"), sy = parseFloat($("hand-sy").value);
      if (!Number.isFinite(zc)) { note.textContent = "손 위치 높이를 넣어 주세요"; return; }
      note.textContent = `계산 중… (높이 ${zc} cm · 높이마다 몇 초)`;
      let d;
      try { d = await api(`/api/ik/reachxyz?z_cm=${encodeURIComponent(zc)}&ref=grip&shoulder_yaw=${encodeURIComponent(Number.isFinite(sy) ? sy : 0)}`); }
      catch (e) { note.textContent = "서버 요청 실패: " + e.message; return; }
      if (!$("reach-chk").checked) return;
      if (!d.ok) { note.textContent = "⚠ 계산 실패: " + (d.reason || "?"); return; }
      M.reach = d; paintCells(M.gReach, d, "rno", "rok");
      note.textContent = `${d.counts ? d.counts.reachable : "?"}칸 (${Math.round(d.step * 100)} cm 격자) · 높이 ${zc} cm · 어깨 옆 ${d.shoulder_yaw != null ? d.shoulder_yaw : 0}°`;
    }
    function bindMap() {
      const svg = $("map");
      let down = false, dragMode = "cube";     // 끌기 동안은 누른 순간의 모드 그대로 (놓을 때 다음 단계로)
      svg.addEventListener("pointerdown", (e) => { down = true; dragMode = state.mode; if (svg.setPointerCapture) svg.setPointerCapture(e.pointerId); pick(e, dragMode); });
      svg.addEventListener("pointermove", (e) => { if (down) pick(e, dragMode); else hoverText(e); });
      svg.addEventListener("pointerup", () => { if (down) advanceMode(); down = false; });
      svg.addEventListener("mouseleave", () => { $("hover").textContent = " "; });
      for (const [id, m] of [["m-cube", "cube"], ["m-target", "target"], ["m-hand", "hand"]]) $(id).addEventListener("click", () => setMode(m));
      $("clear-target").addEventListener("click", () => { $("in-tx").value = ""; $("in-ty").value = ""; updateNote(); });
      for (const id of ["in-x", "in-y", "in-yaw", "in-mass", "in-tx", "in-ty"]) $(id).addEventListener("input", updateNote);
      $("yaw-range").addEventListener("input", () => { $("in-yaw").value = $("yaw-range").value; updateNote(); });
      $("snap").addEventListener("change", () => { M.snap = $("snap").checked; });
      $("grasp3d").addEventListener("change", () => { if (state.viewer && state.viewer.setGraspVisible) state.viewer.setGraspVisible($("grasp3d").checked); });
      $("reach-chk").addEventListener("change", reachLoad);
    }

    // ------------------------------------------------------------ 돌리기
    const PH_KO = { RAISE: "들어올리기", SETTLE: "자세 안정", INSERT: "밀어 넣기", GRASP_LIFT: "쥐고 들기 (정책)", RETURN: "복귀", DONE: "완료",
                    LOWER: "내리기", RELEASE: "놓기 (손가락 힘 빼기)", RETREAT: "손 빼기", HOME: "HOME 으로" };
    // 놓을 자리가 있으면 정책의 '복귀' 구간이 '놓을 자리 위로 옮기기' 로 바뀐다
    function phName(n) {
      const p = state.run && state.run.params, pl = state.run && state.run.summary && state.run.summary.place;
      if (n === "RETURN" && p && p.target_cm) return "옮기기";
      if (n === "RETURN" && pl && pl.put_back) return "제자리 위로";
      return PH_KO[n] || n;
    }
    function setResult(cls, text) {
      const r = $("result");
      r.hidden = !text; r.className = "result " + (cls || ""); r.textContent = text || "";
    }
    async function run() {
      if (state.busy) return;
      if (!spotOk()) { toast("숫자를 확인해 주세요", true); return; }
      state.busy = true; $("btn-run").disabled = true; $("btn-run").textContent = "돌리는 중…";
      setResult("run", "MuJoCo 에서 정책을 돌리는 중입니다 (1~3초)…");
      pause();
      try {
        const body = { x_cm: num("in-x"), y_cm: num("in-y"), yaw_deg: num("in-yaw"), mass_g: num("in-mass"), home: $("home").checked };
        if (targetOk()) { body.target_x_cm = num("in-tx"); body.target_y_cm = num("in-ty"); }
        const r = await api("/api/rl/run", body);
        if (!r.ok) { setResult("bad", r.error || "실패"); return; }
        state.run = r;
        showSummary(r);
        loadFrames(r);
        pollReal();
        play();
      } catch (e) {
        setResult("bad", "서버 요청 실패: " + e.message);
      } finally {
        state.busy = false; $("btn-run").disabled = false; $("btn-run").textContent = "▶ 강화학습으로 돌리기";
      }
    }
    function showSummary(r) {
      const s = r.summary || {};
      const pl = s.place;
      const homeTxt = s.home ? " → HOME 으로 돌아옴" : "";
      const dr = s.drop;
      if (s.done && dr) setResult("ok", `성공 — 큐브를 쥐고 ${dr.lift_cm} cm 들어 올린 뒤 손가락을 펴서 그 자리에 놓았다${homeTxt} (${s.duration_s.toFixed(1)} 초, 시뮬 물리로 확인)`);
      else if (s.done && pl && pl.put_back) setResult("ok", `성공 — 집어 들고 제자리에 다시 내려놓았다${homeTxt} (제자리에서 ${pl.xy_err_cm} cm, ${s.duration_s.toFixed(1)} 초, 시뮬 물리로 확인)`);
      else if (s.done && pl) setResult("ok", `성공 — 집어서 옮겨 놓았다${homeTxt} (목표에서 ${pl.xy_err_cm} cm, ${s.duration_s.toFixed(1)} 초, 시뮬 물리로 확인)`);
      else if (s.done) setResult("ok", `성공 — 큐브를 쥐고 들어 올려 ㄴ자 자세로 복귀했다, 쥔 채로 끝 (${s.duration_s.toFixed(1)} 초, 시뮬 물리로 확인)`);
      else setResult("bad", "실패 — " + (s.fail_reason || "끝내 못 마침") + (s.valid_spot === false ? " · 학습 범위 밖 지점" : ""));
      const kv = $("summary");
      kv.hidden = false; kv.replaceChildren();
      const rows = [["위치 (x, y)", s.spot_cm ? `${s.spot_cm[0]}, ${s.spot_cm[1]} cm` : "-"], ["큐브 각도", s.cube_yaw_deg + "°"],
                    ["학습 범위", s.valid_spot ? "안" : "밖"], ["손 각도 θ (조회표)", Number.isFinite(s.theta_deg) ? s.theta_deg.toFixed(1) + "°" : "-"],
                    ["들어 올린 높이", Number.isFinite(s.max_rise_m) ? (s.max_rise_m * 100).toFixed(1) + " cm" : "-"],
                    ["큐브 무게", s.cube_mass_kg ? Math.round(s.cube_mass_kg * 1000) + " g" : "-"],
                    ["쥐는 힘 배율 (정책 행동)", Number.isFinite(s.grip_force_x) ? s.grip_force_x + "×" : "-"]];
      if (pl && pl.target_cm && !pl.put_back) rows.push(["놓을 자리", `${pl.target_cm[0]}, ${pl.target_cm[1]} cm`], ["놓은 자리 오차", Number.isFinite(pl.xy_err_cm) ? pl.xy_err_cm + " cm" : "-"],
        ["놓은 큐브 기울기", Number.isFinite(pl.tilt_deg) ? pl.tilt_deg + "°" : "-"]);
      if (dr) rows.push(["놓은 큐브", dr.dropped ? `책상 위 · 처음 자리에서 ${dr.moved_cm} cm · 기울기 ${dr.tilt_deg}°` : "안 떨어짐 (손에 걸림)"]);
      for (const [k, v] of rows) { const a = document.createElement("span"), b = document.createElement("span"); a.className = "k"; b.className = "v"; a.textContent = k; b.textContent = v; kv.append(a, b); }
      const ph = $("phases"); ph.replaceChildren();
      (r.phases || []).forEach((p) => {
        const b = document.createElement("button"); b.className = "phase"; b.dataset.t = p.t;
        b.textContent = `${phName(p.name)} ${p.t.toFixed(1)}s`;
        b.addEventListener("click", () => { seekTime(p.t); });
        ph.append(b);
      });
    }

    // ------------------------------------------------------------ 재생
    function frameAt(k) {
      const r = state.run, q = {};
      r.joint_names.forEach((n, i) => { q[n] = r.positions[k][i]; });
      return q;
    }
    function loadFrames(r) {
      $("seek").max = r.positions.length - 1; $("seek").disabled = false; $("btn-play").disabled = false;
      if (state.viewer && r.table) state.viewer.setBoxTable(r.table);
      if (state.viewer && state.viewer.setTarget) state.viewer.setTarget(r.target_robot ? { x: r.target_robot[0], y: r.target_robot[1] } : null);   // 초록 상자 = 놓을 자리
      state.t = 0; show(0);
    }
    function show(k) {
      const r = state.run;
      if (!r) return;
      k = Math.max(0, Math.min(r.positions.length - 1, k)); state.k = k;
      const v = state.viewer;
      if (v && v.ready) { v.setActual(frameAt(k)); v.setCubePose(r.cube[k], r.cube_quat[k]); }
      $("seek").value = k;
      const t = k * r.dt, total = (r.positions.length - 1) * r.dt;
      $("tnum").textContent = `${t.toFixed(1)} / ${total.toFixed(1)} s`;
      let cur = null;
      for (const p of r.phases || []) if (p.t <= t + 1e-6) cur = p;
      $("phase-now").textContent = cur ? `지금: ${phName(cur.name)}` : "";
      root.querySelectorAll(".phase").forEach((b) => b.classList.toggle("now", cur && Math.abs(parseFloat(b.dataset.t) - cur.t) < 1e-6));
    }
    function seekTime(t) { if (state.run) { pause(); state.t = t; show(Math.round(t / state.run.dt)); } }
    function play() {
      if (!state.run) return;
      if (state.k >= state.run.positions.length - 1) { state.t = 0; }
      state.playing = true; state.last = performance.now(); $("btn-play").textContent = "❚❚ 멈춤";
      requestAnimationFrame(tick);
    }
    function pause() { state.playing = false; $("btn-play").textContent = "▶ 재생"; }
    function tick(now) {
      if (!state.playing || !state.run) return;
      state.t += (now - state.last) / 1000 * parseFloat($("speed").value); state.last = now;
      const k = Math.round(state.t / state.run.dt);
      if (k >= state.run.positions.length - 1) { show(state.run.positions.length - 1); pause(); return; }
      show(k);
      requestAnimationFrame(tick);
    }

    // ------------------------------------------------------------ 시작
    function initViewer(meshOk) {
      const make = () => {
        if (state.viewer || !window.JaxViewer) return;
        state.viewer = window.JaxViewer.create($("v3d"), { ghost: true });
        if (state.viewer.setTableD) state.viewer.setTableD(0.4775);   // 정책 장면 책상 0.4775 (대시보드 기본 0.5575 는 실물 책상)
        state.viewer.load(meshOk).then(() => { state.viewer.focus([0, -0.4, -0.3], 1.7); camUpdate(); paint3D(); if (state.run) { if (state.run.table) state.viewer.setBoxTable(state.run.table); show(state.k); } });
      };
      if (window.JaxViewer) make(); else window.addEventListener("jaxviewer-ready", make, { once: true });
    }

    // ------------------------------------------------------------ 카메라 (비전)
    const CAM_KEY = "jaxrl.camera";
    const lsGet = (k) => { try { return localStorage.getItem(k); } catch (e) { return null; } };
    const lsSet = (k, v) => { try { localStorage.setItem(k, v); } catch (e) { /* 저장 못 해도 화면은 그대로 */ } };
    const CAM_IDS = ["cam-x", "cam-y", "cam-z", "cam-roll", "cam-pitch", "cam-yaw", "cam-fov"];
    const CAM_EXAMPLE = [0, -0.35, 0.25, 180, 0, 0, 70];   // 실측 아님: 어깨 위 0.25 m, 로봇 앞 35 cm 에서 아래를 봄 (roll 180° = 카메라 z 가 로봇 −z)
    function camCfg() {
      const v = CAM_IDS.map((id) => parseFloat($(id).value));
      if (!v.every(Number.isFinite) || v[6] <= 5 || v[6] >= 170) return null;
      return { t: v.slice(0, 3), rpy: v.slice(3, 6), hfov: v[6], aspect: 4 / 3, near: 0.3, far: 1.0 };   // 깊이 0.3~1.0 m = vision_node depth_range_m 기본
    }
    function toCamFrame(c, p) {   // 로봇 틀 점 → 카메라 틀 (camera_to_robot 의 역변환)
      let x = p[0] - c.t[0], y = p[1] - c.t[1], z = p[2] - c.t[2];
      const [roll, pitch, yaw] = c.rpy.map((a) => a * Math.PI / 180);
      let cs = Math.cos(yaw), sn = Math.sin(yaw); [x, y] = [cs * x + sn * y, -sn * x + cs * y];
      cs = Math.cos(pitch); sn = Math.sin(pitch); [x, z] = [cs * x - sn * z, sn * x + cs * z];
      cs = Math.cos(roll); sn = Math.sin(roll); [y, z] = [cs * y + sn * z, -sn * y + cs * z];
      return [x, y, z];
    }
    function camVisible(c, p) {   // null = 보임, 아니면 이유
      const q = toCamFrame(c, p), th = Math.tan(c.hfov * Math.PI / 360), tv = th / c.aspect;
      if (q[2] < c.near || q[2] > c.far) return `깊이 ${q[2].toFixed(2)} m 가 범위 ${c.near}~${c.far} m 밖`;
      if (Math.abs(q[0]) > th * q[2] || Math.abs(q[1]) > tv * q[2]) return "시야 밖";
      return null;
    }
    function camUpdate() {
      lsSet(CAM_KEY, JSON.stringify({ on: $("cam-on").checked, v: CAM_IDS.map((id) => $(id).value), path: $("cam-path").value }));
      const c = $("cam-on").checked ? camCfg() : null;
      if (state.viewer && state.viewer.setCamera) state.viewer.setCamera(c);
      const n = $("cam-vis");
      if (!c) { n.textContent = $("cam-on").checked ? "위치·자세·시야각에 숫자를 모두 넣어 주세요." : ""; n.className = "note"; return; }
      if (!spotOk()) { n.textContent = ""; return; }
      const [rx, ry] = spotToRobot(num("in-x"), num("in-y"));
      const why = camVisible(c, [rx / 100, ry / 100, -0.4525]);
      n.textContent = why ? `⚠ 지금 고른 큐브 자리는 카메라에 안 보임: ${why}` : "지금 고른 큐브 자리는 카메라 시야 안";
      n.className = "note" + (why ? " warn" : "");
    }
    async function readCamera() {
      const note = $("cam-note"), c = camCfg();
      note.className = "note"; note.textContent = "읽는 중…";
      let r;
      try { r = await api("/api/rl/vision", { path: $("cam-path").value, camera_to_robot: c ? [...c.t, ...c.rpy] : null }); }
      catch (e) { note.className = "note bad"; note.textContent = "서버 요청 실패: " + e.message; return; }
      if (!r.ok) { note.className = "note bad"; note.textContent = r.error || "읽지 못함"; return; }
      if (!r.spot) { note.className = "note warn"; note.textContent = `카메라 좌표 (${r.camera_xyz.join(", ")}) m — ${r.note}`; return; }
      $("in-x").value = r.spot.x_cm; $("in-y").value = r.spot.y_cm; $("in-yaw").value = r.spot.yaw_deg;
      if (state.viewer && state.viewer.setObject) state.viewer.setObject({ x: r.robot.x, y: r.robot.y, z: r.robot.z, yaw: r.robot.yaw_deg });   // 보라 상자 = 카메라가 본 큐브
      state.vision = { x: r.robot.x, y: r.robot.y, yaw: r.robot.yaw_deg };   // 지도의 보라 사각형
      updateNote();
      note.className = "note" + (r.warn ? " warn" : "");
      note.textContent = `큐브 읽음 (로봇 틀 x ${(r.robot.x * 100).toFixed(1)}, y ${(r.robot.y * 100).toFixed(1)}, z ${(r.robot.z * 100).toFixed(1)} cm, yaw ${r.robot.yaw_deg}°) → 위치 (${r.spot.x_cm}, ${r.spot.y_cm}) cm, 각도 ${r.spot.yaw_deg}°. ` +
        (r.warn ? r.warn + ". " : "") + r.yaw_note;
    }
    // 휴대폰 카메라 실시간 인식 → 큐브 자리 (POST /api/rl/live_cube: 큐브 바닥 = 정책 책상 높이로 잼). run = 가져와서 바로 시뮬
    async function liveCube(run) {
      const note = $("live-note");
      note.className = "note"; note.textContent = "카메라에서 큐브 읽는 중… (몇 초)";
      let r;
      try { r = await api("/api/rl/live_cube", {}); } catch (e) { note.className = "note bad"; note.textContent = "서버 요청 실패: " + e.message; return; }
      if (!r.ok) { note.className = "note bad"; note.textContent = r.error + ((r.warn || []).length ? " · " + r.warn.join(" · ") : ""); return; }
      $("in-x").value = r.spot.x_cm; $("in-y").value = r.spot.y_cm; $("in-yaw").value = r.spot.yaw_deg;
      if ($("yaw-range")) $("yaw-range").value = r.spot.yaw_deg;
      state.vision = { x: r.robot.x, y: r.robot.y, yaw: r.robot.yaw_deg };
      if (state.viewer && state.viewer.setObject) state.viewer.setObject({ x: r.robot.x, y: r.robot.y, z: r.robot.z, yaw: r.robot.yaw_deg });
      updateNote();
      const valid = isValidSpot(r.spot.x_cm, r.spot.y_cm);
      note.className = "note" + (valid === false || (r.warn || []).length ? " warn" : "");
      note.textContent = `카메라 큐브${r.id != null ? " " + r.id + "번" : ""}: 로봇 틀 x ${(r.robot.x * 100).toFixed(1)}, y ${(r.robot.y * 100).toFixed(1)} cm, yaw ${r.robot.yaw_deg.toFixed(0)}° → 위치 (${r.spot.x_cm}, ${r.spot.y_cm}) cm, 각도 ${r.spot.yaw_deg}° · ${r.text || ""}` +
        (valid === false ? " · ⚠ 정책이 학습한 범위(파란 칸) 밖 — 큐브를 파란 칸 쪽으로" : "") + ((r.warn || []).length ? " · " + r.warn.join(" · ") : "");
      if (run) {
        if (valid === false) { toast("학습 범위 밖이라 돌리지 않음 — 큐브를 파란 칸 쪽으로 옮겨 주세요", true); return; }
        await run();
      }
    }
    function initCamera() {
      let saved = null;
      try { saved = JSON.parse(lsGet(CAM_KEY) || "null"); } catch (e) { saved = null; }
      if (saved && Array.isArray(saved.v)) { CAM_IDS.forEach((id, i) => { if (saved.v[i] !== undefined) $(id).value = saved.v[i]; }); $("cam-on").checked = !!saved.on; $("cam-path").value = saved.path || ""; }
      for (const id of [...CAM_IDS, "cam-on", "cam-path"]) $(id).addEventListener("input", camUpdate);
      $("cam-on").addEventListener("change", camUpdate);
      for (const id of ["in-x", "in-y"]) $(id).addEventListener("input", camUpdate);
      $("cam-example").addEventListener("click", () => { CAM_IDS.forEach((id, i) => { $(id).value = CAM_EXAMPLE[i]; }); $("cam-on").checked = true; camUpdate(); });
      $("cam-read").addEventListener("click", readCamera);
      $("live-get").addEventListener("click", () => liveCube(false));
      $("live-run").addEventListener("click", () => liveCube(true));
      camUpdate();
    }

    // ------------------------------------------------------------ 실물 팔로 재생
    const real = { st: null, timer: null };
    function addCheck(box, ok, text) {
      const d = document.createElement("div");
      d.className = "rl-check " + (ok ? "ok" : "bad"); d.textContent = (ok ? "✓ " : "✗ ") + text; box.append(d);
    }
    function renderReal() {
      const st = real.st, box = $("real-checks"); box.replaceChildren();
      const run = state.run, sm = run ? run.summary || {} : null;
      let can = false;
      if (!st) { addCheck(box, false, "상태를 못 읽음"); }
      else if (!st.connected) { addCheck(box, false, st.why || "팔 연결 안 됨"); }
      else {
        addCheck(box, true, "실물(직접 CAN) 연결됨");
        addCheck(box, !(st.not_homed || []).length, (st.not_homed || []).length ? "원점 없는 AK45: " + st.not_homed.join(", ") + " — 각도 제어 탭에서 원점 먼저" : "AK45 원점 4개 잡힘");
        addCheck(box, !(st.unsent_servos || []).length, (st.unsent_servos || []).length ? "한 번도 안 보낸 서보: " + st.unsent_servos.join(", ") + " — 각도 제어 탭에서 시작 자세로 먼저 (첫 명령은 램프 없이 바로 튄다)" : "서보를 모두 한 번은 보냄");
        if (run) {
          addCheck(box, !(st.far || []).length, (st.far || []).length ? "지금 자세가 시작 자세와 다름: " + st.far.join(", ") + " — AK45 는 HOME, 서보는 '서보를 시작 자세로 보내기' (서보가 도는 중이면 몇 초 뒤 맞아짐)" : "지금 자세 = 시작 자세");
          addCheck(box, !(st.clip_big || []).length, (st.clip_big || []).length ? "실물 보정 범위 밖: " + st.clip_big.join("; ") + " — 큐브 각도·위치를 바꿔 다시" : "궤적이 실물 보정 범위 안");
        }
        if (st.calib_dirty) addCheck(box, false, "보정을 바꾸고 아직 저장 안 함 — 각도 제어 탭 보정에서 'w 저장'");
        if ((st.playback || {}).running) addCheck(box, false, st.playback.holding ? `첫 자세 유지 중 — ${st.playback.hold}초 뒤에 움직이기 시작` : `재생 중 ${st.playback.frame}/${st.playback.frames - 1} — 끝나거나 멈춘 뒤 다시`);
        can = !!st.can_play;
      }
      if (!run) addCheck(box, false, "먼저 ② 에서 강화학습을 돌려 궤적을 만든다");
      else if (!sm.done) addCheck(box, false, "시뮬에서 실패한 실행 — 실물로 안 보냄");
      else if (sm.valid_spot === false) addCheck(box, false, "학습 범위 밖 지점 — 실물로 안 보냄");
      const runOk = !!(run && sm.done && sm.valid_spot !== false);
      if (can && runOk && !$("ack").checked) addCheck(box, false, "아래 책상 위치 확인란에 체크해야 '실물로 재생' 이 켜짐");
      $("real-go").disabled = !(can && runOk && $("ack").checked);
      $("real-go").title = !$("ack").checked ? "책상 위치 확인란을 먼저" : "";
      const pb = st && st.playback;
      follow3D(pb);
      if (st) $("grip-note").textContent = st.grip_error ? "⚠ " + st.grip_error : Number.isFinite(st.grip_extra_deg) ? `이 궤적: 시뮬보다 최대 ${st.grip_extra_deg}° 더 닫음` : "";
      const mineRun = !!(pb && state.run && pb.run_id === state.run.run_id);
      if (mineRun && pb.running) real.wasRunning = true;
      else if (real.wasRunning && mineRun && !pb.running) {
        real.wasRunning = false;
        if (pb.reason === "끝까지 재생" && $("real-autohome").checked && typeof goHome === "function") setTimeout(goHome, 300);   // 확인창은 goHome 이 띄운다
      }
      if (pb && !pb.running && pb.reason) {
        const abn = pb.reason !== "끝까지 재생" && pb.reason !== "stopped";
        $("real-note").className = "note" + (abn ? " bad" : "");
        $("real-note").textContent = pb.reason === "끝까지 재생" ? "끝까지 재생함" : pb.reason === "stopped" ? "멈춤 버튼으로 멈춤" :
          `⚠ ${pb.frame}/${pb.frames - 1} 프레임 (약 ${(pb.frame * (pb.period || 0)).toFixed(0)}초)에서 멈춤: ${pb.reason} (연결 탭 로그에도 남음)`;
      }
    }
    // 실물 재생 중엔 3D 가 실물이 지금 보내고 있는 프레임을 따라간다 (서버 play.frame = 이 궤적의 프레임 번호, 같은 run_id 일 때만)
    function follow3D(pb) {
      const mine = !!(pb && state.run && pb.run_id === state.run.run_id);
      if (mine && pb.running) {
        if (!real.following) { pause(); real.following = true; }
        show(pb.frame);
        $("phase-now").textContent = (pb.holding ? "실물: 첫 자세 유지 중 · " : "실물 재생 중 · ") + $("phase-now").textContent;
        clearTimeout(real.fast); real.fast = setTimeout(pollReal, 250);   // 재생 중엔 빨리 읽어 3D 가 부드럽게
      } else if (real.following) {
        real.following = false;
        if (mine) show(pb.frame);                                        // 끝났거나 멈춘 자리
      }
    }
    async function pollReal() {
      if (root.offsetParent === null) return;   // 숨은 탭은 안 읽는다
      const gq = `&grip_gain=${encodeURIComponent($("grip-gain").value)}&grip_add_deg=${encodeURIComponent($("grip-add").value)}`;
      try { real.st = await api("/api/rl/real_state" + (state.run ? "?run_id=" + encodeURIComponent(state.run.run_id) + gq : "")); }
      catch (e) { real.st = null; }
      renderReal();
    }
    async function playReal() {
      if (!state.run) return;
      if (!confirm("⚠ 실물 팔로 이 강화학습 궤적을 재생한다.\n\n열린 고리라 실물에서 큐브가 시뮬과 다르게 움직여도 정책이 다시 고르지 않는다. 시작 자세에서 천천히 시작하고, " +
        "AK45 가 명령에서 15° 넘게 0.5초 벌어지면 멈추고 그 자리 홀드.\n\n팔 주변을 비웠나요? 손으로 받칠 준비를 했나요?")) return;
      if (!confirm("정말 실물로 재생할까요? (멈춤 = 이 카드의 '재생 멈춤', 위 ■ STOP / Esc)")) return;
      const body = { run_id: state.run.run_id, ack_table: $("ack").checked, max_speed: num("real-speed"),
                     grip_gain: num("grip-gain"), grip_add_deg: num("grip-add") };
      let r;
      try {
        r = await api("/api/rl/play_real", body);
        if (!r.ok && r.need_force) {
          if (!confirm("검사에서 문제가 나왔다:\n\n- " + r.problems.join("\n- ") + "\n\n그래도 재생할까요?")) return;
          r = await api("/api/rl/play_real", { ...body, force: true });
        }
      } catch (e) { r = { ok: false, error: "서버 요청 실패: " + e.message }; }
      $("real-note").className = "note" + (r.ok ? "" : " bad");
      $("real-note").textContent = r.ok ? (r.msg || "재생 시작") : (r.error || "거절됨");
      toast(r.ok ? (r.msg || "재생 시작") : (r.error || "거절됨"), !r.ok);
      pollReal();
    }
    // 이 궤적의 첫 프레임 서보 값(°)을 한 번에 보낸다. 서버의 '시작 자세와 다름' 검사와 같은 기준(첫 프레임). finger_rotation = rl_rotation + 180°.
    const RL_AK45 = new Set(["shoulder_roll", "shoulder_yaw", "upper_arm_roll", "elbow"]);
    async function sendStart() {
      if (!state.run) { toast("먼저 ② 에서 강화학습을 돌려 궤적을 만든다", true); return; }
      const joints = {};
      state.run.joint_names.forEach((n, i) => {
        if (RL_AK45.has(n)) return;
        const deg = state.run.positions[0][i] * 180 / Math.PI + (n === "rl_rotation" ? 180 : 0);
        joints[n === "rl_rotation" ? "finger_rotation" : n] = Math.round(deg * 10) / 10;
      });
      if (!confirm("⚠ 실물 서보가 움직인다.\n\n" + Object.entries(joints).map(([k, v]) => `${k} ${v}°`).join(", ") +
        "\n\n서보는 처음 보낼 때 램프 없이 바로 그 각으로 튄다. 특히 finger_rotation 이 크게(최대 180°) 돈다 — 손 주변을 비우세요.")) return;
      let r;
      try { r = await api("/api/joint/send", { joints }); } catch (e) { r = { ok: false, error: "서버 요청 실패: " + e.message }; }
      toast(r.ok ? (r.msg || "서보 보냄") : (r.error || "못 보냄"), !r.ok);
      pollReal();
    }
    function initReal() {
      $("real-start").addEventListener("click", sendStart);
      $("real-go").addEventListener("click", playReal);
      $("real-stop").addEventListener("click", async () => {
        try { const r = await api("/api/ik/stop_real", {}); toast(r.msg || (r.ok ? "멈춤 요청" : r.error || "멈춤 실패"), !r.ok); } catch (e) { toast("멈춤 요청 실패: " + e.message, true); }
        pollReal();
      });
      $("ack").addEventListener("change", renderReal);
      // HOME 으로: 대시보드의 goHome (책상 피하는 길 계획 + 확인창). 별도 페이지(/rl)엔 없어서 숨김
      if (typeof goHome === "function") $("real-home").addEventListener("click", () => goHome());
      else { $("real-home").hidden = true; $("real-autohome").closest("label").hidden = true; }
      for (const [id, key] of [["grip-gain", "rl-grip-gain"], ["grip-add", "rl-grip-add"]]) {   // 브라우저에 기억
        try { const v = localStorage.getItem(key); if (v !== null && v !== "") $(id).value = v; } catch (e) { /* 없어도 됨 */ }
        $(id).addEventListener("change", () => { try { localStorage.setItem(key, $(id).value); } catch (e) { /* 무시 */ } pollReal(); });
      }
      real.timer = setInterval(pollReal, 1500);
      pollReal();
    }


    // ------------------------------------------------------------ 손 위치로 보내기 (역기구학 /api/ik/hand → /api/joint/send, 역기구학 탭 카드와 같은 길)
    const HAND_D = 0.4775;   // 어깨 축 → 책상 윗면 m (viewer.d, 정책 장면과 같음)
    let handTimer = null;
    function handEdit() {   // 입력이 바뀌면 옛 풀이는 무효 (유령·보내기 끔), 잠깐 뒤 다시 푼다
      state.hand = null; $("hand-send").disabled = true; state.handSeq = (state.handSeq || 0) + 1;
      if (state.viewer && state.viewer.setGhost) state.viewer.setGhost(null);
      clearTimeout(handTimer); handTimer = setTimeout(solveHand, 350);
    }
    function handResult(cls, text) { const r = $("hand-result"); r.hidden = !text; r.className = "result " + (cls || ""); r.textContent = text || ""; }
    async function solveHand() {
      clearTimeout(handTimer);
      const x = num("hand-x"), y = num("hand-y"), zc = num("hand-z"), yaw = num("hand-yaw"), sy = parseFloat($("hand-sy").value);
      if (![x, y, zc, yaw].every(Number.isFinite)) { handResult("bad", "숫자를 확인해 주세요"); return; }
      handResult("run", "역기구학으로 푸는 중…");
      const seq = state.handSeq = (state.handSeq || 0) + 1;   // 2026-11-25 검토: 풀이 중 입력이 바뀌면 옛 해가 '보내기' 를 다시 켰다
      let r;
      try {
        r = await api("/api/ik/hand", { x: x / 100, y: y / 100, z: zc / 100 - HAND_D, yaw_deg: yaw, mode: "auto", tilt_max_deg: 60, ref: "grip",
          fix_deg: Number.isFinite(sy) ? { shoulder_yaw: sy } : {} });
      } catch (e) { if (seq === state.handSeq) handResult("bad", "서버 요청 실패: " + e.message); return; }
      if (seq !== state.handSeq) return;   // 그 사이 입력이 바뀜 — 새 풀이가 따로 온다
      if (!r.ok) { state.hand = null; $("hand-send").disabled = true; handResult("bad", "못 풂 — " + (r.reason || "?")); return; }
      state.hand = r; $("hand-send").disabled = false;
      const q = {}; for (const [k, v] of Object.entries(r.joints || {})) q[k] = v * Math.PI / 180;
      if (state.viewer && state.viewer.setGhost) state.viewer.setGhost(q);
      if (state.viewer && state.viewer.setHandMark) state.viewer.setHandMark({ x: x / 100, y: y / 100, z: zc / 100 - HAND_D });
      handResult(r.warn ? "warn" : "ok", `풀림 — 오차 ${Number.isFinite(r.pos_err_mm) ? r.pos_err_mm.toFixed(1) : "?"} mm · 가장 많이 도는 관절 ${r.max_move_deg != null ? r.max_move_deg + "°" : "?"}` + (r.warn ? " · " + r.warn : ""));
    }
    async function sendHand() {
      const h = state.hand;
      if (!h || !h.joints) return;
      if (!confirm("⚠ 실제 모터가 움직인다 (실물 모드면).\n\n지금 자세 → 목표 자세까지 관절마다 직선으로 간다. 중간 경로에서 팔·손이 책상이나 몸에 부딪히는지는 검사하지 않는다 — 길을 비우고 지켜본다.\n\n" +
        `가장 많이 도는 관절: ${h.max_move_deg != null ? h.max_move_deg + "°" : "?"}\n\n계속할까요?`)) return;
      let r;
      try { r = await api("/api/joint/send", { joints: h.joints }); } catch (e) { r = { ok: false, error: "서버 요청 실패: " + e.message }; }
      toast(r.ok ? (r.msg || "보냄") : (r.error || "못 보냄"), !r.ok);
    }
    function initHand() {
      for (const id of ["hand-x", "hand-y", "hand-z", "hand-yaw", "hand-sy"]) $(id).addEventListener("input", () => { handEdit(); mapUpdate(); if ($("reach-chk").checked && (id === "hand-z" || id === "hand-sy")) { clearTimeout(reachTimer); reachTimer = setTimeout(reachLoad, 600); } });
      $("hand-solve").addEventListener("click", solveHand);
      $("hand-send").addEventListener("click", sendHand);
    }

    async function init() {
      const pr = $("presets");
      PRESETS.forEach(([label, x, y, yaw]) => {
        const b = document.createElement("button"); b.className = "btn line small"; b.textContent = label;
        b.addEventListener("click", () => { $("in-x").value = x; $("in-y").value = y; $("in-yaw").value = yaw; updateNote(); });
        pr.append(b);
      });
      buildMap(); setMode(state.mode); bindMap(); mapUpdate(); updateNote();
      $("btn-run").addEventListener("click", run);
      $("btn-play").addEventListener("click", () => (state.playing ? pause() : play()));
      $("seek").addEventListener("input", () => { pause(); if (state.run) { state.t = $("seek").value * state.run.dt; show(+$("seek").value); } });
      let info = null;
      try { info = await api("/api/rl/info"); } catch (e) { info = { ok: false, error: e.message }; }
      const chip = $("chip-pol");
      if (info && info.ok) {
        state.info = info; chip.className = "chip on"; chip.lastChild.textContent = info.policy; paintGrasp();
        const kv = $("policy-info"); kv.replaceChildren();
        for (const [k, v] of [["정책", info.policy], ["신경망", info.network || "-"], ["체크포인트", info.checkpoint || "-"],
                              ["학습 범위 칸", info.valid.flat().filter(Boolean).length + " / " + info.valid.flat().length]]) {
          const a = document.createElement("span"), b = document.createElement("span"); a.className = "k"; b.className = "v"; a.textContent = k; b.textContent = v; kv.append(a, b);
        }
        updateNote();
      } else {
        chip.className = "chip bad"; chip.lastChild.textContent = "정책 없음";
        $("btn-run").disabled = true;
        setResult("bad", (info && info.error) || "정책 정보를 못 읽음");
      }
      initViewer(!!(info && info.robot && info.robot.mesh_ok));
      initCamera(); initReal(); initHand();
    }
    init();
  }

  window.JaxRL = { mount };
  const standalone = document.getElementById("rl-standalone");   // rl.html (별도 페이지)
  if (standalone) {
    mount(standalone);
    // 별도 페이지엔 대시보드의 ■ 전부 정지 가 없다 — 실물 재생·손 보내기 확인창이 말하는 'STOP / Esc' 를 여기서도 (2026-11-25 검토)
    const stop = () => fetch("/api/stop", { method: "POST", headers: { "content-type": "application/json" }, body: "{}" }).catch(() => {});
    document.addEventListener("keydown", (e) => { if (e.key === "Escape") stop(); });
    const bar = document.querySelector(".topbar");
    if (bar) { const b = document.createElement("button"); b.className = "btn-stop"; b.textContent = "■ STOP (Esc)"; b.onclick = stop; bar.append(b); }
  } else if (typeof curTab !== "undefined" && curTab === "rl") {
    const pg = document.getElementById("page-rl");   // app.js 의 첫 render() 가 먼저 돌아 mount 를 건너뜀
    if (pg) mount(pg);
  }
})();
