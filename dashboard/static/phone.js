// 휴대폰 카메라 탭: 큐브 자리를 서버가 찾고 (phone_api.py) → 역기구학 탭과 같은 계획·재생으로 집으러 간다.
// 찾는 흐름은 하나고 사진 올리기는 없다: 로봇에 거치한 S25 의 폰 스테레오 프로그램이 켜져 있으면 서버 스레드(live_api.py)가 새 프레임마다
// ArUco 와 뎁스 결과를 합쳐 (phone_fuse.py: 둘 다면 ArUco 자리 + 뎁스로 맞춰 봄, 안 맞으면 막음) 두고, 화면은 live.js 가 0.5초마다 받는
// GET /api/live/cubes 의 pick 을 쓴다. 프레임이 1.5초 넘게 안 오면 '영상 없음' 으로 막는다. 가상 모드는 카메라를 안 쓰고 가상 큐브만 (지도에서 놓기).
// 폰 프로그램이 꺼져 있으면 큰 '폰 프로그램 켜기' 버튼 하나만 보인다. 끄기와 설정은 아래 접힌 '준비' 에 있다.
// app.js·direct.js 는 안 고친다: 탭 버튼도 여기서 단다. 계획 = POST /api/svc/pick_dry/start (역기구학 탭 칸 그대로, 큐브 칸만 바꿈),
// 재생 = 역기구학 탭의 원래 버튼(가상 팔로 재생 / 실물로 재생)을 그대로 누른다 → 확인창·검사·force 확인이 전부 원래 것.
"use strict";
// 지도 (탑 쌓기 지도와 같은 범위·방향): 큐브들 + 로봇 쪽. 가상 모드면 빈 곳 = 큐브 놓기, 끌기 = 옮기기
const PH_BOX = [-0.27, 0.23, -0.45, -0.13];
const PH = { cube: null, grasp: null, going: false, dp: null, dBusy: false, block: "", last: null, d: null };

(function phInstall() {
  const page = document.getElementById("page-phone");
  if (!page) return;
  document.head.append(el("link", { rel: "stylesheet", href: "/static/phone.css" }));
  // 탭 버튼 (app.js buildTabs 와 같은 모양)
  TABS.push(["phone", "휴대폰 카메라", "사진으로 집기"]);
  TAB_IDS.add("phone");
  const b = el("a", { class: "tab", href: "#phone", role: "tab" }, "휴대폰 카메라", el("small", { class: "tsub" }, "사진으로 집기"), el("span", { class: "tdot" }));
  tabBtns.phone = b;
  $("#tabs").append(b);

  PH.bTh = el("button", { class: "btn line", onclick: phTable }, "책상 높이 재기");
  PH.bThFill = el("button", { class: "btn", disabled: true, onclick: phTableFill }, "역기구학 탭에 넣기");
  PH.thImg = el("img", { class: "ph-img", hidden: true, alt: "책상 높이를 잰 사진" });
  PH.thRes = el("div", { class: "result", hidden: true });
  PH.bOn = el("button", { class: "btn ph-big", hidden: true, onclick: phPipe }, "폰 프로그램 켜기");
  PH.dStat = el("div", { class: "help ph-stat" });
  PH.img = el("img", { class: "ph-img", hidden: true, alt: "지금 프레임" });
  PH.map = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  PH.map.setAttribute("class", "ph-map");
  PH.map.setAttribute("viewBox", `0 0 ${(PH_BOX[1] - PH_BOX[0]) * 1000} ${(PH_BOX[3] - PH_BOX[2]) * 1000}`);
  lvMapAttach(PH.map, { box: PH_BOX, redraw: phMapDraw, edit: () => LQ.virtual(), onCube: () => { PH.cube = null; } });
  PH.src = el("div", { class: "lv-src" });
  PH.res = el("div", { class: "result", hidden: true });
  PH.grasp_el = el("div", { class: "result", hidden: true });
  PH.bGo = el("button", { class: "btn ph-big", disabled: true, onclick: phGo }, "집으러 가기");
  PH.bStop = el("button", { class: "btn red", onclick: () => post(VIRT() ? "/api/ik/stop_virtual" : "/api/ik/stop_real") }, "■ 멈춤");
  PH.goNote = el("div", { class: "result", hidden: true });
  // 강화학습 버전 (2026-11-25 사용자: 카메라에 두 가지 버전) — 강화학습 탭의 카메라 큐브 → 정책 → 실물 재생과 같은 서버 길
  PH.bRL = el("button", { class: "btn ph-big", style: "background:#7c3aed", onclick: phRL }, "강화학습으로 집으러 가기");
  PH.rlNote = el("div", { class: "result", hidden: true });
  // ③ 카메라 보정 (역기구학으로) — 2026-11-25 사용자 '카메라 보정 좀 더 똑디, 일단 역기구학으로'
  PH.cc = { busy: false, stop: false, rows: [] };
  PH.ccFix = el("div", { class: "help", style: "margin:0 0 8px" }, "지금 카메라 보정: …");
  PH.ccStart = el("button", { class: "btn", onclick: ccRun }, "보정 시작 (팔이 큐브를 몇 곳에 놓기)");
  PH.ccStopB = el("button", { class: "btn line", onclick: () => { PH.cc.stop = true; ccShow("이번 자리를 마치고 그만둔다 (팔을 바로 멈추려면 ■ STOP / Esc)", "warn"); } }, "그만");
  PH.ccNote = el("div", { class: "result", hidden: true });
  PH.ccTable = el("div", {});
  PH.ccApply = el("button", { class: "btn", disabled: true, onclick: ccApply }, "적용 (저장)");
  PH.ccClear = el("button", { class: "btn line", disabled: true, onclick: ccClear }, "보정 지우기");
  setTimeout(ccFixText, 500);
  PH.target = el("div", { class: "help", style: "margin:10px 0 0" });
  PH.f = {}; PH.df = {};
  PH.bPipe = el("button", { class: "btn line", onclick: phPipe }, "폰 프로그램 켜기");
  PH.pipeNote = el("div", { class: "help", style: "margin:10px 0 0" });
  PH.cmd = el("code", { class: "ph-cmd" });
  const dfld = (k, lab, unit, wide) => el("label", { class: "field" + (wide ? " ph-wide" : "") }, el("span", {}, lab, el("small", {}, " " + unit)), PH.df[k] = el("input", wide ? {} : { inputmode: "decimal" }));
  const fld = (k, lab, unit) => el("label", { class: "field" }, el("span", {}, lab, el("small", {}, " " + unit)), PH.f[k] = el("input", { inputmode: "decimal" }));
  page.append(
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "1"), "큐브 찾기"),
    el("div", { class: "card" },
      PH.bOn, PH.dStat, PH.img, PH.src, PH.map, lvTools(), PH.res, PH.grasp_el,
      PH.help = el("div", { class: "help", style: "margin:10px 0 0" }, "로봇에 거치한 폰의 영상에서 새 프레임마다 찾는다. 큐브 자리·각은 큐브 윗면 ArUco 마커로만 정한다 (기준 종이는 없어도 된다 — 없으면 거치 자세와 책상 높이로). 뎁스·세그는 화면에 보여 주기만 하고 판정에 안 쓴다 — 마커가 안 보이면 안 집는다. 지도에서 큐브를 누르면 그 큐브를 집는다.")),
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "2"), "집으러 가기 — 두 가지"),
    el("div", { class: "card" },
      el("div", { style: "font-weight:700;margin:0 0 6px" }, "A. 역기구학 (수식으로 계획) — 책상 위 큐브"),
      PH.bGo,
      el("div", { class: "ph-row" }, PH.bStop, el("a", { class: "btn line", href: "#ik" }, "역기구학 탭에서 보기")),
      PH.goNote, PH.target,
      el("div", { class: "help", style: "margin:6px 0 0" }, "계획(역기구학 탭과 같은 것)을 돌린 뒤, 지금 모드의 팔로 재생할지 확인창이 묻는다.")),
    el("div", { class: "card" },
      el("div", { style: "font-weight:700;margin:0 0 6px" }, "B. 강화학습 (정책으로 집기) — 받침대 위 큐브"),
      PH.bRL,
      el("div", { class: "ph-row" }, el("a", { class: "btn line", href: "#rl" }, "강화학습 탭에서 보기")),
      PH.rlNote,
      el("div", { class: "help", style: "margin:6px 0 0" }, "카메라 큐브 → 강화학습 정책을 시뮬로 돌려 성공하면 → 실물 팔로 재생할지 확인창이 묻는다. 정책은 책상이 어깨 아래 47.75 cm 인 장면에서 배워서, 실물 책상이면 큐브를 약 8 cm 받침대 위에 둔다. 멈춤 = 위 ■ STOP / Esc.")),
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "3"), "카메라 보정 (역기구학으로)"),
    el("div", { class: "card" },
      PH.ccFix,
      el("div", { class: "ph-row" }, PH.ccStart, PH.ccStopB),
      PH.ccNote, PH.ccTable,
      el("div", { class: "ph-row" }, PH.ccApply, PH.ccClear),
      el("div", { class: "help", style: "margin:6px 0 0" }, "팔이 역기구학으로 큐브를 정해진 자리 몇 곳에 놓으면 그 자리가 정답이다. 놓을 때마다 HOME 으로 비킨 뒤 카메라가 읽어, ",
        "카메라 자리와 정답의 차이로 회전 + 이동을 맞춘다 (자리 고침 pos_fix 는 한 방향으로 밀기만 해서 카메라가 돌아가 있으면 한쪽만 맞는다). ",
        "첫 번째만 카메라 자리로 집고, 그다음은 팔이 놓은 자리에서 집는다. 저장하면 휴대폰 탭·강화학습의 카메라 큐브 자리에 들어간다.")),
    PH.thH = el("h2", { class: "sec" }, "책상 높이"),
    PH.thCard = el("div", { class: "card" },
      el("div", { class: "ph-row", style: "margin-top:0" }, PH.bTh, PH.bThFill),
      PH.thImg, PH.thRes,
      el("div", { class: "help", style: "margin:10px 0 0" }, "거치한 폰의 화면으로 잰다 (폰 프로그램이 켜져 있어야 한다): 책상 위 큐브의 윗면 마커로 (여러 개면 중앙값), 기준 종이가 있으면 그것으로, 뎁스 맵이 새것이면 뎁스로도 재서 차이를 보여 준다.",
        el("br"), "보정 없이 CAD 거치 자리를 믿는다 — 렌즈 자리·초점거리 때문에 1~3 cm 틀릴 수 있다. 처음 한 번은 줄자로 잰 값(어깨 축 ~ 책상 윗면)과 맞춰 보기.")),
    el("details", { class: "adv" }, el("summary", {}, "준비 — 인쇄물 · 폰 프로그램(뎁스) · 자리 설정"),
      el("h3", { class: "ph-h3" }, "ArUco — 인쇄물 · 기준 종이 자리 (종이는 선택)"),
      el("div", { class: "card" },
        el("a", { class: "btn line", href: "/api/phone/sheet", target: "_blank" }, "인쇄물 PDF 열기 (A4 가로 2쪽)"),
        el("ol", { class: "help" },
          el("li", {}, "실제 크기 100% 로 인쇄 (1쪽의 100 mm 선을 자로 확인)."),
          el("li", {}, "(선택) 1쪽(기준 종이)을 책상에 평평하게 붙인다: '로봇 쪽' 글자가 로봇을 향하게, 가운데 십자가 아래 자리에 오게. 없으면 거치 자세(CAD)와 책상 높이로 자리를 구한다 — 종이가 있으면 더 정확하다."),
          el("li", {}, "2쪽의 마커 한 장을 50 mm 선으로 오려 큐브 윗면에 붙인다.")),
        el("div", { class: "frow" }, el("b", {}, "종이 가운데"), fld("x", "x", "m"), fld("y", "y", "m"), fld("rot_deg", "돌아간 각", "°"), fld("cube_h", "큐브 높이", "m")),
        el("div", { class: "help", style: "margin:8px 0 0" }, "로봇 좌표: 원점 = 어깨 축 바로 아래, +x = 로봇 왼쪽, 앞(책상 쪽) = −y. 각 = 위에서 볼 때 종이가 반시계로 돌아간 만큼."),
        el("div", { class: "ph-row" }, el("button", { class: "btn line", onclick: phSaveCfg }, "종이 자리 저장"))),
      el("h3", { class: "ph-h3" }, "뎁스 — 폰 프로그램 · 거치 자세"),
      el("div", { class: "card" },
        el("div", { class: "help", style: "margin:0" }, "로봇에 거치한 폰 (Galaxy S24/S25, USB) 의 화면과 두 렌즈 깊이로 책상 위 큐브를 찾는다 (표시지가 없어도 뎁스로)."),
        PH.calib = el("div", { class: "help", style: "margin:8px 0 0" }, "보정 파일: …"),
        el("div", { class: "ph-row" }, PH.bPipe), PH.pipeNote,
        el("label", { class: "help", style: "display:flex;gap:8px;align-items:center;margin:14px 0 0" }, PH.df.save_rgbd = el("input", { type: "checkbox" }),
          "뎁스 맵 저장 (책상 높이를 뎁스로도 재려면 — 저장한 뒤 폰 프로그램을 껐다 켜기)"),
        // 2026-11-25: 자로 잴 것 없는 자동 보정 — 폰 중력 센서로 기울기, 큐브 윗면 마커(40 mm)로 책상 높이 d (phone_api.autocal)
        el("div", { class: "ph-row" }, PH.bCal = el("button", { class: "btn", onclick: phAutocal }, "자동 보정 (중력 센서 + 큐브 마커 · 자 필요 없음)")),
        PH.calNote = el("div", { class: "result", hidden: true }),
        // 2026-11-25 사용자: 카메라로 집으면 약 3 cm 앞으로 감 → 찾은 큐브 자리를 고친다 (ArUco·뎁스 둘 다, 기본 y +3 cm)
        el("div", { class: "frow" }, el("b", {}, "자리 고침"), dfld("fx", "x (+ = 로봇 왼쪽)", "cm"), dfld("fy", "y (+ = 로봇 쪽 = 뒤)", "cm")),
        el("div", { class: "help", style: "margin:4px 0 0" }, "카메라로 집을 때 손이 큐브보다 앞으로 가면 y 를 + 로 (그만큼 로봇 쪽으로), 왼쪽으로 치우치면 x 를 − 로. 저장하면 바로 다음 프레임부터."),
        el("div", { class: "frow" }, el("b", {}, "거치 자세"), dfld("c0", "tx", "m"), dfld("c1", "ty", "m"), dfld("c2", "tz", "m"), dfld("c3", "roll", "°"), dfld("c4", "pitch", "°"), dfld("c5", "yaw", "°")),
        el("div", { class: "help", style: "margin:8px 0 0" }, "카메라 좌표 → 로봇 좌표 (팀 vision_node 의 camera_to_robot 과 같은 숫자 6개). 처음 값은 CAD 임시값이라 실물에서 보정해야 한다: ",
          "큐브를 아는 자리 3곳 이상에 놓고 결과 줄의 '카메라 좌표' 를 적어 dashboard/tools/solve_phone_mount.py 로 푼다 (README)."),
        el("div", { class: "frow" }, el("b", {}, "큐브 각"), dfld("yaw", "직접 넣기", "° (비우면 추정)")),
        el("div", { class: "help", style: "margin:8px 0 0" }, "뎁스만으로 찾을 때 쓰는 각. 추정한 각은 보이는 면의 방향에서 구한 근삿값이다 (작은 큐브에서는 10° 안팎 틀릴 수 있다). 큐브는 90° 마다 같아서 −45~45° 로만 나온다. ArUco 가 같이 있으면 ArUco 각을 쓴다."),
        el("div", { class: "frow" }, dfld("repo_dir", "폰 스테레오 저장소", "폴더", true), dfld("run_dir", "결과 폴더", "(비우면 저장소/phone_stereo/runs/dash)", true)),
        // 2026-11-25 사용자: USB 가 불안정 → 핫스팟·Tailscale 로 1 초에 한 장. 비우면 USB(adb), auto = PC 를 폰 핫스팟에 붙였을 때 (기본 게이트웨이 = 폰)
        el("div", { class: "frow" }, dfld("wifi_host", "Wi-Fi 로 받기 (폰 IP)", "비우면 USB · auto = 폰 핫스팟 · Tailscale 은 100.x IP", true), dfld("send_fps", "Wi-Fi 장/초", "기본 1")),
        el("div", { class: "help", style: "margin:6px 0 0" }, "Wi-Fi: PC 를 폰 핫스팟(또는 같은 Tailscale)에 붙이고, 처음 한 번은 USB 를 꽂은 채 '폰 프로그램 켜기' — 앱이 설정을 기억해서 다음부터는 케이블 없이 폰에서 Phone Stereo 앱만 열면 된다. 앱 화면에 폰 주소가 나온다."),
        el("label", { class: "help", style: "display:flex;gap:8px;align-items:center;margin:8px 0 0" }, PH.df.text_rot180 = el("input", { type: "checkbox" }),
          "결과 그림 글자를 거꾸로 써 두기 (폰이 거꾸로 끼워져 화면에서 그림을 뒤집어 볼 때 글자가 바로 읽힘 — 바꾸면 폰 프로그램을 껐다 켜기)"),
        el("div", { class: "ph-row" }, el("button", { class: "btn line", onclick: phSaveDepthCfg }, "폰 뎁스 설정 저장")),
        el("div", { class: "help", style: "margin:10px 0 0" }, "터미널에서 직접 켤 때:"), PH.cmd)));
  fetch("/api/phone/config").then((r) => r.json()).then((d) => { for (const k in PH.f) PH.f[k].value = d.cfg[k]; }).catch(() => {});
  phLoadDepthCfg();
  LQ.on(phLive);
  setInterval(phTick, 500);
  if (location.hash === "#phone" && curTab && curTab !== "phone") showTab("phone");   // 첫 화면이 이 파일보다 먼저 그려졌을 때
})();

async function phSaveCfg() {
  const body = Object.fromEntries(Object.entries(PH.f).map(([k, i]) => [k, parseFloat(i.value)]));
  await post("/api/phone/config", body);
}

function phShow(node, text, cls, small) {
  node.hidden = false; node.className = "result " + (cls || "");
  node.replaceChildren(text, small ? el("small", {}, small) : "");
}

const PH_YAW_FROM = { side: "옆면 방향으로 추정", top: "윗면 모서리로 추정", mixed: "추정(두 면이 섞임)", manual: "직접 넣은 값" };
function phMapDraw() {
  const px = (x) => (x - PH_BOX[0]) * 1000, py = (y) => (PH_BOX[3] - y) * 1000;
  const g = [LV_SVG("circle", { cx: px(0), cy: py(-0.145), r: 7, fill: "#64748b" }), LV_SVG("text", { x: px(0) + 12, y: py(-0.145) + 5, "font-size": 15, fill: "#64748b" }, "로봇 쪽")];
  g.push(...lvDrawCubes(px, py, { empty: [-0.02, -0.30], emptyText: LQ.virtual() ? "눌러서 가상 큐브 놓기" : "보이는 큐브 없음" }));
  PH.map.replaceChildren(...g);
}
// 새 인식 결과마다 (live.js, GET /api/live/cubes): pick = 집을 큐브 하나 (phone_fuse 결과 / 가상 큐브) → 결과 줄·그림·폰 프로그램 상태
function phLive(all) {
  if (curTab !== "phone") return;
  const d = tableD();
  const virt = all.mode === "virtual";
  const r = Object.assign({}, all.pick || { ok: false, reason: all.reason, warn: [] });
  r.pipeline = all.pipeline || {}; r.fresh = !all.stale;
  PH.dp = r;
  phMapDraw();
  PH.src.textContent = lvSrcText(all);
  if (virt) PH.bOn.hidden = PH.dStat.hidden = true;
  PH.thH.hidden = PH.thCard.hidden = virt;
  PH.help.textContent = virt ? "가상 모드: 카메라는 안 쓴다. 지도의 빈 곳을 누르면 가상 큐브를 놓고, 끌어서 옮기고, 누르면 그 큐브를 집는다. 가상 팔로 끝까지 재생하면 큐브가 놓을 자리로 옮겨진다."
    : "로봇에 거치한 폰의 영상에서 새 프레임마다 찾는다. 큐브 자리·각은 큐브 윗면 ArUco 마커로만 정한다 (기준 종이는 없어도 된다 — 없으면 거치 자세와 책상 높이로). 뎁스·세그는 화면에 보여 주기만 하고 판정에 안 쓴다 — 마커가 안 보이면 안 집는다. 지도에서 큐브를 누르면 그 큐브를 집는다.";
  if (virt) {
    PH.img.hidden = true;
    if (PH.imgSrc) { PH.imgSrc = ""; streamSet(PH.img, false); }   // 숨긴 MJPEG 스트림 닫기 (2026-11-25 검토)
    if (PH.inf) PH.inf.hidden = PH.infFlipRow.hidden = true;
    if (!r.ok) { PH.cube = null; PH.grasp = null; PH.block = ""; PH.last = null; PH.grasp_el.hidden = true; phShow(PH.res, "가상 큐브 없음", "", r.reason || ""); return; }
    PH.last = r; PH.block = r.blocked ? r.block : "";
    phShow(PH.res, el("span", {}, el("span", { class: "ph-xy" }, `큐브  x ${r.cube.x.toFixed(3)}  y ${r.cube.y.toFixed(3)} m  yaw ${r.cube.yaw_deg.toFixed(0)}°`), el("b", { class: "ph-src" }, r.text)),
      r.blocked ? "bad" : "", (r.warn || []).join(" · "));
    const c0 = PH.cube;
    if (!c0 || Math.hypot(c0.x - r.cube.x, c0.y - r.cube.y) > 0.003 || Math.abs(c0.yaw_deg - r.cube.yaw_deg) > 2 || PH.d !== d) { PH.d = d; phGrasp(r.cube, " — 큐브를 옮기기"); }
    return;
  }
  const pl = r.pipeline || {}, fresh = !!r.fresh;
  PH.bPipe.textContent = pl.alive ? "폰 프로그램 끄기" : "폰 프로그램 켜기";
  PH.bPipe.classList.toggle("red", !!pl.alive); PH.bPipe.classList.toggle("line", !pl.alive);
  // 2026-11-25: 폰 프로그램은 살아 있는데 영상이 멈춤 (USB 가 끊겨 다시 붙기 5 번 실패한 뒤) → 큰 버튼을 '다시 켜기' 로 (예전엔 숨겨져 끄기를 못 찾았다)
  const age = all.age_s;
  PH.stuck = !!pl.alive && !fresh && (age == null || age > 30) && Date.now() - (PH.pipeT || 0) > 40000;
  PH.bOn.hidden = fresh || (!!pl.alive && !PH.stuck);
  PH.bOn.textContent = PH.stuck ? "폰 카메라 다시 켜기 (영상이 멈춤)" : "폰 프로그램 켜기";
  PH.pipeNote.textContent = pl.alive ? `폰 프로그램 켜짐 (pid ${pl.pid})` + (fresh ? "" : " — 아직 결과 없음: 폰 잠금을 풀고 몇 초 가만히. " + (pl.log || []).slice(-1).join(""))
    : fresh ? "결과가 들어오는 중 (다른 곳에서 켠 프로그램)" : "폰 프로그램 꺼짐" + ((PH.missing || []).length ? " — 이 PC 에 없는 것: " + PH.missing.join(" / ") : "") + ((pl.log || []).length > 1 ? " · 마지막 기록: " + pl.log.slice(-1)[0] : "");
  PH.dStat.hidden = fresh;
  PH.dStat.textContent = PH.stuck ? `폰 프로그램은 켜져 있는데 영상이 ${age == null ? "" : Math.round(age) + "초째 "}없음 — 위 '다시 켜기' (USB 가 끊겼다 붙었을 때 이렇게 멈춘다)`
    : pl.alive ? "폰 프로그램 켜는 중 — 폰 잠금을 풀고 몇 초 가만히 (끄기는 아래 준비)" : PH.pipeNote.textContent;
  // 영상: MJPEG 스트림 하나를 열어 둔다 (새 프레임마다 바로 보임, 최대 15 Hz). 끊기면(영상 없음) 닫았다가 다시 열기
  // 뎁스·세그 결과 (폰 프로그램의 inference.png: 객체 세그 + 뎁스 색) — 원본 영상 아래 (2026-11-24 사용자)
  if (!PH.inf) {
    PH.inf = el("img", { class: "ph-img", alt: "뎁스·세그", title: "폰 프로그램의 뎁스·세그 결과 (1초쯤마다)" });
    // 폰이 거치대에 거꾸로 끼워져 위아래가 뒤집혀 보임 → 기본으로 칸마다 180° 돌려 보인다 (화면만; 인식은 원본). 브라우저마다 기억
    let flip = true;
    try { flip = localStorage.getItem("ph-inf-flip2") !== "0"; } catch (e) { /* 저장소 막힘 → 기본 */ }
    PH.infFlip = el("input", { type: "checkbox" });
    PH.infFlip.checked = flip;
    PH.infFlip.onchange = () => { try { localStorage.setItem("ph-inf-flip2", PH.infFlip.checked ? "1" : "0"); } catch (e) { /* 무시 */ } PH.infT = 0; };
    PH.infFlipRow = el("label", { class: "help", style: "display:flex;gap:6px;align-items:center;margin-top:6px" }, PH.infFlip, "카메라 영상·뎁스 그림 위아래 뒤집어 보기 (폰이 거꾸로 끼워짐 — 글자는 뒤집었을 때 바로 읽히게 써 둠, 인식은 원본으로)");
    PH.img.after(PH.inf, PH.infFlipRow);
  }
  // 영상도 같은 칸으로 뒤집어 본다 (2026-11-25 사용자 '또 왜 반대로': 영상은 안 뒤집혀 거꾸로 보였다)
  const src = !fresh || all.frame_n == null ? "" : "/api/live/stream.mjpg" + (PH.infFlip.checked ? "?flip=1" : "");
  PH.img.hidden = !src;
  if (PH.imgSrc !== src) { PH.imgSrc = src; streamSet(PH.img, !!src, PH.infFlip.checked ? "&flip=1" : ""); }
  PH.inf.hidden = PH.infFlipRow.hidden = !src;
  const now = Date.now();
  if (src && (!PH.infT || now - PH.infT > 1000)) { PH.infT = now; PH.inf.src = "/api/live/inference.png?t=" + now + (PH.infFlip.checked ? "&flip=1" : ""); }
  const tail = (r.note || []).concat(r.warn || []).join(" · ");
  if (!r.ok) {
    PH.cube = null; PH.grasp = null; PH.block = ""; PH.last = null; PH.grasp_el.hidden = true;
    phShow(PH.res, fresh ? "큐브를 못 찾음" : "영상 없음 — 집기 막음", "bad", (r.reason || "") + (tail ? " · " + tail : ""));
    return;
  }
  const a = r.aruco && r.aruco.ok ? r.aruco : null, p = r.depth && r.depth.ok ? r.depth : null;
  const det = [];
  if (a) det.push(`ArUco: ${a.refs.length ? `기준 마커 ${a.refs.length}개` : "기준 종이 없음 (거치 자세 + 책상 높이)"} · 큐브 마커 ${a.cube_marker_mm} mm 로 재짐 (원래 40)`);
  if (p) det.push(`뎁스 (표시만, 판정 안 씀): ${p.age_s.toFixed(0)}초 전 · 크기 ${p.dims_mm[0].toFixed(0)}×${p.dims_mm[1].toFixed(0)} mm · 책상 위 ${p.height_cm.toFixed(1)} cm · 각 ${PH_YAW_FROM[p.yaw_from] || "모름"}` +
    ` · 카메라 좌표 ${p.cam.map((v) => v.toFixed(4)).join(", ")}`);
  if (tail) det.push(tail);
  PH.last = r; PH.block = r.blocked ? r.block : "";
  phShow(PH.res, el("span", {}, el("span", { class: "ph-xy" }, `큐브  x ${r.cube.x.toFixed(3)}  y ${r.cube.y.toFixed(3)} m  yaw ${r.cube.yaw_deg.toFixed(0)}°`), el("b", { class: "ph-src" }, r.text)),
    r.blocked ? "bad" : (r.warn || []).length ? "warn" : "", det.join(" · "));
  // 책상 높이·거치 자세가 안 맞아 막혔으면 그 자리에서 자동 보정 (2026-11-25: 설정 카드 안에 묻혀 있어 못 찾았다)
  const misfit = r.blocked && /층이 확실하지 않음|안 맞음/.test((r.block || "") + (r.text || ""));   // 마커 크기가 틀림 = 책상 높이·거치 자세 (뎁스는 표시만이라 안 봄)
  if (misfit) {
    if (!PH.resCal) PH.resCal = el("button", { class: "btn small", style: "margin-top:8px", onclick: phAutocal }, "자동 보정 — 책상 높이·카메라 기울기 재서 맞추기 (자 필요 없음)");
    PH.res.append(el("div", {}, PH.resCal));
  }
  const c = PH.cube;   // 거의 같은 자리면 그대로 둔다 (2026-11-25 '너무 왔다갔다': 3 mm·2° → 1 cm·5°, 각은 90° 주기 — −45°↔+45° 는 같은 것)
  const dyaw = c ? Math.abs(((c.yaw_deg - r.cube.yaw_deg) % 90 + 135) % 90 - 45) : 0;
  if (!c || Math.hypot(c.x - r.cube.x, c.y - r.cube.y) > 0.01 || dyaw > 5 || PH.d !== d) { PH.d = d; phGrasp(r.cube, " — 큐브를 옮기기"); }
}

// 찾은 큐브를 PH.cube 로 두고, 그 자리를 집을 수 있는지 (지도와 같은 판정) 묻는다
async function phGrasp(cube, hint) {
  PH.cube = cube; PH.grasp = null;
  phShow(PH.grasp_el, "집을 수 있는지 확인 중…", "run");
  let g;
  try {
    g = await (await fetch("/api/ik/graspable", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ x: cube.x, y: cube.y, d: tableD() }) })).json();
  } catch (e) { g = { ok: false, reason: "서버 연결 실패" }; }
  if (PH.cube !== cube) return;   // 그 사이 새 결과
  PH.grasp = g;
  if (g.ok && g.graspable) phShow(PH.grasp_el, "집을 수 있는 자리", "ok");
  else phShow(PH.grasp_el, g.ok ? "여기는 못 집음" : "집을 수 있는지 확인 못 함", g.ok ? "bad" : "warn", (g.reason || "") + (g.ok ? hint : ""));
}

// ---- 책상 높이 재기 (phone_table.py): 거치한 폰의 지금 프레임(큐브 윗면 마커·기준 종이 ArUco) 과 폰 뎁스 맵 → d. 넣는 것은 사람이 버튼으로 ----
async function phTable() {
  if (PH.thBusy) return;
  PH.thBusy = true; PH.th = null; PH.bTh.disabled = PH.bThFill.disabled = true;
  phShow(PH.thRes, "책상 높이 재는 중…", "run");
  let r;
  try {
    r = await (await fetch("/api/phone/table_height", { method: "POST", headers: { "content-type": "application/json" },
      body: JSON.stringify({ d: tableDNum() }) })).json();   // 지금 d 칸 — 큐브 마커 값은 이것·종이·뎁스 중 하나와 2 cm 안이어야 넣는다
  } catch (e) { r = { ok: false, reason: "서버 연결 실패" }; }
  PH.thBusy = false; PH.bTh.disabled = false;
  PH.thImg.hidden = !r.image;
  if (r.image) PH.thImg.src = r.image;
  const warn = (r.warn || []).join(" · ");
  if (!r.ok) { phShow(PH.thRes, "책상 높이를 못 잼", "bad", (r.reason || "") + (warn ? " · " + warn : "")); return; }
  let text = r.text;
  const a = r.aruco && r.aruco.ok ? r.aruco : null, cb = r.cubes && r.cubes.ok ? r.cubes : null, p = r.depth && r.depth.ok ? r.depth : null;
  if (a || cb) PH.thA = { d: (a || cb).d, t: Date.now() };
  else if (PH.thA && Date.now() - PH.thA.t < 600000)    // 이번엔 뎁스만: 10분 안에 ArUco 로 잰 값이 있으면 같이
    text += ` · ArUco ${PH.thA.d.toFixed(3)} m (${Math.round((Date.now() - PH.thA.t) / 60000)}분 전) · 차이 ${Math.round(Math.abs(PH.thA.d - r.d) * 1000)} mm`;
  const m = a || cb || p, now = tableDNum();
  const note = [`기울기 ${m.tilt_deg.toFixed(1)}° (CAD 와 차이, 0 이어야)`, `카메라가 책상 위 ${m.cam_above_table.toFixed(3)} m`,
    a ? `기준 마커 ${a.refs.length}개 · 초점거리 추정 흔들림 ± ${a.d_pm_mm.toFixed(0)} mm`
      : cb ? `큐브 마커 ${cb.n}개 (${cb.ids.join(", ")}) · 한 변 ${cb.marker_px} px · 흔들림 ± ${cb.d_pm_mm.toFixed(0)} mm · 마커 법선으로 ${cb.d_normal.toFixed(3)} m`
      : `평면 점 ${Math.round(p.inlier * 100)} % · 잔차 ${p.rms_mm} mm`,
    `지금 칸 ${now.toFixed(4)} m 와 ${Math.round((r.d - now) * 1000)} mm 차이`];
  if (!r.fill_ok && r.reason) note.unshift(r.reason);
  if (warn) note.push(warn);
  PH.th = r;
  phShow(PH.thRes, el("span", { class: "ph-xy" }, text), !r.fill_ok ? "bad" : warn ? "warn" : "ok", note.join(" · "));
  PH.bThFill.disabled = !r.fill_ok;   // 범위 밖 · 확인 안 된 큐브 마커 값이면 서버가 fill_ok false
}

function phTableFill() {
  const f = M.c && M.c.fields.d;
  if (!f || !PH.th || !PH.th.fill_ok) return;
  f.value = PH.th.d.toFixed(3);
  f.dispatchEvent(new Event("input", { bubbles: true }));   // 지도·집을 수 있는 곳이 새 높이로 다시 그려진다 (app.js 의 칸 처리 그대로)
  PH.bThFill.disabled = true;
  toast(`역기구학 탭 책상 높이 = ${f.value} m (어깨 축 아래)`);   // 큐브는 다음 phLive 가 새 높이로 다시 본다
}

// ---- 자동 보정: 기울기(중력 센서) + 책상 높이(큐브 마커) → 확인 → 폰 뎁스 설정 저장 + 역기구학 탭 d ----
async function phAutocal() {
  PH.bCal.disabled = true;
  phShow(PH.calNote, "재는 중… (폰 프로그램 프레임 5장, 5초쯤)", "run", "큐브를 책상 위에 놓고 윗면 마커가 화면에 보이게, 폰·큐브를 건드리지 않는다");
  try {
    const r = await post("/api/phone/autocal", { apply: false }, true);
    if (!r.ok) { phShow(PH.calNote, "자동 보정 못 함", "bad", r.reason || r.error || "?"); return; }
    const ck = r.check || {}, gap = ck.gap && ck.gap.dpos_mm != null ? ` · ArUco·뎁스 차 ${(ck.gap.dpos_mm / 10).toFixed(1)} cm` : "";
    const info = `책상 높이 d ${r.d} m (어깨 축 아래, 지금 칸 ${M.c && M.c.fields.d && M.c.fields.d.value || "0.5575(기본)"}) · 기울기 ${r.tilt_fix_deg}° 고침 ` +
      `(roll ${r.c2r_old[3].toFixed(1)}→${r.c2r[3].toFixed(1)}°, pitch ${r.c2r_old[4].toFixed(1)}→${r.c2r[4].toFixed(1)}°) · 마커 ${r.n_markers}번 (${r.markers.join(", ")}) · 흔들림 ${r.spread_mm} mm` +
      (ck.marker_mm != null ? ` · 고친 뒤 마커 ${ck.marker_mm} mm (원래 40)` : "") + gap + ((r.warn || []).length ? " · ⚠ " + r.warn.join(" · ") : "");
    phShow(PH.calNote, "잰 값 (아직 저장 안 함)", r.warn && r.warn.length ? "warn" : "ok", info);
    if (!confirm(`자동 보정 결과\n\n${info}\n\n거치 자세(기울기)를 폰 뎁스 설정에 저장하고, 역기구학 탭 책상 높이를 ${r.d} m 로 넣을까요?\n(카메라 자리 tx·ty·tz 와 yaw 는 CAD 그대로 — 중력으로는 못 잰다)`)) return;
    const a = await post("/api/phone/autocal", { apply: true }, true);
    if (!a.ok || !a.applied) { phShow(PH.calNote, "저장 못 함", "bad", a.reason || "?"); return; }
    const f = M.c && M.c.fields.d;
    if (f && a.in_range) { f.value = a.d.toFixed(3); f.dispatchEvent(new Event("input", { bubbles: true })); }
    phShow(PH.calNote, "보정 저장함", "ok", `거치 자세 [${a.c2r.map((v) => (+v).toFixed(3)).join(", ")}] · 역기구학 탭 d = ${a.in_range ? a.d.toFixed(3) + " m" : "범위 밖이라 안 넣음"}`);
    toast(`자동 보정 저장 — 책상 높이 ${a.d.toFixed(3)} m`);
    await phLoadDepthCfg(); lvPoll();
  } catch (e) {
    phShow(PH.calNote, "자동 보정 실패", "bad", e.message);
  } finally { PH.bCal.disabled = false; }
}

// ---- 폰 뎁스: 설정 · 폰 프로그램 켜기·끄기 · 상태 한 줄 ----
async function phLoadDepthCfg() {
  try {
    const d = await (await fetch("/api/phone/depth/config")).json();
    d.cfg.camera_to_robot.forEach((v, i) => { PH.df["c" + i].value = v; });
    PH.df.yaw.value = d.cfg.yaw_manual_deg ?? "";
    PH.df.repo_dir.value = d.cfg.repo_dir; PH.df.run_dir.value = d.cfg.run_dir; PH.df.save_rgbd.checked = !!d.cfg.save_rgbd;
    const pf = d.cfg.pos_fix_m || [0, 0]; PH.df.fx.value = +(pf[0] * 100).toFixed(1); PH.df.fy.value = +(pf[1] * 100).toFixed(1);
    PH.df.wifi_host.value = d.cfg.wifi_host || ""; PH.df.send_fps.value = d.cfg.send_fps ?? 1; PH.df.text_rot180.checked = d.cfg.text_rot180 !== false;
    PH.cmd.textContent = `cd ${d.cfg.repo_dir} && ${d.command}`;
    PH.missing = d.missing || [];
    if (d.calib) PH.calib.textContent = `보정 파일: ${d.calib.file} — 폰 모델로 고름 (${d.calib.note})`;
  } catch (e) { /* 다음에 다시 */ }
}

async function phSaveDepthCfg() {
  const c2r = [0, 1, 2, 3, 4, 5].map((i) => parseFloat(PH.df["c" + i].value));
  const yaw = PH.df.yaw.value.trim();
  const r = await post("/api/phone/depth/config", { camera_to_robot: c2r, yaw_manual_deg: yaw === "" ? null : parseFloat(yaw), repo_dir: PH.df.repo_dir.value, run_dir: PH.df.run_dir.value, save_rgbd: PH.df.save_rgbd.checked,
    pos_fix_m: [(parseFloat(PH.df.fx.value) || 0) / 100, (parseFloat(PH.df.fy.value) || 0) / 100],
    wifi_host: PH.df.wifi_host.value.trim(), send_fps: parseFloat(PH.df.send_fps.value) || 1, text_rot180: PH.df.text_rot180.checked });
  if (r.ok) { await phLoadDepthCfg(); lvPoll(); }
}

async function phPipe(ev) {
  const on = PH.dp && PH.dp.pipeline && PH.dp.pipeline.alive;
  PH.bPipe.disabled = PH.bOn.disabled = true;
  try {
    if (on && PH.stuck && ev && ev.currentTarget === PH.bOn) {        // 멈춘 프로그램: 끄고 다시 켜기
      phShow(PH.res, "폰 카메라 다시 켜는 중… (끄고 켜기)", "run");
      await post("/api/phone/depth/stop", {}, true);
      await phSleep(1500);
      const r = await post("/api/phone/depth/start");
      if (r.ok) PH.pipeT = Date.now();
    } else {
      const r = await post(on ? "/api/phone/depth/stop" : "/api/phone/depth/start");
      if (!on && r.ok) PH.pipeT = Date.now();
    }
  } finally { PH.bPipe.disabled = PH.bOn.disabled = false; }
  phLoadDepthCfg();     // 붙은 폰이 바뀌었으면 보정 파일 줄도
  lvPoll();
}

const phSleep = (ms) => new Promise((ok) => setTimeout(ok, ms));
async function phWait(cond, ms) {
  for (const t0 = Date.now(); Date.now() - t0 < ms; await phSleep(200)) if (cond()) return true;
  return false;
}

// 집으러 가기 = 역기구학 탭에서 사람이 하는 순서 그대로: 큐브 칸 채움 → ▶ 계획 돌려 보기 → (통과하면) 재생 버튼
async function phGo() {
  const cube = PH.cube;
  if (!cube || PH.going) return;
  PH.going = true;
  try { await phPlanPlay(cube, null, PH.goNote); } finally { PH.going = false; }
}

// 역기구학 계획(pick_dry) → 이 모드의 원래 재생 버튼 (원래 확인창). target {x, y} 면 놓을 자리 칸도 채운다 (카메라 보정).
// → {ok: 재생 버튼을 눌렀나, why}. 결과 글은 note 에.
async function phPlanPlay(cube, target, note) {
  const c = cards.pick_dry;
  if (!c) { toast("역기구학 계획 항목이 없다", "error"); return { ok: false, why: "역기구학 계획 항목이 없다" }; }
  const set = (k, v) => { c.fields[k].value = v; };
  set("cube_x", cube.x.toFixed(3)); set("cube_y", cube.y.toFixed(3)); set("cube_yaw", (cube.yaw_deg || 0).toFixed(1));
  if (target) { set("target_x", target.x.toFixed(3)); set("target_y", target.y.toFixed(3)); }
  mapUpdate();
  const params = Object.fromEntries(Object.entries(c.fields).map(([k, i]) => [k, i.type === "checkbox" ? (i.checked ? "1" : "") : i.value]));
  const m0 = S.traj_mtime;
  const st0 = svcById("pick_dry").started;
  phShow(note, "계획 중…", "run");
  const r = await post("/api/svc/pick_dry/start", { params });
  if (!r.ok) { phShow(note, "계획을 못 시작함", "bad", r.error || ""); return { ok: false, why: "계획을 못 시작함 " + (r.error || "") }; }
  // 끝남 = 이번 실행(started 가 바뀜)이 꺼지고 종료코드가 나옴 (꺼진 직후 한 틱은 종료코드가 아직 없다)
  const done = await phWait(() => { const v = svcById("pick_dry"); return v.started !== st0 && !v.alive && v.exit_code != null; }, 180000);
  const sv = svcById("pick_dry");
  if (!done) { phShow(note, "계획이 3분 넘게 안 끝남", "bad", "역기구학 탭에서 확인"); return { ok: false, why: "계획이 3분 넘게 안 끝남" }; }
  const [txt, cls] = pickSummary(sv);   // 역기구학 탭의 결과 한 줄 그대로
  // 궤적이 나왔으면(종료코드 1 = 검사 거절이어도 역기구학 탭처럼) 재생 버튼으로 — 문제는 그 버튼의 검사·확인창이 다시 묻는다
  const got = sv.exit_code !== 3 && await phWait(() => S.traj_mtime != null && S.traj_mtime !== m0 && T.mtime === S.traj_mtime && T.info && T.info.mtime === S.traj_mtime, 4000);
  if (!got || !T.traj) { phShow(note, "계획 실패 — 팔은 안 움직임", "bad", txt || `종료코드 ${sv.exit_code}`); return { ok: false, why: "계획 실패: " + (txt || `종료코드 ${sv.exit_code}`) }; }
  let pass = txt || "계획 나옴";
  const gst = (T.traj && T.traj.grasp) || {};
  const hrI = (T.traj.joint_names || []).indexOf("hand_roll");
  const roll = hrI >= 0 ? Math.max(...T.traj.positions.map((q) => Math.abs(q[hrI] * 180 / Math.PI))) : 0;
  if (gst.style === "tilted") {    // 2026-11-25 '너무 돌아서 못 잡음': 수직으로 안 닿는 자리(왼쪽·먼 곳)는 기울여 잡아 손을 크게 돌린다
    pass += ` · ⚠ 기울여 잡기 (손 돌림 ${Math.round(roll)}°) — 잘 안 잡힐 수 있다: 큐브를 로봇 앞 조금 오른쪽 (x −0.05, y −0.22~−0.26) 으로 옮기면 수직으로 잡는다`;
  }
  await phSleep(600);   // 재생 버튼이 새 궤적으로 켜질 때까지 (다음 화면 갱신)
  const kind = S.mode && S.mode.kind;
  const btn = kind === "virtual" ? T.vplay : kind === "direct" ? (typeof D !== "undefined" && D.play && D.play.go) : null;
  if (!btn) { phShow(note, pass, cls, "이 모드의 재생은 역기구학 탭에서"); return { ok: false, why: "이 모드의 재생은 역기구학 탭에서" }; }
  if (btn.disabled) { const w = btn.title || (kind === "direct" ? "연결 탭에서 팔 연결·원점 먼저" : "이미 재생 중"); phShow(note, pass, "warn", "재생은 아직 못 함: " + w); return { ok: false, why: "재생 못 함: " + w }; }
  phShow(note, pass, gst.style === "tilted" ? "warn" : cls);
  btn.click();   // 원래 버튼 = 원래 확인창 (가상: 1번, 실물: 2번 + 검사 문제면 한 번 더)
  return { ok: true };
}

// ---- ③ 카메라 보정 (역기구학으로, camcal.py): 팔이 큐브를 정해진 자리에 놓고(정답) → HOME 으로 비켜 → 카메라가 읽음 → 회전 + 이동 맞춤 ----
const phPb = () => { const k = S.mode && S.mode.kind; return ((k === "virtual" ? S.ros.virtual_state : S.ros.direct_state) || {}).playback || {}; };
function ccShow(t, cls, small) { phShow(PH.ccNote, t, cls, small); }
async function ccFixText() {
  try {
    const st = await (await fetch("/api/camcal")).json();
    const f = st.fix;
    PH.ccFix.textContent = f ? `지금 카메라 보정: 회전 ${f.theta_deg >= 0 ? "+" : ""}${f.theta_deg.toFixed(2)}° · 자리 ${f.n}곳으로 맞춤 · 오차 ${f.rms_raw_cm} → ${f.rms_fit_cm} cm (${f.t_saved || ""})`
      : "지금 카메라 보정: 없음 (카메라 값 그대로 + 휴대폰 탭 자리 고침 pos_fix 만)";
    PH.ccClear.disabled = !f;
    return st;
  } catch (e) { PH.ccFix.textContent = "카메라 보정 상태를 못 읽음"; return null; }
}
function ccTable() {
  PH.ccTable.replaceChildren(...PH.cc.rows.map((r, i) => el("div", { class: "help", style: "margin:2px 0" },
    `${i + 1}. 놓은 자리 (${r.t.x.toFixed(3)}, ${r.t.y.toFixed(3)}) · 카메라 (${r.rd.fixed[0].toFixed(3)}, ${r.rd.fixed[1].toFixed(3)}) · 차이 ${r.rd.err_cm.toFixed(1)} cm`)));
}
async function ccHome() {
  const real = !VIRT();
  if (!confirm((real ? "⚠ 실물 팔이 움직인다.\n\n" : "(가상) ") + "카메라 보정: 큐브를 놓았다. 카메라가 큐브를 보게 팔을 HOME(모든 관절 0)으로 비킬까요?\n\n멈춤 = ■ STOP / Esc")) return { ok: false, why: "HOME 취소" };
  let r = await post("/api/arm/go_home", { force: false, d: tableD() }, true);
  if (!r.ok && r.need_force) {
    if (!confirm("검사에서 문제가 나왔다:\n\n- " + r.problems.join("\n- ") + "\n\n그래도 HOME 으로 갈까요?")) return { ok: false, why: "HOME 취소" };
    r = await post("/api/arm/go_home", { force: true, d: tableD() }, true);
  }
  return r.ok ? { ok: true } : { ok: false, why: r.error || "HOME 실패" };
}
async function ccRun() {
  if (PH.cc.busy) return;
  const kind = S.mode && S.mode.kind;
  if (kind !== "direct" && kind !== "virtual") { ccShow("실물(직접 CAN)·가상 모드에서만", "bad"); return; }
  if (!PH.cube || PH.block) { ccShow("카메라가 집을 큐브를 못 찾음", "bad", PH.block || "큐브 하나를 카메라에 보이게 책상에 두기 (첫 번째만 카메라 자리로 집는다)"); return; }
  if (phPb().running) { ccShow("다른 재생이 끝난 뒤에", "bad"); return; }
  PH.cc.busy = true; PH.cc.stop = false; PH.cc.rows = []; ccTable();
  try {
    const st = await ccFixText();
    let cube = { x: PH.cube.x, y: PH.cube.y, yaw_deg: PH.cube.yaw_deg };    // 첫 번째만 카메라 자리
    const targets = [];
    ccShow("놓을 자리가 집기·놓기 되는지 확인 중…", "run");
    for (const [x, y] of (st && st.targets) || []) {
      if (Math.hypot(x - cube.x, y - cube.y) < 0.05) continue;            // 지금 큐브 자리와 겹치는 곳은 뺌
      try {
        const g = await (await fetch("/api/ik/graspable", { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ x, y, d: tableD() }) })).json();
        if (g.ok && g.graspable) targets.push({ x, y });
      } catch (e) { /* 그 자리는 뺌 */ }
    }
    if (targets.length < 3) { ccShow(`집기·놓기 되는 자리가 ${targets.length}곳뿐 — 3곳 이상 필요`, "bad", "책상 높이 d 칸·큐브 자리를 확인"); return; }
    if (!confirm(`카메라 보정 (역기구학으로)\n\n팔이 지금 큐브를 집어 ${targets.length}곳에 차례로 놓는다. 놓을 때마다 HOME 으로 비키고 카메라가 읽는다.\n` +
      "자리마다 '재생' 확인창과 'HOME' 확인창이 뜬다. 중간에 그만두려면 확인창에서 취소하거나 '그만' (팔은 ■ STOP / Esc).\n\n시작할까요?")) { ccShow("취소함", ""); return; }
    await post("/api/camcal/reset", {}, true);
    for (let i = 0; i < targets.length && !PH.cc.stop; i++) {
      const t = targets[i], tag = `${i + 1}/${targets.length} (${t.x.toFixed(2)}, ${t.y.toFixed(2)})`;
      const r = await phPlanPlay(cube, t, PH.ccNote);
      if (!r.ok) { ccShow(`${tag}: 멈춤`, "bad", r.why); break; }
      if (!await phWait(() => phPb().running, 30000)) { ccShow(`${tag}: 재생이 시작되지 않음 (확인창 취소?) — 멈춤`, "warn"); break; }
      ccShow(`${tag}: 놓는 중…`, "run");
      if (!await phWait(() => !phPb().running, 300000)) { ccShow(`${tag}: 재생이 5분 넘게 안 끝남 — 멈춤`, "bad"); break; }
      if (phPb().reason !== "끝까지 재생") { ccShow(`${tag}: 재생이 중간에 멈춤 — 보정 멈춤`, "bad", phPb().reason || ""); break; }
      const h = await ccHome();
      if (!h.ok) { ccShow(`${tag}: ${h.why} — 멈춤`, "warn"); break; }
      ccShow(`${tag}: HOME 으로 비키는 중…`, "run");
      if (await phWait(() => phPb().running, 8000)) await phWait(() => !phPb().running, 180000);
      await phSleep(1500);   // 손가락 회전 원점 명령까지 (서버가 HOME 도착 뒤 보냄)
      ccShow(`${tag}: 카메라로 읽는 중… (팔이 비킨 뒤 새 프레임)`, "run");
      const rd = await post("/api/camcal/read", { target: [t.x, t.y] }, true);
      if (!rd.ok) { ccShow(`${tag}: 카메라로 못 읽음 — 멈춤`, "bad", rd.error || ""); break; }
      PH.cc.rows.push({ t, rd }); ccTable();
      cube = { x: t.x, y: t.y, yaw_deg: rd.yaw_deg != null ? rd.yaw_deg : cube.yaw_deg };   // 다음은 팔이 놓은 자리에서 집는다 (카메라 오차와 상관없이)
    }
  } finally { PH.cc.busy = false; }
  if (PH.cc.rows.length) await ccFitShow();
}
async function ccFitShow() {
  const r = await post("/api/camcal/fit", { apply: false }, true);
  if (!r.ok) { ccShow("맞추기 실패", "bad", r.error || ""); PH.ccApply.disabled = true; return; }
  const f = r.fit, ok = !r.problems.length && f.n >= 3;
  ccShow(`${f.n}곳으로 맞춤: 회전 ${f.theta_deg >= 0 ? "+" : ""}${f.theta_deg.toFixed(2)}° · 이동 ${f.shift_cm.toFixed(1)} cm · 오차 ${f.rms_raw_cm} → ${f.rms_fit_cm} cm`,
    ok ? "ok" : "warn", r.problems.length ? r.problems.join(" · ") : f.n < 3 ? "3곳 이상이어야 저장 (회전까지 믿으려면)" : "'적용' 을 누르면 저장 — 그다음부터 휴대폰 탭·강화학습 카메라 큐브 자리에 들어간다");
  PH.ccApply.disabled = !ok;
}
async function ccApply() {
  if (!confirm("이 카메라 보정을 저장할까요? (그다음부터 카메라 큐브 자리에 들어간다 · '보정 지우기' 로 되돌림)")) return;
  const r = await post("/api/camcal/fit", { apply: true }, true);
  if (r.ok && r.saved) { ccShow(r.msg, "ok"); PH.ccApply.disabled = true; } else ccShow("저장 안 함", "bad", r.error || "");
  ccFixText();
}
async function ccClear() {
  if (!confirm("저장한 카메라 보정을 지울까요? (카메라 값 그대로로 돌아간다)")) return;
  const r = await post("/api/camcal/reset", { clear_fix: true }, true);
  ccShow(r.msg || "지움", r.ok ? "" : "bad");
  ccFixText();
}

// ---- B. 강화학습으로 집기: /api/rl/live_cube → /api/rl/run → (실물) 시작 자세 확인 → /api/rl/play_real ----
async function phRL() {
  if (PH.rlBusy) return;
  PH.rlBusy = true;
  const note = (t, cls, small) => phShow(PH.rlNote, t, cls, small);
  try {
    note("카메라에서 큐브 읽는 중…", "run");
    const c = await post("/api/rl/live_cube", {}, true);
    if (!c.ok) { note("카메라 큐브를 못 씀", "bad", (c.error || "") + ((c.warn || []).length ? " · " + c.warn.join(" · ") : "")); return; }
    note(`큐브 위치 (${c.spot.x_cm}, ${c.spot.y_cm}) cm · ${c.spot.yaw_deg}° ${c.insert_down_cm != null ? ` · 손 ${c.insert_down_cm} cm 내림` + (c.marker_drop_cm != null ? ` (마커 높이 ${c.marker_drop_cm} cm 아래)` : "") : ""} — 강화학습 시뮬 돌리는 중…`, "run", (c.text || "") + ((c.warn || []).length ? " · ⚠ " + c.warn.join(" · ") : ""));
    const r = await post("/api/rl/run", { x_cm: c.spot.x_cm, y_cm: c.spot.y_cm, yaw_deg: c.spot.yaw_deg, home: false, ...(c.insert_down_cm != null ? { insert_down_cm: c.insert_down_cm } : {}) }, true);
    if (!r.ok) { note("강화학습 시뮬 실패", "bad", r.error || ""); return; }
    const sm = r.summary || {};
    if (!sm.done) { note("시뮬에서 못 집음 — 실물로 안 보냄", "bad", (sm.fail_reason || "") + (sm.valid_spot === false ? " · 정책이 학습한 범위 밖 — 큐브를 옮기기" : "")); return; }
    if (VIRT()) { note("시뮬에서 집기 성공 (가상 모드 — 실물 재생 없음)", "ok", "강화학습 탭 3D 에서 재생해 볼 수 있다"); return; }
    let st = await (await fetch("/api/rl/real_state?run_id=" + encodeURIComponent(r.run_id))).json();
    if (!st.connected) { note("시뮬 성공 — 실물 팔 연결 안 됨", "warn", st.why || ""); return; }
    // 시작 자세: 서보만 다르면 (강화학습은 손가락 회전 180° 로 시작) 보내 준다. AK45 가 다르면 HOME 먼저
    const AK = new Set(["shoulder_roll", "shoulder_yaw", "upper_arm_roll", "elbow"]);
    const farNames = (st.far || []).map((t) => t.split(" ")[0]);
    if (farNames.some((n) => AK.has(n))) { note("시뮬 성공 — 팔이 시작 자세(HOME)가 아님", "warn", "각도 제어 탭에서 'HOME 으로 가기' 뒤 다시: " + st.far.join(", ")); return; }
    if (farNames.length || (st.unsent_servos || []).length) {
      const joints = {};
      r.joint_names.forEach((n, i) => { if (!AK.has(n)) joints[n === "rl_rotation" ? "finger_rotation" : n] = Math.round((r.positions[0][i] * 180 / Math.PI + (n === "rl_rotation" ? 180 : 0)) * 10) / 10; });
      if (!confirm("⚠ 실물 서보가 움직인다.\n\n강화학습 시작 자세로 서보를 보낸다 (손가락 회전 180° 등):\n" + Object.entries(joints).map(([k, v]) => `${k} ${v}°`).join(", "))) { note("취소함", ""); return; }
      const s1 = await post("/api/joint/send", { joints, instant: true }, true);
      if (!s1.ok) { note("서보를 시작 자세로 못 보냄", "bad", s1.error || ""); return; }
      await new Promise((res) => setTimeout(res, 1500));
      st = await (await fetch("/api/rl/real_state?run_id=" + encodeURIComponent(r.run_id))).json();
    }
    if (!st.can_play) {
      const why = [...(st.not_homed || []).map((n) => n + " 원점 없음"), ...(st.far || []), ...(st.clip_big || []), st.calib_dirty ? "보정 저장 안 함" : "", (st.playback || {}).running ? "다른 재생 중" : ""].filter(Boolean);
      note("시뮬 성공 — 아직 실물로 못 보냄", "warn", why.join(" · ") || st.why || ""); return;
    }
    if (!confirm(`⚠ 실물 팔로 강화학습 궤적을 재생한다 (큐브 ${c.spot.x_cm}, ${c.spot.y_cm} cm · ${c.spot.yaw_deg}°).\n\n` +
      "책상 앞 끝이 어깨에서 22.5 cm, 큐브 윗면 높이가 정책 장면(어깨 아래 47.75 cm 책상)과 맞는가? (실물 책상이면 약 8 cm 받침대)\n열린 고리 — 실물에서 큐브가 달라도 정책이 다시 고르지 않는다. 팔 주변을 비웠나요?")) { note("취소함", ""); return; }
    let pr = await post("/api/rl/play_real", { run_id: r.run_id, ack_table: true, max_speed: 30 }, true);
    if (!pr.ok && pr.need_force) {
      if (!confirm("검사에서 문제가 나왔다:\n\n- " + pr.problems.join("\n- ") + "\n\n그래도 재생할까요?")) { note("취소함", ""); return; }
      pr = await post("/api/rl/play_real", { run_id: r.run_id, ack_table: true, max_speed: 30, force: true }, true);
    }
    note(pr.ok ? "강화학습으로 집는 중 (실물 팔)" : "재생 거절", pr.ok ? "ok" : "bad", pr.ok ? (pr.msg || "") + " · 끝나면 각도 제어 탭 'HOME 으로 가기'" : (pr.error || ""));
  } catch (e) {
    note("강화학습 집기 실패", "bad", e.message);
  } finally { PH.rlBusy = false; }
}

function phTick() {
  if (!S || curTab !== "phone") return;
  const kind = S.mode && S.mode.kind;
  const pb = (kind === "virtual" ? (S.ros.virtual_state || {}) : (S.ros.direct_state || {})).playback || {};
  PH.bGo.disabled = !PH.cube || PH.going || !!pb.running || !PH.grasp || (PH.grasp.ok && !PH.grasp.graspable) || !!PH.block;   // 못 집는 자리로 확인되면 막음 (확인 못 했으면 계획이 판정)
  PH.bGo.textContent = pb.running ? `재생 중 ${pb.frame}/${pb.frames - 1}` : PH.going ? "계획 중…" : PH.block && PH.cube ? PH.block : `역기구학으로 집으러 가기 (${kind === "virtual" ? "가상 팔" : "실물 팔"})`;
  PH.bRL.disabled = !!PH.rlBusy || !!pb.running || !!PH.going || PH.cc.busy;
  if (PH.cc.busy) PH.bGo.disabled = true;     // 카메라 보정이 같은 계획 칸·재생 버튼을 쓰는 동안
  PH.bRL.textContent = PH.rlBusy ? "강화학습 진행 중…" : pb.running ? `재생 중 ${pb.frame}/${pb.frames - 1}` : `강화학습으로 집으러 가기 (${kind === "virtual" ? "가상: 시뮬만" : "실물 팔"})`;
  PH.bStop.disabled = !pb.running;
  PH.ccStart.disabled = PH.cc.busy || !!pb.running || !!PH.going || !!PH.rlBusy;
  PH.ccStart.textContent = PH.cc.busy ? `보정 중… (${PH.cc.rows.length}곳 읽음)` : "보정 시작 (팔이 큐브를 몇 곳에 놓기)";
  PH.ccStopB.disabled = !PH.cc.busy;
  const v = mapVals();
  PH.target.textContent = (v.target ? `놓을 자리: x ${v.target.x.toFixed(3)}, y ${v.target.y.toFixed(3)} m (역기구학 탭에서 바꾼다)` : "놓을 자리: 역기구학 탭에서 정한다") +
    ` · 책상 높이 ${tableDNum().toFixed(3)} m`;
}
