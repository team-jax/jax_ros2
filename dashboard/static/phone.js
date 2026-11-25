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
      PH.help = el("div", { class: "help", style: "margin:10px 0 0" }, "로봇에 거치한 폰의 영상에서 새 프레임마다 찾는다. 큐브 윗면 마커가 보이면 ArUco 자리를 쓰고 뎁스로 맞춰 본다 (기준 종이는 없어도 된다 — 없으면 거치 자세와 책상 높이로). 마커가 없으면 뎁스만 쓴다. 지도에서 큐브를 누르면 그 큐브를 집는다.")),
    el("h2", { class: "sec" }, el("span", { class: "snum" }, "2"), "집으러 가기"),
    el("div", { class: "card" },
      PH.bGo,
      el("div", { class: "ph-row" }, PH.bStop, el("a", { class: "btn line", href: "#ik" }, "역기구학 탭에서 보기")),
      PH.goNote, PH.target,
      el("div", { class: "help", style: "margin:6px 0 0" }, "계획(역기구학 탭과 같은 것)을 돌린 뒤, 지금 모드의 팔로 재생할지 확인창이 묻는다.")),
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
    : "로봇에 거치한 폰의 영상에서 새 프레임마다 찾는다. 큐브 윗면 마커가 보이면 ArUco 자리를 쓰고 뎁스로 맞춰 본다 (기준 종이는 없어도 된다 — 없으면 거치 자세와 책상 높이로). 마커가 없으면 뎁스만 쓴다. 지도에서 큐브를 누르면 그 큐브를 집는다.";
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
  PH.bOn.hidden = !!pl.alive || fresh;
  PH.pipeNote.textContent = pl.alive ? `폰 프로그램 켜짐 (pid ${pl.pid})` + (fresh ? "" : " — 아직 결과 없음: 폰 잠금을 풀고 몇 초 가만히. " + (pl.log || []).slice(-1).join(""))
    : fresh ? "결과가 들어오는 중 (다른 곳에서 켠 프로그램)" : "폰 프로그램 꺼짐" + ((PH.missing || []).length ? " — 이 PC 에 없는 것: " + PH.missing.join(" / ") : "") + ((pl.log || []).length > 1 ? " · 마지막 기록: " + pl.log.slice(-1)[0] : "");
  PH.dStat.hidden = fresh;
  PH.dStat.textContent = pl.alive ? "폰 프로그램 켜는 중 — 폰 잠금을 풀고 몇 초 가만히 (끄기는 아래 준비)" : PH.pipeNote.textContent;
  // 영상: MJPEG 스트림 하나를 열어 둔다 (새 프레임마다 바로 보임, 최대 15 Hz). 끊기면(영상 없음) 닫았다가 다시 열기
  const src = !fresh || all.frame_n == null ? "" : "/api/live/stream.mjpg";
  PH.img.hidden = !src;
  if (PH.imgSrc !== src) { PH.imgSrc = src; streamSet(PH.img, !!src); }
  // 뎁스·세그 결과 (폰 프로그램의 inference.png: 객체 세그 + 뎁스 색) — 원본 영상 아래 (2026-11-24 사용자)
  if (!PH.inf) {
    PH.inf = el("img", { class: "ph-img", alt: "뎁스·세그", title: "폰 프로그램의 뎁스·세그 결과 (1초쯤마다)" });
    // 폰이 거치대에 거꾸로 끼워져 위아래가 뒤집혀 보임 → 기본으로 칸마다 180° 돌려 보인다 (화면만; 인식은 원본). 브라우저마다 기억
    let flip = true;
    try { flip = localStorage.getItem("ph-inf-flip2") !== "0"; } catch (e) { /* 저장소 막힘 → 기본 */ }
    PH.infFlip = el("input", { type: "checkbox" });
    PH.infFlip.checked = flip;
    PH.infFlip.onchange = () => { try { localStorage.setItem("ph-inf-flip2", PH.infFlip.checked ? "1" : "0"); } catch (e) { /* 무시 */ } PH.infT = 0; };
    PH.infFlipRow = el("label", { class: "help", style: "display:flex;gap:6px;align-items:center;margin-top:6px" }, PH.infFlip, "뎁스·세그 그림 위아래 뒤집어 보기 (폰이 거꾸로 끼워짐 — 폰 프로그램이 글자를 미리 거꾸로 써서 뒤집으면 바로 읽힘)");
    PH.img.after(PH.inf, PH.infFlipRow);
  }
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
  if (p) det.push(`뎁스: ${p.age_s.toFixed(0)}초 전 · 크기 ${p.dims_mm[0].toFixed(0)}×${p.dims_mm[1].toFixed(0)} mm · 책상 위 ${p.height_cm.toFixed(1)} cm · 각 ${PH_YAW_FROM[p.yaw_from] || "모름"}` +
    ` · 카메라 좌표 ${p.cam.map((v) => v.toFixed(4)).join(", ")}`);
  if (tail) det.push(tail);
  PH.last = r; PH.block = r.blocked ? r.block : "";
  phShow(PH.res, el("span", {}, el("span", { class: "ph-xy" }, `큐브  x ${r.cube.x.toFixed(3)}  y ${r.cube.y.toFixed(3)} m  yaw ${r.cube.yaw_deg.toFixed(0)}°`), el("b", { class: "ph-src" }, r.text)),
    r.blocked ? "bad" : (r.warn || []).length ? "warn" : "", det.join(" · "));
  // 책상 높이·거치 자세가 안 맞아 막혔으면 그 자리에서 자동 보정 (2026-11-25: 설정 카드 안에 묻혀 있어 못 찾았다)
  const misfit = r.blocked && ((p && (p.height_cm < -2 || p.height_cm > 8)) || /안 맞음/.test(r.text || ""));
  if (misfit) {
    if (!PH.resCal) PH.resCal = el("button", { class: "btn small", style: "margin-top:8px", onclick: phAutocal }, "자동 보정 — 책상 높이·카메라 기울기 재서 맞추기 (자 필요 없음)");
    PH.res.append(el("div", {}, PH.resCal));
  }
  const c = PH.cube;   // 거의 같은 자리면 그대로 둔다 (1초마다 집을 수 있는지 다시 묻지 않게)
  if (!c || Math.hypot(c.x - r.cube.x, c.y - r.cube.y) > 0.003 || Math.abs(c.yaw_deg - r.cube.yaw_deg) > 2 || PH.d !== d) { PH.d = d; phGrasp(r.cube, " — 큐브를 옮기기"); }
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

async function phPipe() {
  const on = PH.dp && PH.dp.pipeline && PH.dp.pipeline.alive;
  PH.bPipe.disabled = true;
  await post(on ? "/api/phone/depth/stop" : "/api/phone/depth/start");
  PH.bPipe.disabled = false;
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
  const c = cards.pick_dry, cube = PH.cube;
  if (!cube || PH.going) return;
  if (!c) { toast("역기구학 계획 항목이 없다", "error"); return; }
  PH.going = true;
  try {
    const set = (k, v) => { c.fields[k].value = v; };
    set("cube_x", cube.x.toFixed(3)); set("cube_y", cube.y.toFixed(3)); set("cube_yaw", cube.yaw_deg.toFixed(1));
    mapUpdate();
    const params = Object.fromEntries(Object.entries(c.fields).map(([k, i]) => [k, i.type === "checkbox" ? (i.checked ? "1" : "") : i.value]));
    const m0 = S.traj_mtime;
    const st0 = svcById("pick_dry").started;
    phShow(PH.goNote, "계획 중…", "run");
    const r = await post("/api/svc/pick_dry/start", { params });
    if (!r.ok) { phShow(PH.goNote, "계획을 못 시작함", "bad", r.error || ""); return; }
    // 끝남 = 이번 실행(started 가 바뀜)이 꺼지고 종료코드가 나옴 (꺼진 직후 한 틱은 종료코드가 아직 없다)
    const done = await phWait(() => { const v = svcById("pick_dry"); return v.started !== st0 && !v.alive && v.exit_code != null; }, 180000);
    const sv = svcById("pick_dry");
    if (!done) { phShow(PH.goNote, "계획이 3분 넘게 안 끝남", "bad", "역기구학 탭에서 확인"); return; }
    const [txt, cls] = pickSummary(sv);   // 역기구학 탭의 결과 한 줄 그대로
    // 궤적이 나왔으면(종료코드 1 = 검사 거절이어도 역기구학 탭처럼) 재생 버튼으로 — 문제는 그 버튼의 검사·확인창이 다시 묻는다
    const got = sv.exit_code !== 3 && await phWait(() => S.traj_mtime != null && S.traj_mtime !== m0 && T.mtime === S.traj_mtime && T.info && T.info.mtime === S.traj_mtime, 4000);
    if (!got || !T.traj) { phShow(PH.goNote, "계획 실패 — 팔은 안 움직임", "bad", txt || `종료코드 ${sv.exit_code}`); return; }
    const pass = txt || "계획 나옴";
    await phSleep(600);   // 재생 버튼이 새 궤적으로 켜질 때까지 (다음 화면 갱신)
    const kind = S.mode && S.mode.kind;
    const btn = kind === "virtual" ? T.vplay : kind === "direct" ? (typeof D !== "undefined" && D.play && D.play.go) : null;
    if (!btn) { phShow(PH.goNote, pass, cls, "이 모드의 재생은 역기구학 탭에서"); return; }
    if (btn.disabled) { phShow(PH.goNote, pass, "warn", "재생은 아직 못 함: " + (btn.title || (kind === "direct" ? "연결 탭에서 팔 연결·원점 먼저" : "이미 재생 중"))); return; }
    phShow(PH.goNote, pass, cls);
    btn.click();   // 원래 버튼 = 원래 확인창 (가상: 1번, 실물: 2번 + 검사 문제면 한 번 더)
  } finally { PH.going = false; }
}

function phTick() {
  if (!S || curTab !== "phone") return;
  const kind = S.mode && S.mode.kind;
  const pb = (kind === "virtual" ? (S.ros.virtual_state || {}) : (S.ros.direct_state || {})).playback || {};
  PH.bGo.disabled = !PH.cube || PH.going || !!pb.running || !PH.grasp || (PH.grasp.ok && !PH.grasp.graspable) || !!PH.block;   // 못 집는 자리로 확인되면 막음 (확인 못 했으면 계획이 판정)
  PH.bGo.textContent = pb.running ? `재생 중 ${pb.frame}/${pb.frames - 1}` : PH.going ? "계획 중…" : PH.block && PH.cube ? PH.block : `집으러 가기 (${kind === "virtual" ? "가상 팔" : "실물 팔"})`;
  PH.bStop.disabled = !pb.running;
  const v = mapVals();
  PH.target.textContent = (v.target ? `놓을 자리: x ${v.target.x.toFixed(3)}, y ${v.target.y.toFixed(3)} m (역기구학 탭에서 바꾼다)` : "놓을 자리: 역기구학 탭에서 정한다") +
    ` · 책상 높이 ${tableDNum().toFixed(3)} m`;
}
