// 팔 3D 뷰 (three.js r160, static/vendor 에 저장 — 오프라인에서도 됨).
// 모델: static/robot/arm.xml (team-jax/IK motion2/arm, MJCF). 관절 이름 = calibration.json 의 sim 관절명, 각은 rad.
// 좌표: 로봇 기준(vision_node·pick_place 와 같음) — 원점 = shoulder_roll 관절 원점, x 로봇 왼쪽+, y 뒤+(앞 = -y), z 위+.
//   이 틀은 arm.xml base_link(Shoulder_Pitch) 틀을 x 로 +0.052 m 옮긴 것과 같다(IK realcoords.py + env.py robot_yaw -90° 에서 계산,
//   pick_place 궤적의 손끝이 큐브 2 cm 안을 지나는 것으로 확인).
// 메시: 서버 /robot/meshes/<obj> (JAX_IK_DIR 등). 없으면 관절 원점을 잇는 막대 + 관절 구.
import * as THREE from "three";
import { OrbitControls } from "three/addons/OrbitControls.js";
import { OBJLoader } from "three/addons/OBJLoader.js";

const REAL_FROM_BASE = new THREE.Vector3(0.052, 0, 0);
const CUBE = 0.05;
let xmlP = null;
const meshCache = new Map();

function loadXml() {
  if (!xmlP) xmlP = fetch("/static/robot/arm.xml").then((r) => { if (!r.ok) throw new Error("arm.xml " + r.status); return r.text(); })
    .then((t) => new DOMParser().parseFromString(t, "application/xml"));
  return xmlP;
}
function loadObj(file) {
  if (!meshCache.has(file)) {
    meshCache.set(file, new OBJLoader().loadAsync("/robot/meshes/" + encodeURIComponent(file)).then((g) => {
      const geos = [];
      g.traverse((o) => { if (o.isMesh) { if (!o.geometry.attributes.normal) o.geometry.computeVertexNormals(); geos.push(o.geometry); } });
      return geos;
    }).catch(() => null));
  }
  return meshCache.get(file);
}
const nums = (s, d) => (s == null ? d : s.trim().split(/\s+/).map(Number));
function placeFrom(el, obj) {
  const p = nums(el.getAttribute("pos"), [0, 0, 0]);
  const q = nums(el.getAttribute("quat"), [1, 0, 0, 0]);   // MJCF: w x y z
  obj.position.set(p[0], p[1], p[2]);
  obj.quaternion.set(q[1], q[2], q[3], q[0]).normalize();
}
const kids = (el, tag) => Array.from(el.children).filter((c) => c.tagName === tag);

// 로봇 한 벌. ghost = 반투명 파랑(슬라이더 목표).
class ArmModel {
  constructor(xml, { meshes, ghost }) {
    this.group = new THREE.Group();
    this.joints = new Map();
    this.ghost = !!ghost;
    this.meshes = meshes;
    this.pending = [];
    const assets = new Map();
    for (const m of xml.querySelectorAll("asset > mesh")) assets.set(m.getAttribute("name"), m.getAttribute("file"));
    const column = Array.from(xml.querySelectorAll("worldbody > body")).find((b) => b.getAttribute("name") === "column");
    const base = column && kids(column, "body").find((b) => b.getAttribute("name") === "base_link");
    if (!column || !base) throw new Error("arm.xml: column/base_link 없음");
    // 로봇 틀(real) = base_link 틀 + 0.052 x. _body(column) 이 column 자신의 pos 도 넣으므로 (column · base_link) 의 역만큼 옮긴다.
    const tc = new THREE.Object3D(); placeFrom(column, tc); tc.updateMatrix();
    const tb = new THREE.Object3D(); placeFrom(base, tb); tb.updateMatrix();
    const wb = tc.matrix.clone().multiply(tb.matrix);
    const m = new THREE.Matrix4().makeTranslation(REAL_FROM_BASE.x, REAL_FROM_BASE.y, REAL_FROM_BASE.z).multiply(wb.invert());
    m.decompose(this.group.position, this.group.quaternion, this.group.scale);
    const glass = (color, opacity) => new THREE.MeshStandardMaterial({ color, transparent: true, opacity, depthWrite: false });
    this.matStick = this.ghost ? glass(0x3b7bf0, 0.35) : new THREE.MeshStandardMaterial({ color: 0x4b5563, metalness: 0.2, roughness: 0.6 });
    this.matJoint = this.ghost ? glass(0x3b7bf0, 0.45) : new THREE.MeshStandardMaterial({ color: 0xf59e0b, metalness: 0.1, roughness: 0.5 });
    this.ghostMat = glass(0x3b7bf0, 0.28);
    this._body(column, this.group, assets, true);
  }
  _material(rgba) {
    if (this.ghost) return this.ghostMat;
    const c = nums(rgba, [0.6, 0.6, 0.6, 1]);
    // 짙은 회색(0.098) 부품은 밝은 배경에서 잘 보이게 조금 밝힌다(색만, 모양 그대로)
    const dark = Math.max(c[0], c[1], c[2]) < 0.2;
    const col = dark ? new THREE.Color(0.26, 0.28, 0.32) : new THREE.Color(c[0], c[1], c[2]);
    return new THREE.MeshStandardMaterial({ color: col, metalness: 0.15, roughness: 0.65 });
  }
  _body(el, parent, assets, isColumn) {
    const g = new THREE.Group();
    placeFrom(el, g);
    parent.add(g);
    let content = g;
    // 손가락 3 의 RotationBody: arm.xml 은 −π 로 굳혀 둠. 강화학습(/rl)은 이 관절을 돌린다 → setPose 의 'rl_rotation'(rad)으로 q0 · Rot_z(θ+π)
    if (el.getAttribute("name") === "RotationBody") this.rotBody = { obj: g, q0: g.quaternion.clone() };
    const j = kids(el, "joint").find((x) => (x.getAttribute("type") || "hinge") === "hinge");
    if (j) {
      const jg = new THREE.Group();
      g.add(jg);
      this.joints.set(j.getAttribute("name"), { obj: jg, axis: new THREE.Vector3(...nums(j.getAttribute("axis"), [0, 0, 1])).normalize() });
      content = jg;
    }
    if (this.meshes) {
      for (const geom of kids(el, "geom")) {
        if (geom.getAttribute("type") !== "mesh" || geom.getAttribute("group") === "3") continue;
        const file = assets.get(geom.getAttribute("mesh"));
        if (!file) continue;
        const holder = new THREE.Group();
        placeFrom(geom, holder);
        content.add(holder);
        const mat = this._material(geom.getAttribute("rgba"));
        this.pending.push(loadObj(file).then((geos) => {
          if (!geos) return false;
          for (const ge of geos) { const mm = new THREE.Mesh(ge, mat); mm.renderOrder = this.ghost ? 2 : 0; holder.add(mm); }
          return true;
        }));
      }
    } else {
      // 막대 모델: 관절 구 + 자식 몸체 원점까지 막대
      if (j) content.add(new THREE.Mesh(new THREE.SphereGeometry(0.014, 16, 12), this.matJoint));
      for (const c of kids(el, "body")) {
        const p = new THREE.Vector3(...nums(c.getAttribute("pos"), [0, 0, 0]));
        const len = p.length();
        if (len < 1e-4) continue;
        const r = isColumn ? 0.02 : 0.008;
        const cyl = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 10), this.matStick);
        cyl.position.copy(p).multiplyScalar(0.5);
        cyl.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), p.clone().normalize());
        content.add(cyl);
      }
    }
    for (const s of kids(el, "site")) {
      if (s.getAttribute("name") !== "end_effector") continue;
      const sp = new THREE.Mesh(new THREE.SphereGeometry(0.008, 12, 8),
        new THREE.MeshBasicMaterial({ color: this.ghost ? 0x3b7bf0 : 0x16a34a, transparent: this.ghost, opacity: this.ghost ? 0.5 : 1 }));
      placeFrom(s, sp);
      content.add(sp);
    }
    for (const c of kids(el, "body")) this._body(c, content, assets, false);
  }
  setPose(q) {
    for (const [name, j] of this.joints) {
      const v = q && Number.isFinite(q[name]) ? q[name] : 0;
      j.obj.quaternion.setFromAxisAngle(j.axis, v);
    }
    if (this.rotBody) {   // 키가 없으면(다른 탭) 굳은 자세 그대로
      const r = q && Number.isFinite(q.rl_rotation) ? q.rl_rotation : null;
      this.rotBody.obj.quaternion.copy(this.rotBody.q0);
      if (r !== null) this.rotBody.obj.quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), r + Math.PI));
    }
  }
}

function textSprite(text, color) {
  const c = document.createElement("canvas");
  c.width = 256; c.height = 64;
  const x = c.getContext("2d");
  x.font = "bold 34px sans-serif"; x.fillStyle = color; x.textBaseline = "middle"; x.fillText(text, 6, 32);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), transparent: true }));
  s.scale.set(0.16, 0.04, 1);
  s.center.set(0, 0.5);
  return s;
}

export class Viewer {
  constructor(container, opts = {}) {
    this.el = container;
    this.opts = opts;
    this.d = 0.5575;   // 실물 책상 기본 (hand_ik.TABLE_D_DEFAULT) — 강화학습 탭은 setTableD 로 정책 장면 0.4775
    this.status = document.createElement("div");
    this.status.className = "v3d-status";
    container.append(this.status);
    try {
      this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    } catch (e) {
      this.status.textContent = "3D 를 못 그림: 이 브라우저에서 WebGL 을 못 씀 (" + e.message + ")";
      this.dead = true;
      return;
    }
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    container.prepend(this.renderer.domElement);
    const scene = this.scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf2f4f6);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.8); sun.position.set(1, -1.5, 2); scene.add(sun);
    const cam = this.camera = new THREE.PerspectiveCamera(40, 1, 0.01, 20);
    cam.up.set(0, 0, 1);
    cam.position.set(-0.75, -1.25, 0.2);   // 앞(-y)·오른쪽(-x) 위에서 (받침대가 팔 뒤로 가게)
    this.controls = new OrbitControls(cam, this.renderer.domElement);
    this.controls.target.set(0.0, -0.12, -0.3);
    this.controls.update();
    this.controls.addEventListener("change", () => this.dirty());
    // 책상 높이 격자(어깨 축 아래 d) + 축
    this.grid = new THREE.GridHelper(1.2, 24, 0x94a3b8, 0xcbd5e1);
    this.grid.rotation.x = Math.PI / 2;
    scene.add(this.grid);
    scene.add(new THREE.AxesHelper(0.15));
    const lx = textSprite("x 왼쪽", "#dc2626"); lx.position.set(0.16, 0, 0); scene.add(lx);
    const ly = textSprite("y 뒤", "#16a34a"); ly.position.set(0, 0.16, 0); scene.add(ly);
    const lz = textSprite("z 위", "#2563eb"); lz.position.set(0, 0, 0.17); scene.add(lz);
    const lf = textSprite("앞 (-y)", "#64748b"); this.frontLabel = lf; scene.add(lf);
    // 책상 (team-jax/IK env 장면 table_top·table_leg_a..d 를 로봇 틀로: 윗면 x −0.228..0.332, y −0.42..−0.14, 두께 5 cm,
    // 다리 2 cm 각 4 개 길이 71 cm). 윗면 = 어깨 축 아래 d (setTableD). 계획의 책상 검사는 앞 끝(y −0.14)부터 뒤로 끝없이로 본다.
    this.table = new THREE.Group();
    const wood = new THREE.MeshStandardMaterial({ color: 0xd6b98c, metalness: 0.0, roughness: 0.85, transparent: true, opacity: 0.85 });
    const top = new THREE.Mesh(new THREE.BoxGeometry(0.56, 0.28, 0.05), wood);
    top.position.set(0.052, -0.28, -0.025 - 0.0006);
    this.table.add(top);
    for (const [x, y] of [[0.052 - 0.25, -0.17], [0.052 + 0.25, -0.17], [0.052 - 0.25, -0.39], [0.052 + 0.25, -0.39]]) {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.04, 0.71), wood);
      leg.position.set(x, y, -0.025 - 0.355);
      this.table.add(leg);
    }
    this.table.visible = false;   // 2026-11-23 사용자: 책상 없다고 보고 계획 (hand_ik.NO_TABLE) → 안 그림
    scene.add(this.table);
    this.setTableD(this.d);
    // 큐브·놓을 자리
    this.cube = new THREE.Mesh(new THREE.BoxGeometry(CUBE, CUBE, CUBE), new THREE.MeshStandardMaterial({ color: 0xf97316 }));
    this.cube.visible = false; scene.add(this.cube);
    const tg = new THREE.Group();
    tg.add(new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(CUBE, CUBE, CUBE)), new THREE.LineBasicMaterial({ color: 0x16a34a })));
    tg.add(new THREE.Mesh(new THREE.BoxGeometry(CUBE, CUBE, CUBE), new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.18, depthWrite: false })));
    this.target = tg; tg.visible = false; scene.add(tg);
    this.objMark = new THREE.Mesh(new THREE.BoxGeometry(CUBE, CUBE, CUBE), new THREE.MeshStandardMaterial({ color: 0xa855f7, transparent: true, opacity: 0.6 }));
    this.objMark.visible = false; scene.add(this.objMark);
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    this._loop = this._loop.bind(this);
    requestAnimationFrame(this._loop);
  }
  setTableD(d) {
    if (!Number.isFinite(d) || (d === this.d && this._dset)) return;
    this.d = d; this._dset = true;
    this.grid.position.set(0, -0.25, -d);
    if (this.table) this.table.position.z = -d;
    if (this.grasp) this.grasp.position.z = -d + 0.001;
    this.frontLabel.position.set(0, -0.62, -d + 0.02);
    this.dirty();
  }
  async load(meshOk) {
    if (this.dead) return;
    try {
      const xml = await loadXml();
      this.actual = new ArmModel(xml, { meshes: meshOk, ghost: false });
      this.scene.add(this.actual.group);
      if (this.opts.ghost) { this.ghost = new ArmModel(xml, { meshes: meshOk, ghost: true }); this.ghost.group.visible = false; this.scene.add(this.ghost.group); }
      this.status.textContent = meshOk ? "메시 불러오는 중…" : "막대 모델 (메시 없음: JAX_IK_DIR 에 team-jax/IK 를 두면 메시로 보임)";
      this.actual.setPose({});
      if (this.ghost) this.ghost.setPose({});
      if (meshOk) {
        const r = await Promise.all(this.actual.pending.concat(this.ghost ? this.ghost.pending : []));
        const bad = r.filter((x) => !x).length;
        this.status.textContent = bad ? `메시 ${r.length - bad}/${r.length} 불러옴 (나머지 못 읽음)` : "메시 (team-jax/IK motion2/arm)";
      }
      this.ready = true;
      this.dirty();
    } catch (e) {
      this.status.textContent = "3D 모델을 못 읽음: " + e.message;
    }
  }
  focus(t, dist) {   // 카메라를 점 t(로봇 틀 [x,y,z])에 맞추고 거리 dist(m)로 (서보 원점 맞추기: 손 가까이)
    const c = this.controls, cam = this.camera;
    const dir = cam.position.clone().sub(c.target).normalize();
    c.target.set(t[0], t[1], t[2]);
    cam.position.copy(c.target.clone().add(dir.multiplyScalar(dist)));
    c.update(); this.dirty();
  }
  setActual(q) { if (this.actual) { this.actual.setPose(q || {}); this.dirty(); } }
  setGhost(q) {
    if (!this.ghost) return;
    this.ghost.group.visible = !!q;
    if (q) this.ghost.setPose(q);
    this.dirty();
  }
  setCube(c) {
    this.cube.visible = !!c;
    if (c) { this.cube.position.set(c.x, c.y, -this.d + CUBE / 2); this.cube.rotation.set(0, 0, (c.yaw || 0) * Math.PI / 180); }
    this.dirty();
  }
  setCubePose(p, q) {   // 강화학습: 큐브 중심 p=[x,y,z](로봇 틀, m), 자세 q=[x,y,z,w] — 들리고 기울어도 그대로 (setCube 는 책상 위 고정)
    this.cube.visible = !!p;
    if (p) { this.cube.position.set(p[0], p[1], p[2]); if (q) this.cube.quaternion.set(q[0], q[1], q[2], q[3]); }
    this.dirty();
  }
  setBoxTable(t) {   // 강화학습 장면의 책상: {center:[x,y,z], size:[sx,sy,sz]} (로봇 틀). 기존 책상(this.table)은 안 건드림
    if (this.boxTable) { this.scene.remove(this.boxTable); this.boxTable.geometry.dispose(); this.boxTable.material.dispose(); this.boxTable = null; }
    if (t) {
      const m = new THREE.Mesh(new THREE.BoxGeometry(...t.size),
        new THREE.MeshStandardMaterial({ color: 0xd6b98c, metalness: 0, roughness: 0.85, transparent: true, opacity: 0.8 }));
      m.position.set(...t.center);
      this.boxTable = m; this.scene.add(m);
    }
    this.dirty();
  }
  // 카메라(비전) 표시: c = {t:[x,y,z] m, rpy:[roll,pitch,yaw] °, hfov °, aspect, near, far} 또는 null. 로봇 틀.
  // 변환은 vision_node.camera_to_robot 과 같다: p_robot = Rz(yaw)·Ry(pitch)·Rx(roll)·p_cam + t, 카메라 틀 = OpenCV (x 오른쪽, y 아래, z 앞).
  // 그려지는 것: 본체 상자, 축(x 빨강·y 초록·z 파랑 = 보는 방향), 시야 원뿔(가까운·먼 면), 이름표.
  setCamera(c) {
    if (this.camGroup) {
      this.scene.remove(this.camGroup);
      this.camGroup.traverse((o) => { if (o.geometry) o.geometry.dispose(); if (o.material) { if (o.material.map) o.material.map.dispose(); o.material.dispose(); } });
      this.camGroup = null;
    }
    if (c) {
      const r = Math.PI / 180;
      const M = new THREE.Matrix4().makeRotationZ(c.rpy[2] * r)
        .multiply(new THREE.Matrix4().makeRotationY(c.rpy[1] * r)).multiply(new THREE.Matrix4().makeRotationX(c.rpy[0] * r));
      M.setPosition(c.t[0], c.t[1], c.t[2]);
      const g = new THREE.Group();
      g.matrixAutoUpdate = false; g.matrix.copy(M);
      const body = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.045, 0.04), new THREE.MeshStandardMaterial({ color: 0x1f2937, metalness: 0.3, roughness: 0.5 }));
      body.position.set(0, 0, -0.02); g.add(body);
      const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, 0.02, 16), new THREE.MeshStandardMaterial({ color: 0x2563eb }));
      lens.rotation.x = Math.PI / 2; lens.position.set(0, 0, 0.008); g.add(lens);
      for (const [dir, col] of [[[0.1, 0, 0], 0xdc2626], [[0, 0.1, 0], 0x16a34a], [[0, 0, 0.1], 0x2563eb]]) {
        g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(...dir)]), new THREE.LineBasicMaterial({ color: col })));
      }
      const th = Math.tan(c.hfov * r / 2), tv = th / (c.aspect || 4 / 3);
      const corner = (d, sx, sy) => new THREE.Vector3(sx * th * d, sy * tv * d, d);
      const pts = [];
      for (const d of [c.near, c.far]) {
        const q = [corner(d, -1, -1), corner(d, 1, -1), corner(d, 1, 1), corner(d, -1, 1)];
        for (let i = 0; i < 4; i++) pts.push(q[i], q[(i + 1) % 4]);
      }
      for (const [sx, sy] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) pts.push(new THREE.Vector3(), corner(c.far, sx, sy));
      g.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0x2563eb, transparent: true, opacity: 0.7 })));
      const far = new THREE.Mesh(new THREE.PlaneGeometry(2 * th * c.far, 2 * tv * c.far),
        new THREE.MeshBasicMaterial({ color: 0x3b82f6, transparent: true, opacity: 0.07, side: THREE.DoubleSide, depthWrite: false }));
      far.position.set(0, 0, c.far); g.add(far);
      const lab = textSprite("카메라", "#1d4ed8"); lab.position.set(0.05, -0.04, 0.02); lab.scale.set(0.2, 0.05, 1); g.add(lab);
      this.camGroup = g; this.scene.add(g);
    }
    this.dirty();
  }
  setTarget(t) {
    this.target.visible = !!t;
    if (t) this.target.position.set(t.x, t.y, -this.d + CUBE / 2);
    this.dirty();
  }
  setGhostTint(bad) {   // 유령 색: 파랑 = 지금 입력의 해, 빨강 = 못 풀어서 마지막 좋은 해를 보이는 중
    if (!this.ghost || this._tint === !!bad) return;
    this._tint = !!bad;
    const c = bad ? 0xef4444 : 0x3b7bf0;
    for (const m of [this.ghost.ghostMat, this.ghost.matStick, this.ghost.matJoint]) if (m) m.color.setHex(c);
    this.dirty();
  }
  setHandMark(p) {   // 손 위치로 보내기의 목표점(손이 쥐는 점): 분홍 구 + 책상까지 세로선
    if (!this.handMark) {
      const g = new THREE.Group();
      g.add(new THREE.Mesh(new THREE.SphereGeometry(0.008, 16, 12), new THREE.MeshBasicMaterial({ color: 0xdb2777 })));
      const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -1)]);
      this.handLine = new THREE.Line(geo, new THREE.LineDashedMaterial({ color: 0xdb2777, dashSize: 0.01, gapSize: 0.008 }));
      g.add(this.handLine);
      this.handMark = g; this.scene.add(g);
    }
    this.handMark.visible = !!p;
    if (p) {
      this.handMark.position.set(p.x, p.y, p.z);
      const h = Math.max(1e-3, p.z + this.d);
      this.handLine.scale.set(1, 1, h); this.handLine.computeLineDistances();
    }
    this.dirty();
  }
  setGraspArea(d) {   // 집을 수 있는 곳 (GET /api/ik/graspmap): 책상 위 반투명 초록, 1 칸 = 1 픽셀 텍스처
    if (this.grasp) { this.scene.remove(this.grasp); this.grasp.geometry.dispose(); this.grasp.material.map.dispose(); this.grasp.material.dispose(); this.grasp = null; }
    if (d && d.rows && d.rows.length) {
      const nx = d.xs.length, ny = d.ys.length, c = document.createElement("canvas");
      c.width = nx; c.height = ny;
      const x = c.getContext("2d"), img = x.createImageData(nx, ny);
      d.rows.forEach((row, j) => {
        for (let i = 0; i < nx; i++) {
          if (row[i] !== "2") continue;
          const k = ((ny - 1 - j) * nx + i) * 4;   // 캔버스 위 = y 큰 쪽
          img.data[k] = 34; img.data[k + 1] = 197; img.data[k + 2] = 94; img.data[k + 3] = 255;
        }
      });
      x.putImageData(img, 0, 0);
      const tex = new THREE.CanvasTexture(c);
      tex.magFilter = THREE.NearestFilter; tex.minFilter = THREE.NearestFilter; tex.generateMipmaps = false;
      const m = new THREE.Mesh(new THREE.PlaneGeometry(nx * d.step, ny * d.step),
        new THREE.MeshBasicMaterial({ map: tex, transparent: true, opacity: 0.55, depthWrite: false, side: THREE.DoubleSide }));
      m.position.set((d.xs[0] + d.xs[nx - 1]) / 2, (d.ys[0] + d.ys[ny - 1]) / 2, -this.d + 0.001);
      m.renderOrder = 1;
      m.visible = this._graspVis !== false;
      this.grasp = m; this.scene.add(m);
    }
    this.dirty();
  }
  setGraspVisible(on) { this._graspVis = !!on; if (this.grasp) this.grasp.visible = !!on; this.dirty(); }
  setObject(o) {   // /object_pose (shoulder_roll_axis 틀): 보라 반투명 상자. o.grasp: true 초록 / false 빨강 / 없음 보라
    this.objMark.visible = !!o;
    if (o) this.objMark.material.color.setHex(o.grasp === true ? 0x22c55e : o.grasp === false ? 0xef4444 : 0xa855f7);
    if (o) { this.objMark.position.set(o.x, o.y, Number.isFinite(o.z) ? o.z : -this.d + CUBE / 2); this.objMark.rotation.set(0, 0, (o.yaw || 0) * Math.PI / 180); }
    this.dirty();
  }
  dirty() { this._dirty = true; }
  resize() {
    if (this.dead) return;
    const w = this.el.clientWidth || 300, h = this.el.clientHeight || 300;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = "100%";
    this.renderer.domElement.style.height = "100%";
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.dirty();
  }
  _loop() {
    requestAnimationFrame(this._loop);
    if (!this._dirty || this.el.offsetParent === null) return;   // 숨은 탭은 안 그린다
    this._dirty = false;
    this.renderer.render(this.scene, this.camera);
  }
}

// WebGL 을 못 써 constructor 가 일찍 끝나면(dead) 장면 물체가 없다 — 모든 메서드를 아무 일 없이 끝나게 한다
// (2026-11-25 검토: setCube 등이 던져서 app.js render() 가 통째로 멈추고, RL 탭은 성공한 시뮬을 '서버 요청 실패' 로 보였다)
for (const k of Object.getOwnPropertyNames(Viewer.prototype)) {
  const f = Viewer.prototype[k];
  if (k === "constructor" || typeof f !== "function") continue;
  Viewer.prototype[k] = function (...a) { return this.dead ? undefined : f.apply(this, a); };
}
window.JaxViewer = { create: (el, opts) => new Viewer(el, opts) };
window.dispatchEvent(new Event("jaxviewer-ready"));
