"""실물로 돌릴 때 모든 것을 한 폴더에 남긴다 (2026-11-24 사용자: "실물로 할 때 모든 정보 다 로깅되게, 영상이나 그런 것들").

재생 하나(역기구학·HOME·탑 쌓기·강화학습) = 폴더 하나  logs/runs/<YYYYmmdd_HHMMSS>_<kind>/   (kind = ik | home | stack | rl | manual)
    video_live.mp4      폰 프로그램 live.jpg (live_api.LIVE.raw_jpeg, 약 10 Hz). 10 fps 고정 — 프레임이 늦으면 앞 프레임을 되풀이해
                        영상 시간 = 실제 시간 (최대 3초까지 메움). video_live_ts.csv = 새 프레임마다 영상 프레임 번호와 실제 시각
    video_infer.mp4     inference.png (뎁스·세그 그림, 1~3 Hz) 가 바뀔 때. 4 fps 고정, 같은 방식 + video_infer_ts.csv
    joints.csv          제어 루프(50 Hz) 한 줄씩: AK45 마다 명령·목표·측정(모터 °)·속도·전류·온도·에러·원점 오프셋·관절 °, 서보 보낸 각, 재생 프레임
                        (실물: real_arm._ctrl_loop 의 한 줄 훅 → deque, 파일 쓰기는 이 모듈의 스레드. 가상: 20 Hz 로 가상 팔을 읽음)
    cubes.csv           인식 결과 (새 프레임마다): id, x, y, yaw, level, tilt, source, age, stale. 가상 모드는 가상 장면이 바뀔 때
    plan.json           재생한 궤적 그대로 + 그 값 (큐브·놓을 자리·책상 d·집는 방향·쌓는 층·강화학습 정보 = 계획 칸 params)
    events.log          그 동안의 direct.log 줄 (STOP·힘 빼기·고장·원점…), 재생 시작/끝/이유, 화면에서 누른 POST 전부(보정·핀 등), CAN 상태 바뀜
    meta.json           시작/끝, 모드, git 커밋, calibration.json·폰 설정 사본(config/), 책상 높이, 버전, 줄 수·프레임 수
보관: 최근 KEEP_N(50) 개 또는 KEEP_GB(5 GB) 중 작은 쪽 — 오래된 것부터 지움.

    GET  /api/record/status            {on, run, kind, t, rows, frames, auto, virtual, list:[최근 N], disk}
    POST /api/record/start {kind?}     손으로 기록 시작 (재생이 없어도. 그 사이 재생은 같은 폴더에 plan_<n>.json 으로 붙음)
    POST /api/record/stop              손으로 끝
    POST /api/record/settings {auto?, virtual?}   재생마다 자동 기록 / 가상 모드도 기록 (logs/runs/settings.json)
    GET  /api/record/zip/<폴더>          폴더를 zip 으로 내려받기

훅 (다른 파일에는 한 줄씩):  real_arm._ctrl_loop: `rec = self.rec; if rec: rec(self, now)`,
    direct_api.play_real / server._go_home / server._play_virtual: 재생이 시작되면 REC.on_play(traj, params, arm, ...).
재생 끝은 이 모듈의 스레드가 arm.play["running"] 이 꺼지는 것을 보고 안다 (+ TAIL_S 초 더 찍고 닫음).
"""
import csv
import json
import os
import platform
import shutil
import subprocess
import sys
import threading
import time
import zipfile
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent

KEEP_N = int(os.environ.get("JAX_DASH_RUNS_KEEP") or 50)
KEEP_GB = float(os.environ.get("JAX_DASH_RUNS_MAX_GB") or 5.0)
TAIL_S = 2.0              # 재생이 끝난 뒤 더 찍는 시간 (멈춘 자리·흔들림)
LOOP_S = 0.05             # 쓰기 스레드 간격 (가상 팔 읽기 20 Hz)
LIVE_FPS, INFER_FPS = 10.0, 4.0
FILL_MAX_S = 3.0          # 프레임이 이보다 오래 안 오면 메우지 않고 다음 프레임을 바로 붙임 (ts.csv 로 알 수 있음)
MANUAL_MAX_S = 30 * 60    # 손으로 켠 기록이 끝없이 커지지 않게
MIN_FREE_GB = float(os.environ.get("JAX_DASH_RUNS_MIN_FREE_GB") or 1.5)   # 남은 디스크가 이보다 적으면 기록을 안 연다
Q_MAX = 50 * 60 * 10      # 제어 루프 줄 버퍼 (10분, 쓰기 스레드가 멈춰도 메모리 상한)
SKIP_POST = ("/api/log/follow", "/api/record/")   # 화면이 자주 보내는 진단·이 모듈 자신은 events 에 안 남김


def _git_hash():
    try:
        r = subprocess.run(["git", "-C", str(REPO), "rev-parse", "HEAD"], capture_output=True, text=True, timeout=3)
        h = r.stdout.strip()
        d = subprocess.run(["git", "-C", str(REPO), "status", "--porcelain", "--untracked-files=no"],
                           capture_output=True, text=True, timeout=3).stdout.strip()
        return h + ("+dirty" if d else "")
    except Exception:
        return None


def _dir_size(p):
    n = 0
    for root, _, files in os.walk(p):
        for f in files:
            try:
                n += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return n


def _r(x, k=3):
    return "" if x is None else round(x, k)


class _Video:
    """고정 fps mp4 (OpenCV mp4v). add(BGR, t) — 늦은 만큼 앞 프레임을 되풀이해 영상 시간 = 실제 시간."""

    def __init__(self, path, fps):
        self.path, self.fps = Path(path), fps
        self.w = None
        self.size = None
        self.t0 = None
        self.n = 0                # 쓴 영상 프레임 수 (되풀이 포함)
        self.uniq = 0
        self.skipped = 0          # 원본이 fps 보다 빨라 건너뛴 프레임 (영상 시간 = 실제 시간 유지)
        self.last = None
        self.bad = False          # VideoWriter 를 못 열었음 (한 번만 알리고 그 뒤로는 조용히 건너뜀)
        self.ts = open(self.path.with_name(self.path.stem + "_ts.csv"), "w", newline="")
        self.tsw = csv.writer(self.ts)
        self.tsw.writerow(["video_frame", "t_unix", "t_rel_s", "src_t_unix", "repeats_before"])

    def add(self, img, t, src_t=None):
        import cv2
        if self.bad:
            return
        if self.w is None:
            h, w = img.shape[:2]
            self.size = (w - w % 2, h - h % 2)
            self.w = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"mp4v"), self.fps, self.size)
            self.t0 = t
            if not self.w.isOpened():
                self.bad = True
                raise RuntimeError(f"{self.path.name} VideoWriter 못 엶 (mp4v, {self.size[0]}x{self.size[1]})")
        if (img.shape[1], img.shape[0]) != self.size:
            img = cv2.resize(img, self.size, interpolation=cv2.INTER_AREA)
        want = int((t - self.t0) * self.fps)     # 이 프레임이 놓일 영상 번호
        if want < self.n:                        # 원본이 fps 보다 빠름: 이 자리는 이미 찼다 (다 쓰면 영상이 실제보다 길어짐)
            self.skipped += 1
            self.last = img                      # 다음에 메울 때는 가장 새 프레임으로
            return
        rep = 0
        if self.last is not None and want > self.n and (want - self.n) / self.fps <= FILL_MAX_S:
            while self.n < want:
                self.w.write(self.last)
                self.n += 1
                rep += 1
        elif self.last is not None and want > self.n:
            self.t0 = t - self.n / self.fps      # 긴 끊김: 메우지 않고 시간축을 다시 맞춤
        self.tsw.writerow([self.n, f"{t:.3f}", f"{t - self.t0:.3f}" if self.t0 else "", f"{src_t:.3f}" if src_t else "", rep])
        self.w.write(img)
        self.n += 1
        self.uniq += 1
        self.last = img

    def close(self):
        if self.w is not None:
            self.w.release()
        self.ts.close()


class Run:
    def __init__(self, rec, kind, mode, auto):
        self.rec, self.kind, self.mode, self.auto = rec, kind, mode, auto
        self.t0 = time.time()
        name = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.t0)) + f"_{kind}"
        d = rec.root / name
        k = 2
        while d.exists():
            d = rec.root / f"{name}_{k}"
            k += 1
        d.mkdir(parents=True)
        self.dir, self.name = d, d.name
        self.q = deque(maxlen=Q_MAX)             # 제어 루프 줄 (실물 훅이 넣음)
        self.plans = []                          # 이 기록 안의 재생들
        self.arm = None                          # 훅을 단 RealArm / 읽는 VirtualArm
        self.play_seen = False                   # 붙은 재생이 running 으로 보였는가
        self.end_at = None
        self.end_reason = None
        self.rows = self.dropped = 0
        self.cube_rows = 0
        self.ev = open(d / "events.log", "a", encoding="utf-8")
        self.jf = open(d / "joints.csv", "w", newline="")
        self.jw = None
        self.cf = open(d / "cubes.csv", "w", newline="")
        self.cw = csv.writer(self.cf)
        self.cw.writerow(["t_unix", "t_frame", "frame_n", "src", "id", "x", "y", "yaw_deg", "level", "tilt_deg", "source", "age_s", "stale"])
        self.vlive = self.vinfer = None
        self.last_raw = None
        self.last_frame_n = None
        self.last_scene_t = None
        self.infer_key = None
        self.infer_check_t = 0.0
        self.log_pos = None
        self.bus_state = None
        self.can_text = None
        self.virtual = False
        self.play_obj = None
        self._sample = None                      # 이 기록이 arm.rec 에 단 함수 (닫을 때 내 것일 때만 지움)
        self.close_req = None                    # 다른 스레드가 끝내 달라고 한 이유 (닫기는 쓰기 스레드만: 영상 쓰는 중 release 를 막음)
        self.closed = threading.Event()
        self.meta = {"name": self.name, "kind": kind, "mode": mode, "auto": auto, "start": self.t0,
                     "start_local": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.t0)),
                     "git": rec.git, "host": platform.node(), "python": sys.version.split()[0]}
        self.event(f"기록 시작 ({kind}, {mode}{', 자동' if auto else ', 손으로'})")

    # -------------------------------------------------- 쓰기 (쓰기 스레드 · 요청 스레드)
    def event(self, text):
        try:
            self.ev.write(time.strftime("%H:%M:%S", time.localtime()) + f".{int(time.time() * 1000) % 1000:03d} [rec] {text}\n")
            self.ev.flush()
        except (OSError, ValueError):
            pass

    def add_plan(self, kind, traj, params, extra):
        k = len(self.plans)
        fn = "plan.json" if k == 0 else f"plan_{k + 1}.json"
        t = dict(traj or {})
        pos = t.pop("positions", None) or []
        t["frames"] = len(pos)                 # (비어 있지 않게: 아래에서 마지막 '}}' 앞에 positions 를 붙인다)
        d = {"kind": kind, "t": time.time(), "params": params, "extra": extra, "trajectory": t}
        try:
            # positions 는 줄마다 따로 dumps: 큰 리스트 하나를 C 인코더가 한 번에 돌면 GIL 을 수십 ms 잡아 50 Hz 제어 틱이 밀린다
            head = json.dumps(d, ensure_ascii=False, default=str)
            rows = ",\n".join([json.dumps(r) for r in pos])
            (self.dir / fn).write_text(head[:-2] + ', "positions": [\n' + rows + "]}}", encoding="utf-8")
        except (OSError, TypeError, ValueError) as e:
            self.event(f"{fn} 못 씀: {e}")
        n = len((traj or {}).get("positions") or [])
        self.plans.append({"file": fn, "kind": kind, "t": d["t"], "frames": n, "planner": (traj or {}).get("planner"),
                           "status": (traj or {}).get("status"), "params": params})
        self.event(f"재생 시작: {kind}, {n} 프레임 → {fn}" + (f" · {json.dumps(params, ensure_ascii=False, default=str)[:400]}" if params else ""))

    def _header(self, arm):
        h = ["t_unix", "t_mono", "play_frame"]
        for m in self.ak_list:
            p = f"{m.joint.joint}"
            h += [f"{p}_cmd", f"{p}_target", f"{p}_pos", f"{p}_vel_erpm", f"{p}_cur_a", f"{p}_temp_c", f"{p}_err", f"{p}_offset",
                  f"{p}_sent", f"{p}_deg"]
        h += [f"{j.joint}_servo_sent" for j in self.servo_list] + [f"{j.joint}_deg" for j in self.servo_list]
        return h

    def attach(self, arm, virtual):
        """훅 달기 (실물: arm.rec = 제어 루프에서 부를 함수). 같은 팔이면 그대로."""
        if arm is self.arm:
            return
        self.detach()
        self.arm, self.virtual = arm, virtual
        if not virtual:
            self.ak_list = list(arm.ak.values())
            self.servo_list = [j for j in arm.cal.joints if j.motor == "servo"]
            pairs = [(m, arm.homing.m[m.name]) for m in self.ak_list]
            servos = [j.name for j in self.servo_list]
            q = self.q

            def sample(a, now):          # 제어 스레드에서, a.lock 안. 숫자 몇 개를 리스트로 deque 에 — 파일 쓰기·변환 없음
                pl = a.play
                row = [time.time(), now, pl.get("frame") if pl.get("running") else -1]
                for m, h in pairs:
                    row.extend((m.cmd, m.target, m.pos, m.spd, m.cur, m.temp, m.err, h.offset_deg if h.homed else None,
                                m.last_sent if m.sent_t == now else None))
                ss = a.servo_sent
                row.extend(ss[n][0] if n in ss else None for n in servos)
                q.append(row)
            if self.jw is None:
                self.jw = csv.writer(self.jf)
                self.jw.writerow(self._header(arm))
            self._sample = sample
            arm.rec = sample
        else:
            self.ak_list, self.servo_list = [], []
            if self.jw is None:
                self.jw = csv.writer(self.jf)
                self.jw.writerow(["t_unix", "play_frame", "joint_deg_json", "servo_sent_json"])
        self.event(f"팔 붙음: {'가상 팔 (20 Hz 로 읽음)' if virtual else '실물 RealArm (제어 루프 50 Hz)'}")

    def detach(self):
        a = self.arm
        if a is not None and self._sample is not None and getattr(a, "rec", None) is self._sample:
            a.rec = None                         # 새 기록이 이미 단 훅은 그대로
        self._sample = None
        self.arm = None

    def _write_joints(self):
        if self.arm is None or self.jw is None:
            return
        if self.virtual:
            a = self.arm
            try:
                with a.lock:
                    deg = {n: round(m.phys, 3) for n, m in a.motors.items()}
                    ss = {n: round(d, 2) for n, (d, _) in a.servo_sent.items()}
                    pl = a.play
                    fr = pl.get("frame") if pl.get("running") else -1
            except Exception:
                return
            self.jw.writerow([f"{time.time():.3f}", fr, json.dumps(deg), json.dumps(ss)])
            self.rows += 1
            return
        q = self.q
        out = []
        while q:
            row = q.popleft()
            r = [f"{row[0]:.3f}", f"{row[1]:.4f}", row[2]]
            k = 3
            for m in self.ak_list:
                cmd, tgt, pos, spd, cur, temp, err, off, sent = row[k:k + 9]
                k += 9
                deg = None
                if pos is not None and off is not None:
                    try:
                        deg = m.joint.to_sim(pos - off)
                    except Exception:
                        deg = None
                r += [_r(cmd), _r(tgt), _r(pos), "" if spd is None else spd, _r(cur, 3), "" if temp is None else temp, err,
                      _r(off), _r(sent), _r(deg, 2)]
            sv = row[k:]
            r += [_r(v, 2) for v in sv]
            r += ["" if v is None else round(j.to_sim(v), 2) for j, v in zip(self.servo_list, sv)]
            out.append(r)
        if out:
            self.jw.writerows(out)
            self.rows += len(out)

    def _write_video(self, live):
        if live is None:
            return
        import cv2
        import numpy as np
        raw, rt = live.raw_jpeg, live.raw_t
        if raw is not None and raw is not self.last_raw:
            self.last_raw = raw
            img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                if self.vlive is None:
                    self.vlive = _Video(self.dir / "video_live.mp4", LIVE_FPS)
                self.vlive.add(img, rt or time.time(), (live.key or (None, 0))[1] / 1e9 if live.key and live.key[0] == "live.jpg" else None)
        now = time.time()
        if now - self.infer_check_t >= 0.25:
            self.infer_check_t = now
            p = self.rec.infer_path()
            if p:
                try:
                    st = os.stat(p)
                except OSError:
                    st = None
                if st is not None and (st.st_mtime_ns, st.st_size) != self.infer_key and now - st.st_mtime < 10:
                    img = cv2.imread(p, cv2.IMREAD_COLOR)
                    if img is not None:            # 쓰는 중이면 다음 번에
                        self.infer_key = (st.st_mtime_ns, st.st_size)
                        if self.vinfer is None:
                            self.vinfer = _Video(self.dir / "video_infer.mp4", INFER_FPS)
                        self.vinfer.add(img, now, st.st_mtime)

    def _write_cubes(self, live, scene, virtual_mode):
        now = time.time()
        if virtual_mode:
            if scene is None:
                return
            s = scene.snapshot()
            if s["t"] == self.last_scene_t:
                return
            self.last_scene_t = s["t"]
            for c in s["cubes"] or [{}]:
                self.cw.writerow([f"{now:.3f}", f"{s['t']:.3f}", "", "virtual", c.get("id", ""), _r(c.get("x"), 4), _r(c.get("y"), 4),
                                  c.get("yaw_deg", ""), c.get("level", ""), "", "virtual" if c else "(없음)", 0, 0])
                self.cube_rows += 1
            return
        if live is None:
            return
        with live.lock:
            res = live.res
        if not res or res.get("frame_n") == self.last_frame_n or "t_frame" not in res:
            return
        self.last_frame_n = res.get("frame_n")
        age = now - res["t_frame"]
        stale = int(age > live.stale_s())
        for c in res.get("cubes") or [{}]:
            self.cw.writerow([f"{now:.3f}", f"{res['t_frame']:.3f}", res.get("frame_n"), res.get("src"), "" if c.get("id") is None else c.get("id"),
                              _r(c.get("x"), 4), _r(c.get("y"), 4), _r(c.get("yaw_deg"), 1), c.get("level", ""), _r(c.get("tilt_deg"), 1),
                              c.get("source", "(없음)" if not c else ""), round(age, 3), stale])
            self.cube_rows += 1

    def _tail_log(self):
        p = self.rec.direct_log
        if not p:
            return
        try:
            size = os.path.getsize(p)
        except OSError:
            return
        if self.log_pos is None:              # 처음: 시작 직전 몇 줄도 (재생 시작 줄은 기록이 열리기 전에 쓰인다)
            self.log_pos = max(0, size - 1500)
            if self.log_pos:
                with open(p, "rb") as f:
                    f.seek(self.log_pos)
                    self.log_pos += f.read(1500).find(b"\n") + 1
            self.ev.write("--- direct.log (시작 직전부터) ---\n")
        if size < self.log_pos:
            self.log_pos = 0
        if size == self.log_pos:
            return
        with open(p, "rb") as f:
            f.seek(self.log_pos)
            data = f.read(min(size - self.log_pos, 1 << 20))
        cut = data.rfind(b"\n") + 1          # 줄 중간이면 다음 번에
        if cut:
            self.log_pos += cut
            for line in data[:cut].decode("utf-8", "replace").splitlines():
                self.ev.write(line + "\n")
            self.ev.flush()

    def _watch_bus(self, arm, can_text):
        b = getattr(arm, "busst", None) if arm is not None and not self.virtual else None
        st = b.get("state") if b else None
        if st != self.bus_state:
            if self.bus_state is not None or st not in (None, "ERROR-ACTIVE"):
                self.event(f"CAN 버스 상태: {self.bus_state} → {st} (err_frames {b.get('err_frames') if b else '-'}, busoff {b.get('busoff_count') if b else '-'})")
            self.bus_state = st
        if can_text != self.can_text:
            if self.can_text is not None:
                self.event(f"can0: {self.can_text} → {can_text}")
            self.can_text = can_text

    def step(self, live, scene, virtual_mode, can_text):
        """부분마다 따로 try: 하나가 계속 실패해도(디스크 가득·direct.log 사라짐) 나머지와 닫기 판정은 돈다."""
        for key, what, fn in (("joints", "joints.csv", self._write_joints),
                              ("video", "영상 쓰기", lambda: self._write_video(live if not virtual_mode else None)),
                              ("cubes", "cubes.csv", lambda: self._write_cubes(live, scene, virtual_mode)),
                              ("tail", "direct.log 따라 읽기", self._tail_log),
                              ("bus", "CAN 상태 보기", lambda: self._watch_bus(self.arm, can_text))):
            try:
                fn()
            except Exception as e:
                self._once(key, f"{what} 오류: {type(e).__name__}: {e}")

    def _once(self, key, text):
        k = "_err_" + key
        if not getattr(self, k, False):
            setattr(self, k, True)
            self.event(text)

    def close(self, reason, extra_meta):
        self.detach()
        for fn in (self._write_joints, self._tail_log):
            try:
                fn()
            except Exception:
                pass
        self.event(f"기록 끝: {reason}")
        for v in (self.vlive, self.vinfer):
            if v is not None:
                try:
                    v.close()
                except Exception:
                    pass
        for f in (self.jf, self.cf, self.ev):
            try:
                f.close()
            except (OSError, ValueError):
                pass
        t1 = time.time()
        self.meta.update(end=t1, end_local=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t1)), seconds=round(t1 - self.t0, 2),
                         end_reason=reason, plays=self.plans, joint_rows=self.rows,
                         joint_hz=round(self.rows / (t1 - self.t0), 1) if t1 > self.t0 else None,
                         video_live_frames=self.vlive.uniq if self.vlive else 0, video_live_written=self.vlive.n if self.vlive else 0,
                         video_live_skipped=self.vlive.skipped if self.vlive else 0,
                         video_infer_frames=self.vinfer.uniq if self.vinfer else 0, cube_rows=self.cube_rows, **extra_meta)
        self.meta["bytes"] = _dir_size(self.dir)
        (self.dir / "meta.json").write_text(json.dumps(self.meta, ensure_ascii=False, indent=1, default=str), encoding="utf-8")


class Recorder:
    def __init__(self):
        self.lock = threading.Lock()
        self.cur = None
        self.root = None
        self.direct_log = None
        self.get_arm = lambda: (None, False)      # → (팔, virtual)
        self.get_mode = lambda: "direct"
        self.get_can = lambda: None
        self.settings = {"auto": True, "virtual": True}
        self.thread = None
        self.last = None                          # 마지막으로 닫은 기록 이름
        self._list_cache = (0.0, None)
        self.git = None
        self.low_disk = None                      # 남은 디스크가 모자라 기록을 안 연 때: {"free": 바이트, "t": 시각}

    # -------------------------------------------------- 설정
    def setup(self, log_dir, get_arm, get_mode, get_can):
        self.root = Path(log_dir) / "runs"
        self.root.mkdir(parents=True, exist_ok=True)
        self.direct_log = str(Path(log_dir) / "direct.log")
        self.log_dir = Path(log_dir)
        self.get_arm, self.get_mode, self.get_can = get_arm, get_mode, get_can
        self.git = _git_hash()            # 서버 시작 때 한 번 (재생 중 fork 하지 않게)
        try:
            self.settings.update(json.loads((self.root / "settings.json").read_text()))
        except (OSError, ValueError):
            pass
        if self.thread is None:
            self.thread = threading.Thread(target=self._loop, daemon=True, name="run-record")
            self.thread.start()

    def set(self, b):
        for k in ("auto", "virtual"):
            if k in b:
                self.settings[k] = bool(b[k])
        try:
            (self.root / "settings.json").write_text(json.dumps(self.settings))
        except OSError:
            pass
        return {"ok": True, "settings": dict(self.settings)}

    def infer_path(self):
        try:
            import phone_depth as DP
            return os.path.join(DP.run_dir(DP.load_cfg(self.log_dir / "phone_depth.json")), "inference.png")
        except Exception:
            return None

    # -------------------------------------------------- 시작 / 끝
    def _disk_ok(self):
        """남은 디스크 ≥ MIN_FREE_GB 인가 (statvfs 한 번 — 빠름). 모자라면 low_disk 에 남겨 칩에 표시."""
        try:
            free = shutil.disk_usage(self.root).free
        except OSError:
            return True
        if free < MIN_FREE_GB * (1 << 30):
            if self.low_disk is None:
                print(f"run_record: 남은 디스크 {free / (1 << 30):.2f} GB < {MIN_FREE_GB:g} GB — 기록 안 함", flush=True)
            self.low_disk = {"free": free, "min_gb": MIN_FREE_GB, "t": time.time()}
            return False
        self.low_disk = None
        return True

    @staticmethod
    def kind_of(traj, params):
        t, p = traj or {}, params or {}
        if t.get("kind") == "home":
            return "home"
        if t.get("rl") is not None or "policy" in t:
            return "rl"
        if str(p.get("stack") or "").strip():
            return "stack"
        return "ik"

    def on_play(self, traj, params=None, arm=None, virtual=False, extra=None):
        """재생이 시작된 직후 (요청 스레드). 자동 기록이면 새 폴더, 이미 기록 중이면 그 폴더에 계획을 붙인다. 실패해도 재생은 그대로."""
        try:
            kind = self.kind_of(traj, params)
            with self.lock:
                if self.root is None:
                    return
                if self.cur is None:
                    if not self.settings.get("auto", True) or (virtual and not self.settings.get("virtual", True)):
                        return
                    if not self._disk_ok():
                        return
                    self.cur = Run(self, kind, "virtual" if virtual else "direct", auto=True)
                    self._snap_config(self.cur)
                r = self.cur
                r.end_at = None                    # 꼬리 기다리던 중 새 재생이 붙으면 계속
                r.play_seen = False
                r.play_obj = arm
                if arm is not None:
                    r.attach(arm, virtual)
                r.add_plan(kind, traj, params, extra)
        except Exception as e:
            print(f"run_record: 기록 시작 실패: {type(e).__name__}: {e}", flush=True)

    def start_manual(self, kind="manual"):
        with self.lock:
            if self.cur is not None:
                return {"ok": False, "error": f"이미 기록 중: {self.cur.name}"}
            if not self._disk_ok():
                return {"ok": False, "error": f"남은 디스크 {self.low_disk['free'] / (1 << 30):.2f} GB — {MIN_FREE_GB:g} GB 아래라 기록 안 함"}
            arm, virtual = self.get_arm()
            self.cur = Run(self, kind, "virtual" if virtual else self.get_mode(), auto=False)
            self._snap_config(self.cur)
            if arm is not None:
                self.cur.attach(arm, virtual)
            return {"ok": True, "msg": f"기록 시작: runs/{self.cur.name}", "run": self.cur.name}

    def stop(self, reason="손으로 끝"):
        with self.lock:
            r = self.cur
            if r is None:
                return {"ok": True, "msg": "기록 중 아님"}
            r.close_req = reason
        if not r.closed.wait(5.0):         # 쓰기 스레드가 닫는다
            return {"ok": False, "error": f"기록이 5초 안에 안 닫힘: runs/{r.name} (기록 스레드 확인)", "run": r.name}
        return {"ok": True, "msg": f"기록 끝: runs/{r.name}", "run": r.name}

    def _close(self, r, reason):
        extra = {}
        try:
            import live_api
            extra["table_d"] = live_api.LIVE.td if live_api.LIVE else None
        except Exception:
            pass
        try:
            r.close(reason, extra)
        except Exception as e:
            print(f"run_record: 기록 닫기 오류 {r.name}: {type(e).__name__}: {e}", flush=True)
        r.closed.set()
        self.last = r.name
        self._list_cache = (0.0, None)
        self.prune()

    def _snap_config(self, r):
        """calibration.json · 원점 자세 · 폰 설정을 config/ 에 복사, meta 에 요약."""
        cfg = r.dir / "config"
        cfg.mkdir(exist_ok=True)
        arm, virtual = self.get_arm()
        cal = getattr(getattr(arm, "cal", None), "path", None) or (REPO / "arm_control" / "calibration.json")
        files = {"calibration.json": cal, "homing.json": REPO / "arm_control" / "homing.json"}
        for n in ("phone_cam.json", "phone_depth.json", "phone_table.json", "virtual_scene.json"):
            files[n] = self.log_dir / n
        got = {}
        for n, p in files.items():
            try:
                if p and Path(p).is_file():
                    shutil.copy(p, cfg / n)
                    got[n] = str(p)
            except OSError:
                pass
        r.meta["config_files"] = got
        try:
            cam = json.loads((cfg / "phone_cam.json").read_text())
            r.meta["phone"] = {k: cam.get(k) for k in ("model", "phone", "device", "name", "calib_file", "intrinsics", "K") if k in cam} or \
                {"keys": sorted(cam)[:30]}
        except (OSError, ValueError):
            r.meta["phone"] = None
        try:
            import live_api
            r.meta["table_d_start"] = live_api.LIVE.td if live_api.LIVE else None
        except Exception:
            pass
        v = {}
        for mod in ("numpy", "cv2", "can", "fastapi"):     # 이미 읽힌 것만 (여기서 import 하면 GIL 을 오래 잡아 제어 루프가 밀림)
            try:
                v[mod] = sys.modules[mod].__version__
            except Exception:
                pass
        r.meta["versions"] = v
        if arm is not None and not virtual:
            r.meta["arm"] = {"interface": arm.interface, "channel": arm.channel, "params": dict(arm.p),
                             "homing": {n: {"homed": h.homed, "offset_deg": h.offset_deg} for n, h in arm.homing.m.items()}}

    # -------------------------------------------------- 쓰기 스레드
    def _loop(self):
        while True:
            t = time.monotonic()
            r = self.cur
            if r is not None:
                try:
                    self._step(r)
                except Exception as e:
                    r._once("loop", f"기록 스레드 오류: {type(e).__name__}: {e}")
            time.sleep(max(0.005, LOOP_S - (time.monotonic() - t)))

    def _step(self, r):
        import live_api
        try:
            mode = self.get_mode()
            arm, virtual = self.get_arm()
            if not r.auto and arm is not None and arm is not r.arm:      # 손으로 켠 기록: 그 사이 팔을 연결하면 붙임
                r.attach(arm, virtual)
            if r.arm is not None and not virtual and not getattr(r.arm, "running", True):
                r.event("팔 연결 끊김")
                r.detach()
            can = self.get_can()
            r.step(live_api.LIVE, live_api.SCENE, mode == "virtual", can)
        except Exception as e:                  # 아래 닫기 판정(손으로 끝·재생 끝·30분)은 꼭 돈다
            r._once("pre", f"기록 스레드 오류: {type(e).__name__}: {e}")
        now = time.time()
        if r.close_req:
            with self.lock:
                if self.cur is r:
                    self.cur = None
            self._close(r, r.close_req)
            return
        if r.auto:
            po = r.play_obj
            pl = getattr(po, "play", None) or {}
            running = bool(po is not None and pl.get("running"))
            if running:
                r.play_seen = True
            if r.end_at is None and (r.play_seen and not running or po is None or (not r.play_seen and now - r.t0 > 5)):
                reason = (pl.get("reason") if po is not None else None) or "재생 끝"
                r.end_reason = reason
                r.event(f"재생 끝: {reason} — {TAIL_S:g}초 더 기록")
                r.end_at = now + TAIL_S
            if r.end_at is not None and now >= r.end_at:
                with self.lock:
                    if self.cur is r and r.end_at is not None:
                        self.cur = None
                    else:
                        return
                self._close(r, r.end_reason or "재생 끝")
        elif now - r.t0 > MANUAL_MAX_S:
            with self.lock:
                if self.cur is r:
                    self.cur = None
            self._close(r, f"손으로 켠 기록이 {MANUAL_MAX_S // 60}분 넘음 — 자동으로 끝")

    def event(self, text):
        r = self.cur
        if r is not None:
            r.event(text)

    # -------------------------------------------------- 목록 · 보관
    def runs(self):
        if self.root is None or not self.root.is_dir():
            return []
        return sorted((p for p in self.root.iterdir() if p.is_dir() and not p.name.startswith(".")), key=lambda p: p.name)

    def prune(self):
        cur = self.cur.name if self.cur else None
        runs = [p for p in self.runs() if p.name != cur]
        sizes = {p: _dir_size(p) for p in runs}
        total = sum(sizes.values())
        cap = KEEP_GB * (1 << 30)
        removed = []
        while runs and (len(runs) > KEEP_N or total > cap):
            p = runs.pop(0)
            shutil.rmtree(p, ignore_errors=True)
            total -= sizes[p]
            removed.append(p.name)
        if removed:
            print(f"run_record: 보관 한도 ({KEEP_N}개 / {KEEP_GB:g} GB) — 지움: {', '.join(removed)}", flush=True)
        return removed

    def listing(self, n=20):
        t, cached = self._list_cache
        if cached is not None and time.time() - t < 5:
            return cached
        rows, total = [], 0
        cur = self.cur.name if self.cur else None
        for p in reversed(self.runs()):
            m = {}
            try:
                m = json.loads((p / "meta.json").read_text(encoding="utf-8"))
                size = int(m.get("bytes") or 0)
            except (OSError, ValueError):
                size = _dir_size(p)
            total += size
            if len(rows) < n:
                rows.append({"name": p.name, "kind": m.get("kind") or p.name.split("_")[-1], "mode": m.get("mode"),
                             "start": m.get("start_local"), "seconds": m.get("seconds"), "reason": m.get("end_reason"),
                             "bytes": size, "video": (p / "video_live.mp4").is_file(), "rows": m.get("joint_rows"),
                             "hz": m.get("joint_hz"), "recording": p.name == cur})
        try:
            free = shutil.disk_usage(self.root).free
        except OSError:
            free = None
        out = {"runs": rows, "count": len(self.runs()), "bytes": total, "free": free, "keep_n": KEEP_N, "keep_gb": KEEP_GB}
        self._list_cache = (time.time(), out)
        return out

    def status(self):
        r = self.cur
        out = {"on": r is not None, "settings": dict(self.settings), "last": self.last, "low_disk": self.low_disk}
        if r is not None:
            out.update(run=r.name, kind=r.kind, auto=r.auto, seconds=round(time.time() - r.t0, 1), rows=r.rows,
                       frames=r.vlive.uniq if r.vlive else 0, infer=r.vinfer.uniq if r.vinfer else 0, ending=r.end_at is not None)
        out["list"] = self.listing()
        free = out["list"].get("free")
        if free is not None and r is None:     # 지금 시작하면 안 열릴 만큼 모자라면 미리 칩에 (listing 이 5초 캐시)
            out["low_disk"] = {"free": free, "min_gb": MIN_FREE_GB} if free < MIN_FREE_GB * (1 << 30) else None
        return out

    def zip_path(self, name):
        """폴더 → zip (mp4 는 압축 안 함). 만든 zip 경로 또는 None."""
        p = self.root / name
        if "/" in name or name.startswith(".") or not p.is_dir() or (self.cur and self.cur.name == name):
            return None
        zd = self.root / ".zip"
        zd.mkdir(exist_ok=True)
        for old in zd.iterdir():          # 지난 zip 은 지움 (하나씩만)
            try:
                old.unlink()
            except OSError:
                pass
        z = zd / f"{name}.zip"
        with zipfile.ZipFile(z, "w") as zf:
            for root, _, files in os.walk(p):
                for f in sorted(files):
                    fp = Path(root) / f
                    comp = zipfile.ZIP_STORED if f.endswith((".mp4", ".png", ".jpg")) else zipfile.ZIP_DEFLATED
                    zf.write(fp, f"{name}/{fp.relative_to(p)}", compress_type=comp)
        return z


REC = Recorder()


def install(app, log_dir, get_arm, get_mode, get_can):
    """server.py 가 부른다. get_arm() → (팔 또는 None, virtual?), get_mode() → 'direct'|'virtual'|'ros', get_can() → can0 글."""
    from fastapi import Request
    from fastapi.responses import FileResponse, JSONResponse

    REC.setup(log_dir, get_arm, get_mode, get_can)

    class _PostLog:
        """화면에서 누른 POST 를 기록 중이면 events.log 에 (몸통 앞 300자). 순수 ASGI — 요청은 그대로 지나간다."""

        def __init__(self, app_):
            self.app = app_

        async def __call__(self, scope, receive, send):
            if scope.get("type") != "http" or scope.get("method") != "POST" \
                    or not scope.get("path", "").startswith("/api/") or scope["path"].startswith(SKIP_POST):
                return await self.app(scope, receive, send)
            buf = bytearray()

            async def recv():
                m = await receive()
                if m.get("type") == "http.request" and len(buf) < 2048:
                    buf.extend(m.get("body", b"")[:2048])
                return m

            status = {}

            async def snd(m):
                if m.get("type") == "http.response.start":
                    status["code"] = m.get("status")
                await send(m)
            try:
                await self.app(scope, recv, snd)
            finally:
                if REC.cur is not None:     # 끝난 뒤에 본다: 재생 요청이 기록을 시작한 경우도 남게
                    body = bytes(buf).decode("utf-8", "replace").replace("\n", " ")[:300]
                    REC.event(f"화면 POST {scope['path']} {body} → {status.get('code', '?')}")

    app.add_middleware(_PostLog)

    @app.get("/api/record/status")
    def _status():
        return JSONResponse(REC.status())

    @app.post("/api/record/start")
    async def _start(req: Request):
        try:
            b = await req.json()
        except Exception:
            b = {}
        if not isinstance(b, dict):              # [] · null · "str" → 기본 (전엔 b.get 에서 500)
            b = {}
        kind = str(b.get("kind") or "manual")
        if kind not in ("manual", "ik", "home", "stack", "rl"):
            kind = "manual"
        return JSONResponse(REC.start_manual(kind))

    @app.post("/api/record/stop")
    def _stop():
        return JSONResponse(REC.stop())

    @app.post("/api/record/settings")
    async def _settings(req: Request):
        try:
            b = await req.json()
        except Exception:
            b = {}
        return JSONResponse(REC.set(b if isinstance(b, dict) else {}))

    @app.get("/api/record/zip/{name}")
    def _zip(name: str):
        z = REC.zip_path(name)
        if z is None:
            return JSONResponse({"ok": False, "error": "없는 기록 (또는 기록 중)"}, status_code=404)
        return FileResponse(str(z), media_type="application/zip", filename=z.name)
