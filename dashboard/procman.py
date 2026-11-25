"""프로세스 관리자: 항목마다 새 프로세스 그룹으로 띄우고, 로그 파일에 쓰고, 끌 때 SIGINT → SIGTERM → SIGKILL.

- Linux: bash -lc "source ROS; source ws; exec stdbuf -oL ..." , start_new_session=True, 그룹 전체에 신호(터미널 Ctrl+C 와 같음).
- Windows(시험용): CREATE_NEW_PROCESS_GROUP, SIGINT 대신 CTRL_BREAK_EVENT, SIGTERM/SIGKILL 둘 다 TerminateProcess.
- 대시보드를 껐다 켜도 Linux 에서는 logs/pids.json 으로 아직 살아 있는 항목을 다시 붙잡는다(입양).
- 가상 모드(set_virtual): ROS 노드 항목은 프로세스를 띄우지 않고 virtual_arm.VirtualArm 의 가상 노드를 켜고 끈다.
  bash 한 번 실행형(can0 올리기, vcan 시험)은 '가상: 할 일 없음' 으로 바로 끝난다. argv 항목(가짜 시험, 로컬
  pick_place --dry-run)은 그대로 이 PC 에서 실행. 로그는 logs/virtual/<id>.log (실물 로그와 섞이지 않게).
"""
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from services import ROS_DISTRO, WS, WATCH_PROCS, visible_services

try:
    from virtual_arm import SERVICE_NODES as VIRTUAL_NODES
except Exception:   # virtual_arm 을 못 읽어도(arm_control 없음) 실물 모드는 돈다
    VIRTUAL_NODES = {}
MODE = {"virtual": False}
FORCE_LOCAL = {"on": False}   # 직접 CAN 모드(ROS 없음): argv 로컬 실행을 쓰는 항목(pick_place --dry-run)을 이 PC 의 python 으로

IS_LINUX = sys.platform.startswith("linux")
LOG_DIR = Path(os.environ.get("JAX_DASH_LOGS") or Path(__file__).resolve().parent / "logs")
PID_FILE = LOG_DIR / "pids.json"
SIGINT_WAIT = float(os.environ.get("JAX_DASH_SIGINT_WAIT", "5"))
SIGTERM_WAIT = float(os.environ.get("JAX_DASH_SIGTERM_WAIT", "3"))
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


def _ros_prefix():
    setup = shlex.quote(f"{WS}/install/setup.bash")
    distro = shlex.quote(f"/opt/ros/{ROS_DISTRO}/setup.bash")
    return (f"source {distro} || {{ echo {distro} '없음'; exit 3; }}\n"
            f"[ -f {setup} ] || {{ echo {setup} '없음 — colcon build 먼저'; exit 3; }}\n"
            f"source {setup}\n")


def _proc_start_time(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]
    except Exception:
        return None


def tail_file(path, n=40, max_bytes=65536):
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - max_bytes))
            data = f.read()
    except FileNotFoundError:
        return []
    lines = ANSI.sub("", data.decode("utf-8", "replace")).splitlines()
    return lines[-n:]


class Item:
    def __init__(self, svc):
        self.svc = svc
        self.proc = None          # Popen (우리가 띄운 것)
        self.pid = None           # 입양한 것은 pid 만
        self.started = None
        self.ended = None
        self.exit_code = None
        self.stopping = False
        self.stop_note = ""       # "SIGINT 로 끝남" / "SIGINT 로 안 끝나 SIGKILL" …
        self.error = ""           # 켜기 거절 이유 등
        self.cmd = ""
        self.params = {p.key: p.default for p in svc.params}
        self.adopted = False
        self.vm = None            # 가상 모드에서 켠 가상 노드 (VirtualArm)
        self.lock = threading.Lock()

    @property
    def log_path(self):
        return (LOG_DIR / "virtual" if MODE["virtual"] else LOG_DIR) / f"{self.svc.id}.log"

    def alive(self):
        if self.vm is not None:
            return self.vm.service_alive(self.svc.id)
        if self.proc is not None:
            return self.proc.poll() is None
        if self.pid:
            try:
                os.kill(self.pid, 0)
            except OSError:
                return False
            st = Path(f"/proc/{self.pid}/stat")
            try:
                if st.read_text().rsplit(")", 1)[1].split()[0] == "Z":
                    return False
            except Exception:
                return False
            return True
        return False

    def available(self):
        if MODE["virtual"]:
            return True, ""
        if self.svc.linux and not IS_LINUX:
            return False, "이 PC에서 못 씀 (Linux 로봇 PC 전용)"
        return True, ""


class ProcessManager:
    def __init__(self):
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.items = {s.id: Item(s) for s in visible_services()}
        self.strays = []
        self.virtual = None       # VirtualArm (가상 모드) 또는 None
        self._adopt()
        threading.Thread(target=self._watch, daemon=True).start()

    # ---------- 켜기 ----------
    def start(self, sid, params=None):
        it = self.items.get(sid)
        if it is None:
            return {"ok": False, "error": f"없는 항목 {sid}"}
        with it.lock:
            ok, why = it.available()
            if not ok:
                return {"ok": False, "error": why}
            if it.alive():
                return {"ok": False, "error": f"{it.svc.label} 이미 실행 중"}
            if it.stopping:
                return {"ok": False, "error": "끄는 중"}
            for c in it.svc.conflicts:
                o = self.items.get(c)
                if o and o.alive():
                    return {"ok": False, "error": f"{o.svc.label} 가 켜져 있어 같이 못 켬 (같은 모터·같은 노드). 먼저 끄세요."}
            outside = [s for s in self.strays if s["name"] in it.svc.procs]
            if outside:
                return {"ok": False, "error": "대시보드 밖에서 이미 떠 있음: " +
                        ", ".join(f"{s['name']}(pid {s['pid']})" for s in outside)}
            old = dict(it.params)    # 2026-11-25 검토: 거절된 값이 남아 다음 실행(가상 재생 속도 0 → 500)에 쓰였다 → 거절이면 되돌림
            if isinstance(params, dict):
                for k in it.params:
                    if k in params:
                        it.params[k] = str(params[k])
            if self.virtual is not None and it.svc.id in VIRTUAL_NODES:
                try:     # 가상 노드도 실물과 같은 값 검사 (예전엔 건너뛰어 "abc" 는 500·반쯤 켜짐, "-30" 은 관절이 거꾸로 달아남)
                    it.svc.build(dict(it.params, _local=True))
                except ValueError as e:
                    it.params, it.error = old, str(e)
                    return {"ok": False, "error": str(e)}
                return self._start_virtual(it)
            try:
                body = it.svc.build(dict(it.params, _local=self.virtual is not None or not IS_LINUX or FORCE_LOCAL["on"]))
            except ValueError as e:
                it.params, it.error = old, str(e)
                return {"ok": False, "error": str(e)}
            env = dict(os.environ, PYTHONUNBUFFERED="1", RCUTILS_COLORIZED_OUTPUT="0",
                       RCUTILS_LOGGING_BUFFERED_STREAM="0")
            if isinstance(body, str) and self.virtual is not None:
                return self._once_virtual(it, body)
            if isinstance(body, str):
                if not IS_LINUX:
                    return {"ok": False, "error": "bash 명령은 Linux 에서만"}
                body = body.replace("exec ros2", "exec stdbuf -oL -eL ros2")
                script = (_ros_prefix() if it.svc.ros else "") + body
                argv = ["bash", "-lc", script]
                it.cmd = body.strip()
            else:
                argv = body
                it.cmd = " ".join(argv)
            log = open(it.log_path, "ab", buffering=0)
            log.write(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} 켜기: {it.cmd}\n".encode())
            kw = {}
            if IS_LINUX:
                kw["start_new_session"] = True
            else:
                kw["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
            try:
                it.proc = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                           env=env, cwd=str(Path(__file__).resolve().parent.parent), **kw)
            except OSError as e:
                log.close()
                it.error = f"실행 실패: {e}"
                return {"ok": False, "error": it.error}
            log.close()
            it.pid = it.proc.pid
            it.adopted = False
            it.started, it.ended, it.exit_code = time.time(), None, None
            it.stop_note, it.error = "", ""
            self._save_pids()
            return {"ok": True, "msg": f"{it.svc.label} 켬 (pid {it.pid})"}

    # ---------- 끄기 ----------
    def stop(self, sid, wait=False):
        it = self.items.get(sid)
        if it is None:
            return {"ok": False, "error": f"없는 항목 {sid}"}
        if not it.alive():
            return {"ok": True, "msg": f"{it.svc.label} 이미 꺼짐"}
        if it.vm is not None:
            return self._stop_virtual(it)
        if it.stopping:
            return {"ok": True, "msg": "끄는 중"}
        it.stopping = True
        t = threading.Thread(target=self._stop_seq, args=(it,), daemon=True)
        t.start()
        if wait:
            t.join()
        return {"ok": True, "msg": f"{it.svc.label} 끄는 중 (SIGINT)"}

    def _signal(self, it, stage):
        pid = it.pid
        try:
            if IS_LINUX:
                os.killpg(pid, {"INT": signal.SIGINT, "TERM": signal.SIGTERM, "KILL": signal.SIGKILL}[stage])
            else:
                if stage == "INT":
                    os.kill(pid, signal.CTRL_BREAK_EVENT)
                elif it.proc is not None:
                    it.proc.kill()   # Windows: TERM/KILL 둘 다 TerminateProcess
        except (ProcessLookupError, PermissionError, OSError):
            pass

    def _wait_dead(self, it, sec):
        end = time.time() + sec
        while time.time() < end:
            if not it.alive():
                return True
            time.sleep(0.1)
        return not it.alive()

    def _stop_seq(self, it):
        t0 = time.time()
        try:
            self._signal(it, "INT")
            if self._wait_dead(it, SIGINT_WAIT):
                it.stop_note = f"SIGINT 로 끝남 ({time.time() - t0:.1f}초)"
            else:
                self._signal(it, "TERM")
                if self._wait_dead(it, SIGTERM_WAIT):
                    it.stop_note = f"SIGINT 로 {SIGINT_WAIT:g}초 안에 안 끝나 SIGTERM 으로 끝냄"
                else:
                    self._signal(it, "KILL")
                    if self._wait_dead(it, 3):
                        it.stop_note = ("SIGINT·SIGTERM 으로 안 끝나 SIGKILL 로 끝냄 "
                                        "(ak45_node 라면 STATUS.md 의 알려진 문제: 브레이크 0A 프레임이 안 갔을 수 있음)")
                    else:
                        it.stop_note = "SIGKILL 후에도 살아 있음! 터미널에서 pkill -KILL -f <이름> 확인"
            # 그룹에 남은 자식도 정리(bash 가 먼저 죽고 자식이 남는 경우)
            if IS_LINUX:
                try:
                    os.killpg(it.pid, 0)
                    os.killpg(it.pid, signal.SIGKILL)
                    it.stop_note += " / 그룹에 남은 프로세스 SIGKILL"
                except OSError:
                    pass
            self._log(it, f"끄기: {it.stop_note}")
        finally:
            it.stopping = False
            self._reap(it)

    def stop_all(self):
        threads = []
        for it in self.items.values():
            if it.vm is not None and it.alive():
                self._stop_virtual(it)
                threads.append(it.svc.label + "(가상)")
                continue
            if it.alive() and not it.stopping:
                it.stopping = True
                t = threading.Thread(target=self._stop_seq, args=(it,), daemon=True)
                t.start()
                threads.append(it.svc.label)
        return {"ok": True, "msg": ("정지: " + ", ".join(threads)) if threads else "켜진 항목 없음"}

    # ---------- 가상 모드 ----------
    def set_virtual(self, vm):
        """vm = VirtualArm 이면 가상 모드, None 이면 실물. 실물 프로세스가 켜져 있으면 가상으로 못 바꾼다."""
        if vm is not None:
            real = [it.svc.label for it in self.items.values()
                    if it.vm is None and it.alive() and (it.svc.id in VIRTUAL_NODES or it.svc.ros)]
            if real:
                return {"ok": False, "error": "실물 노드가 켜져 있어 가상으로 못 바꿈: " + ", ".join(real) + " — 먼저 끄세요"}
        else:
            for it in self.items.values():
                if it.vm is not None:
                    it.vm.stop_service(it.svc.id)
                    it.vm = None
        self.virtual = vm
        MODE["virtual"] = vm is not None
        (LOG_DIR / "virtual").mkdir(parents=True, exist_ok=True)
        for it in self.items.values():
            if it.proc is None and not it.pid:
                it.started = it.ended = it.exit_code = None
                it.stop_note = it.error = it.cmd = ""
        return {"ok": True}

    def vlog(self, sid, text):
        it = self.items.get(sid)
        if it is not None:
            try:
                with open(it.log_path, "ab") as f:
                    f.write(f"{time.strftime('%H:%M:%S')} {text}\n".encode())
            except OSError:
                pass

    def _start_virtual(self, it):
        vm = self.virtual
        it.cmd = f"(가상) {it.svc.label}: " + ", ".join(VIRTUAL_NODES[it.svc.id]) + " 가상 노드"
        self._log(it, f"켜기: {it.cmd}")
        it.vm = vm
        vm.start_service(it.svc.id, dict(it.params))
        it.started, it.ended, it.exit_code = time.time(), None, None
        it.stop_note, it.error, it.pid, it.proc = "", "", None, None
        return {"ok": True, "msg": f"{it.svc.label} 켬 (가상)"}

    def _stop_virtual(self, it):
        it.vm.stop_service(it.svc.id)
        it.vm = None
        it.stop_note = "가상 노드: 바로 끝남"
        it.ended, it.exit_code = time.time(), 0
        self._log(it, "끄기: 가상 노드 끝남")
        return {"ok": True, "msg": f"{it.svc.label} 끔 (가상)"}

    def _once_virtual(self, it, body):
        it.cmd = "(가상) " + body.strip().splitlines()[0][:120]
        self._log(it, f"켜기: {it.cmd}")
        self.vlog(it.svc.id, "[가상] 실물 버스·스크립트가 없어 할 일 없음. 가상 can0 은 늘 UP 으로 본다.")
        it.started, it.ended, it.exit_code = time.time(), time.time(), 0
        it.stop_note, it.error = "", ""
        self._log(it, "끝남: 종료코드 0 (가상)")
        return {"ok": True, "msg": f"{it.svc.label}: 가상 — 할 일 없음 (종료코드 0)"}

    def restart(self, sid, params=None):
        it = self.items.get(sid)
        if it is None:
            return {"ok": False, "error": f"없는 항목 {sid}"}
        if it.alive():
            self.stop(sid, wait=True)
        return self.start(sid, params)

    # ---------- 감시 ----------
    def _log(self, it, text):
        try:
            with open(it.log_path, "ab") as f:
                f.write(f"===== {time.strftime('%H:%M:%S')} {text}\n".encode())
        except OSError:
            pass

    def _reap(self, it):
        if it.ended is None and it.started is not None and not it.alive():
            it.ended = time.time()
            if it.proc is not None:
                it.exit_code = it.proc.poll()
            self._log(it, f"끝남: 종료코드 {it.exit_code}")
            self._save_pids()

    def _watch(self):
        while True:
            for it in self.items.values():
                if not it.stopping:
                    self._reap(it)
            if IS_LINUX and self.virtual is None:
                self.strays = self._scan_strays()
            elif self.virtual is not None:
                self.strays = []
            time.sleep(1.0)

    def _scan_strays(self):
        """대시보드가 띄우지 않았는데 떠 있는 노드(터미널에서 켠 것)."""
        ours = set()
        for it in self.items.values():
            if it.alive() and it.pid:
                ours.add(it.pid)
        out = []
        for d in Path("/proc").iterdir():
            if not d.name.isdigit():
                continue
            try:
                argv = (d / "cmdline").read_bytes().split(b"\0")
                pgid = os.getpgid(int(d.name))
            except Exception:
                continue
            if pgid in ours:
                continue
            names = {os.path.basename(a.decode("utf-8", "replace")) for a in argv[:3] if a}
            for w in WATCH_PROCS:
                if w in names:
                    out.append({"name": w, "pid": int(d.name),
                                "cmd": b" ".join(argv).decode("utf-8", "replace")[:200]})
        return out

    def _save_pids(self):
        if not IS_LINUX:
            return
        data = {sid: {"pid": it.pid, "start": _proc_start_time(it.pid), "t": it.started, "cmd": it.cmd}
                for sid, it in self.items.items() if it.pid and it.alive()}
        try:
            PID_FILE.write_text(json.dumps(data))
        except OSError:
            pass

    def _adopt(self):
        if not IS_LINUX or not PID_FILE.exists():
            return
        try:
            data = json.loads(PID_FILE.read_text())
        except Exception:
            return
        for sid, d in data.items():
            it = self.items.get(sid)
            if not it or not d.get("pid"):
                continue
            pid = d["pid"]
            if _proc_start_time(pid) != d.get("start"):
                continue   # pid 재사용
            it.pid, it.adopted, it.started, it.cmd = pid, True, d.get("t"), d.get("cmd", "")

    # ---------- 상태 ----------
    def snapshot(self):
        now = time.time()
        out = []
        for it in self.items.values():
            ok, why = it.available()
            alive = it.alive()
            s = it.svc
            out.append({
                "id": s.id, "label": s.label, "kind": s.kind, "group": s.group, "help": s.help, "tab": s.tab,
                "available": ok, "why": why, "alive": alive, "stopping": it.stopping,
                "pid": it.pid if alive else None, "adopted": it.adopted and alive,
                "uptime": (now - it.started) if alive and it.started else None,
                "started": it.started, "ended": it.ended, "exit_code": it.exit_code,
                "stop_note": it.stop_note, "error": it.error, "cmd": it.cmd,
                "params": [{"key": p.key, "label": p.label, "value": it.params.get(p.key, ""), "hint": p.hint}
                           for p in s.params],
                "stop_warn": s.stop_warn, "conflicts": list(s.conflicts),
                "virtual": it.vm is not None or (MODE["virtual"] and not s.demo and s.id != "pick_dry"),
                "log": tail_file(it.log_path, s.tail), "tail": s.tail,
            })
        return out
