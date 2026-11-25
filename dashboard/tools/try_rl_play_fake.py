#!/usr/bin/env python3
"""강화학습 실물 재생 (server._rl_play_real) 을 가짜 CAN 팔 (fake_can_arm, python-can virtual) 로 끝까지 — 실물·CAN·폰 없음.

  JAX_IK_DIR=~/Desktop/IK .venv/bin/python dashboard/tools/try_rl_play_fake.py

시뮬로 RL 을 한 번 돌린 뒤 (실물 보정 범위·기본 밀어 넣기·숙이기·더 쥐기 그대로) → 거절 경로 · 시작 자세 확인(real_state) · 재생 ·
끝까지, 첫 자세 유지 2 초 · 늦추기 없음(빠른 구간만 늦춘 궤적) · src_frame (RL 탭 3D) · 보정 범위로 크게 잘리는 관절 없음. 실행 기록은 안 남긴다.
"""
import os, shutil, sys, tempfile, time
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]
os.environ.setdefault("JAX_IK_DIR", str(Path.home() / "Desktop" / "IK"))
import server, direct_api, rl_run, play_phase, run_record  # noqa: E402
from fake_can_arm import FakeArm  # noqa: E402
import real_arm as R  # noqa: E402
FAIL = []


def check(n, ok, d=""):
    print(("✓ " if ok else "✗ ") + n + (f" — {d}" if d else ""), flush=True)
    if not ok:
        FAIL.append(n)


def main():
    run_record.REC.on_play = lambda *a, **k: None          # 사용자 logs/runs 에 시험 기록을 안 남김
    server.MODE["virtual"] = False
    server.MODE["ros"] = False
    res = rl_run.run(os.environ["JAX_IK_DIR"], {"x_cm": -3.0, "y_cm": 1.3, "yaw_deg": 20.0, "home": False, "limits_deg": server._rl_real_limits()})
    check("시뮬 RL 성공", res.get("ok") and (res.get("summary") or {}).get("done"), str((res.get("summary") or {}).get("insert_push")))
    rid = res["run_id"]
    # 2026-11-26 '잡기 전에 손목 고정, 손가락으로 확실히 잡기': 실물 재생에서 잡는 자세에 팔을 세워 두고 손가락만 먼저 닫는다
    tq = rl_run.retime(rl_run.real_traj(res, grip=rl_run.grip_params()), rl_run.RL_SPEED_DEFAULT)
    P, names, fg = tq["positions"], tq["joint_names"], tq["phase_frames"]["GRASP_LIFT"]
    fc = rl_run._finger_cols(names)
    arm_c = [i for i in range(len(names)) if i not in fc]
    k0 = tq["src_map"].index(fg)
    still = next((k for k in range(k0, len(P)) if max(abs(P[k][i] - P[k0][i]) for i in arm_c) > 1e-9), len(P)) - k0
    closed = max(P[k0 + still - 1][i] - P[k0][i] for i in fc)
    want = rl_run.RL_GRASP_DWELL_S + rl_run.RL_GRASP_CLOSE_S + rl_run.RL_GRASP_HOLD_S - 0.1
    check("잡을 때 팔 고정 → 손가락 먼저 닫음", still * tq["dt"] >= want and closed > 0.1, f"팔 멈춤 {still * tq['dt']:.2f} s · 손가락 최대 {closed * 57.3:.1f}° 닫힘")
    check("연결 전 → 거절", not server._rl_play_real({"run_id": rid, "ack_table": True}).get("ok"))
    tmp = Path(tempfile.mkdtemp())
    cal = tmp / "calibration.json"
    shutil.copy(R.REPO / "arm_control" / "calibration.json", cal)
    fake = FakeArm("virtual", "fake_rlplay0")
    arm = R.RealArm(calibration_file=cal, interface="virtual", channel="fake_rlplay0", lock_path=None, log=lambda t: None, log_file=str(tmp / "d.log"))
    arm.start()
    time.sleep(1.0)
    direct_api.DIRECT.arm = arm
    try:
        arm.call_trigger("/arm_control/home_all")
        time.sleep(3)
        for body, why in (({"run_id": rid}, "책상 확인 없음"), ({"run_id": "nope", "ack_table": True}, "없는 실행"),
                          ({"run_id": rid, "ack_table": True, "max_speed": "abc"}, "속도 글자"), ({"run_id": rid, "ack_table": True, "max_speed": 500}, "속도 범위 밖")):
            r = server._rl_play_real(body)
            check(f"{why} → 거절", not r.get("ok"), r.get("error", "")[:70])
        st = server.api_rl_real_state(run_id=rid)
        check("real_state: 서보 안 보냄 → can_play False", not st["can_play"])
        t = rl_run.real_traj(res, grip=rl_run.grip_params(None, None), limits=server._rl_real_limits())
        f0 = dict(zip(t["joint_names"], t["positions"][0]))
        arm.send_joint_command(list(f0), list(f0.values()), instant=True)
        time.sleep(4)
        st = server.api_rl_real_state(run_id=rid)
        check("real_state: 첫 프레임으로 보낸 뒤 → can_play (크게 잘리는 관절 없음)", st["can_play"],
              str({k: st.get(k) for k in ("far", "unsent_servos", "clip_big")})[:120])
        check("real_state playback 에 긴 목록 없음", "src_map" not in (st.get("playback") or {}) and "phases" not in (st.get("playback") or {}))
        r = server._rl_play_real({"run_id": rid, "ack_table": True})
        if not r.get("ok") and r.get("need_force"):
            print("  (검사 경고:", r["problems"][:2], ") → force")
            r = server._rl_play_real({"run_id": rid, "ack_table": True, "force": True})
        check("재생 시작 (기본 속도)", r.get("ok"), r.get("error", r.get("msg", ""))[:80])
        t0 = time.time()
        check("첫 자세 2 초 · 늦추기 없음", arm.play.get("hold") == rl_run.RL_HOLD_S and abs(arm.play.get("period", 0) - t["dt"]) < 1e-9,
              f"hold {arm.play.get('hold')} period {arm.play.get('period')}")
        snap = play_phase.view(dict(arm.play))
        check("src_frame 이 나옴 · src_map 은 화면에 안 나감", "src_frame" in snap and "src_map" not in snap)
        while arm.play.get("running"):
            time.sleep(0.3)
        el = time.time() - t0
        check("끝까지 재생 (60 초 안)", arm.play.get("reason") == "끝까지 재생" and el < 60, f"{el:.1f}s · {arm.play.get('reason')}")
    finally:
        try:
            arm.close(brake=False)
        except Exception:
            pass
        direct_api.DIRECT.arm = None
    print("통과" if not FAIL else f"실패 {len(FAIL)}: {FAIL}")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
