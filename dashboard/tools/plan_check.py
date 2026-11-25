"""계획 회귀 검사: 지도(graspmap)의 초록 칸 둘을 무작위로 골라 pick_plan.plan 을 N 번 돌려 'playable' 비율을 본다.

    python dashboard/tools/plan_check.py gm.json 30 1          # gm.json = GET /api/ik/graspmap 저장본, N, seed
    python dashboard/tools/plan_check.py --make-map gm.json    # 지도를 이 프로세스에서 계산해 저장 (서버 없이)
    python dashboard/tools/plan_check.py --make-map gm_045.json --d 0.45   # 다른 책상 높이 (어깨 축 아래 m) — 계획도 그 지도의 d 로
    python dashboard/tools/plan_check.py --make-map gm_v.json --grasp vertical              # 집는 방향 (auto / vertical / horizontal) — 계획도 그 지도의 방향으로
    python dashboard/tools/plan_check.py --make-map gm_h.json --grasp horizontal --yaw 0    # 수평은 큐브 yaw 하나의 지도 (계획도 그 yaw 로)

한 줄씩 JSON (큐브, yaw, 놓을 자리, 상태, 이유, 시간, 주요 검사값), 끝에 상태별 개수. 결과는 gm.json 옆 plan_check_<seed>.json.
yaw 는 (0, 20, 45, -30, 70) 중 무작위 (큐브는 90° 마다 같아 이것으로 0~90 을 고루 본다).
"""
import argparse
import collections
import json
import os
import random
import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import pick_plan   # noqa: E402


def make_map(path, d=None, parts=None, grasp="auto", yaw=None):
    import hand_ik
    s = hand_ik._Solver(str(pick_plan._find_ik(None)))
    s.set_table_d(d)
    ranges = pick_plan._cmd_ranges(pick_plan.REPO / "arm_control" / "calibration.json")
    x0, x1, y0, y1 = hand_ik.MAP_BOX
    r = s.graspmap({"ranges_deg": ranges, "shoulder_yaw": 0.0, "step": 0.01, "d": d, "parts": parts, "grasp": grasp, "yaw": yaw, "x0": x0, "x1": x1, "y0": y0, "y1": y1})
    Path(path).write_text(json.dumps(r), encoding="utf-8")
    print(f"map: grasp {r['grasp']} yaw {r['yaw']} {r['counts']} {r['ms']} ms -> {path}")


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument("gm")
    ap.add_argument("n", nargs="?", type=int, default=30)
    ap.add_argument("seed", nargs="?", type=int, default=1)
    ap.add_argument("--make-map", action="store_true")
    ap.add_argument("--d", type=float, default=None, help="책상 높이 (어깨 축 아래 m). 비우면 지도 파일의 d, 그것도 없으면 기본 0.4775")
    ap.add_argument("--parts", type=int, default=None, help="--make-map: 지도를 나눠 푸는 프로세스 수 (기본 hand_ik.MAP_PARTS)")
    ap.add_argument("--grasp", choices=pick_plan.GRASP_MODES, default=None, help="집는 방향. 비우면 지도 파일의 grasp, 그것도 없으면 auto")
    ap.add_argument("--yaw", type=float, default=None, help="--make-map: 이 큐브 yaw (실좌표 °) 만 본 지도 (수평용). 계획도 그 yaw 로")
    a = ap.parse_args()
    if a.make_map:
        return make_map(a.gm, a.d, a.parts, a.grasp or "auto", a.yaw)
    gm = json.loads(Path(a.gm).read_text(encoding="utf-8"))
    d = a.d if a.d is not None else (gm.get("d") if abs((gm.get("d") or 0.4775) - 0.4775) > 1e-9 else None)   # 기본 높이면 None (예전과 같은 호출)
    grasp = a.grasp or gm.get("grasp") or "auto"
    yaws = [gm["yaw"]] if gm.get("yaw") is not None else [0, 20, 45, -30, 70]   # yaw 하나로 만든 지도면 그 yaw 로만
    green = [(x, y) for y, r in zip(gm["ys"], gm["rows"]) for x, c in zip(gm["xs"], r) if c == "2"]
    random.seed(a.seed)
    out = []
    for _ in range(a.n):
        c = random.choice(green); t = random.choice(green); yaw = random.choice(yaws)
        ns = argparse.Namespace(cube=[c[0], c[1], yaw], target=list(t), target_yaw=None, d=d, ik=None, calibration=None, max_speed=30.0, grasp=grasp, pick_drop=0.0)   # 지도(낮춤 없음)와 같은 조건
        t0 = time.time()
        try:
            r = pick_plan.plan(ns)
            row = {"cube": c, "yaw": yaw, "target": t, "status": r["status"], "reason": r.get("reason"), "s": round(time.time() - t0, 1),
                   "checks": {k: r.get("checks", {}).get(k) for k in ("lowest_finger_above_table_mm", "joint_space_fallback",
                                                                       "carry_cube_bottom_min_above_table_mm", "approach")},
                   "n": len(r.get("positions") or []), "tilt": (r.get("sim") or {}).get("hand_tilt_deg"), "grasp": (r.get("grasp") or {}).get("text")}
        except Exception:
            row = {"cube": c, "yaw": yaw, "target": t, "status": "EXC", "reason": traceback.format_exc()[-600:], "s": round(time.time() - t0, 1)}
        out.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    cnt = collections.Counter(r["status"] for r in out)
    print(f"summary: playable {cnt['playable']}/{len(out)} ({100 * cnt['playable'] / max(1, len(out)):.0f}%) · " +
          ", ".join(f"{k} {v}" for k, v in cnt.most_common()), flush=True)
    Path(a.gm).with_name(f"plan_check_{a.seed}.json").write_text(json.dumps(out, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    main()
