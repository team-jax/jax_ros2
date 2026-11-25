"""역기구학 탭 2D 지도: team-jax/IK 의 도달 맵(reachmap_real*.npz)과 판정(realcoords.plan)을 실좌표로 준다.

realcoords.py 는 numpy 만 쓴다(패키지 __init__ 은 MuJoCo 를 부르므로 파일을 직접 읽는다).
실좌표: 원점 = 어깨 roll 축, x = 로봇 왼쪽 +, y = 뒤 + (앞 = −y), m. 칸 중심 = realcoords 의 sim 그리드를 되돌린 것
(x_real = 0.052 − y_sim, y_real = x_sim − 0.34 — 상수는 realcoords.ORIGIN_XY 에서 읽는다).
"""
import importlib.util
import math
from pathlib import Path

SUB = Path("motion1") / "src" / "humanoid_grasp_rl"
_cache = {"key": None, "rc": None, "base": None, "yaw": None}


def _load(ik_dir):
    """(realcoords 모듈, 기본 맵 dict, yaw 맵 dict). 파일이 바뀌면 다시 읽는다. 없으면 FileNotFoundError."""
    import numpy as np
    if ik_dir is None:
        raise FileNotFoundError("team-jax/IK 를 못 찾음 (JAX_IK_DIR)")
    d = Path(ik_dir) / SUB
    files = [d / "realcoords.py", d / "reachmap_real.npz", d / "reachmap_real_yaw.npz"]
    for f in files:
        if not f.is_file():
            raise FileNotFoundError(f"없음: {f}")
    key = tuple((str(f), f.stat().st_mtime) for f in files)
    if _cache["key"] != key:
        spec = importlib.util.spec_from_file_location("_jax_realcoords", files[0])
        rc = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(rc)
        with np.load(files[1]) as z:
            base = {k: z[k] for k in ("xs", "ys", "grasp", "place", "feasible")}
        with np.load(files[2]) as z:
            yaw = {k: z[k] for k in ("yaws", "offsets", "xs", "ys", "grasp", "place", "feasible")}
            yaw["by_flip"] = "feasible_by_flip" in z.files   # 파지(손 그대로/반 바퀴)별 맵이 있나
        _cache.update(key=key, rc=rc, base=base, yaw=yaw)
    return _cache["rc"], _cache["base"], _cache["yaw"]


def _real_xy(rc, xs, ys):
    """sim 그리드 (xs[i], ys[j]) -> 실좌표 칸 중심: to_sim 의 역(−90° 회전 + 원점)."""
    ox, oy = rc.ORIGIN_XY
    c, s = math.cos(math.radians(rc.SIM_FROM_REAL_YAW)), math.sin(math.radians(rc.SIM_FROM_REAL_YAW))
    out = []
    for i, xv in enumerate(xs):
        for j, yv in enumerate(ys):
            dx, dy = float(xv) - ox, float(yv) - oy
            out.append((i, j, c * dx + s * dy, -s * dx + c * dy))   # R(-yaw) · (sim - origin)
    return out


def _yaw_masks(rc, ym, yaw_real):
    """실 yaw 하나에 대해 (sim yaw, 고른 맵 yaw, 맵 yaw 번호, 파지 목록 [(손 돌림, flip, feasible[i,j])]) —
    realcoords.grasp_maps 와 같은 yaw·손 돌림·손 모양(그대로/반 바퀴) 고르기."""
    yaw_sim = rc.fold_yaw(yaw_real + rc.SIM_FROM_REAL_YAW)
    yaws = ym["yaws"]
    k = min(range(len(yaws)), key=lambda q: abs(rc.fold_yaw(float(yaws[q]) - yaw_sim)))
    _, _, maps = rc.grasp_maps(yaw_sim)
    return yaw_sim, float(yaws[k]), k, maps


def _gkey(turn, flip):
    return {"hand_yaw": float(turn), "flip": flip}


def reachmap(ik_dir, yaw_real=0.0):
    """GET /api/reachmap?yaw= : 칸 목록(실좌표) + 그 큐브 yaw 에서의 grasp/place/feasible + 칸마다 되는 파지.

    grasps = 그 칸에서 집기·놓기가 다 되는 파지 [{hand_yaw, flip}] (flip = 손 반 바퀴). 한 배달은 한 파지로
    끝까지 가므로 큐브 칸과 놓을 자리 칸에 같은 파지가 있어야 됨(realcoords.plan)."""
    try:
        rc, base, ym = _load(ik_dir)
    except FileNotFoundError as e:
        return {"ok": False, "error": f"도달 맵 없음 — {e}"}
    except Exception as e:
        return {"ok": False, "error": f"도달 맵을 못 읽음: {type(e).__name__}: {e}"}
    if not hasattr(rc, "grasp_maps"):
        return {"ok": False, "error": "realcoords.py 가 옛 버전 (grasp_maps 없음) — team-jax/IK handroll-limit 최신으로"}
    yaw_sim, map_yaw, k, maps = _yaw_masks(rc, ym, float(yaw_real))
    turns_all = sorted({t for t, _, _ in maps}, key=abs)
    os_ = {t: min(range(len(ym["offsets"])), key=lambda q: abs(float(ym["offsets"][q]) - (t - (map_yaw + rc.fold_yaw(yaw_sim - map_yaw)))))
           for t in turns_all}
    by_flip = bool(ym["by_flip"])
    cells = []
    for i, j, x, y in _real_xy(rc, ym["xs"], ym["ys"]):
        g = any(bool(ym["grasp"][k, os_[t], i, j]) for t in turns_all)
        p = any(bool(ym["place"][k, os_[t], i, j]) for t in turns_all)
        grasps = [_gkey(t, f) for t, f, m in maps if m[i, j]]
        turns = sorted({gg["hand_yaw"] for gg in grasps}, key=abs)
        cells.append({"x": round(x, 4), "y": round(y, 4), "grasp": g, "place": p, "feasible": bool(grasps),
                      "turns": turns, "grasps": grasps,
                      "plain": any(gg["flip"] is False for gg in grasps), "flip": any(gg["flip"] is True for gg in grasps),
                      "base": bool(base["feasible"][i, j])})
    xs, ys = ym["xs"], ym["ys"]
    return {"ok": True, "yaw_real": float(yaw_real), "yaw_sim": yaw_sim, "map_yaw_sim": map_yaw,
            "hand_turns": turns_all, "by_flip": by_flip,
            "cell": {"dx": abs(float(ys[1] - ys[0])), "dy": abs(float(xs[1] - xs[0]))},   # 실 x 폭 = sim y 간격
            "d": rc.MAP_D, "cube_size": rc.CUBE_SIZE, "cells": cells,
            "files": [k_[0] for k_ in _cache["key"]],
            "note": "feasible = 어떤 파지로든 집기·놓기 다 되고 상하좌우 칸도 됨. grasps = 그 칸이 되는 파지(flip = 손 반 바퀴). "
                    "배달은 큐브 칸과 놓을 자리 칸에 같은 파지가 있어야 됨(realcoords.plan). 칸 판정은 가장 가까운 칸, 사이 보간 없음."}


def check(ik_dir, body):
    """POST /api/reachmap/check {cube:[x,y,yaw], target:[x,y], d?} -> realcoords.plan() 결과 + 큐브·놓을 자리 따로 판정."""
    try:
        rc, _, _ = _load(ik_dir)
    except FileNotFoundError as e:
        return {"ok": False, "error": f"도달 맵 없음 — {e}"}
    except Exception as e:
        return {"ok": False, "error": f"도달 맵을 못 읽음: {type(e).__name__}: {e}"}
    try:
        cube = [float(v) for v in body["cube"]][:3]
        target = [float(v) for v in body["target"]][:2]
        if len(cube) == 2:
            cube.append(0.0)
        d = body.get("d")
        d = rc.MAP_D if d in (None, "") else float(d)
        if len(cube) != 3 or len(target) != 2 or not all(math.isfinite(v) for v in cube + target + [d]):
            raise ValueError
    except (KeyError, TypeError, ValueError):
        return {"ok": False, "error": "cube: [x, y, yaw], target: [x, y] 숫자"}
    if not hasattr(rc, "grasps_for"):   # reachmap() 와 같은 가드: IK 레포가 옛 버전이면 서버가 500 으로 죽지 말고 이유를 화면에
        return {"ok": False, "error": "realcoords.py 가 옛 버전 (grasps_for 없음) — team-jax/IK handroll-limit 최신으로. "
                "지도 판정은 '옛 맵' 을 끄면 실제 IK(/api/ik/pair)로 한다"}
    try:
        r = rc.plan(tuple(cube), tuple(target), d)
    except Exception as e:
        return {"ok": False, "error": f"realcoords.plan 실패: {type(e).__name__}: {e}"}
    out = {k: ([round(float(x), 4) for x in v] if hasattr(v, "tolist") else v) for k, v in r.items()}
    yaw_sim = out.get("yaw", rc.fold_yaw(cube[2] + rc.SIM_FROM_REAL_YAW))
    one = {}
    for name, xy in (("cube", out["cube"][:2]), ("target", out["target"][:2])):
        g = rc.grasps_for([xy], yaw_sim)
        one[name] = {"reachable": bool(g), "hand_yaw": g[0]["hand_yaw"] if g else None, "grasps": g or [],
                     "reason": "outside the reach map's grid" if g is None else ("in the reach band" if g else "no grasp serves it")}
    out.update(ok=True, alone=one, d=d, map_d=rc.MAP_D, input={"cube": cube, "target": target})
    return out
