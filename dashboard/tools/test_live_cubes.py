#!/usr/bin/env python3
"""live_api 시험 (폰·GPU 없이): 실시간 인식 스레드 + 가상 장면.

    /usr/bin/python3 dashboard/tools/test_live_cubes.py

1. 가짜 결과 폴더에 그린 프레임을 inference.png (뎁스 색 칸까지 붙인 모양) 로 → 스레드가 큐브 여럿 (ID·층) 을 찾는다, 휴대폰 탭 pick 도.
2. live.jpg 를 쓰기 시작하면 그것으로 바뀐다 (5 Hz).
3. 프레임이 끊기면 1.5초 뒤 '영상 없음' (stale, pick 막음), 다시 오면 풀림.
4. 가상 장면: 놓기·옮기기(위에 쌓기)·지우기 규칙, 재생이 끝까지 가면 옮김, 어긋남 ±1 cm·±5°, 파일에 남음.
5. stack_api.steps 의 걸음 목록 (완료·가려짐·어긋남·기울기·층·떨어짐·겹침).
실패하면 종료코드 1.
"""
import math
import os
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import live_api as L             # noqa: E402
import phone_detect as PD        # noqa: E402
import stack_api as SA           # noqa: E402
import test_phone_detect as T    # noqa: E402

BAD = []


def check(name, ok, detail=""):
    print(("✓ " if ok else "✗ ") + name + (f" — {detail}" if detail else ""))
    if not ok:
        BAD.append(name)


def wait(cond, s):
    t0 = time.time()
    while time.time() - t0 < s:
        if cond():
            return True
        time.sleep(0.05)
    return False


def camera(tmp):
    run = tmp / "run"
    run.mkdir()
    (tmp / "phone_depth.json").write_text('{"run_dir": "%s"}' % run)
    L.LIVE = L.Live(tmp, lambda: "direct")
    cubes = [(0, 0.02, -0.27, 10.0, 0), (1, -0.14, -0.30, 5.0, 0), (2, -0.14, -0.30, 5.0, 1)]
    img = T.render(dict(PD.DEFAULT_CFG), None, (-0.10, -0.80, 0.50), 1250, cubes)
    panel = np.hstack([img, np.zeros((img.shape[0], img.shape[1] + 70, 3), np.uint8)])   # inference.png: [객체 그림 | 뎁스 색 + 눈금 70 px]
    L.LIVE.start()

    def put(name, im):
        tmpf = run / ("tmp_" + name)
        cv2.imwrite(str(tmpf), im)
        os.replace(tmpf, run / name)

    put("inference.png", panel)
    L.LIVE_MAX_LEVELS = 5   # 이 장면은 쌓인 큐브(탑 쌓기 시절) — 층 판정 코드를 계속 시험한다 (실제 서버는 1: 탑 쌓기 없앰)
    L.LIVE.td = 0.4775      # 이 장면은 책상 0.4775 로 그렸다 (서버 기본은 2026-11-25 부터 실물 0.5575)
    ok = wait(lambda: L.LIVE.res is not None and L.LIVE.res.get("src") == "inference.png", 8)
    r = L.cubes(0.4775, None)
    got = {c["id"]: c for c in r["cubes"]}
    good = ok and not r["stale"] and set(got) == {0, 2} and got[2]["level"] == 1 and got[0]["level"] == 0
    check("inference.png (오른쪽 뎁스 칸 잘라 냄) → 큐브 0 (1층) · 2 (2층, 1 은 가려짐)", good,
          ", ".join(f"{i}: ({c['x']:.3f}, {c['y']:.3f}) {c['level'] + 1}층 기울기 {c.get('tilt_deg')}°" for i, c in got.items()) or r.get("reason"))
    if good:
        check("자리 1 cm 안", math.hypot(got[0]["x"] - 0.02, got[0]["y"] + 0.27) < 0.01 and math.hypot(got[2]["x"] + 0.14, got[2]["y"] + 0.30) < 0.01)
    p = r["pick"]
    check("휴대폰 탭 pick = ArUco 0", bool(p.get("ok")) and p.get("used") == "aruco" and abs(p["cube"]["x"] - 0.02) < 0.01, p.get("text") or p.get("reason"))
    p12 = L.cubes(None, 2)["pick"]
    check("pick=2 (2층) → 막음 '쌓인 큐브'", bool(p12.get("ok")) and p12.get("blocked"), p12.get("block"))
    r = L.cubes(0.4775, None)
    check("응답 table.d = 계산에 쓴 d", r["table"]["d"] == 0.4775 and not r.get("td_mismatch"), str(r["table"]))
    L.LIVE.td_done = L.LIVE.td   # 스레드가 다시 계산하기 전 (프레임이 안 바뀌면 다음 _step 에서 다시) — 그 사이 다른 d 로 부름
    with L.LIVE.lock:
        L.LIVE.res = dict(L.LIVE.res, td=0.50)
    r = L.cubes(0.4775, 0)
    check("다른 d (0.50) 로 계산된 결과 → 이 화면 (0.4775) 에는 안 줌 (큐브·pick 없음)", not r["ok"] and r.get("td_mismatch") and not r["cubes"] and not r["pick"]["ok"]
          and r["table"]["d"] == 0.50, r.get("reason"))

    t0, n0 = time.time(), L.LIVE.n
    while time.time() - t0 < 2.0:
        put("live.jpg", img)
        time.sleep(0.2)
    time.sleep(0.3)
    r = L.cubes(None, None)
    check("live.jpg 가 있으면 그것 (5 Hz 로 씀 → 프레임마다 인식)", r["src"] == "live.jpg" and L.LIVE.n - n0 >= 7,
          f"src {r['src']}, 프레임 {L.LIVE.n - n0}개, {r['rate_hz']} Hz, 한계 {r['stale_s']} s")
    time.sleep(L.STALE_S + 0.3)
    r = L.cubes(None, None)
    check("프레임이 1.5초 넘게 없으면 영상 없음 (stale, 목록 비움, pick 막음)", r["stale"] and not r["cubes"] and not r["pick"]["ok"], r["reason"])
    put("live.jpg", img)
    check("새 프레임이 오면 풀림", wait(lambda: not L.cubes(None, None)["stale"], 3))


def scene(tmp):
    S = L.SCENE = L.Scene(tmp / "virtual_scene.json")
    a = S.op({"op": "add", "x": 0.02, "y": -0.27})
    b = S.op({"op": "add", "x": 0.10, "y": -0.21, "yaw_deg": 50})
    c = S.op({"op": "add", "x": 0.02, "y": -0.27})
    cs = S.snapshot()["cubes"]
    check("놓기: 다음 빈 번호 0, 1 · 같은 자리는 거절 · 각은 −45~45", a["ok"] and b["ok"] and not c["ok"] and [k["id"] for k in cs] == [0, 1] and cs[1]["yaw_deg"] == -40.0)
    m = S.op({"op": "move", "id": 1, "x": 0.025, "y": -0.268})
    k = {q["id"]: q for q in S.snapshot()["cubes"]}
    check("다른 큐브 위로 끌면 그 위 (자리·각 맞춤, 2층)", m["ok"] and k[1]["level"] == 1 and k[1]["x"] == 0.02 and k[1]["yaw_deg"] == k[0]["yaw_deg"])
    check("밑의 큐브는 못 옮기고 못 지움", not S.op({"op": "move", "id": 0, "x": 0, "y": -0.3})["ok"] and not S.op({"op": "delete", "id": 0})["ok"])
    S.op({"op": "move", "id": 1, "x": 0.10, "y": -0.21})
    end = S.on_play({"real": {"cube": [0.10, -0.21, 0.0], "target": [-0.16, -0.28], "target_yaw": 30.0}})
    end("stopped")
    check("재생이 중간에 멈추면 안 옮김", next(q for q in S.snapshot()["cubes"] if q["id"] == 1)["x"] == 0.10)
    end("끝까지 재생")
    q = next(q for q in S.snapshot()["cubes"] if q["id"] == 1)
    check("끝까지 재생 → 놓을 자리로 (각 = target_yaw)", (q["x"], q["y"], q["yaw_deg"], q["level"]) == (-0.16, -0.28, 30.0, 0), S.snapshot()["last_move"]["text"])
    S.op({"op": "jitter", "on": True})
    S.on_play({"real": {"cube": [0.02, -0.27, 10.0], "target": [-0.16, -0.28], "target_yaw": 30.0}})("끝까지 재생")
    q = next(q for q in S.snapshot()["cubes"] if q["id"] == 0)
    dx, dy, dyaw = q["x"] + 0.16, q["y"] + 0.28, q["yaw_deg"] - 30.0
    check("어긋남 켬 → 큐브 1 위 (2층), ±1 cm · ±5°", q["level"] == 1 and abs(dx) <= 0.0101 and abs(dy) <= 0.0101 and abs(dyaw) <= 5.01,
          S.snapshot()["last_move"]["text"])
    S2 = L.Scene(tmp / "virtual_scene.json")
    check("파일에 남음 (서버를 다시 켜도)", S2.snapshot()["cubes"] == S.snapshot()["cubes"] and S2.jitter)
    # 탑 쌓기 걸음 목록 (상태 없음: 지금 보이는 것 + 순서·탑 자리만)
    cubes = [{"id": 1, "x": -0.16, "y": -0.28, "yaw_deg": 30.0, "level": 0}, {"id": 0, "x": -0.152, "y": -0.284, "yaw_deg": 33.0, "level": 1},
             {"id": 2, "x": 0.10, "y": -0.21, "yaw_deg": 0.0, "level": 0}]
    base, order = {"x": -0.16, "y": -0.28, "yaw_deg": 30.0}, [1, 0, 2]
    st = lambda r: [q["state"] for q in r["steps"]]   # noqa: E731
    r = SA.steps([dict(cubes[0], x=0.02, y=-0.27), dict(cubes[1], level=0, x=0.10, y=-0.36), cubes[2]], base, order)
    check("steps: 처음 → 1층 걸음만 누를 수 있음 (탑 자리 바닥)", st(r) == ["next", "wait", "wait"] and r["next"]["target"]["x"] == -0.16, r["text"])
    r = SA.steps(cubes, base, order)
    check("steps: 9 mm 어긋난 큐브 0 → 1·2층 완료, 3층은 실제 큐브 0 위 (각 33°)",
          st(r) == ["done", "done", "next"] and r["next"]["level"] == 2 and r["next"]["target"]["x"] == -0.152 and r["next"]["target"]["yaw_deg"] == 33.0, r["text"])
    r = SA.steps(cubes[1:], base, order)
    check("steps: 카메라는 맨 위만 (11 가려짐) → 1층 완료 (가려짐)", st(r) == ["done", "done", "next"], r["steps"][0]["why"])
    r = SA.steps([cubes[0], dict(cubes[1], x=-0.13), cubes[2]], base, order)
    check("steps: 3 cm 어긋남 → 2층 문제 (이유), 3층 막음", st(r) == ["done", "bad", "blocked"] and "어긋남" in r["steps"][1]["why"] and "어긋남" in r["steps"][2]["why"], r["text"])
    r = SA.steps([cubes[0], dict(cubes[1], tilt_deg=25.0), cubes[2]], base, order)
    check("steps: 25° 기울어짐 → 막음", st(r) == ["done", "bad", "blocked"] and "기울어짐" in r["text"], r["text"])
    r = SA.steps([cubes[0], dict(cubes[1], level=2), cubes[2]], base, order, {"0": 1, "1": 0})
    check("steps: 기록과 다른 층 (2층에 놓았는데 3층으로 보임) → 막음", "bad" in st(r) and "next" not in st(r) and r.get("bad"), r["text"])
    r = SA.steps([cubes[0], dict(cubes[1], level=0, x=0.0, y=-0.40), cubes[2]], base, order)
    check("steps: 2층이 책상으로 떨어짐 → 2층 걸음을 다시 누를 수 있음", st(r) == ["done", "next", "wait"], r["text"])
    r = SA.steps([cubes[0], dict(cubes[1], level=0, x=-0.12, y=-0.30), cubes[2]], base, order)
    check("steps: 탑 옆 (8 cm 안) 으로 떨어짐 → 겹침으로 막음", st(r) == ["done", "blocked", "wait"] and "겹침" in r["text"], r["text"])
    r = SA.steps(cubes[:2] + [{"id": 2, "x": -0.152, "y": -0.284, "yaw_deg": 33.0, "level": 2}], base, order)
    check("steps: 다 쌓임 → 완료 3개", st(r) == ["done", "done", "done"] and r.get("done"), r["text"])
    r = SA.steps(cubes[:2] + [{"id": 3, "x": -0.152, "y": -0.284, "yaw_deg": 33.0, "level": 2}], base, order)
    check("steps: 순서에 없는 큐브가 3층에 → 그래도 높이를 믿음 (완료)", st(r) == ["done", "done", "done"], r["text"])
    one = [cubes[0], dict(cubes[2])]   # 책상 위에 큐브 하나씩 (사용자): 탑 1 + 2 만
    r = SA.steps(one, base, [1, 0])
    check("steps: 순서의 0 이 없고 2 만 보임 → '2층: 보이는 큐브 2번'", st(r) == ["done", "next"] and r["steps"][1]["text"] == "2층: 보이는 큐브 2번 옮기기", r["text"])
    r = SA.steps(one, base, [])
    check("steps: 순서 비움 → 보이는 큐브 아무거나, 3층까지", st(r) == ["done", "next", "wait"] and r["steps"][1]["text"] == "2층: 보이는 큐브 2번 옮기기", r["text"])
    r = SA.steps(cubes[:2] + [dict(cubes[2], unsure=True)], base, order)
    check("steps: 집을 큐브가 unsure → 3층 막음", st(r) == ["done", "done", "blocked"] and "확실하지" in r["text"], r["text"])
    r = SA.steps([cubes[0], dict(cubes[1], unsure=True), cubes[2]], base, order)
    check("steps: 탑 맨 위가 unsure → 막음", "next" not in st(r) and "확실하지" in r["text"], r["text"])
    r = SA.steps(cubes, base, order, None, True)
    check("steps: level_unsure 프레임 → 막음", "next" not in st(r) and "층이 확실하지" in r["text"], r["text"])
    r = SA.steps([cubes[1]], base, order, {"0": 1})
    check("steps: 탑만 보이고 책상 위 큐브 없음 → 3층 막음 (이유), 1층 번호는 기록에서", st(r) == ["done", "done", "blocked"] and r["steps"][0]["id"] == 1, r["text"])


def main():
    tmp = Path(tempfile.mkdtemp(prefix="live_test"))
    camera(tmp)
    scene(tmp)
    print("통과" if not BAD else f"실패 {len(BAD)}: {BAD}")
    sys.exit(1 if BAD else 0)


if __name__ == "__main__":
    main()
