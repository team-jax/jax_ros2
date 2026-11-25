"""팔 재생 중 '지금 무슨 동작' 한 줄 (2026-11-24 사용자: "각 동작 중 지금 뭔 동작을 하는 것인지 알 수 있게 텍스트 표시").

재생하는 쪽(virtual_arm.VirtualArm / real_arm.RealArm 의 play_trajectory)이 시작할 때 phases(traj) 로 단계표를 만들어 play["phases"] 에 두고,
snapshot 이 view(play) 로 지금 단계를 붙인다 → /api/state 의 ros.virtual_state.playback / ros.direct_state.playback:
    phase    "큐브 위로 가는 중" … 끝나면 "끝" / "멈춤 (STOP)" / "멈춤: 이유"
    phase_k  지금 단계 번호 (1 부터), phase_n 단계 수
    left_s   남은 초 (재생 중만)
단계표 = [(시작 프레임, 글), …]. 지금 프레임 f 의 단계 = 시작 프레임 ≤ f 인 마지막 것. 같은 글이 이어지면 하나로 합친다.

어디서 오나 (궤적 종류마다):
  - pick_plan.plan (역기구학 탭 계획, 탑 쌓기 걸음), plan_place_from: marks {경유점 이름: 그 경유점에 닿는 프레임}.
    경유점 k 에 닿는 프레임이 f_k 면 (f_이전, f_k] 동안이 'k 로 가는 중' (MARK_TEXT).
  - pick_plan.plan_home (HOME 버튼): marks (yaw0 / exit_back / home) — 같은 규칙, kind "home" 이면 HOME_TEXT.
  - rl_api.build_traj (강화학습 들기 + 내리기): phase_marks [{frame, text}] 를 직접 준다 (체인 단계 RAISE… + 내리기 CARRY…).
  - 그 밖(옛 궤적 파일): waypoints [{label, frame}] 의 label 로 '→ label', 그것도 없으면 '궤적 재생 중' 하나.
"""
import time

MARK_TEXT = {
    "start": "첫 자세로 가는 중",
    "into_limits": "관절 한계 안으로 들어오는 중",
    "exit_out": "HOME 나가는 중",
    "above_pick": "큐브 위로 가는 중",
    "grasp": "내려가는 중",
    "close": "손가락 닫는 중",
    "closed": "쥐고 기다리는 중",
    "lift": "들어 올리는 중",
    "above_place": "놓을 자리로 옮기는 중",
    "place": "내려놓는 중",
    "open": "손 펴는 중",
    "opened": "손 편 채 기다리는 중",
    "lift2": "손 빼고 올라가는 중",
    "away": "탑에서 멀어지는 중",
    "narrow": "손가락 좁히는 중",
    "exit_back": "HOME 으로 돌아가는 중",
    "home0": "HOME 으로 돌아가는 중",
    "home": "HOME 으로 돌아가는 중",
    "fingers_home": "손가락을 원점(0°)으로",
}
HOME_TEXT = {   # plan_home (HOME 버튼)
    "yaw0": "HOME 으로: 팔 돌림 0 으로 맞추는 중",
    "exit_back": "HOME 으로: 책상 위 경유점으로 가는 중",
    "home": "HOME 으로 들어가는 중",
}
RL_TEXT = {     # rl_runner: 체인(정책) 단계 chain.PHASES + 내리기 PLACE_PHASES
    "RAISE": "팔 들어 큐브 쪽으로 가는 중",
    "SETTLE": "자세 가라앉히는 중",
    "INSERT": "큐브로 손 넣는 중",
    "GRASP_LIFT": "쥐고 들어 올리는 중 (정책)",
    "RETURN": "든 채 돌아오는 중",
    "DONE": "든 채 기다리는 중",
    "CARRY": "놓을 자리로 옮기는 중",
    "LOWER": "내려놓는 중",
    "OPEN": "손 펴는 중",
    "RETREAT": "손 빼고 올라가는 중",
    "HOME": "HOME 으로 돌아가는 중",
}
HOLD_TEXT = "첫 자세 잡는 중"


def _merge(rows):
    out = []
    for f, text in sorted(rows, key=lambda r: r[0]):
        if out and out[-1][1] == text:
            continue
        if out and out[-1][0] == f:
            out[-1] = (f, text)
        else:
            out.append((int(f), text))
    return out


def phases(traj):
    """궤적 → [(시작 프레임, 글), …] (프레임 오름차순, 같은 글 합침). 모르는 꼴이면 [(0, '궤적 재생 중')]."""
    try:
        pm = traj.get("phase_marks")
        if pm:
            return _merge([(int(p["frame"]), str(p["text"])) for p in pm])
        mk = traj.get("marks")
        if isinstance(mk, dict) and mk:
            table = HOME_TEXT if traj.get("kind") == "home" else MARK_TEXT
            level = ((traj.get("real") or {}).get("place_level") or 0)
            rows, prev = [], 0
            for key, f in sorted(mk.items(), key=lambda kv: kv[1]):
                f = int(f)
                if key == "start" and f == 0:
                    continue
                text = table.get(key) or MARK_TEXT.get(key) or f"{key} 로 가는 중"
                if level and key == "above_place":
                    text = f"탑 {level + 1}층 자리로 옮기는 중"
                rows.append((prev, text))
                prev = f
            if rows:
                return _merge(rows)
        wp = traj.get("waypoints")
        if wp:
            rows, prev = [], 0
            for w in sorted(wp, key=lambda w: w["frame"]):
                if int(w["frame"]) == 0:
                    continue
                rows.append((prev, f"→ {w['label']}"))
                prev = int(w["frame"])
            if rows:
                return _merge(rows)
    except (KeyError, TypeError, ValueError):
        pass
    return [(0, "HOME 으로 가는 중" if traj.get("kind") == "home" else "궤적 재생 중")]


def view(play, now=None):
    """play (재생 상태 dict, phases 포함) → 화면에 보낼 dict: phases 표는 빼고 phase·phase_k·phase_n·left_s 를 붙인다."""
    p = dict(play)
    m = p.pop("src_map", None)      # RL 실물 재생: 늦춘 궤적 프레임 → 원래 RL 프레임 (RL 탭 3D 가 따라가는 번호)
    if m:
        p["src_frame"] = int(m[max(0, min(int(p.get("frame") or 0), len(m) - 1))])
    rows = p.pop("phases", None) or []
    if not rows:
        return p
    now = time.time() if now is None else now
    p["phase_n"] = len(rows)
    f = int(p.get("frame") or 0)
    k = max(i for i, (f0, _) in enumerate(rows) if f0 <= f or i == 0)
    p["phase_k"] = k + 1
    if p.get("running"):
        hold_end = (p.get("started") or now) + (p.get("hold") or 0.0)
        p["phase"] = HOLD_TEXT if f == 0 and now < hold_end else rows[k][1]
        left = (p.get("started") or now) + (p.get("seconds") or 0.0) - now
        p["left_s"] = round(max(0.0, left), 1)
    else:
        why = p.get("reason") or ""
        at = rows[k][1]
        if why == "끝까지 재생" or not why:
            p["phase"], p["phase_k"] = "끝", len(rows)
        else:   # 멈춘 단계도 같이 ("멈춤 (STOP) · 그때: 내려가는 중")
            p["phase"] = ("멈춤 (STOP)" if why == "stopped" else f"멈춤: {why}") + f" · 그때: {at}"
        p["left_s"] = 0.0
    return p
