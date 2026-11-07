"""Splitting check: the same sim commands through SimToReal's sim2real_node (before) and through
arm_control_node + stm_bridge_node (after) must give the same motor commands, tick by tick.

    python3 tests/check_split.py --original /path/to/SimToReal

No bus and no spinning: the nodes' on_command / on_timer are called directly with the same sequence, and
what they publish (AK45) or send (servo frames) is recorded. A servo angle makes the round trip
degrees -> rad (arm_control) -> degrees (stm_bridge) -> x10 count; a difference of 1 count is
recorded and passes, anything more fails (the user's rule, 2026-11-07).

Also checks the servo range ends (0 deg, 180 deg = pi rad) against stm_bridge's +-0.05 deg tolerance.
"""
import argparse
import random
import sys
import time
from math import degrees, isnan, pi, radians
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "stm_bridge"))

import rclpy
from sensor_msgs.msg import JointState

failures = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f": {detail}" if detail != "" else ""))
    if not ok:
        failures.append(name)


class Recorder:
    """Stands in for ServoCan: keeps the frames."""
    def __init__(self, interface="socketcan", channel="can0"):
        self.frames = []

    def send(self, frame):
        self.frames.append(frame)

    def close(self):
        pass


def msg(names, degs=None, rad=None):
    m = JointState()
    m.name = list(names)
    m.position = list(rad) if rad is not None else [radians(d) for d in degs]
    return m


# (tick, message): valid, clipped, partial, and the three kinds the node must drop whole
SCENARIO = [
    (0, msg(["elbow", "wrist_pitch", "finger1_joint1", "shoulder_roll", "upper_arm_roll"], [-10, 20, 45, 10, 100])),
    (5, msg(["elbow", "nonexistent"], [5, 0])),
    (6, msg(["elbow", "wrist_pitch"], rad=[0.1])),
    (7, msg(["elbow"], rad=[float("nan")])),
    (10, msg(["elbow", "hand_roll", "wrist_yaw"], [120, 150, -100])),
    (200, msg(["elbow", "finger2_joint2", "finger_rotation", "shoulder_yaw", "finger3_joint1"], [30, 120, 0, -5, -45.55])),
    (400, msg(["wrist_pitch", "hand_roll", "finger1_joint2"], [-90, -150, 33.33])),
]
TICKS = 900
FEEDBACK = {"ak45_1": 0.0, "ak45_2": 0.0, "ak45_4": 2.0}      # ak45_3 answers only from tick 40


def run_before(orig_dir, params):
    sys.path.insert(0, str(orig_dir))
    import sim2real.node as old
    old.ServoCan = Recorder
    node = old.Sim2RealNode()
    ak45 = []
    node.ak45.pub.publish = lambda m: ak45.append((list(m.name), list(m.position)))
    events = drive(node, lambda: node.servo.frames, ak45, node.on_command, node.on_timer, node.ak45)
    node.destroy_node()
    return events


def run_after(params):
    import stm_bridge.stm_bridge_node as bridge_mod
    from arm_control.arm_control_node import ArmControlNode
    bridge_mod.ServoCan = Recorder
    node = ArmControlNode()
    bridge = bridge_mod.StmBridgeNode()
    ak45 = []
    node.ak45.pub.publish = lambda m: ak45.append((list(m.name), list(m.position)))
    node.servo.pub.publish = lambda m: bridge.on_command(m)       # /servo/command -> stm_bridge
    events = drive(node, lambda: bridge.servo.frames, ak45, node.on_command, node.on_timer, node.ak45)
    node.destroy_node()
    bridge.destroy_node()
    return events


def drive(node, frames_of, ak45, on_command, on_timer, topics):
    events = []
    script = dict(SCENARIO)
    seen = 0
    for tick in range(TICKS):
        now = time.monotonic()
        for n, d in FEEDBACK.items():
            topics.state[n] = (d, now)
        if tick >= 40:
            topics.state["ak45_3"] = (0.0, now)
        if tick in script:
            on_command(script[tick])
        n_ak = len(ak45)
        on_timer()
        frames = frames_of()
        events.append((ak45[n_ak:], [str(f) for f in frames[seen:]]))
        seen = len(frames)
    return events


def x10(frame):
    return (int(frame[5:7], 16) << 8) | int(frame[7:9], 16)


def compare(before, after):
    ak_bad = 0
    ticks_with_ak = 0
    frames_total = 0
    diffs = {}
    structure_bad = 0
    for t, ((ak_b, fr_b), (ak_a, fr_a)) in enumerate(zip(before, after)):
        ticks_with_ak += bool(ak_b)
        if ak_b != ak_a:
            ak_bad += 1
        if [f[:6] for f in fr_b] != [f[:6] for f in fr_a]:       # same ids, same order
            structure_bad += 1
            continue
        for fb, fa in zip(fr_b, fr_a):
            frames_total += 1
            d = x10(fa) - x10(fb)
            diffs[d] = diffs.get(d, 0) + 1
    return ak_bad, ticks_with_ak, frames_total, diffs, structure_bad


def sweep(bridge_mod, bridge):
    """degrees -> rad -> stm_bridge, for every 0.05 deg of 0..180 and 20000 random angles."""
    from stm_bridge.protocol import servo_frame
    rec = bridge.servo
    angles = [i * 0.05 for i in range(3601)]
    rng = random.Random(7)
    angles += [rng.uniform(0.0, 180.0) for _ in range(20000)]
    dropped, diffs = 0, {}
    for deg in angles:
        n = len(rec.frames)
        bridge.on_command(msg(["W_PITCH"], rad=[radians(deg)]))
        if len(rec.frames) == n:
            dropped += 1
            continue
        want = (servo_frame(9, deg).data[1] << 8) | servo_frame(9, deg).data[2]
        got = (rec.frames[-1].data[1] << 8) | rec.frames[-1].data[2]
        diffs[got - want] = diffs.get(got - want, 0) + 1
    return len(angles), dropped, diffs


def ends(bridge):
    rec = bridge.servo

    def sent(rad):
        n = len(rec.frames)
        bridge.on_command(msg(["F2_2"], rad=[rad]))
        return None if len(rec.frames) == n else rec.frames[-1].data.hex().upper()

    zero, top = "030000", "030708"
    check("0 rad -> 0 deg frame", sent(0.0) == zero, sent(0.0))
    check("pi rad (math.pi) -> 180 deg frame", sent(pi) == top, sent(pi))
    check("radians(180.0) -> 180 deg frame", sent(radians(180.0)) == top, sent(radians(180.0)))
    check("degrees(pi) is 180.0 exactly", degrees(pi) == 180.0, degrees(pi))
    check("-0.049 deg (inside the 0.05 tolerance) is sent as 0", sent(radians(-0.049)) == zero)
    check("180.049 deg (inside the tolerance) is sent as 180", sent(radians(180.049)) == top)
    check("-0.06 deg (outside) is dropped", sent(radians(-0.06)) is None)
    check("180.06 deg (outside) is dropped", sent(radians(180.06)) is None)
    check("-1 deg and 181 deg are dropped", (sent(radians(-1.0)), sent(radians(181.0))) == (None, None))
    check("nextafter(pi) = pi + 4.4e-16 rad is sent as 180", sent(pi + 4.5e-16) == top)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--original", required=True, help="SimToReal checkout (package sim2real)")
    a = ap.parse_args()
    rclpy.init()

    print("== same input, before (sim2real_node) and after (arm_control_node + stm_bridge_node)")
    before = run_before(Path(a.original), {})
    after = run_after({})
    ak_bad, ak_ticks, frames, diffs, structure_bad = compare(before, after)
    n_ak = sum(len(e[0]) for e in before)
    print(f"      {TICKS} ticks, {n_ak} /ak45/command messages on {ak_ticks} ticks, {frames} servo frames (before)")
    check("before and after publish at least something", n_ak > 0 and frames > 0)
    check("/ak45/command: names and positions identical on every tick", ak_bad == 0, f"{ak_bad} ticks differ")
    check("servo frames: same ids in the same order on every tick", structure_bad == 0, f"{structure_bad} ticks differ")
    print(f"      servo x10 count difference (after - before): {dict(sorted(diffs.items()))}")
    check("servo count differences are within 1 count", all(abs(d) <= 1 for d in diffs))
    print("== servo round trip degrees -> rad -> degrees, 0..180 deg")
    from stm_bridge.stm_bridge_node import StmBridgeNode
    import stm_bridge.stm_bridge_node as bridge_mod
    bridge_mod.ServoCan = Recorder
    bridge = StmBridgeNode()
    total, dropped, d2 = sweep(bridge_mod, bridge)
    print(f"      {total} angles, dropped {dropped}, count difference {dict(sorted(d2.items()))}")
    check("no angle in 0..180 deg is dropped", dropped == 0)
    check("count differences are within 1 count", all(abs(d) <= 1 for d in d2))
    print("== the ends of the range")
    ends(bridge)
    bridge.destroy_node()
    rclpy.shutdown()
    print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
