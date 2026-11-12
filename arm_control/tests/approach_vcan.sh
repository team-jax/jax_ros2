#!/usr/bin/env bash
# Camera pose -> approach_node -> arm_control_node -> ak45_node / stm_bridge -> vcan, numbers checked.
#
#   tests/approach_vcan.sh          (after colcon build; no sudo: a private network namespace with vcan can0)
#
# The driver plays vision_node (a 1 Hz /object_pose in shoulder_roll_axis); ak45_can's ak45_vcan_sim.py
# plays the four AK45s. Scenarios:
#   A refused: unverified calibration without force; a camera-frame pose
#   B the whole approach with force: 60 Hz, the plan's frames, last CAN frames = the plan's last pose
#   C AK45 id 4 silent: the tracking watch stops the run and nothing more is sent
here="$(cd "$(dirname "$0")" && pwd)"
if [ -z "${IN_NETNS:-}" ]; then
    exec env IN_NETNS=1 unshare -Urn bash "$0" "$@"
fi
set -m
ws="$(cd "$here/../../../.." && pwd)"
sim="$here/../../ak45_can/tools/ak45_vcan_sim.py"
drv="$here/approach_driver.py"
out="$(mktemp -d)"
ip link set lo up
ip link add can0 type vcan && ip link set can0 up || { echo "cannot create vcan can0 (is the vcan module loaded?)"; exit 2; }
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
export ROS_DOMAIN_ID=77

fails=0
check() { if [ "$2" = "1" ]; then echo "PASS  $1"; else echo "FAIL  $1"; fails=$((fails + 1)); fi; }
field() { python3 -c "import json,sys; d=json.load(open('$1')); print($2)"; }

up() {   # $1 = sim ids, $2.. = approach_node params
    local ids="$1"; shift
    : > "$out/dump.txt"
    python3 "$sim" --iface can0 --ids "$ids" > "$out/sim.log" 2>&1 & simpid=$!
    stdbuf -oL candump -L can0 >> "$out/dump.txt" 2>&1 & dumppid=$!
    ros2 launch arm_control arm.launch.py > "$out/launch.log" 2>&1 & launchpid=$!
    ros2 run arm_control approach_node --ros-args -p hold_s:=1.0 "$@" > "$out/node.log" 2>&1 & nodepid=$!
    sleep 6
}
down() {
    kill -INT "$nodepid" 2>/dev/null; kill -INT "$launchpid" 2>/dev/null; sleep 3
    kill -KILL -- -"$launchpid" -"$nodepid" 2>/dev/null
    kill -- -"$simpid" 2>/dev/null; kill "$dumppid" 2>/dev/null; sleep 1
}

echo "== A: refusals"
up 1,2,3,4
python3 "$drv" --out "$out/a1.json" --wait 3 --timeout 3
check "unverified calibration without force: start refused" "$(field "$out/a1.json" "int(d['start']['success'] is False and 'check' in d['start']['message'])")"
python3 "$drv" --out "$out/a2.json" --frame camera --wait 3 --timeout 3
check "camera-frame pose: start refused, says camera_to_robot" "$(field "$out/a2.json" "int(d['start']['success'] is False and 'camera_to_robot' in d['start']['message'])")"
check "nothing sent on /arm_control/joint_command" "$(field "$out/a2.json" "int(not d['commands'])")"
down

echo "== B: the whole approach (force)"
up 1,2,3,4 -p force:=true
python3 "$drv" --out "$out/b.json" --timeout 20
sleep 4          # arm_control_node's 30 deg/s limit finishes walking to the last target
echo "   $(field "$out/b.json" "d['start']['message']")"
check "start accepted" "$(field "$out/b.json" "int(d['start']['success'])")"
check "phases HOLD_FIRST RAISE SETTLE INSERT HOLD" "$(field "$out/b.json" "int([p for _,p in d['phases']][-5:] == ['HOLD_FIRST','RAISE','SETTLE','INSERT','HOLD'])")"
python3 - "$out/b.json" "$out/dump.txt" "$here/.." <<'EOF' > "$out/b_checks.txt"
import json, re, sys
from math import degrees
sys.path[:0] = [sys.argv[3], sys.argv[3] + "/../stm_bridge"]
from arm_control import calibration
from arm_control.approach_plan import plan_from_pose
from arm_control.protocol import ak45_pos_frame
from stm_bridge.protocol import servo_frame
d = json.load(open(sys.argv[1])); dump = open(sys.argv[2]).read()
plan = plan_from_pose((-0.048, -0.285, -0.4475), 20.0, sys.argv[3] + "/model")
cmds = [c for c in d["commands"] if c[0] >= d["start_t"]]
hold = 60    # hold_s 1.0 at 60 Hz
body = cmds[hold:]
def ok(name, cond): print(("PASS  " if cond else "FAIL  ") + name)
ok(f"{len(cmds)} commands = 60 hold + {len(plan.frames)} plan frames", len(cmds) == hold + len(plan.frames))
same = all(c[1] == list(f) and max(abs(a - b) for a, b in zip(c[2], f.values())) < 1e-12 for c, (_, f) in zip(body, plan.frames))
ok("every published frame equals the plan (names and rad)", same and len(body) == len(plan.frames))
span = body[-1][0] - body[0][0]
ok(f"rate: {len(body) - 1} intervals in {span:.2f} s = {(len(body) - 1) / span:.1f} Hz (60 +-5%)", abs((len(body) - 1) / span - 60) < 3)
cal = calibration.load(sys.argv[3] + "/calibration.json")
last = plan.frames[-1][1]
for j in cal.joints:
    if j.joint not in last:
        continue
    m = j.to_motor(degrees(last[j.joint]))[0]
    if j.motor == "ak45":
        want = str(ak45_pos_frame(j.id, m))
        got = re.findall(re.escape(want.split("#")[0]) + r"#[0-9A-F]+", dump)
    else:
        want = str(servo_frame(j.id, m))
        got = re.findall(rf"005#{j.id:02X}[0-9A-F]{{4}}", dump)
    ok(f"{j.joint}: last CAN frame {got[-1] if got else None} = plan end {want} ({m:.2f} motor deg)", bool(got) and got[-1] == want)
ok("finger_rotation (servo 6) never commanded", not re.search(r"005#06", dump))
EOF
cat "$out/b_checks.txt"
fails=$((fails + $(grep -c '^FAIL' "$out/b_checks.txt")))
check "no ERROR/Traceback in approach_node log" "$(grep -qE 'Traceback|\[ERROR\]' "$out/node.log" && echo 0 || echo 1)"
down

echo "== C: AK45 id 4 (elbow) silent -> the watch stops the run"
up 1,2,3 -p force:=true
python3 "$drv" --out "$out/c.json" --timeout 10
check "phase STOPPED" "$(field "$out/c.json" "int(d['final_phase'] == 'STOPPED')")"
check "stopped in about 0.5 s after the hold (watch grace)" "$(field "$out/c.json" "int(len([c for c in d['commands'] if c[0] >= d['start_t']]) < 60 + 60)")"
check "log names ak45_4" "$(grep -q 'stopped: ak45_4' "$out/node.log" && echo 1 || echo 0)"
down

echo "== D: /approach/stop during RAISE"
up 1,2,3,4 -p force:=true
python3 "$drv" --out "$out/d.json" --timeout 10 --stop-after 3
check "stop accepted, phase STOPPED" "$(field "$out/d.json" "int(d['stop']['success'] and d['final_phase'] == 'STOPPED')")"
n1=$(field "$out/d.json" "len(d['commands'])"); sleep 2
check "no frames after the stop (but the one hold command)" "$(field "$out/d.json" "int(d['commands'][-1][0] - d['start_t'] < 3.3)")"
down

[ "$fails" != 0 ] && { echo "---- node.log"; tail -20 "$out/node.log"; echo "---- launch.log"; tail -20 "$out/launch.log"; }
rm -rf "$out"
echo; [ "$fails" = 0 ] && echo "all passed" || echo "$fails failed"
exit $((fails != 0))
