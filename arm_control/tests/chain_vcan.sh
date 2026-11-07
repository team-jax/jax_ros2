#!/usr/bin/env bash
# arm_control_node -> /servo/command -> stm_bridge_node -> vcan0, with a fake ak45_node, at 50 Hz:
# the number of frames candump sees must equal the number of servo joints arm_control_node published.
# Needs vcan0 (sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0), python3-can, can-utils.
#   tests/chain_vcan.sh [vcan0]
# pgrep before and after: nothing of ours may be left running.
here="$(cd "$(dirname "$0")" && pwd)"
chan="${1:-vcan0}"
out="$(mktemp -d)"
source /opt/ros/jazzy/setup.bash
source "$(cd "$here/../../../.." && pwd)/install/setup.bash"

echo "== pgrep before"; pgrep -fa 'stm_bridge_node|ak45_node|arm_control_node' || echo "(none)"

pids=()
start() { setsid "$@" > "$out/$1.$$.$RANDOM.log" 2>&1 & pids+=($!); }
start ros2 run stm_bridge stm_bridge_node --ros-args -p servo_channel:="$chan"
start python3 "$here/fake_ak45.py"
start ros2 run arm_control arm_control_node
timeout 60 stdbuf -oL candump -L "$chan" > "$out/dump.txt" 2>&1 &
cd_pid=$!
sleep 4
python3 "$here/chain_load.py" > "$out/load.json" 2> "$out/load.err"
sleep 1
kill "$cd_pid" 2>/dev/null
cat "$out/load.json"

python3 - "$out" <<'PY'
import json, sys
out = sys.argv[1]
load = json.loads(open(out + "/load.json").read().strip().splitlines()[-1])
frames = [l.split()[-1] for l in open(out + "/dump.txt") if "#" in l]
sent = load["servo_joints_published"]
print(f"servo joints published on /servo/command: {sent} in {load['servo_messages']} messages")
print(f"frames on the bus (candump):             {len(frames)}")
print(f"/servo/state messages:                   {load['servo_state_messages']}")
ok = len(frames) == sent and sent > 0
print("PASS" if ok else "FAIL", " frame count == published count")
last = {}
for f in frames:
    data = f.split("#")[1]
    last[int(data[:2], 16)] = int(data[2:6], 16)
want = {}
import re
ids = {"W_PITCH": 9, "W_YAW": 8, "W_X": 7, "F1_1": 0, "F1_2": 1, "F2_1": 2}
for name, x10 in load["final_x10"].items():
    want[ids[name]] = x10
print("PASS" if last == want else "FAIL", " last frame of every servo is its goal:", last, "want", want)
sys.exit(0 if ok and last == want else 1)
PY
rc=$?

for p in "${pids[@]}"; do kill -- -"$p" 2>/dev/null; done
sleep 2
echo "== pgrep after"; pgrep -fa 'stm_bridge_node|ak45_node|arm_control_node' || echo "(none)"
rm -rf "$out"
exit $rc
