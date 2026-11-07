#!/usr/bin/env bash
# NODES.md section 4 (the topic table) against the running system: type, who publishes, who subscribes,
# and what the messages carry. Whole launch + vision_node on a vcan can0 in a private network namespace
# (unshare -Urn, no sudo), the AK45s played by ak45_can/tools/ak45_vcan_sim.py.
#   tests/check_topics.sh
here="$(cd "$(dirname "$0")" && pwd)"
if [ -z "${IN_NETNS:-}" ]; then
    exec env IN_NETNS=1 unshare -Urn bash "$0" "$@"
fi
set -m
ws="$(cd "$here/../../../.." && pwd)"
sim="$here/../../ak45_can/tools/ak45_vcan_sim.py"
out="$(mktemp -d)"
ip link set lo up
ip link add can0 type vcan && ip link set can0 up || { echo "cannot create vcan can0"; exit 2; }
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"

fails=0
check() { if [ "$2" = "1" ]; then echo "PASS  $1"; else echo "FAIL  $1  $3"; fails=$((fails + 1)); fi; }

python3 "$sim" --iface can0 --ids 1,2,3,4 > "$out/sim.log" 2>&1 &
simpid=$!
ros2 launch arm_control arm.launch.py max_speed_deg_s:=300.0 > "$out/launch.log" 2>&1 &
launchpid=$!
ros2 run vision_node vision_node --ros-args -p fixed_pose:="[0.10, -0.30, 20.0]" > "$out/vision.log" 2>&1 &
visionpid=$!
sleep 8

# ---- who publishes / subscribes, and the type (ros2 topic info -v)
# usage: topic_row TOPIC  ->  "type|pubs|subs"  (node names, sorted, comma-separated)
topic_row() {
    ros2 topic info -v "$1" 2>/dev/null | python3 -c '
import sys, re
text = sys.stdin.read()
typ = re.search(r"Type: (\S+)", text)
def nodes(kind):
    block = re.search(kind + r" count: \d+(.*?)(?=\n(?:Publisher|Subscription) count:|\Z)", text, re.S)
    return sorted(set(re.findall(r"Node name: (\S+)", block.group(1)))) if block else []
print("%s|%s|%s" % (typ.group(1) if typ else "-", ",".join(nodes("Publisher")), ",".join(nodes("Subscription"))))
'
}
# TOPIC TYPE PUBLISHERS SUBSCRIBERS (what NODES.md section 4 says; subscribers may include others, e.g. a tool)
table() {
    cat <<'EOF'
/object_pose|geometry_msgs/msg/PoseStamped|vision_node|arm_control_node
/arm_control/joint_command|sensor_msgs/msg/JointState||arm_control_node
/ak45/command|sensor_msgs/msg/JointState|arm_control_node|ak45_node
/servo/command|sensor_msgs/msg/JointState|arm_control_node|stm_bridge
/joint_states|sensor_msgs/msg/JointState|ak45_node|arm_control_node
/servo/state|sensor_msgs/msg/JointState|stm_bridge|
/diagnostics|diagnostic_msgs/msg/DiagnosticArray|ak45_node|
EOF
}
echo "== NODES.md section 4: type, publisher, subscriber"
while IFS='|' read -r topic want_type want_pub want_sub; do
    row="$(topic_row "$topic")"
    got_type="${row%%|*}"; rest="${row#*|}"; got_pub="${rest%%|*}"; got_sub="${rest#*|}"
    check "$topic: type $want_type" "$([ "$got_type" = "$want_type" ] && echo 1 || echo 0)" "got $got_type"
    check "$topic: published by '${want_pub:-nobody}'" "$([ "$got_pub" = "$want_pub" ] && echo 1 || echo 0)" "got '$got_pub'"
    if [ -n "$want_sub" ]; then
        check "$topic: subscribed by $want_sub" "$(echo ",$got_sub," | grep -q ",$want_sub," && echo 1 || echo 0)" "got '$got_sub'"
    fi
done < <(table)
check "/pressure does not exist yet (NODES.md says: later)" "$(ros2 topic list 2>/dev/null | grep -qx /pressure && echo 0 || echo 1)"

# ---- what the messages carry
echo "== message contents"
timeout 20 ros2 topic echo --once /object_pose > "$out/pose.txt" 2>&1
check "/object_pose frame_id is shoulder_roll_axis" "$(grep -q "frame_id: shoulder_roll_axis" "$out/pose.txt" && echo 1 || echo 0)"
check "/object_pose position x 0.1, y -0.3 (m), z 0.0 (unused)" \
    "$(python3 - "$out/pose.txt" <<'PY'
import re, sys
t = open(sys.argv[1]).read()
v = lambda k, blk: float(re.search(blk + r".*?\n\s+" + k + r": (\S+)", t, re.S).group(1))
print(1 if (v("x", "position"), v("y", "position"), v("z", "position")) == (0.1, -0.3, 0.0) else 0)
PY
)"
check "/object_pose rotation is a z-axis yaw only: x = y = 0, 20 deg" \
    "$(python3 - "$out/pose.txt" <<'PY'
import re, sys
from math import atan2, degrees
t = open(sys.argv[1]).read()
o = t[t.index("orientation"):]
q = {k: float(re.search(r"\n\s+" + k + r": (\S+)", o).group(1)) for k in "xyzw"}
print(1 if q["x"] == 0 and q["y"] == 0 and round(degrees(2 * atan2(q["z"], q["w"])), 6) == 20.0 else 0)
PY
)"

timeout 25 ros2 topic echo --once /ak45/command > "$out/ak45cmd.txt" 2>&1 &
e1=$!
timeout 25 ros2 topic echo --once /servo/command > "$out/servocmd.txt" 2>&1 &
e2=$!
timeout 25 ros2 topic echo --once /servo/state > "$out/servostate.txt" 2>&1 &
e3=$!
sleep 2
# elbow -10 deg (AK45 4), wrist_pitch 20 deg -> servo motor 90 + 20 = 110 deg
timeout 20 ros2 topic pub -1 --wait-matching-subscriptions 1 /arm_control/joint_command sensor_msgs/msg/JointState \
    "{name: [elbow, wrist_pitch], position: [-0.17453292519943295, 0.3490658503988659]}" > /dev/null 2>&1
wait $e1 $e2 $e3 2>/dev/null
nm() { python3 - "$1" <<'PY'
import re, sys
t = open(sys.argv[1]).read()
names = re.findall(r"- (\S+)", t.split("position:")[0].split("name:")[1]) if "name:" in t else []
pos = [float(x) for x in re.findall(r"- (\S+)", t.split("position:")[1].split("velocity:")[0])] if "position:" in t else []
print(names, [round(p, 6) for p in pos])
PY
}
echo "   /ak45/command: $(nm "$out/ak45cmd.txt")   /servo/command: $(nm "$out/servocmd.txt")   /servo/state: $(nm "$out/servostate.txt")"
check "/ak45/command: name ak45_4, motor angle in rad (first step of the ramp, -6 deg = -0.104720)" \
    "$([ "$(nm "$out/ak45cmd.txt")" = "['ak45_4'] [-0.10472]" ] && echo 1 || echo 0)"
check "/servo/command: name W_PITCH, MOTOR angle in rad (110 deg = 1.919862, not the sim 20 deg)" \
    "$([ "$(nm "$out/servocmd.txt")" = "['W_PITCH'] [1.919862]" ] && echo 1 || echo 0)"
check "/servo/state: W_PITCH = the angle last sent (1.919862 rad)" \
    "$([ "$(nm "$out/servostate.txt")" = "['W_PITCH'] [1.919862]" ] && echo 1 || echo 0)"

kill -INT "$launchpid" "$visionpid" 2>/dev/null
for _ in $(seq 1 20); do kill -0 "$launchpid" 2>/dev/null || break; sleep 0.5; done
kill -KILL -- -"$launchpid" -"$visionpid" 2>/dev/null     # ak45_node can hang once frames stop (STATUS.md)
kill -- -"$simpid" 2>/dev/null
sleep 2
echo "== pgrep after"; pgrep -fa 'stm_bridge_node|ak45_node|arm_control_node|vision_node' || echo "(none)"
if [ "$fails" != 0 ]; then echo "---- launch.log"; tail -15 "$out/launch.log"; fi
rm -rf "$out"
echo; [ "$fails" = 0 ] && echo "all passed" || echo "$fails failed"
exit $((fails != 0))
