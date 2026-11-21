#!/usr/bin/env bash
# up / zero / start / stop on a virtual bus, no sudo (private network namespace with vcan can0).
#   jax_bringup/test/bringup_vcan.sh          (after colcon build)
# ak45_vcan_sim.py plays the AK45s, arm_control/tests/fake_hand.py the pressure sensors.
here="$(cd "$(dirname "$0")" && pwd)"
if [ -z "${IN_NETNS:-}" ]; then exec env IN_NETNS=1 unshare -Urn bash "$0" "$@"; fi
set -m
repo="$(cd "$here/../.." && pwd)"
ws="$(cd "$repo/../.." && pwd)"
out="$(mktemp -d)"
ip link set lo up
ip link add can0 type vcan && ip link set can0 up || { echo "cannot create vcan can0"; exit 2; }
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
export ROS_DOMAIN_ID=79 JAX_YES=1 JAX_CAN=skip
fails=0
check() { if [ "$2" = "1" ]; then echo "PASS  $1"; else echo "FAIL  $1  $3"; fails=$((fails + 1)); fi; }
nodes() { ros2 node list --no-daemon 2>/dev/null | sort; }

python3 "$repo/ak45_can/tools/ak45_vcan_sim.py" --iface can0 --ids 1,2,3,4 > "$out/sim.log" 2>&1 & sim=$!
ros2 run vision_node vision_node --ros-args -p fixed_pose:="[0.5, 0.5, 0.0]" > "$out/old.log" 2>&1 & old=$!
sleep 3

echo "== up (fixed pose) stops the old vision_node and starts 5 nodes"
ros2 run jax_bringup up "[-0.048, -0.285, 20.0]" force:=true > "$out/up1.log" 2>&1 & up1=$!
sleep 12
n="$(nodes | tr '\n' ' ')"
check "5 nodes: ak45_node approach_node arm_control_node stm_bridge vision_node" \
    "$([ "$n" = "/ak45_node /approach_node /arm_control_node /stm_bridge /vision_node " ] && echo 1 || echo 0)" "got: $n"
check "the old vision_node (pose 0.5, 0.5) was stopped" "$(kill -0 $old 2>/dev/null && echo 0 || echo 1)"
pose="$(timeout 10 ros2 topic echo --once /object_pose --field pose.position --no-daemon 2>/dev/null | tr -d ' \n')"
check "/object_pose = the fixed pose" "$([[ "$pose" == "x:-0.048y:-0.285z:0.0"* ]] && echo 1 || echo 0)" "got: $pose"
check "up log names the stopped process and the launch" "$(grep -q 'stopping:' "$out/up1.log" && grep -q 'jax_bringup: fixed pose' "$out/up1.log" && echo 1 || echo 0)"

echo "== zero"
python3 "$repo/arm_control/tests/fake_hand.py" > "$out/hand.log" 2>&1 & hand=$!
echo n | ros2 run jax_bringup zero > "$out/zero_n.log" 2>&1
check "zero, answer n: nothing sent" "$(grep -q 'cancelled, nothing sent' "$out/zero_n.log" && echo 1 || echo 0)"
echo y | ros2 run jax_bringup zero > "$out/zero.log" 2>&1
check "zero, answer y: AK45s at their zero_deg after 5 s (ak45_3 89.90)" "$(grep -qE '^ak45_3 +89.90' "$out/zero.log" && echo 1 || echo 0)"

echo "== start: approach + grasp with the fake hand"
timeout 120 ros2 run jax_bringup start > "$out/start.log" 2>&1
echo "   $(grep -o 'phase: [A-Z_]*' "$out/start.log" | tr '\n' ' ')"
check "start accepted" "$(grep -q 'success=True' "$out/start.log" && echo 1 || echo 0)"
check "phases end GRASP RETURN DONE" "$(grep -o 'phase: [A-Z_]*' "$out/start.log" | tail -3 | tr '\n' ' ' | grep -q 'GRASP phase: RETURN phase: DONE' && echo 1 || echo 0)"
r="$(ros2 run jax_bringup stop 2>&1)"
check "stop after the run: 'not running (DONE)'" "$(echo "$r" | grep -q 'not running (DONE)' && echo 1 || echo 0)" "$r"

echo "== a second up replaces the first: finger test, no pressure sensors"
kill $hand 2>/dev/null
ros2 run jax_bringup up "[-0.048, -0.285, 20.0]" force:=true fingers:=true > "$out/up2.log" 2>&1 & up2=$!
sleep 14
n="$(nodes | uniq -c | awk '{print $1}' | sort -u | tr '\n' ' ')"
check "every node once (no doubles)" "$([ "$n" = "1 " ] && echo 1 || echo 0)" "counts: $n"
check "the first up has ended" "$(kill -0 $up1 2>/dev/null && echo 0 || echo 1)"
timeout 120 ros2 run jax_bringup start > "$out/start2.log" 2>&1
echo "   $(grep -o 'phase: [A-Z_]*' "$out/start2.log" | tr '\n' ' ')"
check "finger test phases end INSERT FINGERS CLOSED OPEN DONE" "$(grep -o 'phase: [A-Z_]*' "$out/start2.log" | tail -5 | tr '\n' ' ' | grep -q 'INSERT phase: FINGERS phase: CLOSED phase: OPEN phase: DONE' && echo 1 || echo 0)"
check "log: no pressure, fingers closed to 30, 30, 0, 60, 60, 60 deg" "$(grep -q 'NO pressure' "$out/up2.log" && grep -q 'fingers at 30.0, 30.0, 0.0, 60.0, 60.0, 60.0 deg' "$out/up2.log" && echo 1 || echo 0)"

kill -INT -$up2 2>/dev/null; sleep 3; kill -KILL -$up2 -$up1 2>/dev/null; kill $hand $sim 2>/dev/null; sleep 1
echo "   logs: $out"
echo "== pgrep after"; pgrep -fa 'ak45_node|stm_bridge_node|arm_control_node|approach_node|vision_node' || echo "(none)"
echo; [ "$fails" = 0 ] && echo "all passed" || echo "$fails failed"; [ "$fails" = 0 ]
