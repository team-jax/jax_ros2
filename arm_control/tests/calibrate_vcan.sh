#!/usr/bin/env bash
# calibrate --ros against real processes: the first servo command must reach the bus even when it is
# typed (piped) the moment calibrate starts, i.e. before ROS discovery has matched stm_bridge.
# Needs vcan0 (sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0), python3-can, can-utils.
#   tests/calibrate_vcan.sh [vcan0]
here="$(cd "$(dirname "$0")" && pwd)"
chan="${1:-vcan0}"
out="$(mktemp -d)"
source /opt/ros/jazzy/setup.bash
source "$(cd "$here/../../../.." && pwd)/install/setup.bash"

fails=0
check() { if [ "$2" = "1" ]; then echo "PASS  $1"; else echo "FAIL  $1"; fails=$((fails + 1)); fi; }

echo "== pgrep before"; pgrep -fa 'stm_bridge_node|ak45_node|arm_control_node' || echo "(none)"

setsid python3 "$here/fake_ak45.py" > "$out/fake.log" 2>&1 &
fake=$!
setsid ros2 run stm_bridge stm_bridge_node --ros-args -p servo_channel:="$chan" > "$out/bridge.log" 2>&1 &
bridge=$!
timeout 90 stdbuf -oL candump -L "$chan" > "$out/dump.txt" 2>&1 &
dump=$!
sleep 4

echo "== 1) stm_bridge running, 'j W_PITCH / m 100 / m +10' piped at once"
start=$(date +%s.%N)
printf 'j W_PITCH\nm 100\nm +10\nq\n' | timeout 30 ros2 run arm_control calibrate --ros > "$out/cal1.txt" 2>&1
took=$(echo "$(date +%s.%N) - $start" | bc)
sleep 1
first=$(grep -m1 -o '005#[0-9A-F]*' "$out/dump.txt")
last=$(grep -o '005#[0-9A-F]*' "$out/dump.txt" | tail -1)
echo "   first frame: $first   last frame: $last   frames: $(grep -c '005#' "$out/dump.txt")   ($took s)"
check "first command is a jump to 100 deg: 005#0903E8" "$([ "$first" = "005#0903E8" ] && echo 1 || echo 0)"
check "ramp ends at 110 deg: 005#09044C" "$([ "$last" = "005#09044C" ] && echo 1 || echo 0)"

echo "== 2) stm_bridge not running: a servo command is refused, nothing is sent"
kill -- -"$bridge" 2>/dev/null; sleep 2
: > "$out/dump.txt"
start=$(date +%s.%N)
( sleep 2; printf 'j W_PITCH\nm 100\nq\n' ) | timeout 40 ros2 run arm_control calibrate --ros > "$out/cal2.txt" 2>&1
took=$(echo "$(date +%s.%N) - $start" | bc)
grep -i "subscriber" "$out/cal2.txt" | head -2
check "calibrate says nobody listens on /servo/command" "$(grep -qi 'no subscriber on /servo/command' "$out/cal2.txt" && echo 1 || echo 0)"
check "no frame on the bus" "$([ ! -s "$out/dump.txt" ] && echo 1 || echo 0)"
check "the joint is not reported as moved" "$(grep -q 'W_PITCH at' "$out/cal2.txt" && echo 0 || echo 1)"

echo "== 3) stm_bridge not running: an AK45 joint is not delayed or refused"
( sleep 3; printf 'j elbow\nm -10\nq\n' ) | timeout 40 ros2 run arm_control calibrate --ros > "$out/cal3.txt" 2>&1
check "AK45_4 moved" "$(grep -q 'AK45_4 at -10 deg' "$out/cal3.txt" && echo 1 || echo 0)"
check "no servo message for an AK45" "$(grep -qi 'subscriber' "$out/cal3.txt" && echo 0 || echo 1)"

kill -- -"$fake" 2>/dev/null; kill "$dump" 2>/dev/null
sleep 2
echo "== pgrep after"; pgrep -fa 'stm_bridge_node|ak45_node|arm_control_node' || echo "(none)"
rm -rf "$out"
echo; [ "$fails" = 0 ] && echo "all passed" || echo "$fails failed"
exit $((fails != 0))
