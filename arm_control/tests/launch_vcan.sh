#!/usr/bin/env bash
# The whole launch on a virtual bus: ros2 launch arm_control arm.launch.py (ak45_node + stm_bridge_node +
# arm_control_node) and ak45_can's tools/ak45_vcan_sim.py as the four AK45s.
#
# ak45_node's interface name is the compile-time constant "can0", so this script makes its own private
# network namespace (unshare -Urn, no sudo, the host's network is untouched) with a vcan named can0.
#   tests/launch_vcan.sh
here="$(cd "$(dirname "$0")" && pwd)"
if [ -z "${IN_NETNS:-}" ]; then
    exec env IN_NETNS=1 unshare -Urn bash "$0" "$@"
fi
set -m      # job control: background jobs keep the default SIGINT (a plain & would ignore it)
ws="$(cd "$here/../../../.." && pwd)"
sim="$here/../../ak45_can/tools/ak45_vcan_sim.py"
out="$(mktemp -d)"
ip link set lo up
ip link add can0 type vcan && ip link set can0 up || { echo "cannot create vcan can0 (is the vcan module loaded?)"; exit 2; }
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"

fails=0
check() { if [ "$2" = "1" ]; then echo "PASS  $1"; else echo "FAIL  $1"; fails=$((fails + 1)); fi; }

echo "== pgrep before"; pgrep -fa 'stm_bridge_node|ak45_node|arm_control_node' || echo "(none)"

python3 "$sim" --iface can0 --ids 1,2,3,4 > "$out/sim.log" 2>&1 &
simpid=$!
timeout 120 stdbuf -oL candump -L can0 >> "$out/dump.txt" 2>&1 &     # >> : the file is emptied below
dumppid=$!
ros2 launch arm_control arm.launch.py max_speed_deg_s:=300.0 > "$out/launch.log" 2>&1 &
launchpid=$!
sleep 8

nodes="$(ros2 node list --no-daemon 2>/dev/null)"
echo "nodes: $(echo $nodes)"
for n in /ak45_node /stm_bridge /arm_control_node; do
    check "node $n is up" "$(echo "$nodes" | grep -qx "$n" && echo 1 || echo 0)"
done
check "no ERROR, traceback or dead process in the launch log" \
    "$(grep -qE 'ERROR|Traceback|process has died|FATAL' "$out/launch.log" && echo 0 || echo 1)"
fb="$(timeout 10 ros2 topic echo --once /joint_states --field name 2>/dev/null | tr -d "[]', \n-")"
check "/joint_states from ak45_node names the four motors" "$([ "$fb" = "ak45_1ak45_2ak45_3ak45_4" ] && echo 1 || echo 0)"

echo "== one sim command: elbow -10 deg (AK45 id 4), wrist_pitch 20 deg (servo 9)"
: > "$out/dump.txt"
timeout 15 ros2 topic echo --once /servo/state --field position > "$out/state.txt" 2>&1 &     # /servo/state is published per command
echostate=$!
sleep 1
timeout 20 ros2 topic pub -1 --wait-matching-subscriptions 1 /arm_control/joint_command sensor_msgs/msg/JointState \
    "{name: [elbow, wrist_pitch], position: [-0.17453292519943295, 0.3490658503988659]}" > /dev/null 2>&1
sleep 3
ak45="$(grep -ao '00000404#[0-9A-F]*' "$out/dump.txt" | tail -1)"
servo="$(grep -ao '005#[0-9A-F]*' "$out/dump.txt" | tail -1)"
echo "   last AK45 id 4 frame: $ak45   servo frames: $(grep -ac '005#' "$out/dump.txt")   last: $servo"
check "AK45 id 4 ends at -10 deg: mode 4, int32 -100000 = FFFE7960" "$([ "$ak45" = "00000404#FFFE7960" ] && echo 1 || echo 0)"
check "servo 9 (W_PITCH) at 90 + 20 deg: 005#09044C" "$([ "$servo" = "005#09044C" ] && echo 1 || echo 0)"
check "the other AK45s were not commanded" \
    "$(grep -aE '0000040[123]#' "$out/dump.txt" | grep -q . && echo 0 || echo 1)"
wait "$echostate" 2>/dev/null
check "/servo/state holds 110 deg = 1.919862 rad" "$(grep -q '1.919862' "$out/state.txt" && echo 1 || echo 0)"

echo "== Ctrl+C (SIGINT) on the launch"
: > "$out/dump.txt"
kill -INT "$launchpid" 2>/dev/null
for _ in $(seq 1 20); do kill -0 "$launchpid" 2>/dev/null || break; sleep 0.5; done
check "launch exited" "$(kill -0 "$launchpid" 2>/dev/null && echo 0 || echo 1)"
sleep 1
echo "   frames at shutdown (ak45_can sends Current Brake 0A on exit, ids 1..6): $(grep -aoc '0000020[0-9]#' "$out/dump.txt")"
check "no ExternalShutdownException / traceback at SIGINT" "$(grep -qE 'Traceback|ExternalShutdownException' "$out/launch.log" && echo 0 || echo 1)"

kill -- -"$launchpid" 2>/dev/null; sleep 2
kill -KILL -- -"$launchpid" 2>/dev/null     # ak45_node can hang in ak45_close() once frames stop (see STATUS.md)
kill -- -"$simpid" 2>/dev/null; kill "$dumppid" 2>/dev/null
sleep 2
echo "== pgrep after"; pgrep -fa 'stm_bridge_node|ak45_node|arm_control_node' || echo "(none)"
if [ "$fails" != 0 ]; then echo "---- launch.log"; tail -30 "$out/launch.log"; fi
rm -rf "$out"
echo; [ "$fails" = 0 ] && echo "all passed" || echo "$fails failed"
exit $((fails != 0))
