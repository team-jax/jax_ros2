#!/usr/bin/env bash
# Build jax_ros2 in ros:jazzy and run the device-free checks, including the vcan chains.
#   tools/docker/run_tests.sh [command ...]     default: build + approach checks
# The repo is mounted read-only at /ws/src/jax_ros2; the build (plain install, so the share/ copies of
# calibration.json and model/ are what runs) stays in the container.
# vcan: IN_NETNS=1 makes the test scripts create can0 in the container's own network namespace (NET_ADMIN);
# the host needs the vcan module (it loads on demand).
set -e
repo="$(cd "$(dirname "$0")/../.." && pwd)"
docker build -q -t jax_ros2:jazzy -f "$repo/tools/docker/Dockerfile.jazzy" "$repo/tools/docker" >/dev/null
cmd="${*:-arm_control/tests/approach_vcan.sh}"
docker run --rm --cap-add NET_ADMIN --security-opt seccomp=unconfined --security-opt apparmor=unconfined \
    -v "$repo:/ws/src/jax_ros2:ro" -e IN_NETNS=1 jax_ros2:jazzy bash -c "
    set -e
    source /opt/ros/jazzy/setup.bash
    cd /ws && colcon build --event-handlers console_direct- > /tmp/build.log 2>&1 || { tail -40 /tmp/build.log; exit 1; }
    cd /ws/src/jax_ros2
    python3 -B arm_control/tests/check_approach.py | tail -1
    python3 -B arm_control/tests/check_approach_plan.py | tail -1
    $cmd"
