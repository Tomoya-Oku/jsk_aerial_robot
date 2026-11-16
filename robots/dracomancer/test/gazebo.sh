#!/usr/bin/env bash
# Isolated contact-plugin build and physics smoke test, without robot flight.
set -eo pipefail
source /opt/ros/noetic/setup.bash
set -u
task_package=$(cd "$(dirname "$0")/.." && pwd)
task_build=$(mktemp -d /tmp/dracomancer-contact.XXXXXX)
g++ -std=c++17 -fPIC -shared "$task_package/src/wall_contact.cpp" \
  -o "$task_build/libwall_contact.so" $(pkg-config --cflags --libs gazebo roscpp)
export GAZEBO_PLUGIN_PATH="$task_build:${GAZEBO_PLUGIN_PATH:-}"
roslaunch gazebo_ros empty_world.launch gui:=false > "$task_build/gazebo.log" 2>&1 &
task_gazebo_pid=$!
trap 'kill "$task_gazebo_pid" 2>/dev/null || true' EXIT
python3 -B "$task_package/test/test_gazebo.py"
