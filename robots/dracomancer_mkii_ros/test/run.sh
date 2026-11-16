#!/usr/bin/env bash
# Isolated ROS/Qt/Gazebo integration checks. Never reuse a real hardware master.
set -euo pipefail
mode=${1:-all}
case "$mode" in viewer|gazebo|direct|all) ;; *) echo 'Usage: run.sh [viewer|gazebo|direct|all]' >&2; exit 2;; esac
command -v roslaunch >/dev/null || { echo 'Source the ROS/Catkin environment first.' >&2; exit 2; }
package=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=$(mktemp -d "${TMPDIR:-/tmp}/mkii-check.XXXXXX")
export ROS_LOG_DIR="$output/roslog"
export ROS_HOME="$output/ros-home"
mkdir -p "$ROS_HOME"
export ROS_HOSTNAME=localhost
export LIBGL_ALWAYS_SOFTWARE=1
unset ROS_IP ROS_NAMESPACE
pid=''
cleanup() {
  if [[ -n "$pid" ]]; then
    kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
    pid=''
  fi
}
trap cleanup EXIT
free_ports() {
  python3 - "$@" <<'PY'
import socket, sys
for port in map(int, sys.argv[1:]):
    with socket.socket() as sock:
        sock.settimeout(0.2)
        if sock.connect_ex(('127.0.0.1', port)) == 0:
            raise SystemExit(f'Test port {port} is busy; refusing to attach to an existing server')
PY
}
run_case() {
  local kind=$1 port
  case "$kind" in viewer) port=11371;; gazebo) port=11372;; direct) port=11373;; esac
  free_ports "$port"
  export ROS_MASTER_URI="http://localhost:$port"
  if [[ "$kind" == gazebo ]]; then
    free_ports 11382
    export GAZEBO_MASTER_URI=http://localhost:11382
    roslaunch -p "$port" dracomancer mkii.launch backend:=gazebo gui:=false rviz:=false panel:=false >"$output/gazebo-launch.log" 2>&1 &
    pid=$!
    export MKII_RESULTS="$output/gazebo-results.json"
    timeout --signal=INT 180s python3 -u "$package/test/simulation.py" >"$output/gazebo-test.log" 2>&1 || {
      cat "$output/gazebo-test.log" >&2; return 1;
    }
  elif [[ "$kind" == viewer ]]; then
    roslaunch -p "$port" dracomancer mkii.launch rviz:=false panel:=false >"$output/viewer-launch.log" 2>&1 &
    pid=$!
    timeout --signal=INT 60s python3 -u "$package/test/view_test.py" >"$output/viewer-test.log" 2>&1 || {
      cat "$output/viewer-test.log" >&2; return 1;
    }
  else
    roscore -p "$port" >"$output/direct-master.log" 2>&1 &
    pid=$!
    export DIRECT_RESULTS="$output/direct-results.json"
    timeout --signal=INT 90s python3 -u "$package/test/direct_test.py" >"$output/direct-test.log" 2>&1 || {
      cat "$output/direct-test.log" >&2; return 1;
    }
  fi
  cat "$output/$kind-test.log"
  cleanup
}
if [[ "$mode" == all ]]; then
  python3 "$package/test/model.py"
  python3 "$package/test/current.py"
  for kind in viewer gazebo direct; do run_case "$kind"; done
else
  run_case "$mode"
fi
printf 'Results: %s\n' "$output"
