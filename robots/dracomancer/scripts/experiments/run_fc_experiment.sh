#!/usr/bin/env bash
# Orchestrates the fc threshold-calibration experiment (see
# docs/fc_threshold_calibration.md). Assumes a DRAGON Gazebo sim is already
# hovering (flight_state == 5).
#
#   1. start rosbag       (fc / joints / odom / flight_state)
#   2. Part A shape sweep -> fc_sweep.csv
#   3. (optional) Part B disturbance test per axis -> *_disturbance.csv
#   4. stop rosbag
#   5. analyze -> proposed [hard_min, min]
#
# Usage: run_fc_experiment.sh [out_dir] [--with-disturbance]
set -euo pipefail

OUT_DIR="${1:-$HOME/dracomancer_fc_experiment}"
WITH_DIST="${2:-}"
mkdir -p "$OUT_DIR"
PKG=$(rospack find dracomancer)/scripts/experiments

echo "[1/5] start rosbag -> $OUT_DIR/fc_sweep.bag"
rosbag record -O "$OUT_DIR/fc_sweep.bag" __name:=fc_rosbag \
  /dragon/debug/fc_f_min /dragon/debug/fc_t_min \
  /dragon/joint_states /dragon/joints_ctrl \
  /dragon/uav/cog/odom /dragon/flight_state &
sleep 3

echo "[2/5] Part A shape sweep"
rosrun dracomancer collect_fc_data.py \
  _out_path:="$OUT_DIR/fc_sweep.csv" _n_points_1d:=7 _n_random:=40

if [ "$WITH_DIST" = "--with-disturbance" ]; then
  echo "[3/5] Part B disturbance test (fx, fz, tx)"
  for AX in fx fz tx; do
    rosrun dracomancer disturbance_test.py \
      _axis:="$AX" _out_path:="$OUT_DIR/${AX}_disturbance.csv"
  done
fi

echo "[4/5] stop rosbag"
rosnode kill /fc_rosbag || true
sleep 2

echo "[5/5] analyze"
python3 "$PKG/analyze_fc_data.py" "$OUT_DIR/fc_sweep.csv" --hard-pct 5 --min-frac 0.8 \
  | tee "$OUT_DIR/proposed_thresholds.txt"

echo "done. artifacts in $OUT_DIR"
