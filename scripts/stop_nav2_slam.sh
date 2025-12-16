#!/usr/bin/env bash
set -euo pipefail

# Stop Nav2+SLAM processes started by run_nav2_slam.sh.
#
# This is important because running multiple Nav2 / slam_toolbox instances in
# parallel will publish conflicting TFs (e.g., multiple map->odom) and the robot
# pose / lidar will appear to "jump" in RViz.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STATE_DIR="${ROS_HOME:-$HOME/.ros}/go2_nav2_slam"
PID_FILE="$STATE_DIR/pids"

QUIET=0
if [[ "${1-}" == "--quiet" ]]; then
  QUIET=1
fi

log() {
  if [[ "$QUIET" != "1" ]]; then
    echo "$@"
  fi
}

kill_group() {
  local pid="$1"
  local sig="${2:-INT}"
  if [[ -z "$pid" ]]; then
    return 0
  fi
  if ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi

  # Prefer killing the *process group* since ros2 launch spawns many children.
  # Note: the PID is often NOT the PGID, so we query it explicitly.
  local pgid=""
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  if [[ -n "$pgid" ]]; then
    kill "-$sig" -- "-$pgid" 2>/dev/null || true
  fi
  kill "-$sig" "$pid" 2>/dev/null || true
}

mkdir -p "$STATE_DIR"

if [[ -f "$PID_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$PID_FILE" || true
  log "[go2_nav2] Stopping processes from $PID_FILE ..."
  # ros2 launch often ignores SIGINT when detached; use SIGTERM for reliable shutdown.
  kill_group "${NAV2_PID-}" TERM
  kill_group "${SCAN_RELAY_PID-}" TERM
  kill_group "${PC2LS_PID-}" TERM
  kill_group "${FOOTPRINT_TF_PID-}" TERM
  rm -f "$PID_FILE" 2>/dev/null || true
  log "[go2_nav2] Done."
  exit 0
fi

# Fallback: try to stop processes that reference our repo param files.
PARAMS_FILE="$ROOT_DIR/nav2/nav2_slam_params.yaml"
PC2LS_PARAMS="$ROOT_DIR/nav2/pointcloud_to_laserscan.yaml"

log "[go2_nav2] No PID file found. Trying best-effort cleanup by pattern..."
for pid in $(pgrep -f "$PARAMS_FILE" 2>/dev/null || true); do
  kill_group "$pid" INT
done
for pid in $(pgrep -f "$PC2LS_PARAMS" 2>/dev/null || true); do
  kill_group "$pid" TERM
done
for pid in $(pgrep -f "scan_qos_relay.py" 2>/dev/null || true); do
  kill_group "$pid" TERM
done
for pid in $(pgrep -f "base_footprint_tf.py" 2>/dev/null || true); do
  kill_group "$pid" TERM
done
log "[go2_nav2] Done."
