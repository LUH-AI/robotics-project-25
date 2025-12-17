#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STATE_DIR="$ROOT_DIR/outputs/agent_state"
FRONTIER_PID_FILE="$STATE_DIR/frontier.pid"
mkdir -p "$STATE_DIR"

ensure_ros2() {
  if command -v ros2 >/dev/null 2>&1; then
    return 0
  fi
  local env_name="${GO2_ROS_ENV:-ros2_humble}"
  local conda_sh=""
  for candidate in \
    "/opt/conda/etc/profile.d/conda.sh" \
    "$HOME/miniconda3/etc/profile.d/conda.sh" \
    "$HOME/.miniconda3/etc/profile.d/conda.sh" \
    "$HOME/anaconda3/etc/profile.d/conda.sh" \
    "$HOME/.anaconda3/etc/profile.d/conda.sh"; do
    if [[ -f "$candidate" ]]; then
      conda_sh="$candidate"
      break
    fi
  done
  if [[ -n "$conda_sh" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$conda_sh"
    if command -v conda >/dev/null 2>&1; then
      conda activate "$env_name" >/dev/null 2>&1 || true
    fi
    set -u
  fi
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "[start_agent] ERROR: 'ros2' CLI not found. Activate your ROS 2 environment." >&2
    exit 127
  fi
}

cleanup() {
  local code=$?
  if [[ -n ${STATUS_GUI_PID:-} ]] && kill -0 "$STATUS_GUI_PID" >/dev/null 2>&1; then
    kill -TERM "$STATUS_GUI_PID" >/dev/null 2>&1 || true
    wait "$STATUS_GUI_PID" 2>/dev/null || true
  fi
  if [[ -n ${OBJECT_PURSUIT_PID:-} ]] && kill -0 "$OBJECT_PURSUIT_PID" >/dev/null 2>&1; then
    kill -TERM "$OBJECT_PURSUIT_PID" >/dev/null 2>&1 || true
    wait "$OBJECT_PURSUIT_PID" 2>/dev/null || true
  fi
  if [[ -n ${FRONTIER_PID:-} ]] && kill -0 "$FRONTIER_PID" >/dev/null 2>&1; then
    kill -TERM "$FRONTIER_PID" >/dev/null 2>&1 || true
    wait "$FRONTIER_PID" 2>/dev/null || true
  fi
  if [[ -n ${NAV2_STACK_PID:-} ]] && kill -0 "$NAV2_STACK_PID" >/dev/null 2>&1; then
    kill -INT "$NAV2_STACK_PID" >/dev/null 2>&1 || true
    wait "$NAV2_STACK_PID" 2>/dev/null || true
  fi
  rm -f "$FRONTIER_PID_FILE" >/dev/null 2>&1 || true
  return $code
}
# Kill any zombie processes from previous runs
echo "[start_agent] Cleaning up zombie processes..."
pkill -9 -f "python.*go2_object_detection_node.py" 2>/dev/null || true
pkill -9 -f "explorer.py" 2>/dev/null || true
pkill -9 -f "go2_status_gui.py" 2>/dev/null || true
pkill -9 -f "go2_object_goal_manager.py" 2>/dev/null || true
pkill -9 -f "lifecycle_manager_slam" 2>/dev/null || true
sleep 1
echo "[start_agent] Cleanup complete."

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

ensure_ros2

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$ROOT_DIR/nav2/fastdds_no_shm.xml}"

# 1) Launch Nav2 + SLAM (includes object detector).
echo "[start_agent] Launching Nav2 + SLAM pipeline..."
"$ROOT_DIR/scripts/run_nav2_slam.sh" &
NAV2_STACK_PID=$!

# Give Nav2/SLAM a few seconds to start up
echo "[start_agent] Waiting for Nav2/SLAM to initialize..."
sleep 5

echo "[start_agent] Waiting for Nav2 action server..."
nav2_ready=0
for _ in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q "^/navigate_to_pose$"; then
    nav2_ready=1
    break
  fi
  sleep 1
done
if [[ "$nav2_ready" -ne 1 ]]; then
  echo "[start_agent] ERROR: Nav2 action server not available." >&2
  exit 1
fi

# 2) Start the cloned frontier explorer package.
EXPLORER_ROOT="$ROOT_DIR/exploration_algorithm/Autonomous-Explorer-and-Mapper-ros2-nav2"
if [[ ! -d "$EXPLORER_ROOT/custom_explorer" ]]; then
  echo "[start_agent] ERROR: Explorer package not found at $EXPLORER_ROOT." >&2
  exit 1
fi
export PYTHONPATH="$EXPLORER_ROOT:$ROOT_DIR/src:${PYTHONPATH:-}"

echo "[start_agent] Starting frontier exploration node..."
python3 -m custom_explorer.explorer &
FRONTIER_PID=$!
printf '%s\n' "$FRONTIER_PID" > "$FRONTIER_PID_FILE"

# 3) Start the status GUI window
echo "[start_agent] Starting status GUI window..."
python3 "$ROOT_DIR/src/ros2/go2_status_gui.py" &
STATUS_GUI_PID=$!

# 4) Start the detection-aware pursuit manager.
export GO2_FRONTIER_PID_FILE="$FRONTIER_PID_FILE"

echo "[start_agent] Monitoring detections and switching to pursuit when needed..."
python3 "$ROOT_DIR/src/ros2/go2_object_goal_manager.py" &
OBJECT_PURSUIT_PID=$!

wait "$OBJECT_PURSUIT_PID"

# When object pursuit finishes, the trap handler cleans up Nav2 + frontier.
echo "[start_agent] Object pursuit node exited; shutting down stack."
