#!/usr/bin/env bash
set -euo pipefail

# Launch the Grounded-SAM-2 detector node for the Go2 camera.
# Requires the same ROS 2 Humble environment used for Nav2.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

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
    echo "[go2_object_detection] ERROR: 'ros2' not found in PATH." >&2
    echo "[go2_object_detection] Activate your ROS2 Humble environment (GO2_ROS_ENV=${GO2_ROS_ENV:-ros2_humble})." >&2
    exit 127
  fi
}

ensure_ros2

export PYTHONPATH="$ROOT_DIR/src:$ROOT_DIR/object-detection:$ROOT_DIR/object-detection/Grounded-SAM-2:${PYTHONPATH:-}"

# Keep DDS consistent with the simulator bridge.
export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[go2_object_detection] Starting detector node..."
exec python3 "$ROOT_DIR/src/ros2/go2_object_detection_node.py" "$@"
