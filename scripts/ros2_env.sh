#!/usr/bin/env bash
set -euo pipefail

# Source this in any shell to get ROS2 Humble (conda) + consistent DDS settings.

# Make robostack/ament happy even when the caller used `set -u`.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-0}"
# Will be overwritten after activation, keeps setup scripts from tripping over unset vars.
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-$HOME/.conda/envs/ros2_humble/bin/python}"

# Prefer system-wide conda if present.
if [[ -f /opt/conda/etc/profile.d/conda.sh ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/conda/etc/profile.d/conda.sh
  set -u
else
  echo "[go2_ros2] ERROR: /opt/conda not found. Install/enable conda first." >&2
  return 1
fi

ENV_PATH="${GO2_ROS_ENV_PATH:-$HOME/.conda/envs/ros2_humble}"

# Robostack activation is not always `set -u` clean.
set +u
conda activate "$ENV_PATH"
set -u

# shellcheck disable=SC1090
set +u
source "$CONDA_PREFIX/setup.bash"
set -u

# Refresh AMENT_PYTHON_EXECUTABLE now that the env is active.
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-$CONDA_PREFIX/bin/python}"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# FastDDS: allow disabling SHM via our profile (default in run_nav2_slam.sh).
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  export FASTRTPS_DEFAULT_PROFILES_FILE="$repo_root/nav2/fastdds_no_shm.xml"
fi
