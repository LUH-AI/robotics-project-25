#!/usr/bin/env bash
#
# One-shot helper to prep this repo on a new workstation.
# - Ensures basic folders exist.
# - (Optionally) creates a robostack ROS2 Humble env with Nav2 + SLAM.
# - Prints the exact commands needed to run Isaac Sim + Nav2 after setup.
#
# Usage:
#   ./scripts/after_clone.sh
#   ./scripts/after_clone.sh --ros2-env my_ros --isaac-env env_isaaclab_py311
#   GO2_SKIP_ROS2_INSTALL=1 ./scripts/after_clone.sh   # just verify/prereqs

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ISAAC_ENV="${GO2_ISAAC_ENV:-env_isaaclab_py311}"
ROS2_ENV="${GO2_ROS_ENV:-ros2_humble}"
SKIP_ROS2="${GO2_SKIP_ROS2_INSTALL:-0}"
ROS2_PKGS="${GO2_ROS2_PACKAGES:-ros-humble-desktop ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan}"
YES_FLAG="-y"

print_usage() {
  cat <<EOF
Usage: ${0##*/} [options]

Options:
  --ros2-env NAME       Conda environment name to create/use for ROS2 (default: ${ROS2_ENV})
  --isaac-env NAME      Conda environment name that hosts Isaac Lab/Sim (default: ${ISAAC_ENV})
  --skip-ros2           Skip creating the ROS2 env (useful if you manage it manually)
  --ros2-packages LIST  Override the robostack package list for the ROS2 env
  -h, --help            Show this help message and exit

Environment overrides:
  GO2_ROS_ENV, GO2_ISAAC_ENV, GO2_SKIP_ROS2_INSTALL, GO2_ROS2_PACKAGES

This script does NOT install Isaac Sim/Lab for you. Install the NVIDIA packages
first, then run this helper to create the Nav2/SLAM environment and generate
ready-to-copy launch commands.
EOF
}

log()  { echo -e "[setup] $*"; }
warn() { echo -e "[setup][warn] $*" >&2; }
die()  { echo -e "[setup][error] $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros2-env) shift; ROS2_ENV="${1:-}";;
    --isaac-env) shift; ISAAC_ENV="${1:-}";;
    --skip-ros2) SKIP_ROS2=1;;
    --ros2-packages) shift; ROS2_PKGS="${1:-}";;
    -h|--help) print_usage; exit 0;;
    *) die "Unknown option: $1";;
  esac
  shift
done

ensure_dirs() {
  mkdir -p "$ROOT_DIR/logs" "$ROOT_DIR/outputs"
  if [[ ! -f "$ROOT_DIR/logs/.gitkeep" ]]; then
    touch "$ROOT_DIR/logs/.gitkeep"
  fi
}

find_conda_sh() {
  if [[ -n "${GO2_CONDA_SH:-}" && -f "$GO2_CONDA_SH" ]]; then
    echo "$GO2_CONDA_SH"
    return 0
  fi
  if command -v conda >/dev/null 2>&1; then
    local conda_exe
    conda_exe="$(command -v conda)"
    local base_dir
    base_dir="$(cd "$(dirname "$conda_exe")/.." && pwd)"
    if [[ -f "$base_dir/etc/profile.d/conda.sh" ]]; then
      echo "$base_dir/etc/profile.d/conda.sh"
      return 0
    fi
  fi
  local candidates=(
    "/opt/conda"
    "$HOME/mambaforge"
    "$HOME/miniforge3"
    "$HOME/miniconda3"
    "$HOME/.miniconda3"
    "$HOME/anaconda3"
    "$HOME/.anaconda3"
  )
  for prefix in "${candidates[@]}"; do
    [[ -z "$prefix" ]] && continue
    if [[ -f "$prefix/etc/profile.d/conda.sh" ]]; then
      echo "$prefix/etc/profile.d/conda.sh"
      return 0
    fi
  done
  return 1
}

ensure_conda() {
  local conda_sh
  if ! conda_sh="$(find_conda_sh)"; then
    die "Could not find conda. Install Miniconda/Mambaforge and re-run."
  fi
  # shellcheck disable=SC1090
  set +u
  source "$conda_sh"
  set -u
}

env_exists() {
  local name="$1"
  conda env list | awk '{print $1}' | grep -Fxq "$name"
}

choose_solver() {
  if command -v mamba >/dev/null 2>&1; then
    echo "mamba"
  elif command -v micromamba >/dev/null 2>&1; then
    echo "micromamba"
  else
    echo "conda"
  fi
}

create_ros2_env() {
  local solver
  solver="$(choose_solver)"
  log "Creating ROS2 env '$ROS2_ENV' using $solver (this can take a while)..."
  "$solver" create -n "$ROS2_ENV" $YES_FLAG -c conda-forge -c robostack-staging $ROS2_PKGS
}

verify_ros2_env() {
  log "Verifying ROS2 env '$ROS2_ENV'..."
  if ! conda run -n "$ROS2_ENV" python - <<'PY' >/dev/null 2>&1; then
import rclpy
PY
    die "ROS2 env '$ROS2_ENV' is missing rclpy. Inspect the conda output above."
  fi
}

ensure_dirs
log "Repo root: $ROOT_DIR"
ensure_conda

if [[ "$SKIP_ROS2" != "1" ]]; then
  if env_exists "$ROS2_ENV"; then
    log "ROS2 env '$ROS2_ENV' already exists."
  else
    create_ros2_env
  fi
  verify_ros2_env
else
  warn "Skipping ROS2 env creation (GO2_SKIP_ROS2_INSTALL=1 or --skip-ros2)."
fi

cat <<EOF

============================================================
Setup summary
============================================================
- Isaac Sim / Isaac Lab conda env: ${ISAAC_ENV}
- ROS2 Humble env: ${ROS2_ENV}  (source via:  source scripts/ros2_env.sh )
- Logs directory: ${ROOT_DIR}/logs

Next steps:
1) Ensure Isaac Sim 5.1 + Isaac Lab 2.3 are installed inside '$ISAAC_ENV'.
2) Export the Isaac ROS bridge libraries before launching the sim, e.g.
     export ISAAC_ROS_LIB=\$(python -c "import isaacsim.ros2.bridge as b; from pathlib import Path; print(Path(b.__file__).resolve().parent/'humble'/'lib')")
     export LD_LIBRARY_PATH=\$ISAAC_ROS_LIB:\${LD_LIBRARY_PATH:-}
3) Start the simulator (GUI):
     source /opt/conda/etc/profile.d/conda.sh
     conda activate ${ISAAC_ENV}
     export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
     export ROS_DOMAIN_ID=0
     GO2_HEADLESS=0 python src/isaac_go2_ros2.py
4) In a second terminal, bring up Nav2 + SLAM:
     source scripts/ros2_env.sh
     ./scripts/run_nav2_slam.sh

Common pitfalls & fixes:
- If isaacsim.ros2.bridge complains about libstdc++ versions, set
    export GO2_LIBSTDCXX_OVERRIDE=\$CONDA_PREFIX/lib/libstdc++.so.6
  before launching python and add that path to LD_LIBRARY_PATH.
- On remote desktops (ThinLinc/VNC), set GO2_RVIZ_SOFTWARE=1 when running
  ./scripts/run_nav2_slam.sh to force software GL for RViz.
- Make sure only one Nav2/SLAM stack runs at a time. Use ./scripts/stop_nav2_slam.sh
  before relaunching to avoid conflicting TF trees.

Happy mapping!
EOF
