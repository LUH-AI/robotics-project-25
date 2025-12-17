#!/usr/bin/env bash
#
# Convenience launcher for isaac_go2_ros2.py
# Usage:
#   ./scripts/run_go2.sh                # GUI (default)
#   GO2_HEADLESS=1 ./scripts/run_go2.sh # headless
#   ./scripts/run_go2.sh env_name=office sensor.enable_camera=false   # pass Hydra overrides

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$ROOT_DIR/src"
ISAAC_ENV="${GO2_ISAAC_ENV:-env_isaaclab_py311}"
export OMNI_KIT_ACCEPT_EULA="${OMNI_KIT_ACCEPT_EULA:-YES}"

# Activate the Isaac Lab/Sim conda environment once.
find_conda_sh() {
  for candidate in \
    "/opt/conda/etc/profile.d/conda.sh" \
    "$HOME/miniconda3/etc/profile.d/conda.sh" \
    "$HOME/.miniconda3/etc/profile.d/conda.sh" \
    "$HOME/anaconda3/etc/profile.d/conda.sh" \
    "$HOME/.anaconda3/etc/profile.d/conda.sh"; do
    if [[ -f "$candidate" ]]; then
      echo "$candidate"
      return 0
    fi
  done
  return 1
}

CONDA_SH="$(find_conda_sh || true)"
if [[ -z "${CONDA_SH:-}" ]]; then
  echo "[go2_sim] ERROR: Could not locate conda.sh. Install Miniconda/Anaconda and retry." >&2
  exit 1
fi

# Conda activation scripts reference variables that may be unset under `set -u`.
set +u
# shellcheck disable=SC1091
source "$CONDA_SH"
conda activate "$ISAAC_ENV"
set -u

# Helper: locate Isaac Sim's ROS2 bridge library so the internal ROS nodes load.
resolve_ros_bridge() {
  python - <<'PY'
from pathlib import Path
import os
import sys

def echo(path: Path) -> bool:
    if path and path.is_dir():
        print(path.as_posix())
        return True
    return False

def iter_roots():
    """Yield likely installation roots without requiring SimulationApp imports."""
    prefix = Path(sys.prefix).resolve()
    pyver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    guesses = {
        prefix,
        prefix / "lib",
        prefix / "lib" / pyver,
        prefix / "lib" / pyver / "site-packages",
        prefix / "lib" / pyver / "site-packages" / "isaacsim",
    }
    for env_key in ("ISAACSIM_PATH", "ISAACSIM_ROOT", "ISAAC_PATH"):
        val = os.environ.get(env_key)
        if val:
            try:
                guesses.add(Path(val).resolve())
            except Exception:
                pass
    for root in list(guesses):
        if root.exists():
            yield root

# Preferred path: python module exposes the lib directory (works on Isaac Sim 4.1+)
try:
    import isaacsim.ros2.bridge as bridge  # type: ignore[import]
except Exception:
    bridge = None

if bridge:
    lib_dir = Path(bridge.__file__).resolve().parent / "humble" / "lib"
    if echo(lib_dir):
        raise SystemExit(0)

for root in iter_roots():
    matches = sorted(root.glob("**/isaacsim.ros2.bridge-*/humble/lib"), reverse=True)
    for candidate in matches:
        if echo(candidate):
            raise SystemExit(0)
PY
}

ISAAC_ROS_LIB="${ISAAC_ROS_LIB:-$(resolve_ros_bridge)}"
if [[ -z "$ISAAC_ROS_LIB" || ! -d "$ISAAC_ROS_LIB" ]]; then
  # Hard fallback to the known install path used by this repo/env.
  FALLBACK_LIB="$HOME/miniconda3/envs/env_isaaclab_py311/lib/python3.11/site-packages/isaacsim/kit/data/Kit/Isaac-Sim/5.1/exts/3/isaacsim.ros2.bridge-4.12.4+107.3.3.lx64/humble/lib"
  if [[ -d "$FALLBACK_LIB" ]]; then
    ISAAC_ROS_LIB="$FALLBACK_LIB"
  fi
fi

if [[ -z "$ISAAC_ROS_LIB" || ! -d "$ISAAC_ROS_LIB" ]]; then
  echo "[go2_sim] WARNING: Could not auto-detect isaacsim.ros2.bridge lib directory."
  echo "           Set ISAAC_ROS_LIB to the '.../isaacsim/ros2/bridge/humble/lib' path if ROS topics fail."
else
  export LD_LIBRARY_PATH="$ISAAC_ROS_LIB:${LD_LIBRARY_PATH:-}"
fi

# Optional libstdc++ override (helps when ROS bridge complains about missing CXXABI symbols).
if [[ -n "${GO2_LIBSTDCXX_OVERRIDE:-}" && -f "$GO2_LIBSTDCXX_OVERRIDE" ]]; then
  export LD_LIBRARY_PATH="$(dirname "$GO2_LIBSTDCXX_OVERRIDE"):${LD_LIBRARY_PATH:-}"
fi

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT_DIR/nav2/fastdds_no_shm.xml"
fi

# Default GO2_HEADLESS to 0 unless already set by the user/environment.
export GO2_HEADLESS="${GO2_HEADLESS:-0}"

# --- Detection Cube Mode Selection ---
if [[ -z "${GO2_CUBE_MODE:-}" ]]; then
  echo "--------------------------------------------------"
  echo "Select Detection Cube Mode:"
  echo "  1) EASY   - 1 Cube 5m in front"
  echo "  2) MEDIUM - 10 Cubes random (15m radius)"
  echo "  3) HARD   - 1 Cube random (20m radius) [Default]"
  echo "  --------------------------------------------------"
  read -p "Enter choice [1-3]: " choice

  case "$choice" in
    1)
      export GO2_CUBE_MODE="EASY"
      echo "-> EASY MODE selected."
      ;;
    2)
      export GO2_CUBE_MODE="MEDIUM"
      echo "-> MEDIUM MODE selected."
      ;;
    3)
      export GO2_CUBE_MODE="HARD"
      echo "-> HARD MODE selected."
      ;;
    *)
      export GO2_CUBE_MODE="HARD"
      echo "-> Defaulting to HARD MODE."
      ;;
  esac
  echo "--------------------------------------------------"
fi


echo "[go2_sim] Starting Isaac Sim (env: $ISAAC_ENV, headless=$GO2_HEADLESS)"
cd "$SRC_DIR"
exec python isaac_go2_ros2.py "$@"
