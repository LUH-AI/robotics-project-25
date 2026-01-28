#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$ROOT_DIR/dimos"

if [[ ! -x ".venv/bin/dimos" ]]; then
  echo "ERROR: Missing dimos virtualenv at '$ROOT_DIR/dimos/.venv'." >&2
  echo "Hint: create it via:  cd dimos && uv venv && . .venv/bin/activate && uv pip install -e '.[base,unitree]'" >&2
  exit 1
fi

# shellcheck disable=SC1091
source ".venv/bin/activate"

# Ensure this repo copy is imported (even if the venv was created from another checkout).
export PYTHONPATH="$ROOT_DIR/dimos${PYTHONPATH:+:$PYTHONPATH}"

# Load optional per-project defaults.
if [[ -f ".env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source ".env"
  set +a
fi

usage() {
  cat <<'EOF'
Usage:
  ./run_sam3_exploration.sh [--robot-ip <IP>] [--workers <n>] [--memory-limit <str>] [--sam3-model <path>] [--ui-port <port>] [--viewer-backend <mode>] [--no-browser] [-- <dimos run args...>]

Notes:
  - Robot connection is configured via env/.env (ROBOT_IP) or --robot-ip.
  - UI runs on http://localhost:<port> (default 7788).
  - Viewer backend: rerun-web (default), rerun-native, or foxglove.
  - Exploration is started manually from the UI.
EOF
}

# Allow a small set of friendly flags and forward the rest.
RUN_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --robot-ip)
      ROBOT_IP="${2:-}"
      shift 2
      ;;
    --robot-ip=*)
      ROBOT_IP="${1#*=}"
      shift 1
      ;;
    --workers|--n-dask-workers)
      N_DASK_WORKERS="${2:-}"
      shift 2
      ;;
    --workers=*|--n-dask-workers=*)
      N_DASK_WORKERS="${1#*=}"
      shift 1
      ;;
    --memory-limit)
      MEMORY_LIMIT="${2:-}"
      shift 2
      ;;
    --memory-limit=*)
      MEMORY_LIMIT="${1#*=}"
      shift 1
      ;;
    --sam3-model|--sam3-model-path)
      SAM3_MODEL_PATH="${2:-}"
      shift 2
      ;;
    --sam3-model=*|--sam3-model-path=*)
      SAM3_MODEL_PATH="${1#*=}"
      shift 1
      ;;
    --ui-port|--sam3-ui-port)
      SAM3_UI_PORT="${2:-}"
      shift 2
      ;;
    --ui-port=*|--sam3-ui-port=*)
      SAM3_UI_PORT="${1#*=}"
      shift 1
      ;;
    --viewer|--viewer-backend)
      VIEWER_BACKEND="${2:-}"
      shift 2
      ;;
    --viewer=*|--viewer-backend=*)
      VIEWER_BACKEND="${1#*=}"
      shift 1
      ;;
    --no-browser|--no-open-browser)
      OPEN_BROWSER=0
      shift 1
      ;;
    --open-browser)
      OPEN_BROWSER=1
      shift 1
      ;;
    --)
      shift
      RUN_ARGS+=("$@")
      break
      ;;
    *)
      RUN_ARGS+=("$1")
      shift 1
      ;;
  esac
done

# Default for this setup (can be overridden via env or flags above).
ROBOT_IP="${ROBOT_IP:-192.168.8.184}"
echo "Using robot IP: $ROBOT_IP"

# SAM3 weights (required)
SAM3_MODEL_PATH="${SAM3_MODEL_PATH:-${DIMOS_SAM3_MODEL:-}}"
if [[ -z "$SAM3_MODEL_PATH" && -f "$ROOT_DIR/dimos/sam3.pt" ]]; then
  SAM3_MODEL_PATH="$ROOT_DIR/dimos/sam3.pt"
fi

if [[ -z "$SAM3_MODEL_PATH" ]]; then
  echo "ERROR: SAM3_MODEL_PATH is not set and '$ROOT_DIR/dimos/sam3.pt' does not exist." >&2
  echo "Set SAM3_MODEL_PATH (or DIMOS_SAM3_MODEL) to the SAM3 weights file." >&2
  exit 1
fi

if [[ ! -f "$SAM3_MODEL_PATH" ]]; then
  echo "ERROR: SAM3 weights not found at: $SAM3_MODEL_PATH" >&2
  exit 1
fi

export SAM3_MODEL_PATH

# Optional UI port (defaults to 7788)
export SAM3_UI_PORT="${SAM3_UI_PORT:-7788}"

# Optional: control browser auto-open (OPEN_BROWSER=0/1)
if [[ -n "${OPEN_BROWSER:-}" ]]; then
  export OPEN_BROWSER
fi

# Sit once when the target is reached and centered in front.
export SAM3_SIT_ON_ARRIVAL="${SAM3_SIT_ON_ARRIVAL:-1}"

# Global CLI options must appear before `run` for Typer.
GLOBAL_ARGS=(--robot-ip "$ROBOT_IP")
if [[ -n "${N_DASK_WORKERS:-}" ]]; then
  GLOBAL_ARGS+=(--n-dask-workers "$N_DASK_WORKERS")
fi
if [[ -n "${MEMORY_LIMIT:-}" ]]; then
  GLOBAL_ARGS+=(--memory-limit "$MEMORY_LIMIT")
fi
if [[ -n "${VIEWER_BACKEND:-}" ]]; then
  GLOBAL_ARGS+=(--viewer-backend "$VIEWER_BACKEND")
fi

# Pass robot ip as a global CLI option (Typer requires it before `run`).
exec ".venv/bin/dimos" "${GLOBAL_ARGS[@]}" run unitree-go2-sam3-explore "${RUN_ARGS[@]}"
