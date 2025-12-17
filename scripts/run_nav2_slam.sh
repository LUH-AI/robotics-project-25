#!/usr/bin/env bash
set -euo pipefail

# One-shot launcher for Go2 Nav2 + SLAM.
# - Starts pointcloud_to_laserscan (PointCloud2 -> /scan)
# - Starts nav2_bringup in SLAM mode
# - If a GUI is available, auto-opens RViz with the Nav2 default view.
#
# Disable auto-RViz by setting GO2_NO_RVIZ=1.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARAMS_FILE="$ROOT_DIR/nav2/nav2_slam_params.yaml"
PC2LS_PARAMS="$ROOT_DIR/nav2/pointcloud_to_laserscan.yaml"
MAP_FILE="$ROOT_DIR/nav2/empty_map.yaml"
STATE_DIR="${ROS_HOME:-$HOME/.ros}/go2_nav2_slam"
PID_FILE="$STATE_DIR/pids"
FAST_DDS_PROFILE="$ROOT_DIR/nav2/fastdds_no_shm.xml"

# Thin clients often need software GL to keep RViz from crashing.
if [[ "${GO2_RVIZ_SOFTWARE:-0}" == "1" ]] || [[ -n "${THINLINCSESSION:-}" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
fi

ensure_ros2() {
  if command -v ros2 >/dev/null 2>&1; then
    return 0
  fi

  # Try to auto-activate a conda env (robostack) if available.
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
    # Robostack activation scripts are not `set -u` clean (they reference variables
    # like CONDA_BUILD). Temporarily disable nounset during activation.
    set +u
    # shellcheck disable=SC1090
    source "$conda_sh"
    if command -v conda >/dev/null 2>&1; then
      conda activate "$env_name" >/dev/null 2>&1 || true
    fi
    set -u
  fi

  if ! command -v ros2 >/dev/null 2>&1; then
    echo "[go2_nav2] ERROR: 'ros2' not found in PATH." >&2
    echo "[go2_nav2] Activate your ROS2 Humble environment, e.g.:" >&2
    echo "  source /opt/conda/etc/profile.d/conda.sh" >&2
    echo "  conda activate ${GO2_ROS_ENV:-ros2_humble}" >&2
    return 127
  fi
}

echo "[go2_nav2] Using params: $PARAMS_FILE"
echo "[go2_nav2] Using pointcloud_to_laserscan params: $PC2LS_PARAMS"
echo "[go2_nav2] Using dummy map: $MAP_FILE"

cleanup() {
  rm -f "$PID_FILE" 2>/dev/null || true
  if [[ -n "${IMAGE_VIEW_PID-}" ]] && kill -0 "$IMAGE_VIEW_PID" 2>/dev/null; then
    kill -TERM "$IMAGE_VIEW_PID" 2>/dev/null || true
  fi
  if [[ -n "${PROMPT_GUI_PID-}" ]] && kill -0 "$PROMPT_GUI_PID" 2>/dev/null; then
    kill -TERM "$PROMPT_GUI_PID" 2>/dev/null || true
  fi
  if [[ -n "${RVIZ_PID-}" ]] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill -TERM "$RVIZ_PID" 2>/dev/null || true
  fi
  if [[ -n "${DETECTION_PID-}" ]] && kill -0 "$DETECTION_PID" 2>/dev/null; then
    pgid="$(ps -o pgid= -p "$DETECTION_PID" 2>/dev/null | tr -d ' ' || true)"
    [[ -n "${pgid:-}" ]] && kill -TERM -- "-$pgid" 2>/dev/null || true
    kill -TERM "$DETECTION_PID" 2>/dev/null || true
  fi
  if [[ -n "${NAV2_PID-}" ]] && kill -0 "$NAV2_PID" 2>/dev/null; then
    # Kill the whole process group (ros2 launch + all children). PID != PGID on older runs.
    pgid="$(ps -o pgid= -p "$NAV2_PID" 2>/dev/null | tr -d ' ' || true)"
    [[ -n "${pgid:-}" ]] && kill -TERM -- "-$pgid" 2>/dev/null || true
    kill -TERM "$NAV2_PID" 2>/dev/null || true
  fi
  if [[ -n "${SCAN_RELAY_PID-}" ]] && kill -0 "$SCAN_RELAY_PID" 2>/dev/null; then
    pgid="$(ps -o pgid= -p "$SCAN_RELAY_PID" 2>/dev/null | tr -d ' ' || true)"
    [[ -n "${pgid:-}" ]] && kill -TERM -- "-$pgid" 2>/dev/null || true
    kill -TERM "$SCAN_RELAY_PID" 2>/dev/null || true
  fi
  if [[ -n "${PC2LS_PID-}" ]] && kill -0 "$PC2LS_PID" 2>/dev/null; then
    pgid="$(ps -o pgid= -p "$PC2LS_PID" 2>/dev/null | tr -d ' ' || true)"
    [[ -n "${pgid:-}" ]] && kill -TERM -- "-$pgid" 2>/dev/null || true
    kill -TERM "$PC2LS_PID" 2>/dev/null || true
  fi
  if [[ -n "${FOOTPRINT_TF_PID-}" ]] && kill -0 "$FOOTPRINT_TF_PID" 2>/dev/null; then
    pgid="$(ps -o pgid= -p "$FOOTPRINT_TF_PID" 2>/dev/null | tr -d ' ' || true)"
    [[ -n "${pgid:-}" ]] && kill -TERM -- "-$pgid" 2>/dev/null || true
    kill -TERM "$FOOTPRINT_TF_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ensure_ros2

# Ensure DDS matches Isaac Sim's ROS2 bridge defaults (FastDDS).
export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$FAST_DDS_PROFILE}"
mkdir -p "$STATE_DIR"

# Stop any previous stacks (PID file or best-effort pattern), otherwise you'll get
# duplicate node names + conflicting TF (pose/map will "jump" in RViz).
if [[ "${GO2_SKIP_STOP-0}" != "1" ]]; then
  "$ROOT_DIR/scripts/stop_nav2_slam.sh" --quiet || true
fi

# NOTE: Isaac Sim bridge in this repo timestamps sensor/TF messages using wall-time by default.
# Running Nav2 with sim time enabled will cause TF / message-filter drops.
# Opt-in to sim time with GO2_USE_SIM_TIME=1 once the bridge publishes sim-time stamps.
USE_SIM_TIME="${GO2_USE_SIM_TIME:-0}"
if [[ "$USE_SIM_TIME" == "1" ]]; then
  USE_SIM_TIME_STR="True"
else
  USE_SIM_TIME_STR="False"
fi

start_detection() {
  if [[ "${GO2_SKIP_OBJECT_DETECTION:-0}" == "1" ]]; then
    echo "[go2_nav2] Skipping object detection (GO2_SKIP_OBJECT_DETECTION=1)"
    return
  fi

  local prompt="${GO2_DETECTION_PROMPT:-green cube.}"
  local image_topic="${GO2_DETECTION_IMAGE_TOPIC:-/unitree_go2/front_cam/color_image}"
  local detection_cmd=(
    "$ROOT_DIR/scripts/run_object_detection.sh"
    --prompt-text "$prompt"
    --image-topic "$image_topic"
  )

  # Auto-wire local SAM3 checkpoint if available (allows running without gated HF access).
  if [[ "${GO2_DETECTION_ARGS:-}" != *"--sam3-checkpoint"* ]]; then
    local sam3_ckpt="${GO2_SAM3_CHECKPOINT:-$ROOT_DIR/object-detection/sam3_modelscope/sam3.pt}"
    if [[ -f "$sam3_ckpt" ]]; then
      detection_cmd+=(--sam3-checkpoint "$sam3_ckpt")
    fi
  fi

  if [[ "${GO2_DETECTION_ENABLE_MASKS:-1}" == "1" ]]; then
    detection_cmd+=(--enable-masks)
  fi
  if [[ -n "${GO2_DETECTION_DEVICE:-}" ]]; then
    detection_cmd+=(--device "${GO2_DETECTION_DEVICE}")
  fi
  local EXTRA_ARGS=()
  if [[ -n "${GO2_DETECTION_ARGS:-}" ]]; then
    # shellcheck disable=SC2206
    EXTRA_ARGS=(${GO2_DETECTION_ARGS})
    detection_cmd+=("${EXTRA_ARGS[@]}")
  fi

  echo "[go2_nav2] Starting object detector (prompt='${prompt}')..."
  setsid "${detection_cmd[@]}" &
  DETECTION_PID=$!
}

start_detection

echo "[go2_nav2] Starting base_footprint TF (base_link -> base_footprint)..."
setsid python3 "$ROOT_DIR/scripts/base_footprint_tf.py" \
  --odom-frame odom \
  --base-link unitree_go2/base_link \
  --base-footprint unitree_go2/base_footprint &
FOOTPRINT_TF_PID=$!

echo "[go2_nav2] Starting pointcloud_to_laserscan..."
setsid python3 "$ROOT_DIR/scripts/pc_to_scan.py" \
  --in /unitree_go2/lidar/point_cloud \
  --out /scan_raw \
  --params-file "$PC2LS_PARAMS" &
PC2LS_PID=$!

sleep 2

echo "[go2_nav2] Starting scan QoS relay (/scan_raw -> /scan)..."
setsid python3 "$ROOT_DIR/scripts/scan_qos_relay.py" --in /scan_raw --out /scan &
SCAN_RELAY_PID=$!

echo "[go2_nav2] Starting Nav2 bringup (SLAM mode)..."
setsid ros2 launch nav2_bringup bringup_launch.py \
  slam:=True \
  use_sim_time:="$USE_SIM_TIME_STR" \
  autostart:=True \
  params_file:="$PARAMS_FILE" \
  map:="$MAP_FILE" &
NAV2_PID=$!

cat >"$PID_FILE" <<EOF
FOOTPRINT_TF_PID=$FOOTPRINT_TF_PID
PC2LS_PID=$PC2LS_PID
SCAN_RELAY_PID=$SCAN_RELAY_PID
NAV2_PID=$NAV2_PID
DETECTION_PID=${DETECTION_PID:-}
EOF

# Launch RViz immediately - topics will appear when ready


if [[ "${GO2_NO_RVIZ-0}" != "1" ]] && [[ -n "${DISPLAY-}" ]]; then
  profile="${GO2_RVIZ_PROFILE:-slam}"
  RVIZ_CFG="$ROOT_DIR/nav2/go2_nav2.rviz"
  if [[ "$profile" == "full" ]]; then
    RVIZ_CFG="$ROOT_DIR/nav2/go2_nav2_full.rviz"
  fi
  if [[ -n "${GO2_RVIZ_CFG-}" ]]; then
    # Allow absolute or repo-relative override.
    if [[ "$GO2_RVIZ_CFG" = /* ]]; then
      RVIZ_CFG="$GO2_RVIZ_CFG"
    else
      RVIZ_CFG="$ROOT_DIR/$GO2_RVIZ_CFG"
    fi
  fi
  if [[ ! -f "$RVIZ_CFG" ]]; then
    RVIZ_CFG="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz"
  fi
  echo "[go2_nav2] Launching RViz: $RVIZ_CFG"
  # RViz can segfault on remote desktops (e.g. ThinLinc/Mesa GL). Keep Nav2 running even if RViz crashes.
  (
    set +e
    rviz2 -d "$RVIZ_CFG"
    code=$?
    echo "[go2_nav2] RViz exited (code=$code). Nav2+SLAM is still running." >&2
    echo "[go2_nav2] Tip: set GO2_NO_RVIZ=1 and run RViz on a machine with proper OpenGL." >&2
  ) &
  RVIZ_PID=$!
  echo "[go2_nav2] RViz PID: $RVIZ_PID (Ctrl-C here will stop Nav2+SLAM)"

  # Optional convenience: open the annotated camera stream in a dedicated window.
  # RViz's Image display renders inside the main render panel; rqt_image_view gives a
  # straightforward "camera feed" window with overlays.
  if [[ "${GO2_NO_IMAGE_VIEW-0}" != "1" ]]; then
    if ros2 pkg prefix rqt_image_view >/dev/null 2>&1; then
      echo "[go2_nav2] Launching rqt_image_view (/go2/object_detections/image)..."
      ( set +e; ros2 run rqt_image_view rqt_image_view /go2/object_detections/image ) &
      IMAGE_VIEW_PID=$!
    else
      echo "[go2_nav2] NOTE: rqt_image_view not installed; skipping camera window." >&2
    fi
    
    # Launch prompt changer GUI for easy live prompt updates
    echo "[go2_nav2] Launching prompt changer GUI..."
    ( set +e; python3 "$ROOT_DIR/src/ros2/prompt_changer_gui.py" ) &
    PROMPT_GUI_PID=$!
  fi

  wait "$NAV2_PID"
else
  echo "[go2_nav2] Nav2+SLAM is running. Open RViz in another terminal if desired:"
  echo "  rviz2 -d $ROOT_DIR/nav2/go2_nav2.rviz"
  echo "[go2_nav2] Press Ctrl-C here to stop Nav2+SLAM."
  wait "$NAV2_PID"
fi
