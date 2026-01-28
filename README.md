# RL Project 2025/26: Object Localization with the Unitree Go2 Robot

## Overview
This repository contains code for a research project exploring object localization using the
Unitree Go2 robot equipped with a camera. The system leverages the SAM3 model for
segmentation based on text prompts, enabling the robot to identify and navigate towards specified objects
in its environment.

The main components of the system include:
- **SAM3 Object Detection**: A segmentation model that processes camera images to identify objects based on text prompts.
- **Mapping and Navigation**: The robot builds a voxel map of its environment and uses pathplanning for navigation through the map.
- **Exploration Strategy**: A wavefront frontier exploration algorithm is used to discover new areas and explore the map while searching for the specified object.
- **User Interface**: A web-based UI allows users to input text prompts and visualize the robot's camera feed with segmentation overlays.
---

## Setup

### 1) System prerequisites (Ubuntu)

On Ubuntu 22.04/24.04, install a minimal set of deps used by DimOS:

```bash
sudo apt-get update
sudo apt-get install -y curl g++ portaudio19-dev git-lfs libturbojpeg python3-dev
# sudo apt-get update
# sudo apt-get install -y git git-lfs curl g++ portaudio19-dev libturbojpeg0-dev python3-dev
# git lfs install
```

If you plan to run SAM3 on GPU, make sure your NVIDIA/CUDA setup is working. (If you don’t have CUDA,
you can disable the “require CUDA” check via `SAM3_REQUIRE_CUDA=0`.)

### 2) Python environment (uv + venv)

From the repo root:

```bash
cd dimos
curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv && . .venv/bin/activate
uv pip install 'dimos[base,unitree]'
```

DimOS requires Python `>=3.10` (see `dimos/pyproject.toml`).

<!-- ### 3) SAM3 weights

You need a weights file on disk and an env var pointing at it:

- Put it at `dimos/sam3.pt` (auto-detected), **or**
- Export `SAM3_MODEL_PATH=/absolute/path/to/sam3.pt` (also accepts `DIMOS_SAM3_MODEL`).

--- -->

## Running it on the Robot

From the repo root:

```bash
./run_sam3_exploration.sh --robot-ip <GO2_IP>
```

Then open the UI:
- `http://localhost:7788` (default)

Useful flags:

```bash
# change UI port
./run_sam3_exploration.sh --ui-port 7790 --robot-ip <GO2_IP>

# choose a viewer backend (rerun-web is the default in the script)
./run_sam3_exploration.sh --viewer-backend rerun-web --robot-ip <GO2_IP> # prefered
./run_sam3_exploration.sh --viewer-backend rerun-native --robot-ip <GO2_IP> 
./run_sam3_exploration.sh --viewer-backend foxglove --robot-ip <GO2_IP>
```

### Configuration via `.env`

The script loads `dimos/.env` if present. Common knobs:
- `ROBOT_IP` (if you don’t want to pass `--robot-ip`)
- `SAM3_MODEL_PATH`, `SAM3_UI_PORT`
- `N_DASK_WORKERS`, `MEMORY_LIMIT` (helps keep large weights stable)
- `OPEN_BROWSER=0/1`

---

## Architecture (High Level)

The runnable you start is:

- `run_sam3_exploration.sh` → launches `dimos` CLI with `run unitree-go2-sam3-explore`
- Blueprint wiring: `dimos/dimos/robot/unitree_webrtc/unitree_go2_sam3_blueprints.py`

That blueprint composes these main modules:
- **Go2 WebRTC connection**: streams camera/telemetry + accepts motion commands
- **Mapping**: voxel map → costmap
- **Navigation**: replanning A* planner
- **Exploration**: wavefront frontier exploration (started/stopped from the UI)
- **SAM3 exploration UI**: `dimos/dimos/robot/unitree_webrtc/sam3_exploration.py`
  - Serves a small FastAPI web app (prompt + live annotated stream + logs)
  - Runs SAM3 text-prompt segmentation on the live `color_image`
  - When a target is confirmed, stops exploration and sends navigation goals to approach it

Data flow (simplified): camera → SAM3 segmentation → target selection → navigation goal updates → Go2.

---
