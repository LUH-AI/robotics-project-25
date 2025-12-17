# Go2 Navigation Stack (Isaac Sim 5.1 + ROS 2 Humble)

## Quick start (two terminals)

Terminal 1 (sim):

```bash
./scripts/run_go2.sh
```

Terminal 2 (Nav2 + SLAM):

```bash
./scripts/run_nav2_slam.sh
```

Stop Nav2/SLAM from any shell:

```bash
./scripts/stop_nav2_slam.sh
```

Wait until the simulator is fully loaded and publishing `/tf` before starting Nav2.

## What this repo is

This repo connects a Unitree Go2 simulation in Isaac Sim/Isaac Lab to a ROS 2 Nav2 + SLAM stack.
The goal is: launch the simulator and get mapping + navigation working without manually sourcing a dozen scripts.

## Setup (one-time)

You need two environments:

- **Isaac env** (default conda env name: `env_isaaclab_py311`): Isaac Lab/Sim 5.1 + Python entrypoint.
- **ROS 2 env** (default conda env name: `ros2_humble`): ROS 2 Humble + `nav2_bringup` + `slam_toolbox` + `pointcloud_to_laserscan`.

If you use robostack, this helper can create/verify the ROS 2 environment:

```bash
./scripts/after_clone.sh
```

This does not install Isaac Sim/Lab for you; it only helps with the ROS 2 side.

If `./scripts/run_go2.sh` cannot find your Isaac env, set:

- `GO2_ISAAC_ENV=your_env_name`

If `./scripts/run_nav2_slam.sh` cannot find your ROS 2 env, set:

- `GO2_ROS_ENV=your_env_name`

## How it works (0 → 100)

There are two processes, and they talk over ROS 2 (FastDDS):

1) **Simulator side** (`./scripts/run_go2.sh`)

- Activates the Isaac conda env.
- Locates Isaac’s ROS 2 bridge libraries (`isaacsim.ros2.bridge`) and adjusts `LD_LIBRARY_PATH` so the bridge can load.
- Forces FastDDS (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`) and `ROS_DOMAIN_ID=0`.
- Runs `src/isaac_go2_ros2.py`, which publishes simulator state and sensors to ROS 2 and subscribes to `/cmd_vel`.

2) **Navigation side** (`./scripts/run_nav2_slam.sh`)

- Activates the ROS 2 conda env (robostack or whatever you use).
- Converts the simulator LiDAR point cloud into a 2D laser scan:
  - input: `/unitree_go2/lidar/point_cloud`
  - output: `/scan_raw`
- Relays `/scan_raw` to `/scan` with a QoS profile Nav2 expects.
- Adds a small TF helper: `unitree_go2/base_link -> unitree_go2/base_footprint`.
- Launches Nav2 in SLAM mode (`slam_toolbox`) using `nav2/nav2_slam_params.yaml`.
- Optionally opens RViz.

Important: the Isaac bridge in this repo timestamps messages using wall-time by default.
Running Nav2 with sim time enabled can break TF/message filters unless the bridge publishes sim-time timestamps.

## Where to change things

- Simulator entrypoint: `src/isaac_go2_ros2.py`
- Nav2 + SLAM launch pipeline: `scripts/run_nav2_slam.sh`
- Nav2 configuration: `nav2/nav2_slam_params.yaml`
- PointCloud → LaserScan parameters: `nav2/pointcloud_to_laserscan.yaml`
- DDS profile (disables shared memory, helps in containers): `nav2/fastdds_no_shm.xml`
- RViz configs: `nav2/go2_nav2.rviz`, `nav2/go2_nav2_full.rviz`

## Useful options (details, not required)

- Headless simulator: `GO2_HEADLESS=1 ./scripts/run_go2.sh`
- Disable RViz: `GO2_NO_RVIZ=1 ./scripts/run_nav2_slam.sh`
- Disable the camera overlay window: `GO2_NO_IMAGE_VIEW=1 ./scripts/run_nav2_slam.sh`
- Force software OpenGL for RViz (remote desktops): `GO2_RVIZ_SOFTWARE=1 ./scripts/run_nav2_slam.sh`
- Opt into sim time (only if your bridge publishes sim time correctly): `GO2_USE_SIM_TIME=1 ./scripts/run_nav2_slam.sh`

## Object detection (SAM3 realtime + Go2 camera)

- `./scripts/run_nav2_slam.sh` now auto-starts the detector with default arguments (prompt `green cube.` on `/unitree_go2/front_cam/color_image`). Disable this with `GO2_SKIP_OBJECT_DETECTION=1 ./scripts/run_nav2_slam.sh`.
- The simulator spawns a bright green cube in the environment by default to make testing easy (toggle with `GO2_ENABLE_DETECTION_CUBE=0`). Default pose can be overridden with `GO2_DETECTION_CUBE_POS="x,y,z"` (meters, env frame). The detector prompt defaults to `green cube.` so you immediately see a bounding box/mask in RViz.
- Detector node: `./scripts/run_object_detection.sh --prompt-text "person."` (runs in the ROS 2 env you use for Nav2) – useful if you want to launch it manually outside the Nav2 script.
- Topics:
  - Camera: `/unitree_go2/front_cam/color_image` (default subscriber)
  - Detections: `/go2/object_detections` (`vision_msgs/Detection2DArray`)
  - Annotated view: `/go2/object_detections/image` (added to `nav2/go2_nav2*.rviz` as “Go2 Detection Image”, and also opened via `rqt_image_view` by default)
  - Prompt updates: set param `prompt_text` or publish `std_msgs/String` to `/go2/object_detection/prompt`
- Install SAM3 realtime fork once per machine (run inside your ROS 2 env, e.g. `ros2_humble`): `pip install -e object-detection/sam3-realtime`.
- Checkpoint note: HuggingFace `facebook/sam3` is gated for many users. If you can’t download via HF, fetch weights via ModelScope and point the detector at `sam3.pt`:
  - `pip install modelscope`
  - `modelscope download --model facebook/sam3 --local_dir object-detection/sam3_modelscope`
  - `GO2_DETECTION_ARGS="--sam3-checkpoint $(pwd)/object-detection/sam3_modelscope/sam3.pt" ./scripts/run_nav2_slam.sh`
- Optional: keep the previous Grounded-SAM-2 pipeline by passing `GO2_DETECTION_ARGS="--backend grounded_sam2 --enable-masks"` (requires checkpoints via `object-detection/Grounded-SAM-2/checkpoints/download_ckpts.sh`).
- Runtime knobs (export before running Nav2):
  - `GO2_DETECTION_PROMPT="forklift."`
  - `GO2_DETECTION_IMAGE_TOPIC=/unitree_go2/front_cam/color_image`
  - `GO2_DETECTION_DEVICE=cuda` (or `cpu`)
  - `GO2_DETECTION_ARGS="--backend grounded_sam2 --enable-masks"` to fall back to the previous detector
  - `GO2_DETECTION_ARGS="--sam3-checkpoint /models/sam3_large.pt"` to point to a local checkpoint
  - `GO2_DETECTION_ARGS="--max-hz 1.0"` for any extra CLI flags
 
## Quick sanity checks

In a ROS 2 terminal (the same one you use for Nav2), after the simulator is up:

```bash
ros2 topic list
```

Expect at least: `/tf`, `/tf_static`, `/unitree_go2/lidar/point_cloud`.

After Nav2 is up:

```bash
ros2 topic echo /scan --once
```

## Troubleshooting (only the common stuff)

- **Nav2 complains about TF / “Invalid frame ID 'odom'”**: start the sim first, wait for `/tf`, then start Nav2.
- **No `/scan`**: check if `/scan_raw` exists; then check `/unitree_go2/lidar/point_cloud`.
- **RViz crashes remotely**: use `GO2_RVIZ_SOFTWARE=1` or run RViz on a machine with a proper OpenGL stack.
