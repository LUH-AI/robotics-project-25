# I WANT TO UNDERSTAND THE REPO

This repository is a “glue layer” between two worlds:

On one side you have Isaac Sim / Isaac Lab running a simulated Unitree Go2 with sensors and a controller. On the other side you have a normal ROS 2 Humble navigation stack (Nav2) plus SLAM (slam_toolbox). The job of this repo is to make those two sides behave like one coherent robot system by taking care of environment activation, ROS 2 middleware settings, message compatibility, and the small conversions that Nav2 typically expects.

If you only want to run it, the quick start in `README.md` is enough. This document is for when you want to change things confidently: how data moves through the system, why specific helper nodes exist, which files are “the truth”, and what usually breaks.

## The mental model (two terminals, one robot)

You always run two long‑lived processes:

The simulator process (Terminal 1) is responsible for producing robot state and sensor data and for accepting velocity commands. In practice it runs Python (`src/isaac_go2_ros2.py`) inside an Isaac conda environment and loads Isaac’s ROS 2 bridge libraries so the simulator can publish and subscribe as a ROS 2 participant.

The navigation process (Terminal 2) is responsible for taking those ROS 2 topics and turning them into navigation behavior. It runs inside a ROS 2 Humble environment and launches a small pipeline plus Nav2 bringup in SLAM mode.

Everything is ROS 2 messages over DDS. There is no “special magic connection” besides the Isaac ROS bridge and correct middleware settings.

## What “working” looks like

When the system is healthy, the simulator publishes a TF tree and some sensor topics. Nav2 consumes TF and a 2D LaserScan (`/scan`) and produces a map (`/map`) while also publishing a velocity command (`/cmd_vel`) to drive the robot. RViz visualizes the map, TF, and robot state and lets you send goals.

The minimum things you should be able to observe once everything is running are:

- `/tf` and `/tf_static` exist and are non-empty.
- A LiDAR sensor topic exists (in this repo it is a point cloud at `/unitree_go2/lidar/point_cloud`).
- A LaserScan topic exists at `/scan` (the repo generates it for Nav2).
- Nav2 is publishing `/cmd_vel` and the simulator is reacting (the robot moves when you set a goal in RViz).
- A map topic `/map` exists and changes as you explore.

If any of these are missing, the “why” is almost always in the data flow described below.

## Data flow: from simulator sensors to Nav2

Nav2 historically expects a 2D planar LaserScan for obstacle layers and costmaps. Modern simulators often produce a 3D point cloud. This repo therefore does two conversions before Nav2 sees the data:

First, the simulator publishes a `sensor_msgs/PointCloud2` from the Go2 LiDAR (here: `/unitree_go2/lidar/point_cloud`). A helper node converts this point cloud into a `sensor_msgs/LaserScan` (`/scan_raw`). This step is implemented as a Python script wrapper in `scripts/pc_to_scan.py` and configured via `nav2/pointcloud_to_laserscan.yaml`.

Second, Nav2 can be strict about QoS settings for `/scan` (reliability, history, depth). Simulator bridges sometimes publish with QoS profiles that do not match Nav2’s defaults. To avoid “it exists but Nav2 does not receive it” situations, the repo runs a small QoS relay from `/scan_raw` to `/scan` (`scripts/scan_qos_relay.py`).

Only after those steps does Nav2 start, so it can subscribe to a LaserScan topic with the QoS profile it expects.

This design is intentionally boring: it makes the system debuggable. If mapping fails, you can inspect each stage in isolation: point cloud exists, scan_raw exists, scan exists, Nav2 consumes it.

## TF: why frames matter more than you think

Nav2 is TF‑driven. If TF is wrong or incomplete, everything downstream becomes confusing: costmaps fail, slam_toolbox drops messages, RViz shows warnings, and goals will not be executed.

At a high level, Nav2 wants a consistent transform chain that connects:

- a global frame (often `map`)
- a local odometry frame (often `odom`)
- the robot base frame (often `base_link` or `base_footprint`)

slam_toolbox typically produces `map -> odom`. The robot/simulator produces `odom -> base_link` and static transforms for sensors. If that chain exists, then any sensor frame can be transformed into costmap frames, and navigation works.

This repo adds one small helper TF because Nav2 stacks frequently use `base_footprint` as the base frame for 2D navigation, while simulators and URDFs often expose `base_link`. If the simulator does not provide `base_footprint`, the script `scripts/base_footprint_tf.py` publishes a transform `unitree_go2/base_link -> unitree_go2/base_footprint`. This makes costmap configuration and footprints predictable.

If you change frame names in the simulator, you must keep Nav2 parameters in sync (base frame, odom frame, sensor frame). The fastest way to debug is to open RViz TF display or run `ros2 run tf2_tools view_frames` in the ROS 2 environment and look for disconnected subtrees.

## Time: sim time vs wall time (and why it breaks silently)

ROS 2 nodes can operate either with wall time (the computer clock) or simulated time (from `/clock`). Nav2 and slam_toolbox are extremely sensitive to timestamps because TF lookups and message filters are time‑based. If your sensors or TF are stamped with wall time but Nav2 uses sim time (or vice versa), you will see message drops and “transform not available” errors.

In this repo, the Isaac bridge is treated as wall‑time by default. The Nav2 launcher therefore defaults to not using sim time. There is an opt‑in flag `GO2_USE_SIM_TIME=1` in `scripts/run_nav2_slam.sh`, but you should only use it once you are sure the simulator publishes correct simulated timestamps (including `/clock` and consistent stamping).

Rule of thumb: make all components agree on time, then debug everything else.

## DDS and discovery: why FastDDS is forced

ROS 2 communication is DDS under the hood. Isaac’s ROS bridge uses a specific middleware configuration, and mismatches here cause the most frustrating “topics exist in one terminal but not in the other” problems.

This repo forces:

- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (FastDDS / FastRTPS)
- `ROS_DOMAIN_ID=0` (so both sides join the same domain)

It also sets a FastDDS profile file (`nav2/fastdds_no_shm.xml`) which disables shared memory transport. This is a practical choice when running inside containers or mixed environments where shared memory transport causes discovery or runtime issues.

If you ever need multiple robots, the simplest separation mechanism is `ROS_DOMAIN_ID`. If you change it, change it on both the simulator and navigation side.

## The launcher scripts: what they really do

### `scripts/run_go2.sh`

This script exists because Isaac Sim needs a very specific runtime environment: the right conda env, the right shared libraries for the ROS bridge, and the correct middleware variables. It activates your Isaac env (default `env_isaaclab_py311`), tries to locate the `isaacsim.ros2.bridge` “humble/lib” directory, and adds it to `LD_LIBRARY_PATH`. This is what avoids the common `librmw_*` / C++ ABI errors when a Python process loads ROS 2 bridge libraries built against different libstdc++ versions.

It then exports the ROS 2 settings (FastDDS, domain id) and runs `python src/isaac_go2_ros2.py`.

If you change how Isaac is installed, the only part that usually needs adjustment is how the bridge library path is found. The script already tries multiple strategies and has a fallback path for a known Isaac Sim layout.

### `src/isaac_go2_ros2.py`

This is the core simulator integration. Conceptually, it does three things:

It configures the simulated robot (Go2) and sensors and steps the simulation. It publishes state into ROS 2 (TF, odometry, sensor topics). And it subscribes to velocity commands (`/cmd_vel`) from Nav2 so that the robot in simulation moves based on navigation output.

When you want to change the “robot behavior” or add/remove sensors, this is the file you edit.

### `scripts/run_nav2_slam.sh`

This is the one‑shot launcher for the navigation side. It ensures a ROS 2 environment is active (it can auto‑activate a conda env if `ros2` is not found), exports the same middleware variables as the simulator, and then starts a small set of helper nodes:

- a TF helper that creates `base_footprint`
- a pointcloud → laserscan converter
- a QoS relay to produce `/scan` with Nav2-friendly QoS

After that it launches Nav2 bringup with SLAM enabled and your parameter file (`nav2/nav2_slam_params.yaml`). It waits for key topics to appear and optionally launches RViz with a provided config.

If you want to swap SLAM toolbox settings, costmap configuration, planners, controller parameters, or frames, this script and the YAML in `nav2/` are where you do it.

### `scripts/stop_nav2_slam.sh`

Nav2 launches many processes. If you restart often while developing, you can easily end up with duplicated node names, duplicated TF publishers, and unpredictable behavior. This helper tries to kill the previous process group and related helper scripts. Use it when things start to “feel haunted”.

### `scripts/after_clone.sh`

This is a convenience setup helper. It can create a robostack ROS 2 env with the packages you need and verify that `rclpy` is importable. It does not install Isaac Sim/Lab for you.

## Configuration files: what to edit, in what order

If you want to change navigation behavior, start with `nav2/nav2_slam_params.yaml`. That file controls Nav2 bringup and slam_toolbox parameters. Common changes include:

You might change frame names (base frame, odom frame), robot footprint/radius, costmap resolution, inflation radius, and planner/controller tuning. If your robot seems to “refuse to move”, it is often a costmap/footprint/inflation issue rather than a controller bug.

If you want to change how the LaserScan is generated from the point cloud (height filtering, angle limits, range limits, frame id), edit `nav2/pointcloud_to_laserscan.yaml`. If obstacle avoidance feels wrong, verify that your scan resembles a 2D planar scan and is in the correct frame.

If discovery is flaky or you run in a container/cluster environment, look at `nav2/fastdds_no_shm.xml` and keep the simulator side consistent.

RViz views are in `nav2/go2_nav2.rviz` and `nav2/go2_nav2_full.rviz`. These should stay “visualization only” (do not encode critical system configuration there).

## How to debug without guessing

The fastest debugging strategy is to validate the pipeline in the same order it is built:

First confirm the simulator is actually publishing (`/tf` exists and updates, point cloud topic exists). Then confirm that conversion nodes are producing `/scan_raw` and `/scan`. Only then look at Nav2 logs and SLAM behavior. This avoids losing hours inside planner tuning when the real issue is missing TF or a QoS mismatch.

If something is visible in `ros2 topic list` but a node does not receive it, suspect QoS. The relay exists specifically to make `/scan` “boring and compatible”.

If you see TF warnings, do not tune Nav2 parameters yet. Fix TF first. A correct TF tree makes everything else suddenly readable.

If you change time settings, change them everywhere. Mixed sim time / wall time is the most common cause of intermittent message filter drops.

## Typical modifications and where they live

If you want to add a second sensor (camera, IMU), add it in `src/isaac_go2_ros2.py` and make sure TF includes the sensor frame. If the sensor should influence navigation, you then integrate it on the Nav2 side (e.g., a depth image conversion pipeline, a voxel layer, or a different costmap source).

If you want to change topics (e.g., publish `/scan` directly from Isaac), you can remove the conversion steps in `scripts/run_nav2_slam.sh`. However, keep in mind that you then own QoS compatibility and frame consistency. The current design keeps these issues explicit and local.

If you want to run multiple robots, the simplest starting point is separate `ROS_DOMAIN_ID`s. If you want multiple robots in the same domain, you must namescape topics and TF frames consistently and update Nav2 parameters accordingly.

If you want to change how navigation is launched (SLAM vs localization), the switch is in `scripts/run_nav2_slam.sh` where Nav2 bringup is launched with `slam:=True`. For localization with a saved map, you would typically set `slam:=False` and provide a map YAML, then tune AMCL (or another localization stack).

## A note on simplicity (why things are split this way)

This repo intentionally keeps “the hard parts” split into small, observable pieces. A monolithic launch file that does everything can look cleaner at first, but it makes debugging much harder. Here you can always ask: do I have TF, do I have scan, does Nav2 have map, does cmd_vel move the robot. Each answer points to a specific file and a specific layer.

If you keep that mental model, you can refactor, extend, or replace components without breaking everything at once.

