# Go2 Navigation Stack on Isaac Sim 5.1 + ROS 2 Humble

This repository links Isaac Sim/Isaac Lab, the `isaacsim.ros2.bridge`, and a ROS 2 Nav2 + SLAM stack. The scripts hide all middleware setup so operators can launch the simulator and Nav2 stack with two commands.

---

## 1. Architecture overview

- **Isaac Sim / Isaac Lab (`env_isaaclab_py311`)**  
  Runs `src/isaac_go2_ros2.py`, publishes `/tf`, `/odom`, `/unitree_go2/lidar/point_cloud`, and exposes `/cmd_vel` subscriptions. The script auto-loads the Isaac ROS bridge libraries (FastDDS) and exports `ROS_DISTRO=humble`.

- **ROS 2 Humble (`ros2_humble`)**  
  `scripts/run_nav2_slam.sh` starts three helpers (base-footprint TF, point cloud → laser scan, scan QoS relay) and the Nav2 bringup with `slam_toolbox`. Everything uses FastDDS (`rmw_fastrtps_cpp`) and `ROS_DOMAIN_ID=0` to match the simulator.

---

## 2. Requirements

| Component | Notes |
|-----------|-------|
| Isaac Lab / Isaac Sim 5.1 | Installed inside `env_isaaclab_py311` (override via `GO2_ISAAC_ENV`). |
| ROS 2 Humble (robostack)  | Includes `nav2_bringup`, `slam_toolbox`, `pointcloud_to_laserscan`. Default env name `ros2_humble`, override via `GO2_ROS_ENV`. |
| GPU workstation           | Tested on Ubuntu 22.04, RTX 4090, CUDA/Vulkan stack from NVIDIA driver 550. |

Run `scripts/after_clone.sh` once to create / update the ROS 2 environment automatically.

---

## 3. Running the system

### 3.1 Simulator terminal (Isaac Lab env)

```bash
./scripts/run_go2.sh
# Options:
#   GO2_HEADLESS=1 ./scripts/run_go2.sh        # headless render
#   GO2_ISAAC_ENV=my_env ./scripts/run_go2.sh  # custom Isaac env
#   GO2_HEADLESS=1 GO2_MAX_STEPS=2000 ./scripts/run_go2.sh  # CI-style limited run
```

What happens:
- Activates the Isaac env.
- Finds the `isaacsim.ros2.bridge` library path (fallback to known Kit path) and prepends it to `LD_LIBRARY_PATH`/`PATH`.
- Forces FastDDS (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`) and `ROS_DISTRO=humble`.
- Launches Isaac Sim and the Go2 controller.

Wait until the simulator finishes loading and `/tf` appears (use `ros2 topic list` from another terminal) **before** starting Nav2.

### 3.2 Nav2 + SLAM terminal (ROS 2 Humble env)

```bash
./scripts/run_nav2_slam.sh
# Options:
#   GO2_NO_RVIZ=1 ./scripts/run_nav2_slam.sh        # skip RViz
#   GO2_RVIZ_SOFTWARE=1 ./scripts/run_nav2_slam.sh  # software GL for thin clients
#   GO2_USE_SIM_TIME=1 ./scripts/run_nav2_slam.sh   # opt into sim time
#   GO2_RVIZ_CFG=nav2/go2_nav2_full.rviz            # custom RViz profile
```

What happens:
- Activates the ROS 2 env (robostack).
- Starts `base_link → base_footprint` TF helper, pointcloud-to-scan converter, QoS relay to reliable `/scan`.
- Launches `nav2_bringup` in SLAM mode with `nav2/nav2_slam_params.yaml`.
- Waits for `/scan`, `/map`, `/tf`, then launches RViz (unless disabled).

### 3.3 Stopping

Use Ctrl‑C in each terminal. To guarantee Nav2 shutdown from any shell:

```bash
./scripts/stop_nav2_slam.sh
```

---

## 4. Sanity checks

After `run_go2.sh` finishes loading:

```bash
conda run -n ros2_humble ros2 topic list
# Expect /tf, /tf_static, /unitree_go2/lidar/point_cloud

conda run -n ros2_humble ros2 run tf2_tools view_frames
```

While Nav2 is running:

```bash
conda run -n ros2_humble ros2 topic echo /scan --once
conda run -n ros2_humble ros2 service call /slam_toolbox/async_reset std_srvs/srv/Empty
```

If `/tf` is missing, restart `run_go2.sh` and wait longer; the Nav2 stack must only be launched after the simulator publishes TF.

---

## 5. Repository structure

| Path | Purpose |
|------|---------|
| `scripts/run_go2.sh` | Simulator launcher + ROS bridge bootstrap. |
| `src/isaac_go2_ros2.py` | Actual Isaac Sim entry point (policy, sensors, ROS transport). |
| `scripts/run_nav2_slam.sh` | Nav2 + SLAM helper pipeline. |
| `scripts/stop_nav2_slam.sh` | Best-effort shutdown for Nav2, scan relay, TF helper. |
| `nav2/` | Nav2 params, RViz configs, FastDDS no-SHM profile. |

---

## 6. Troubleshooting

- **`librmw_*` errors during sim launch:** ensure you used `run_go2.sh`; it prepends the bundled ROS libs. If the warning persists, check that `~/miniconda3/envs/env_isaaclab_py311/.../isaacsim.ros2.bridge-*/humble/lib` exists.
- **Nav2 spam “Invalid frame ID 'odom'”:** simulator didn’t publish TF yet or was stopped. Restart `run_go2.sh`, confirm `/tf`, then rerun Nav2.
- **RViz crashes on remote desktop:** set `GO2_RVIZ_SOFTWARE=1` when running Nav2 (forces Mesa software GL).
- **No `/scan` topic:** check `/scan_raw` first. The QoS relay only starts after `pointcloud_to_laserscan` is running.
- **Need logs for support:** run `GO2_HEADLESS=1 ./scripts/run_go2.sh > debug/go2_run.log 2>&1 &` and `GO2_NO_RVIZ=1 ./scripts/run_nav2_slam.sh > debug/nav2_run.log 2>&1`, then share the log tails.

---

## 7. Clean start commands (copy/paste checklist)

```bash
# Terminal 1
cd /path/to/ISAAC-EXP
./scripts/run_go2.sh

# Terminal 2
cd /path/to/ISAAC-EXP
./scripts/run_nav2_slam.sh

# Optional stop
./scripts/stop_nav2_slam.sh
```

That’s the complete workflow—nothing else needs manual export or sourcing. Once both terminals show healthy output (no TF errors, RViz map filling in), you can set Nav2 goals directly from RViz.
