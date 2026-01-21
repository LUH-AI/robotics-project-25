# 🤖 Autonomous Go2 Navigation & Detection Agent

**Isaac Sim 5.1 + ROS 2 Humble + Nav2 SLAM + SAM3 Object Detection**

An autonomous quadruped robot agent that explores unknown environments, detects objects using real-time SAM3 vision AI, and navigates to targets using frontier-based exploration.

---

## 🚀 Quick Start (Two Terminals)

### Terminal 1: Launch Isaac Sim

```bash
cd ~/Desktop/OPUS_MAX/ISAAC-EXP 

./scripts/run_go2.sh
# Select detection mode:
# 1) EASY   - 1 Cube 5m in front
# 2) MEDIUM - 10 Cubes random (15m radius)  
# 3) HARD   - 1 Cube random (20m radius)
```

Wait for: **"Simulation App Startup Complete"**

### (Alternatively for Deployment) Terminal 1: Launch Bridges

```bash
cd ~/Desktop/OPUS_MAX/ISAAC-EXP 
./scripts/run_own_bridges.sh
```

### Terminal 2: Launch Autonomous Agent

```bash
./scripts/start_agent.sh

# With logging file
./scripts/start_agent.sh > run.log 2>&1


# Both
./scripts/start_agent.sh 2>&1 | tee run.log
```

This will:
- ✅ Launch Nav2 + SLAM
- ✅ Start SAM3 object detection
- ✅ Open RViz visualization
- ✅ Launch frontier explorer
- ✅ Start status GUI

**The robot will automatically explore, find green cubes, and navigate to them!** 🎯

**Changing Search Target:** Use the Status GUI to enter a new prompt (e.g., "plant", "chair") and click "Update" to search for different objects. When the robot reaches a target, click "Start New Search" with a new prompt to begin a fresh exploration cycle.


#### Terminate Nav2:
```bash
./scripts/stop_nav2_slam.sh 
```

### Terminal 3 (Optional): Manual Nav2 Control

For debugging or manual navigation without the autonomous agent:

```bash
./scripts/run_nav2_slam.sh
```

This launches **only** Nav2 + SLAM + SAM3 + RViz, without the frontier explorer. Useful for:
- **Manual navigation**: Use RViz "2D Nav Goal" tool to send waypoints
- **SLAM debugging**: Observe map building in real-time
- **Detection testing**: Monitor `/go2/object_detections` topic
- **Manual robot control**: Use WASD keys in Isaac Sim viewport

---

## 📖 I Want to Understand This Repo

### What is This?

This is an **autonomous agent system** combining:
- **Isaac Sim**: Photo-realistic robotics simulator
- **Unitree Go2**: Quadruped robot with LiDAR + camera
- **Nav2 + SLAM**: Simultaneous mapping and navigation
- **SAM3**: Real-time object detection AI (Segment Anything Model 3)
- **Frontier Explorer**: Autonomous exploration algorithm
- **Object Pursuit**: Goal switching when objects detected

### How Does the Agent Work?

The agent operates in **two modes**:

#### 1. **Exploration Mode** (Default)
- Maps the environment using SLAM (Simultaneous Localization and Mapping)
- Identifies "frontiers" (boundaries between known and unknown space)
- Navigates to frontiers to expand the map
- Continuously scans for target objects using SAM3 vision AI

#### 2. **Pursuit Mode** (Triggered by detection)
- When SAM3 detects a target object **3+ times** (confirmation threshold)
- Switches from exploration to pursuit
- Navigates directly to the detected object's location
- Stops when within goal tolerance

### Detection Cube Modes

Choose difficulty when launching Isaac Sim:

| Mode | Cubes | Distance | Difficulty |
|------|-------|----------|------------|
| **DEBUG** | 1 | 3m straight ahead | Testing only |
| **EASY** | 1 | 5m straight ahead | Beginner |
| **MEDIUM** | 10 | Random 15m radius | Intermediate |
| **HARD** | 1 | Random 20m radius | Expert |

Cubes spawn at **1.5m height** (enough clearance above for safe spawning).

### SAM3 Object Detection

**SAM3** (Segment Anything Model 3) is a state-of-the-art vision model that:
- Runs **real-time** on GPU (~10-15 FPS)
- Detects objects from **text prompts** (e.g., "green cube", "person", "forklift")
- Provides **bounding boxes** and **segmentation masks**
- Works on **any object** without training

**Default Configuration:**
- **Input**: `/unitree_go2/front_cam/color_image` (640x480 RGB camera)
- **Output**: `/go2/object_detections` (vision_msgs/Detection2DArray)
- **Prompt**: `"green cube."` (configurable via `GO2_CUBE_MODE`)
- **Device**: CUDA (GPU accelerated)

**GPU Memory:** ~3.5GB VRAM for SAM3 model + inference buffers

### System Architecture

```
┌─────────────────┐
│   Isaac Sim     │  ← Spawns Go2 + Environment + Cubes
│  (Terminal 1)   │  → Publishes: /tf, /odom, /lidar, /camera
└────────┬────────┘
         │ ROS 2 Topics
         ↓
┌─────────────────────────────────────────────┐
│         Nav2 + SLAM Stack                   │
│  ┌─────────────┐  ┌──────────────┐         │
│  │ SLAM Toolbox│→ │ Costmap 2D   │         │
│  └─────────────┘  └──────────────┘         │
│  ┌─────────────┐  ┌──────────────┐         │
│  │ Nav2 Planner│  │ Controller   │         │
│  └─────────────┘  └──────────────┘         │
└─────────┬───────────────────────────────────┘
          │                      
          ├──→ RViz (Visualization)
          │
┌─────────┴───────────────────────────────────┐
│         Agent Intelligence Layer            │
│  ┌─────────────────┐  ┌──────────────────┐ │
│  │ SAM3 Detector   │  │ Frontier Explorer│ │
│  │ (Vision AI)     │  │ (Autonomy)       │ │
│  └────────┬────────┘  └────────┬─────────┘ │
│           │                    │           │
│  ┌────────┴────────────────────┴─────────┐ │
│  │   Object Goal Manager (Coordinator)   │ │
│  │  • Monitors detections (3x threshold) │ │
│  │  • Switches Explorer ↔ Pursuit mode   │ │
│  │  • Publishes navigation goals         │ │
│  └───────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
```

### Key Components

#### 1. **Frontier Explorer** (`custom_explorer/explorer.py`)
- Analyzes SLAM map to find unexplored areas
- Calculates frontier points (border between known/unknown)
- Sends navigation goals to Nav2
- Vectorized numpy implementation for performance

#### 2. **Object Goal Manager** (`go2_object_goal_manager.py`)
- Subscribes to `/go2/object_detections`
- Counts detections per object class
- Triggers mode switch when threshold reached (default: 3 detections)
- Cancels exploration goal and sends pursuit goal

#### 3. **Status GUI** (`go2_status_gui.py`)
- Real-time tkinter window showing:
  - Current mode (EXPLORING / PURSUING)
  - Detection cube difficulty (EASY/MEDIUM/HARD)
  - Target coordinates
  - Detection count

#### 4. **SAM3 Detector** (`go2_object_detection_node.py`)
- Loads SAM3 model (~3.5GB VRAM)
- Processes camera feed at 5-10 FPS
- Publishes vision_msgs/Detection2DArray
- Broadcasts annotated images for RViz

### Environment Variables

**Isaac Sim (run_go2.sh):**
```bash
GO2_CUBE_MODE=MEDIUM       # Detection difficulty
GO2_HEADLESS=1             # Disable GUI (faster)
GO2_ISAAC_ENV=my_env       # Custom Isaac env name
```

**Agent Stack (start_agent.sh):**
```bash
GO2_ROS_ENV=ros2_humble    # Custom ROS env name
GO2_NO_RVIZ=1              # Skip RViz launch
GO2_SKIP_CLEANUP=1         # Keep zombie processes
```

**Object Detection:**
```bash
GO2_DETECTION_PROMPT="person"        # Change detection target
GO2_DETECTION_DEVICE=cpu             # Use CPU (slow)
GO2_SKIP_OBJECT_DETECTION=1          # Disable SAM3
```

### File Structure

```
ISAAC-EXP/
├── scripts/
│   ├── run_go2.sh              # Launch Isaac Sim (Terminal 1)
│   ├── start_agent.sh          # Launch full agent stack (Terminal 2)
│   ├── run_nav2_slam.sh        # Nav2+SLAM+SAM3 launcher
│   └── stop_nav2_slam.sh       # Clean shutdown
├── src/
│   ├── isaac_go2_ros2.py       # Isaac Sim ROS bridge
│   ├── env/sim_env.py          # Cube spawning logic
│   └── ros2/
│       ├── go2_object_detection_node.py   # SAM3 detector
│       ├── go2_object_goal_manager.py     # Agent coordinator
│       └── go2_status_gui.py              # Status window
├── exploration_algorithm/
│   └── custom_explorer/explorer.py  # Frontier exploration
├── nav2/
│   ├── nav2_slam_params.yaml       # Nav2 configuration
│   ├── pointcloud_to_laserscan.yaml # LiDAR processing
│   └── go2_nav2.rviz               # RViz layout
└── object-detection/
    └── sam3-realtime/              # SAM3 model code
```

---

## Setup (One-Time)

### Prerequisites
- **Isaac Sim 5.1** (or Isaac Lab 0.47+)
- **CUDA 11.8+** (for GPU acceleration)
- **24GB+ RAM, 8GB+ VRAM** (RTX 3060 or better)

### Installation

1. **Clone the repository:**
```bash
git clone <your-repo-url>
cd ISAAC-EXP
```

2. **Setup ROS 2 environment:**
```bash
./scripts/after_clone.sh
# Creates ros2_humble conda env with Nav2, SLAM, object detection
```

3. **Install SAM3:**
```bash
conda activate ros2_humble
pip install -e object-detection/sam3-realtime
```

4. **Download SAM3 weights** (if HuggingFace is gated):
```bash
pip install modelscope
modelscope download --model facebook/sam3 --local_dir object-detection/sam3_modelscope
```

5. **Verify Isaac Sim environment:**
```bash
conda activate env_isaaclab_py311
python -c "import isaaclab; print('Isaac Lab OK')"
```

---

## Usage Examples

### Standard Autonomous Mission
```bash
# Terminal 1
./scripts/run_go2.sh
# Choose: 2 (MEDIUM)

# Terminal 2
./scripts/start_agent.sh
# Watch robot explore and find cubes automatically
```

### Debug Mode (Quick Testing)
```bash
# Terminal 1
GO2_CUBE_MODE=DEBUG ./scripts/run_go2.sh
# Cube spawns 3m in front - easy to test detection

# Terminal 2
./scripts/start_agent.sh
```

### Custom Detection Prompt
```bash
# Detect "person" instead of "green cube"
GO2_DETECTION_PROMPT="person" ./scripts/start_agent.sh
```

### Headless (No GUI, for servers)
```bash
# Terminal 1
GO2_HEADLESS=1 ./scripts/run_go2.sh

# Terminal 2
GO2_NO_RVIZ=1 ./scripts/start_agent.sh
```

---

## 🐛 Troubleshooting

### "CUDA out of memory"
- **Cause**: SAM3 model too large for GPU
- **Fix**: Reduce camera resolution or use CPU mode
  ```bash
  GO2_DETECTION_DEVICE=cpu ./scripts/start_agent.sh
  ```

### "No /scan topic"
- **Cause**: LiDAR not publishing or Isaac Sim paused
- **Fix**: Press PLAY (spacebar) in Isaac Sim viewport

### "Explorer not moving"
- **Cause**: No frontiers found or Nav2 action server busy
- **Fix**: Wait for SLAM to build more map, or move robot manually (WASD keys)

### "RViz crashes immediately"
- **Cause**: OpenGL issues (remote desktop)
- **Fix**: Use software rendering
  ```bash
  GO2_RVIZ_SOFTWARE=1 ./scripts/start_agent.sh
  ```

### "Detection window shows black screen"
- **Cause**: SAM3 still loading or camera not publishing
- **Fix**: Wait ~15-20s for model initialization

---

## 📊 Performance Metrics

**On RTX 4090 + i9-13900K:**
- Isaac Sim: ~60 FPS (GUI), ~200 FPS (headless)
- SAM3 Detection: ~10-15 FPS (640x480)
- Nav2 Planning: ~20 Hz
- SLAM Update: ~5 Hz
- **Total System Latency**: ~200ms (perception → action)

**GPU Memory Usage:**
- Isaac Sim: ~2GB
- SAM3 Model: ~3.5GB
- **Total VRAM**: ~5.5-6GB

---

## 🤝 Contributing

Contributions welcome! Areas for improvement:
- [ ] Multi-object tracking (pursue closest/specific object)
- [ ] Dynamic obstacle avoidance
- [ ] 3D frontier exploration
- [ ] Distributed multi-agent coordination

---

## 📄 License

[Your License Here]

---

## 🙏 Acknowledgments

- **Isaac Sim**: NVIDIA Omniverse Isaac Sim
- **SAM3**: Meta AI Segment Anything Model 3
- **Nav2**: ROS 2 Navigation Stack
- **SLAM Toolbox**: Steve Macenski
- **Frontier Explorer**: Modified from [Autonomous-Explorer-and-Mapper-ros2-nav2](https://github.com/yourusername/explorer)
