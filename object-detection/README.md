## Grounded-SAM-2 integration plan (Go2)

### What was added
- Cloned https://github.com/IDEA-Research/Grounded-SAM-2 into `object-detection/Grounded-SAM-2`.
- Reusable detector wrapper: `object-detection/go2_grounded_sam2.py`.
- ROS 2 node: `src/ros2/go2_object_detection_node.py` (starts from `scripts/run_object_detection.sh`).
- RViz overlays: `/go2/object_detections/image` is pre-wired into `nav2/go2_nav2*.rviz`.
- Local sanity script: `object-detection/run_prompt_demo.py` (uses the apple pie image in `assets/2025_apple-pie.jpg`).

### Runtime architecture
1. Isaac Sim already publishes `/unitree_go2/front_cam/color_image`.
2. Detector node subscribes to that topic, runs GroundingDINO (HF) + optional SAM 2 masks, and throttles to `--max-hz` (default 2 Hz) to keep ROS responsive.
3. Outputs:
   - `/go2/object_detections` (`vision_msgs/Detection2DArray`) with pixel-space boxes + scores.
   - `/go2/object_detections/image` (`sensor_msgs/Image`) with bounding boxes drawn for RViz.
4. Prompt control:
   - ROS param: `ros2 param set /go2_grounded_sam2 prompt_text "forklift"`.
   - Topic: publish `std_msgs/String` to `/go2/object_detection/prompt`.
5. Logging: every frame prints detection counts + latency.

### Install (CPU-friendly by default)
Use Python 3.10 (matches ROS Humble tooling):
```bash
python3.10 -m pip install -r object-detection/requirements.txt
```

Optional SAM 2 masks (downloads several GB):
```bash
cd object-detection/Grounded-SAM-2
bash checkpoints/download_ckpts.sh
```
Then start the node with `--enable-masks`.

### Quick verification with the apple-pie image
```bash
python3.10 object-detection/run_prompt_demo.py \
  --prompt "apple pie" \
  --image assets/2025_apple-pie.jpg
# Output saved to outputs/object_detection/apple_pie_detection.jpg
```

### Run alongside Go2 sim + Nav2
Terminal 1: simulator (`./scripts/run_go2.sh`)

Terminal 2: Nav2 + RViz (`./scripts/run_nav2_slam.sh`)

Terminal 3: detector (runs in the same ROS env as Nav2):
```bash
./scripts/run_object_detection.sh \
  --prompt-text "person." \
  --image-topic /unitree_go2/front_cam/color_image \
  --max-hz 2.0
```
RViz already contains a display named “Go2 Detection Image” bound to `/go2/object_detections/image`.

### Notes / knobs
- Change the GroundingDINO model: `--model-id IDEA-Research/grounding-dino-base`.
- Bump thresholds for stricter boxes: `--box-threshold 0.4 --text-threshold 0.35`.
- If running on GPU: add `--device cuda` (auto-detected otherwise).
- If you prefer publishing only when new prompts arrive, set `--max-hz` low (e.g., 0.5) to reduce load.
