"""ROS 2 node that runs text-prompted detection on the Go2 front camera.

- Subscribes: /unitree_go2/front_cam/color_image (or configurable topic)
- Publishes:  /go2/object_detections        (vision_msgs/Detection2DArray)
              /go2/object_detections/image  (sensor_msgs/Image with boxes)
- Prompt control: set param `prompt_text` or publish std_msgs/String to
  /go2/object_detection/prompt.
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

# Make sure the helper modules + cloned detector repos are importable.
REPO_ROOT = Path(__file__).resolve().parents[2]
DETECTION_ROOT = REPO_ROOT / "object-detection"
GROUNDED_SAM2_ROOT = DETECTION_ROOT / "Grounded-SAM-2"
SAM3_REALTIME_ROOT = DETECTION_ROOT / "sam3-realtime"
for pth in (DETECTION_ROOT, GROUNDED_SAM2_ROOT, SAM3_REALTIME_ROOT):
    if str(pth) not in sys.path:
        sys.path.insert(0, str(pth))

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    try:
        from rclpy.parameter import SetParametersResult
    except ImportError:  # older rclpy exposes it via node
        from rclpy.node import SetParametersResult  # type: ignore
except Exception as exc:  # pragma: no cover - ROS only
    raise RuntimeError(
        "This node requires ROS 2 (rclpy). "
        "Activate your ROS 2 Humble environment before running."
    ) from exc

try:
    from cv_bridge import CvBridge  # type: ignore
except Exception:
    CvBridge = None  # type: ignore

from sensor_msgs.msg import Image  # type: ignore
from std_msgs.msg import String  # type: ignore

try:
    from vision_msgs.msg import (  # type: ignore
        BoundingBox2D,
        Detection2D,
        Detection2DArray,
        ObjectHypothesisWithPose,
    )
    VISION_MSGS_AVAILABLE = True
except Exception:
    VISION_MSGS_AVAILABLE = False

from go2_grounded_sam2 import GroundedSAM2Detector, draw_detections
from go2_sam3_detector import Sam3RealtimeDetector


def _image_to_bgr(msg: Image) -> np.ndarray:
    """Convert a ROS image to BGR numpy array."""
    if CvBridge is not None:
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Manual conversion fallback; handles rgb8/bgr8 encodings.
    dtype = np.uint8
    if msg.encoding.lower() not in ("rgb8", "bgr8"):
        raise RuntimeError(
            f"Unsupported image encoding '{msg.encoding}'. Install cv_bridge for more formats."
        )
    channels = 3
    data = np.frombuffer(msg.data, dtype=dtype)
    image = data.reshape((msg.height, msg.width, channels)).copy()
    if msg.encoding.lower() == "rgb8":
        image = image[:, :, ::-1]
    return image


BACKENDS = ("sam3", "grounded_sam2")


class Go2GroundedSAM2Node(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("go2_grounded_sam2")
        self.prompt_text = args.prompt_text
        self.min_interval = 1.0 / max(args.max_hz, 0.1)
        self._last_inference = 0.0
        self._queue: queue.Queue[tuple[np.ndarray, Image]] = queue.Queue(maxsize=1)

        # Camera topics coming from Isaac / bridges are typically BEST_EFFORT.
        qos_in = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        # RViz Image display defaults to RELIABLE. Publish RELIABLE so it can satisfy
        # both RELIABLE and BEST_EFFORT subscribers.
        qos_out_image = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.image_sub = self.create_subscription(
            Image, args.image_topic, self._image_cb, qos_in
        )
        self.prompt_sub = self.create_subscription(
            String, "/go2/object_detection/prompt", self._prompt_cb, 10
        )

        self.detections_pub = None
        if VISION_MSGS_AVAILABLE:
            self.detections_pub = self.create_publisher(
                Detection2DArray, "/go2/object_detections", 10
            )
        self.image_pub = self.create_publisher(
            Image, "/go2/object_detections/image", qos_profile=qos_out_image
        )

        # Reconfigure thresholds + prompt dynamically.
        self.declare_parameter("prompt_text", self.prompt_text)
        self.declare_parameter("box_threshold", args.box_threshold)
        self.declare_parameter("text_threshold", args.text_threshold)
        self.add_on_set_parameters_callback(self._on_param_update)

        backend = args.backend.lower()
        if backend not in BACKENDS:
            raise ValueError(f"Unsupported backend '{backend}'. Expected one of {BACKENDS}")
        self.backend = backend

        try:
            if backend == "sam3":
                self.detector = Sam3RealtimeDetector(
                    checkpoint_path=args.sam3_checkpoint,
                    device=args.device,
                    compile=args.sam3_compile,
                )
            else:
                self.detector = GroundedSAM2Detector(
                    model_id=args.model_id,
                    device=args.device,
                    box_threshold=args.box_threshold,
                    text_threshold=args.text_threshold,
                    enable_masks=args.enable_masks,
                    sam2_config=args.sam2_config,
                    sam2_checkpoint=args.sam2_checkpoint,
                    verbose=True,
                )
        except Exception as exc:
            self.get_logger().error(str(exc))
            raise

        self._stop_evt = threading.Event()
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"Detector backend={self.backend} ready. Prompt='{self.prompt_text}' | topic={args.image_topic}"
        )

    # -- ROS callbacks -----------------------------------------------------

    def _image_cb(self, msg: Image) -> None:
        try:
            bgr = _image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warning(f"Image conversion failed: {exc}")
            return

        if self._queue.full():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
        try:
            self._queue.put_nowait((bgr, msg))
        except queue.Full:
            pass

    def _prompt_cb(self, msg: String) -> None:
        self.prompt_text = msg.data.strip()
        self.get_logger().info(f"Updated prompt via topic: '{self.prompt_text}'")

    def _on_param_update(self, params):
        updated = []
        for param in params:
            if param.name == "prompt_text":
                self.prompt_text = str(param.value)
                updated.append(f"prompt='{self.prompt_text}'")
            elif param.name == "box_threshold":
                self.detector.box_threshold = float(param.value)
                updated.append(f"box_threshold={self.detector.box_threshold}")
            elif param.name == "text_threshold":
                self.detector.text_threshold = float(param.value)
                updated.append(f"text_threshold={self.detector.text_threshold}")
        if updated:
            self.get_logger().info("Param update: " + ", ".join(updated))
        return SetParametersResult(successful=True)

    # -- Worker loop ------------------------------------------------------

    def _inference_loop(self) -> None:
        bridge = CvBridge() if CvBridge is not None else None
        while not self._stop_evt.is_set():
            try:
                bgr, msg = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

            now = time.time()
            if now - self._last_inference < self.min_interval:
                continue
            self._last_inference = now

            try:
                result = self.detector.detect(bgr, self.prompt_text)
            except Exception as exc:
                self.get_logger().error(f"Inference failed: {exc}")
                continue

            if self.detections_pub is not None:
                self._publish_detections(msg, result)
            if bridge is not None:
                # Select best target and draw with color coding
                annotated = self._draw_detections_with_selection(bgr, result)
                out_msg = bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                out_msg.header = msg.header
                self.image_pub.publish(out_msg)

    def _select_best_target(self, result, img_width, img_height):
        """Select detection closest to image center"""
        if len(result.boxes_xyxy) == 0:
            return None
        
        center_x = img_width / 2.0
        center_y = img_height / 2.0
        
        best_idx = None
        min_distance = float('inf')
        
        for i, box in enumerate(result.boxes_xyxy):
            bbox_cx = (box[0] + box[2]) * 0.5
            bbox_cy = (box[1] + box[3]) * 0.5
            
            # Distance to center
            import math
            dist = math.sqrt((bbox_cx - center_x)**2 + (bbox_cy - center_y)**2)
            
            if dist < min_distance:
                min_distance = dist
                best_idx = i
        
        return best_idx
    
    def _draw_detections_with_selection(self, bgr, result):
        """Draw detections with green for selected, yellow for others"""
        import cv2
        overlay = bgr.copy()
        
        if len(result.boxes_xyxy) == 0:
            return overlay
        
        img_height, img_width = bgr.shape[:2]
        selected_idx = self._select_best_target(result, img_width, img_height)
        
        for i, (box, score, label) in enumerate(zip(result.boxes_xyxy, result.scores, result.labels)):
            x1, y1, x2, y2 = map(int, box)
            
            # Color code: green for selected, yellow for others
            if i == selected_idx:
                color = (0, 255, 0)  # Green (BGR)
                label_text = f"{label} {score:.2f} 🎯"
                thickness = 3
            else:
                color = (0, 255, 255)  # Yellow (BGR)
                label_text = f"{label} {score:.2f}"
                thickness = 2
            
            # Draw bbox
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, thickness)
            
            # Draw label background
            (text_width, text_height), _ = cv2.getTextSize(
                label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            cv2.rectangle(
                overlay,
                (x1, y1 - text_height - 10),
                (x1 + text_width, y1),
                color,
                -1
            )
            
            # Draw label text
            cv2.putText(
                overlay,
                label_text,
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),  # Black text
                2
            )
        
        return overlay
    
    def _publish_detections(self, src_msg: Image, result) -> None:
        """Publish only the selected best target"""
        if len(result.boxes_xyxy) == 0:
            # Publish empty array
            arr = Detection2DArray()
            arr.header = src_msg.header
            self.detections_pub.publish(arr)
            return
        
        # Get image dimensions from message
        img_width = src_msg.width
        img_height = src_msg.height
        
        # Select best target
        selected_idx = self._select_best_target(result, img_width, img_height)
        if selected_idx is None:
            arr = Detection2DArray()
            arr.header = src_msg.header
            self.detections_pub.publish(arr)
            return
        
        # Publish only selected detection
        arr = Detection2DArray()
        arr.header = src_msg.header
        
        box = result.boxes_xyxy[selected_idx]
        score = result.scores[selected_idx]
        label = result.labels[selected_idx]
        
        det = Detection2D()
        det.header = src_msg.header
        bbox = BoundingBox2D()
        # BoundingBox2D.center is a Pose2D (has position.x/position.y, not x/y directly)
        bbox.center.position.x = float((box[0] + box[2]) * 0.5)
        bbox.center.position.y = float((box[1] + box[3]) * 0.5)
        bbox.center.theta = 0.0
        bbox.size_x = float(max(0.0, box[2] - box[0]))
        bbox.size_y = float(max(0.0, box[3] - box[1]))
        det.bbox = bbox

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = label
        hyp.hypothesis.score = float(score)
        det.results.append(hyp)

        arr.detections.append(det)
        self.detections_pub.publish(arr)
        
        # Log selected detection
        self.get_logger().info(
            f"Published selected target: {label} (score={score:.2f})"
        )

    def destroy_node(self) -> None:
        self._stop_evt.set()
        return super().destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Go2 Grounded-SAM-2 detector node")
    parser.add_argument(
        "--image-topic",
        default="/unitree_go2/front_cam/color_image",
        help="Image topic to subscribe to.",
    )
    parser.add_argument(
        "--prompt-text",
        default="green cube.",
        help="Initial text prompt. Can be updated via ROS param or /go2/object_detection/prompt.",
    )
    parser.add_argument(
        "--backend",
        choices=BACKENDS,
        default="sam3",
        help="Detector backend: 'sam3' (default) or 'grounded_sam2'.",
    )
    parser.add_argument(
        "--model-id",
        default="IDEA-Research/grounding-dino-tiny",
        help="HuggingFace model id for Grounding DINO (only used with grounded_sam2 backend).",
    )
    parser.add_argument(
        "--device",
        default=None,
        help="Force device (cuda/cpu). Default: auto-detect.",
    )
    parser.add_argument("--box-threshold", type=float, default=0.3)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument(
        "--max-hz",
        type=float,
        default=2.0,
        help="Max inference rate (Hz) to avoid starving the ROS executor.",
    )
    parser.add_argument(
        "--enable-masks",
        action="store_true",
        help="Enable SAM 2 mask prediction (requires checkpoints, grounded_sam2 backend only).",
    )
    parser.add_argument(
        "--sam2-config",
        default="configs/sam2.1/sam2.1_hiera_l.yaml",
        help="Path to SAM 2 config file (relative to Grounded-SAM-2 repo).",
    )
    parser.add_argument(
        "--sam2-checkpoint",
        default="checkpoints/sam2.1_hiera_large.pt",
        help="Path to SAM 2 checkpoint.",
    )
    parser.add_argument(
        "--sam3-checkpoint",
        default=None,
        help="Optional local checkpoint for SAM3. If unset, downloads via HuggingFace auth cache.",
    )
    parser.add_argument(
        "--sam3-compile",
        action="store_true",
        help="Enable torch.compile when building the SAM3 streaming predictor.",
    )
    return parser.parse_args()


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args()
    rclpy.init(args=argv)
    node = Go2GroundedSAM2Node(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
