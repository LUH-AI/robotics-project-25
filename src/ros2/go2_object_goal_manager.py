"""ROS 2 helper node: switch from frontier exploration to object pursuit.

Listens to `/go2/object_detections` (SAM3 realtime output). When detections
match the requested label a configurable number of times, the node cancels the
frontier explorer process and sends a NavigateToPose goal to Nav2.

The target location defaults to the detection cube location used inside the
simulator (3m ahead, 1m to the left in the map frame), but can be overridden via
`GO2_OBJECT_TARGET="x,y,yaw"`.
"""

from __future__ import annotations

import math
import os
import signal
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

try:  # vision_msgs is optional but required for detections
    from vision_msgs.msg import Detection2DArray
except Exception as exc:  # pragma: no cover - ROS only
    raise RuntimeError(
        "vision_msgs is required for detection-driven navigation. "
        "Install ros-humble-vision-msgs inside your ROS environment."
    ) from exc


def _parse_target(raw: str | None) -> tuple[float, float, float]:
    if not raw:
        return (3.0, 1.0, 0.0)
    try:
        parts = [float(p.strip()) for p in raw.split(",") if p.strip()]
        if len(parts) == 2:
            return (parts[0], parts[1], 0.0)
        if len(parts) >= 3:
            return (parts[0], parts[1], parts[2])
    except Exception:
        pass
    return (3.0, 1.0, 0.0)


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class ObjectPursuitNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_object_pursuit")
        
        # TF Buffer for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.required_hits = int(os.environ.get("GO2_DETECTIONS_REQUIRED", "3"))
        self.target_label = os.environ.get("GO2_OBJECT_LABEL", "").strip().lower()
        pid_file = os.environ.get("GO2_FRONTIER_PID_FILE", "").strip()
        self.frontier_pid_file = Path(pid_file) if pid_file else None
        self._detections_seen = 0
        self._goal_in_flight = False
        self._goal_future = None
        self._goal_handle = None
        
        # Store last detection for calculating goal
        self._last_detection_bbox = None  # (center_x, center_y, image_width, image_height)
        
        # Camera parameters (Go2 front camera from sim)
        self.camera_hfov = 69.4  # degrees (horizontal field of view)
        self.approach_distance = 2.0  # meters to move per update (increased for closer approach)
        
        # Visual servoing parameters
        self.target_bbox_percentage = 0.50  # Stop when bbox is 50% of frame (close!)
        self.stable_bbox_percentage = 0.40  # Can stop at 40% if stable (not growing)
        self.min_bbox_percentage = 0.05     # Ignore detections < 5% of frame (noise)
        self.goal_update_rate = 0.5         # seconds between goal updates
        self.last_goal_update_time = 0.0
        self.pursuit_start_time = None      # Track when pursuit started
        self.min_pursuit_time = 5.0         # Minimum 5 seconds before can stop
        
        # Stability tracking - detect when bbox stops growing
        self.bbox_history = []              # Last N bbox sizes
        self.bbox_stable_count = 0          # How many times bbox was stable
        self.bbox_stable_threshold = 3      # Need 3 stable readings

        topic = os.environ.get("GO2_DETECTION_TOPIC", "/go2/object_detections")
        self.get_logger().info(
            "Watching %s (target label='%s', hits=%d, visual servoing enabled)"
            % (topic, self.target_label or "any", self.required_hits)
        )
        self.det_sub = self.create_subscription(
            Detection2DArray, topic, self._detection_cb, 10
        )
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
        # Publisher for status GUI
        from std_msgs.msg import String, Bool
        self.mode_pub = self.create_publisher(String, "/go2/agent_mode", 10)
        self.exploration_pub = self.create_publisher(Bool, "/go2/exploration/enable", 10)
        
        # Subscribe to prompt changes to reset search
        self.prompt_sub = self.create_subscription(
            String, "/go2/object_detection/prompt", self._prompt_cb, 10
        )
        
        self._publish_mode("SEARCHING")
        self.create_timer(1.0, self._log_status)

    # ------------------------------------------------------------------
    def _publish_mode(self, mode: str) -> None:
        """Publish current mode for status GUI"""
        from std_msgs.msg import String
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
    
    def _log_status(self) -> None:
        if self._goal_in_flight:
            return
        if self._detections_seen > 0:
            self.get_logger().info(
                "Waiting for %d/%d detection confirmations..."
                % (self._detections_seen, self.required_hits)
            )
    
    def _prompt_cb(self, msg) -> None:
        """Reset search when new prompt is entered"""
        if self._goal_in_flight:
            # Cancel current pursuit
            if self._goal_handle:
                self.get_logger().info("Cancelling current pursuit goal for new search...")
                cancel_future = self._goal_handle.cancel_goal_async()
                self._goal_in_flight = False
        
        # Reset detection counter and pursuit timer
        self._detections_seen = 0
        self.pursuit_start_time = None
        self._publish_mode("SEARCHING")
        
        # Restart exploration
        from std_msgs.msg import Bool
        enable_msg = Bool()
        enable_msg.data = True
        self.exploration_pub.publish(enable_msg)
        
        self.get_logger().info(f"New search started with prompt: '{msg.data}' - exploration restarted")

    def _detection_cb(self, msg: Detection2DArray) -> None:
        if not msg.detections:
            return
        
        # Find matching detection and store its bounding box
        matched_detection = None
        if self.target_label:
            for detection in msg.detections:
                for result in detection.results:
                    label = getattr(result.hypothesis, "class_id", "")
                    if isinstance(label, str) and label.lower() == self.target_label:
                        matched_detection = detection
                        break
                if matched_detection:
                    break
            if not matched_detection:
                return
        else:
            # Take first detection if no specific label
            matched_detection = msg.detections[0]
        
        # Get bbox info
        bbox = matched_detection.bbox
        bbox_width = bbox.size_x
        bbox_height = bbox.size_y
        
        # Calculate bbox as percentage of frame
        img_w, img_h = 640.0, 480.0
        bbox_pct_w = bbox_width / img_w
        bbox_pct_h = bbox_height / img_h
        bbox_percentage = max(bbox_pct_w, bbox_pct_h)
        
        # Store bbox info (center position + image dimensions + percentage)
        self._last_detection_bbox = (
            bbox.center.position.x,
            bbox.center.position.y,
            640.0,  # image width
            480.0,  # image height
            bbox_percentage
        )
        
        # Check if object is close enough (bbox is large AND enough time passed)
        if self._goal_in_flight:
            import time
            time_pursuing = time.time() - self.pursuit_start_time if self.pursuit_start_time else 0
            
            # Track bbox history for stability check
            self.bbox_history.append(bbox_percentage)
            if len(self.bbox_history) > 5:
                self.bbox_history.pop(0)  # Keep last 5
            
            # Check if bbox is stable (not growing much)
            is_stable = False
            if len(self.bbox_history) >= 3:
                recent = self.bbox_history[-3:]
                max_diff = max(recent) - min(recent)
                is_stable = max_diff < 0.03  # Less than 3% variance = stable
            
            # Stop if: (reached 50% target) OR (at 40%+ and stable and enough time)
            stop_condition_met = (
                (bbox_percentage >= self.target_bbox_percentage) or
                (bbox_percentage >= self.stable_bbox_percentage and is_stable)
            ) and time_pursuing >= self.min_pursuit_time
            
            if stop_condition_met:
                reason = "target 50%" if bbox_percentage >= 0.50 else "stable at 40%+"
                self.get_logger().info(
                    f"Object reached ({reason})! Bbox={bbox_percentage*100:.1f}% "
                    f"(target={self.target_bbox_percentage*100:.0f}%), pursued for {time_pursuing:.1f}s"
                )
                # Cancel goal and mark as complete
                if self._goal_handle:
                    self._goal_handle.cancel_goal_async()
                self._goal_in_flight = False
                self.pursuit_start_time = None
                self.bbox_history = []
                self._publish_mode("PURSUIT")  # Search complete
                self.destroy_node()
                rclpy.shutdown()
                return
        
        # Ignore tiny detections (noise/far away)
        if bbox_percentage < self.min_bbox_percentage:
            return
        
        # If not pursuing yet, count confirmations
        if not self._goal_in_flight:
            self._detections_seen += 1
            self.get_logger().info(
                f"Detection {self._detections_seen}/{self.required_hits} - "
                f"bbox @ ({bbox.center.position.x:.0f}, {bbox.center.position.y:.0f}) "
                f"coverage={bbox_percentage*100:.1f}%"
            )
            if self._detections_seen >= self.required_hits:
                self._begin_object_pursuit()
        else:
            # Already pursuing - update goal based on current detection
            import time
            now = time.time()
            if now - self.last_goal_update_time >= self.goal_update_rate:
                self._update_pursuit_goal()
                self.last_goal_update_time = now

    def _calculate_goal_from_bbox(self):
        """Calculate Nav2 goal position from current detection bbox"""
        if not self._last_detection_bbox:
            return None
        
        # Get robot's current pose
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'unitree_go2/base_link', rclpy.time.Time()
            )
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y
            
            # Get robot's current yaw
            quat = trans.transform.rotation
            siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            
        except Exception as e:
            self.get_logger().error(f"Could not get robot pose: {e}")
            return None
        
        # Calculate bearing to object from bbox
        bbox_cx, _, img_w, _, bbox_size = self._last_detection_bbox
        
        # Horizontal offset from image center (normalized -0.5 to 0.5)
        offset_normalized = (bbox_cx - img_w / 2.0) / img_w
        
        # Convert to angle using camera HFOV
        bearing_offset = offset_normalized * math.radians(self.camera_hfov)
        
        # Object direction = robot yaw + bearing offset
        object_direction = robot_yaw + bearing_offset
        
        # Goal position: approach_distance meters in direction of object
        goal_x = robot_x + self.approach_distance * math.cos(object_direction)
        goal_y = robot_y + self.approach_distance * math.sin(object_direction)
        
        return (goal_x, goal_y, object_direction, bbox_size)
    
    def _begin_object_pursuit(self) -> None:
        if self._goal_in_flight:
            return
        if not self._last_detection_bbox:
            self.get_logger().error("No detection bbox stored!")
            return
            
        self._goal_in_flight = True
        self._publish_mode("PURSUING")  # Notify GUI
        self._stop_frontier_process()
        
        # Start pursuit timer
        import time
        self.pursuit_start_time = time.time()
        
        # Send initial goal
        self._send_pursuit_goal()
    
    def _send_pursuit_goal(self):
        """Send/update Nav2 goal based on current detection"""
        goal_data = self._calculate_goal_from_bbox()
        if not goal_data:
            self.get_logger().error("Failed to calculate goal from bbox")
            self._goal_in_flight = False
            return
        
        goal_x, goal_y, object_direction, bbox_percentage = goal_data
        
        import time
        time_pursuing = time.time() - self.pursuit_start_time if self.pursuit_start_time else 0
        
        self.get_logger().info(
            f"Visual servoing: goal=({goal_x:.2f}, {goal_y:.2f}) "
            f"bbox={bbox_percentage*100:.1f}% time={time_pursuing:.1f}s"
        )
        
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 action server not available")
            self._goal_in_flight = False
            return

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quaternion(object_direction)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Cancel old goal if exists
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
        
        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)
    
    def _update_pursuit_goal(self):
        """Update pursuit goal based on new detection (visual servoing)"""
        self._send_pursuit_goal()

    def _goal_response_cb(self, future) -> None:
        try:
            self._goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - future errors
            self.get_logger().error(f"Failed to send goal: {exc}")
            self._goal_in_flight = False
            return
        if not self._goal_handle.accepted:
            self.get_logger().warning("NavigateToPose goal rejected")
            self._goal_in_flight = False
            return
        self.get_logger().info("Object pursuit goal accepted; waiting for result...")
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future) -> None:
        try:
            result = future.result()
            status = getattr(result, "status", None)
            self.get_logger().info(
                f"Object pursuit finished (status={status}). Mission complete."
            )
        except Exception as exc:  # pragma: no cover - future errors
            self.get_logger().error(f"Object pursuit failed: {exc}")
        finally:
            self._goal_in_flight = False
            self.destroy_node()
            rclpy.shutdown()

    # ------------------------------------------------------------------
    def _stop_frontier_process(self) -> None:
        pid = self._read_frontier_pid()
        if pid is None:
            self.get_logger().warning("Frontier PID file missing; nothing to stop")
            return
        self.get_logger().info(f"Stopping frontier explorer process (pid={pid})")
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                os.kill(pid, sig)
                break
            except ProcessLookupError:
                break
            except PermissionError:
                continue
        if self.frontier_pid_file:
            try:
                if self.frontier_pid_file.exists():
                    self.frontier_pid_file.unlink()
            except Exception:
                pass

    def _read_frontier_pid(self) -> Optional[int]:
        if not self.frontier_pid_file or not self.frontier_pid_file.exists():
            return None
        try:
            raw = self.frontier_pid_file.read_text().strip()
            return int(raw)
        except Exception:
            return None


def main() -> None:
    rclpy.init()
    node = ObjectPursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Object pursuit interrupted")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
