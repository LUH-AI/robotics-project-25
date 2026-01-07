"""ROS 2 helper node: robust visual servoing for object pursuit.

Listens to `/go2/object_detections` (SAM3 realtime output). When detections
match the requested label consistently, switches from exploration to pursuit
using visual servoing with Nav2.

Features:
- Detection instance locking (doesn't jump between targets)
- EMA smoothing for bearing and bbox size
- Dynamic step sizing (rotate-first when off-center)
- Goal update gating (prevents Nav2 thrashing)
- Lost-detection watchdog with auto-return to exploration
"""

from __future__ import annotations

import math
import os
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

try:  # vision_msgs is optional but required for detections
    from vision_msgs.msg import Detection2DArray
except Exception as exc:  # pragma: no cover - ROS only
    raise RuntimeError(
        "vision_msgs is required for detection-driven navigation. "
        "Install ros-humble-vision-msgs inside your ROS environment."
    ) from exc


def _wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]"""
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert yaw (radians) to quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


@dataclass
class _DetMeas:
    """Detection measurement"""
    cx: float               # bbox center x (pixels)
    cy: float               # bbox center y (pixels)
    offset_norm: float      # horizontal offset from center [-0.5, 0.5]
    bbox_pct: float         # max(width, height) as fraction of frame [0, 1]
    score: float           # detection confidence


class ObjectPursuitNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_object_pursuit")
        
        # TF Buffer for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Frames (configurable)
        self.map_frame = os.environ.get("GO2_MAP_FRAME", "map").strip()
        self.base_frame = os.environ.get("GO2_BASE_FRAME", "unitree_go2/base_footprint").strip() # Ch
        
        # Target configuration
        self.required_hits = int(os.environ.get("GO2_DETECTIONS_REQUIRED", "3"))
        self.target_label = os.environ.get("GO2_OBJECT_LABEL", "").strip().lower()
        pid_file = os.environ.get("GO2_FRONTIER_PID_FILE", "").strip()
        self.frontier_pid_file = Path(pid_file) if pid_file else None
        
        # Camera parameters (Go2 front camera from sim)
        self.camera_hfov = 69.4  # degrees (horizontal field of view)
        
        # Detection robustness
        self.min_score = float(os.environ.get("GO2_MIN_DETECTION_SCORE", "0.25"))
        self.candidate_px_gate = float(os.environ.get("GO2_CANDIDATE_PX_GATE", "80"))  # px for hit counting
        self.lock_px_gate = float(os.environ.get("GO2_LOCK_PX_GATE", "140"))          # px when pursuing
        
        # Visual servoing parameters
        self.target_bbox_percentage = 0.50  # Stop when bbox is 50% of frame (close!)
        self.stable_bbox_percentage = 0.40  # Can stop at 40% if stable (not growing)
        self.min_bbox_percentage = 0.05     # Ignore detections < 5% of frame (noise)
        self.goal_update_rate = float(os.environ.get("GO2_GOAL_UPDATE_RATE", "1.5")) # define GO2_GOAL_UPDATE_RATE please
        self.min_pursuit_time = 5.0         # Minimum 5 seconds before can stop
        
        # Pursuit safety/timeouts (ROS time)
        self.lost_detection_timeout = float(os.environ.get("GO2_LOST_DETECTION_TIMEOUT", "4.0"))
        self.abort_after_lost = float(os.environ.get("GO2_ABORT_AFTER_LOST", "20.0"))
        
        # Goal update gating (prevents thrashing)
        self.goal_update_distance = float(os.environ.get("GO2_GOAL_UPDATE_DIST", "0.6"))  # meters
        self.goal_update_yaw = math.radians(float(os.environ.get("GO2_GOAL_UPDATE_YAW_DEG", "10.0")))
        self.goal_hold_distance = float(os.environ.get("GO2_GOAL_HOLD_DIST", "0.9"))
        self.goal_hold_timeout = float(os.environ.get("GO2_GOAL_HOLD_TIMEOUT", "4.0"))
        
        # Dynamic step sizing
        self.max_step = float(os.environ.get("GO2_MAX_STEP", "2.5"))
        self.min_step = float(os.environ.get("GO2_MIN_STEP", "0.4"))
        self.yaw_only_threshold = float(os.environ.get("GO2_YAW_ONLY_OFFSET", "0.35"))  # normalized [-0.5..0.5]
        self.center_tolerance = float(os.environ.get("GO2_CENTER_TOL", "0.08"))
        
        # Smoothing
        self.bearing_ema_alpha = float(os.environ.get("GO2_BEARING_EMA_ALPHA", "0.25"))
        self.bbox_ema_alpha = float(os.environ.get("GO2_BBOX_EMA_ALPHA", "0.25"))
        
        # Internal state
        self._mode = "SEARCHING"
        self._candidate_center: Optional[tuple[float, float]] = None  # (cx,cy)
        self._candidate_hits = 0
        self._locked_center: Optional[tuple[float, float]] = None     # (cx,cy) once pursuing
        
        self._last_meas: Optional[_DetMeas] = None
        self._last_detection_time = 0.0   # ROS seconds
        self._bearing_ema: Optional[float] = None
        self._bbox_ema: Optional[float] = None
        
        self._current_goal: Optional[tuple[float, float, float]] = None  # (x,y,yaw)
        self._goal_seq = 0                # ignore stale callbacks
        self._goal_handle = None
        _goal_in_flight = False
        self._last_goal_sent_time = 0.0
        
        self.pursuit_start_time: Optional[float] = None
        self.last_goal_update_time = 0.0
        
        topic = os.environ.get("GO2_DETECTION_TOPIC", "/go2/object_detections")
        self.get_logger().info(
            f"Robust visual servoing enabled: {topic} (label='{self.target_label or 'any'}', hits={self.required_hits})"
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
        self.create_timer(0.25, self._pursuit_watchdog)

    # ------------------------------------------------------------------
    # Helper methods
    # ------------------------------------------------------------------
    def _now_sec(self) -> float:
        """Get current ROS time in seconds"""
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_exploration(self, enabled: bool) -> None:
        """Enable/disable exploration"""
        from std_msgs.msg import Bool
        msg = Bool()
        msg.data = enabled
        self.exploration_pub.publish(msg)

    def _cancel_nav_goal(self) -> None:
        """Cancel current Nav2 goal"""
        if self._goal_handle:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None

    def _mark_reached(self, reason: str) -> None:
        '''
        End if reach
        '''

        self.get_logger().info(f"Object reached ({reason}). Stopping pursuit.")
        self._cancel_nav_goal()
        self.pursuit_start_time = None
        self._publish_mode("REACHED")
        self._set_exploration(False)

    def _publish_mode(self, mode: str) -> None:
        """Publish current mode for status GUI"""
        self._mode = mode
        from std_msgs.msg import String
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def _stop_frontier_process(self) -> None:
        """Stop frontier explorer process"""
        if not self.frontier_pid_file or not self.frontier_pid_file.exists():
            return
        try:
            pid_str = self.frontier_pid_file.read_text().strip()
            pid = int(pid_str)
            os.kill(pid, signal.SIGTERM)
            self.get_logger().info(f"Stopped frontier process PID {pid}")
            self.frontier_pid_file.unlink()
        except Exception as exc:
            self.get_logger().warning(f"Could not stop frontier: {exc}")

    # ------------------------------------------------------------------
    # Detection picking and locking
    # ------------------------------------------------------------------
    def _pick_detection(self, msg: Detection2DArray) -> Optional[_DetMeas]:
        """Select best detection from array, with instance locking during pursuit"""
        if not msg.detections:
            return None

        img_w, img_h = 640.0, 480.0
        candidates: list[_DetMeas] = []
        
        for det in msg.detections:
            best_score = -1.0
            # best label match inside this detection
            for res in det.results:
                label = getattr(res.hypothesis, "class_id", "")
                score = float(getattr(res.hypothesis, "score", 0.0))
                if self.target_label and (not isinstance(label, str) or label.lower() != self.target_label):
                    continue
                if score < self.min_score:
                    continue
                best_score = max(best_score, score)

            if best_score < 0.0:
                continue

            bbox = det.bbox
            cx = float(bbox.center.position.x)
            cy = float(bbox.center.position.y)
            bbox_pct = max(float(bbox.size_x) / img_w, float(bbox.size_y) / img_h)
            offset_norm = (cx - img_w / 2.0) / img_w  # [-0.5..0.5]
            candidates.append(_DetMeas(cx=cx, cy=cy, offset_norm=offset_norm, bbox_pct=bbox_pct, score=best_score))

        if not candidates:
            return None

        # During pursuing: stick to the locked instance (nearest center)
        if self._mode == "PURSUING" and self._locked_center is not None:
            lx, ly = self._locked_center
            best = min(candidates, key=lambda m: (m.cx - lx) ** 2 + (m.cy - ly) ** 2)
            if math.hypot(best.cx - lx, best.cy - ly) > self.lock_px_gate:
                return None  # likely a different instance -> treat as lost
            return best

        # Not pursuing: take strongest (score, then bbox size)
        return max(candidates, key=lambda m: (m.score, m.bbox_pct))

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _prompt_cb(self, msg) -> None:
        """Reset search when new prompt is entered"""
        # Cancel current nav goal (if any)
        self._cancel_nav_goal()

        # Reset tracking
        self._candidate_center = None
        self._candidate_hits = 0
        self._locked_center = None

        self._last_meas = None
        self._bearing_ema = None
        self._bbox_ema = None
        self._current_goal = None

        self.pursuit_start_time = None
        self._goal_seq += 1

        # Restart exploration
        self._set_exploration(True)
        self._publish_mode("SEARCHING")

        self.get_logger().info(f"New search started with prompt: '{msg.data}' - exploration restarted")

    def _detection_cb(self, msg: Detection2DArray) -> None:
        """Handle incoming detections with robust tracking and visual servoing"""
        if self._mode == "REACHED":
            return

        meas = self._pick_detection(msg)
        if meas is None:
            return

        # Ignore tiny detections (noise/far away)
        if meas.bbox_pct < self.min_bbox_percentage:
            return

        now = self._now_sec()
        self._last_detection_time = now
        self._last_meas = meas

        # If pursuing, keep lock center updated smoothly
        if self._mode == "PURSUING" and self._locked_center is not None:
            lx, ly = self._locked_center
            self._locked_center = (0.7 * lx + 0.3 * meas.cx, 0.7 * ly + 0.3 * meas.cy)

        # --- Reached condition (safer): big bbox + centered + min time
        if self._mode == "PURSUING":
            time_pursuing = (now - self.pursuit_start_time) if self.pursuit_start_time else 0.0
            if (meas.bbox_pct >= self.target_bbox_percentage and
                abs(meas.offset_norm) <= self.center_tolerance and
                time_pursuing >= self.min_pursuit_time):
                self.get_logger().info(
                    f"Object reached: bbox={meas.bbox_pct*100:.1f}% "
                    f"offset={meas.offset_norm:+.3f} pursued={time_pursuing:.1f}s"
                )
                self._mark_reached("vision")
                return

        # --- Not pursuing yet: require consistent hits near same center
        if self._mode != "PURSUING":
            if self._candidate_center is None:
                self._candidate_center = (meas.cx, meas.cy)
                self._candidate_hits = 1
            else:
                cx0, cy0 = self._candidate_center
                if math.hypot(meas.cx - cx0, meas.cy - cy0) <= self.candidate_px_gate:
                    self._candidate_hits += 1
                    self._candidate_center = (0.8 * cx0 + 0.2 * meas.cx, 0.8 * cy0 + 0.2 * meas.cy)
                else:
                    self._candidate_center = (meas.cx, meas.cy)
                    self._candidate_hits = 1

            self.get_logger().info(
                f"Detection {self._candidate_hits}/{self.required_hits} "
                f"bbox={meas.bbox_pct*100:.1f}% cx={meas.cx:.0f} cy={meas.cy:.0f}"
            )

            if self._candidate_hits >= self.required_hits:
                # lock on this instance
                self._locked_center = self._candidate_center
                self._begin_object_pursuit()
            return

        # --- Pursuing: update goal only at rate + significance
        if self._mode == "PURSUING":
            if now - self.last_goal_update_time >= self.goal_update_rate:
                self._update_pursuit_goal()
                self.last_goal_update_time = now

    # ------------------------------------------------------------------
    # Pursuit logic
    # ------------------------------------------------------------------
    def _calculate_goal_from_bbox(self):
        """Calculate Nav2 goal with rotate-first and dynamic step sizing"""
        if not self._last_meas:
            return None

        # Get robot pose
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y

            quat = trans.transform.rotation
            siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        except Exception as e:
            self.get_logger().error(f"Could not get robot pose: {e}")
            return None

        meas = self._last_meas

        # Smooth bbox pct and bearing
        if self._bbox_ema is None:
            self._bbox_ema = meas.bbox_pct
        else:
            a = self.bbox_ema_alpha
            self._bbox_ema = (1.0 - a) * self._bbox_ema + a * meas.bbox_pct

        bearing_offset = meas.offset_norm * math.radians(self.camera_hfov)  # [-hfov/2..hfov/2]
        if self._bearing_ema is None:
            self._bearing_ema = bearing_offset
        else:
            a = self.bearing_ema_alpha
            self._bearing_ema = (1.0 - a) * self._bearing_ema + a * bearing_offset

        object_direction = _wrap_angle(robot_yaw + self._bearing_ema)

        # Dynamic step: smaller when bbox grows.
        # 07.01 dont rotate
        bbox = self._bbox_ema
        if bbox >= self.target_bbox_percentage:
            step = 0.0
        else:
            ratio = max(0.0, (self.target_bbox_percentage - bbox) / self.target_bbox_percentage)
            step = self.min_step + ratio * (self.max_step - self.min_step)

        # If object is off-center, reduce step but keep small forward
        if abs(meas.offset_norm) > self.yaw_only_threshold:
            step = max(step * 0.3, self.min_step * 0.25)
        else:
            # Reduce step when not centered (safer)
            step *= max(0.25, 1.0 - abs(meas.offset_norm) / 0.5)

        # If the target is still small in view, bias toward a larger step
        if self._bbox_ema is not None and self._bbox_ema < (self.stable_bbox_percentage * 0.5):
            step = max(step, self.min_step * 1.5)

        # Build goal
        if step <= 1e-3:
            goal_x, goal_y = robot_x, robot_y
        else:
            goal_x = robot_x + step * math.cos(object_direction)
            goal_y = robot_y + step * math.sin(object_direction)

        return (goal_x, goal_y, object_direction, bbox, robot_x, robot_y, robot_yaw)
    
    def _begin_object_pursuit(self) -> None:
        """Start visual servoing pursuit"""
        if self._mode == "PURSUING":
            return
        if not self._last_meas:
            self.get_logger().error("No detection stored!")
            return

        self._publish_mode("PURSUING")
        self._set_exploration(False)
        self._stop_frontier_process()

        self.pursuit_start_time = self._now_sec()
        self.last_goal_update_time = 0.0

        # reset smoothing at pursuit start
        self._bearing_ema = None
        self._bbox_ema = None
        self._current_goal = None

        self._send_pursuit_goal()
    
    def _send_pursuit_goal(self):
        """Send/update Nav2 goal with gating to prevent thrashing"""
        goal_data = self._calculate_goal_from_bbox()
        if not goal_data:
            return

        goal_x, goal_y, object_yaw, bbox, robot_x, robot_y, robot_yaw = goal_data
        now = self._now_sec()
        goal_yaw = object_yaw
        if bbox < self.stable_bbox_percentage:
            # Avoid rotate in place when the target is still far (does it make sense to rotate?)
            goal_yaw = robot_yaw

        # Update gating: only if goal meaningfully changes
        if self._current_goal is not None:
            gx0, gy0, yaw0 = self._current_goal
            if (math.hypot(goal_x - gx0, goal_y - gy0) < self.goal_update_distance and
                abs(_wrap_angle(goal_yaw - yaw0)) < self.goal_update_yaw):
                return
            dist_to_goal = math.hypot(gx0 - robot_x, gy0 - robot_y)
            if dist_to_goal > self.goal_hold_distance and (now - self._last_goal_sent_time) < self.goal_hold_timeout:
                return

        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 action server not available")
            return

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(goal_x)
        pose.pose.position.y = float(goal_y)
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quaternion(goal_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._goal_seq += 1
        seq = self._goal_seq
        self._current_goal = (goal_x, goal_y, goal_yaw)
        self._last_goal_sent_time = now

        self.get_logger().info(
            f"Pursuit goal: ({goal_x:.2f}, {goal_y:.2f}) yaw={math.degrees(goal_yaw):.1f}° bbox={bbox*100:.1f}%"
        )

        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda fut, seq=seq: self._goal_response_cb(fut, seq))
    
    def _update_pursuit_goal(self):
        """Update pursuit goal based on new detection (visual servoing)"""
        self._send_pursuit_goal()

    def _goal_response_cb(self, future, seq: int) -> None:
        """Handle Nav2 goal acceptance"""
        if seq != self._goal_seq or self._mode != "PURSUING":
            return
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send goal: {exc}")
            return
        if not self._goal_handle.accepted:
            self.get_logger().warning("NavigateToPose goal rejected")
            return

        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut, seq=seq: self._goal_result_cb(fut, seq))

    def _goal_result_cb(self, future, seq: int) -> None:
        """Handle Nav2 goal completion (don't shutdown - keep servoing)"""
        if seq != self._goal_seq:
            return
        try:
            wrapped = future.result()
            status = getattr(wrapped, "status", None)

            if status == GoalStatus.STATUS_SUCCEEDED:
                if self._mode == "PURSUING":
                    if self._last_meas and self._last_meas.bbox_pct >= self.stable_bbox_percentage:
                        self._mark_reached("nav2_goal_succeeded")
                    else:
                        self.get_logger().debug(
                            "Nav2 goal reached but target still small; continuing pursuit."
                        )
                else:
                    # Intermediate goal reached (normal for servoing)
                    self.get_logger().debug("Intermediate Nav2 goal reached.")
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warning("Nav2 goal aborted (will continue pursuing if detections continue).")
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().debug("Nav2 goal canceled/preempted.")
            else:
                self.get_logger().info(f"Nav2 goal finished with status={status}.")
        except Exception as exc:
            self.get_logger().error(f"Nav2 goal result error: {exc}")
        finally:
            # Keep node alive; pursuit continues via detections
            self._goal_handle = None

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------
    def _pursuit_watchdog(self) -> None:
        """Monitor for lost detections and abort pursuit if needed"""
        if self._mode != "PURSUING":
            return
        if self._last_detection_time <= 0.0:
            return

        now = self._now_sec()
        lost_for = now - self._last_detection_time

        if lost_for > self.abort_after_lost:
            self.get_logger().warning(f"Lost object for {lost_for:.1f}s -> abort pursuit, resume exploration.")
            self._cancel_nav_goal()
            self.pursuit_start_time = None

            # reset tracking
            self._candidate_center = None
            self._candidate_hits = 0
            self._locked_center = None
            self._last_meas = None
            self._bearing_ema = None
            self._bbox_ema = None
            self._current_goal = None

            self._publish_mode("SEARCHING")
            self._set_exploration(True)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
