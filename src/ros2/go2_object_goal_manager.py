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
        target_xyz = _parse_target(os.environ.get("GO2_OBJECT_TARGET"))
        self.target_x = target_xyz[0]
        self.target_y = target_xyz[1]
        self.target_yaw = target_xyz[2]
        self.required_hits = int(os.environ.get("GO2_DETECTIONS_REQUIRED", "3"))
        self.target_label = os.environ.get("GO2_OBJECT_LABEL", "").strip().lower()
        pid_file = os.environ.get("GO2_FRONTIER_PID_FILE", "").strip()
        self.frontier_pid_file = Path(pid_file) if pid_file else None
        self._detections_seen = 0
        self._goal_in_flight = False
        self._goal_future = None
        self._goal_handle = None

        topic = os.environ.get("GO2_DETECTION_TOPIC", "/go2/object_detections")
        self.get_logger().info(
            "Watching %s (target label='%s', hits=%d)"
            % (topic, self.target_label or "any", self.required_hits)
        )
        self.det_sub = self.create_subscription(
            Detection2DArray, topic, self._detection_cb, 10
        )
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_timer(1.0, self._log_status)

    # ------------------------------------------------------------------
    def _log_status(self) -> None:
        if self._goal_in_flight:
            return
        if self._detections_seen > 0:
            self.get_logger().info(
                "Waiting for %d/%d detection confirmations..."
                % (self._detections_seen, self.required_hits)
            )

    def _detection_cb(self, msg: Detection2DArray) -> None:
        if self._goal_in_flight:
            return
        if not msg.detections:
            return
        if self.target_label:
            matched = False
            for detection in msg.detections:
                for result in detection.results:
                    label = getattr(result.hypothesis, "class_id", "")
                    if isinstance(label, str) and label.lower() == self.target_label:
                        matched = True
                        break
                if matched:
                    break
            if not matched:
                return
        self._detections_seen += 1
        self.get_logger().info(
            "Detection confirmation %d/%d"
            % (self._detections_seen, self.required_hits)
        )
        if self._detections_seen >= self.required_hits:
            self._begin_object_pursuit()

    def _begin_object_pursuit(self) -> None:
        if self._goal_in_flight:
            return
        self._goal_in_flight = True
        self._stop_frontier_process()
        self.get_logger().info(
            "Sending NavigateToPose goal to (%.2f, %.2f, yaw=%.2f)"
            % (self.target_x, self.target_y, self.target_yaw)
        )
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available")
            self._goal_in_flight = False
            return

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.target_x
        pose.pose.position.y = self.target_y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quaternion(self.target_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

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
