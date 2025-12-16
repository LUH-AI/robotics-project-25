#!/usr/bin/env python3

import argparse
import math
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
import yaml


def load_pc2scan_params(path: str) -> dict:
    p = Path(path)
    data = yaml.safe_load(p.read_text())
    params = data.get("pointcloud_to_laserscan", {}).get("ros__parameters", {})
    if not isinstance(params, dict):
        raise ValueError(f"Invalid params structure in {path}")
    return params


class PcToScan(Node):
    def __init__(self, in_topic: str, out_topic: str, params: dict) -> None:
        super().__init__("go2_pc_to_scan")

        self.target_frame = str(params.get("target_frame", ""))
        self.min_height = float(params.get("min_height", -0.5))
        self.max_height = float(params.get("max_height", 0.5))
        self.angle_min = float(params.get("angle_min", -math.pi))
        self.angle_max = float(params.get("angle_max", math.pi))
        self.angle_increment = float(params.get("angle_increment", 0.0087))
        self.scan_time = float(params.get("scan_time", 0.1))
        self.range_min = float(params.get("range_min", 0.1))
        self.range_max = float(params.get("range_max", 30.0))
        self.use_inf = bool(params.get("use_inf", True))
        self.inf_epsilon = float(params.get("inf_epsilon", 1.0))
        # Isaac Sim sometimes timestamps sensors a few ms ahead of TF/odom; slam_toolbox then refuses to
        # lookup transforms "in the future". Shift scan stamps slightly into the past.
        self.stamp_offset_sec = float(params.get("stamp_offset", 0.05))
        self.max_pub_rate = float(params.get("max_pub_rate", 5.0))
        self._last_pub_wall = 0.0
        self._last_scan = None  # type: LaserScan

        if self.angle_increment <= 0:
            raise ValueError("angle_increment must be > 0")
        if self.angle_max <= self.angle_min:
            raise ValueError("angle_max must be > angle_min")

        self._count = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1

        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(LaserScan, out_topic, pub_qos)
        self._sub = self.create_subscription(PointCloud2, in_topic, self._on_cloud, sub_qos)

        self._last_warn = 0.0
        self.get_logger().info(
            f"Converting '{in_topic}' -> '{out_topic}' bins={self._count} range=[{self.range_min},{self.range_max}] "
            f"height=[{self.min_height},{self.max_height}] stamp_offset={self.stamp_offset_sec}s "
            f"max_pub_rate={self.max_pub_rate}Hz"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self.max_pub_rate > 0.0:
            now_wall = time.time()
            min_period = 1.0 / self.max_pub_rate
            if now_wall - self._last_pub_wall < min_period:
                return
            self._last_pub_wall = now_wall

        total = int(msg.width) * int(msg.height)
        if total == 0:
            # Isaac/Omniverse occasionally publishes empty PointCloud2 messages (width=0).
            # Dropping the message is safer than re-using stale data (stale ranges smear the map).
            now = time.time()
            if now - self._last_warn > 5.0:
                self._last_warn = now
                self.get_logger().warning(
                    f"Empty PointCloud2 (frame={msg.header.frame_id}, width={msg.width}, height={msg.height}). "
                    f"Skipping publish to avoid injecting stale ranges."
                )
            return

        ranges = [math.inf] * self._count

        min_r = float("inf")
        max_r = 0.0
        min_z = float("inf")
        max_z = -float("inf")
        used = 0
        for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            z = float(z)
            if z < min_z:
                min_z = z
            if z > max_z:
                max_z = z
            if z < self.min_height or z > self.max_height:
                continue
            x = float(x)
            y = float(y)
            r = math.hypot(x, y)
            if r < min_r:
                min_r = r
            if r > max_r:
                max_r = r
            if r < self.range_min or r > self.range_max:
                continue
            angle = math.atan2(y, x)
            if angle < self.angle_min or angle > self.angle_max:
                continue
            idx = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= idx < self._count and r < ranges[idx]:
                ranges[idx] = r
                used += 1

        if self.use_inf:
            out_ranges = ranges
        else:
            out_ranges = [r if math.isfinite(r) else (self.range_max + self.inf_epsilon) for r in ranges]

        scan = LaserScan()
        # Clamp into the past to avoid TF extrapolation errors.
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        stamp_ns -= int(self.stamp_offset_sec * 1_000_000_000)
        if stamp_ns < 0:
            stamp_ns = 0
        scan.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        scan.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        scan.header.frame_id = self.target_frame or msg.header.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = out_ranges
        self._pub.publish(scan)
        self._last_scan = scan

        # If we get a non-empty cloud but produce no finite ranges, warn occasionally.
        if used == 0:
            now = time.time()
            if now - self._last_warn > 5.0:
                self._last_warn = now
                self.get_logger().warning(
                    f"Cloud had no usable points after filtering (frame={msg.header.frame_id}, "
                    f"points={total}, z[min,max]=[{min_z:.3f},{max_z:.3f}], r[min,max]=[{min_r:.3f},{max_r:.3f}], "
                    f"min/max height={self.min_height}/{self.max_height}, range=[{self.range_min},{self.range_max}])."
                )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--in", dest="in_topic", default="/unitree_go2/lidar/point_cloud")
    parser.add_argument("--out", dest="out_topic", default="/scan_raw")
    parser.add_argument("--params-file", required=True)
    args = parser.parse_args()

    params = load_pc2scan_params(args.params_file)
    rclpy.init()
    node = PcToScan(args.in_topic, args.out_topic, params)
    try:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
