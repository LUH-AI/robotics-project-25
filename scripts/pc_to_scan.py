#!/usr/bin/env python3





# NOTE: ADDED AT 20.01 tofix lidar frame handling (all changes)
import argparse
import math
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def load_pc2scan_params(path: str) -> dict:
    p = Path(path)
    data = yaml.safe_load(p.read_text())
    params = data.get("pointcloud_to_laserscan", {}).get("ros__parameters", {})
    if not isinstance(params, dict):
        raise ValueError(f"Invalid params structure in {path}")
    return params


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class PcToScan(Node):
    def __init__(self, in_topic: str, out_topic: str, params: dict) -> None:
        super().__init__("go2_pc_to_scan")

        self.target_frame = str(params.get("target_frame", ""))
        self.transform_tolerance = float(params.get("transform_tolerance", 0.2))
        self.max_stamp_age = float(params.get("max_stamp_age", 2.0))
        self.use_latest_tf_on_failure = bool(params.get("use_latest_tf_on_failure", True))
        self.flatten_roll_pitch = bool(params.get("flatten_roll_pitch", False))
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

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_tf_warn = 0.0

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
        self._last_stamp_warn = 0.0
        self.get_logger().info(
            f"Converting '{in_topic}' -> '{out_topic}' bins={self._count} range=[{self.range_min},{self.range_max}] "
            f"height=[{self.min_height},{self.max_height}] target_frame='{self.target_frame or 'input'}' "
            f"flatten_roll_pitch={self.flatten_roll_pitch} "
            f"stamp_offset={self.stamp_offset_sec}s "
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

        src_frame = msg.header.frame_id or ""
        if not src_frame and self.target_frame:
            src_frame = self.target_frame
            now = time.time()
            if now - self._last_tf_warn > 5.0:
                self._last_tf_warn = now
                self.get_logger().warning(
                    "PointCloud2 header.frame_id is empty; assuming target_frame for scan conversion."
                )

        transform = None
        use_latest = False
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            use_latest = True
        elif self.max_stamp_age > 0.0:
            now = self.get_clock().now()
            age = abs(now.nanoseconds - stamp.nanoseconds) * 1e-9
            if age > self.max_stamp_age:
                use_latest = True
                warn_now = time.time()
                if warn_now - self._last_stamp_warn > 2.0:
                    self._last_stamp_warn = warn_now
                    self.get_logger().warning(
                        f"PointCloud2 stamp is out of sync by {age:.2f}s; using latest TF + wall time."
                    )

        if self.target_frame and src_frame and self.target_frame != src_frame:
            try:
                tf_stamp = Time() if use_latest else stamp
                transform = self._tf_buffer.lookup_transform(
                    self.target_frame,
                    src_frame,
                    tf_stamp,
                    timeout=Duration(seconds=self.transform_tolerance),
                )
            except TransformException as exc:
                if self.use_latest_tf_on_failure and not use_latest:
                    try:
                        transform = self._tf_buffer.lookup_transform(
                            self.target_frame,
                            src_frame,
                            Time(),
                            timeout=Duration(seconds=self.transform_tolerance),
                        )
                        use_latest = True
                    except TransformException:
                        transform = None
                if transform is None:
                    now = time.time()
                    if now - self._last_tf_warn > 2.0:
                        self._last_tf_warn = now
                        self.get_logger().warning(
                            f"TF lookup failed ({src_frame} -> {self.target_frame}): {exc}. "
                            "Skipping scan publish."
                        )
                    return

        ranges = [math.inf] * self._count

        if transform is not None:
            t = transform.transform.translation
            r = transform.transform.rotation
            qx = float(r.x)
            qy = float(r.y)
            qz = float(r.z)
            qw = float(r.w)
            if self.flatten_roll_pitch:
                yaw = yaw_from_quat(qx, qy, qz, qw)
                cy = math.cos(yaw)
                sy = math.sin(yaw)
                r00 = cy
                r01 = -sy
                r02 = 0.0
                r10 = sy
                r11 = cy
                r12 = 0.0
                r20 = 0.0
                r21 = 0.0
                r22 = 1.0
            else:
                xx = qx * qx
                yy = qy * qy
                zz = qz * qz
                xy = qx * qy
                xz = qx * qz
                yz = qy * qz
                wx = qw * qx
                wy = qw * qy
                wz = qw * qz
                r00 = 1.0 - 2.0 * (yy + zz)
                r01 = 2.0 * (xy - wz)
                r02 = 2.0 * (xz + wy)
                r10 = 2.0 * (xy + wz)
                r11 = 1.0 - 2.0 * (xx + zz)
                r12 = 2.0 * (yz - wx)
                r20 = 2.0 * (xz - wy)
                r21 = 2.0 * (yz + wx)
                r22 = 1.0 - 2.0 * (xx + yy)
            tx = float(t.x)
            ty = float(t.y)
            tz = float(t.z)

        min_r = float("inf")
        max_r = 0.0
        min_z = float("inf")
        max_z = -float("inf")
        used = 0
        for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if transform is not None:
                x = float(x)
                y = float(y)
                z = float(z)
                x, y, z = (
                    r00 * x + r01 * y + r02 * z + tx,
                    r10 * x + r11 * y + r12 * z + ty,
                    r20 * x + r21 * y + r22 * z + tz,
                )
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
        if use_latest:
            stamp_ns = int(self.get_clock().now().nanoseconds)
        else:
            stamp_ns = stamp.nanoseconds
        stamp_ns -= int(self.stamp_offset_sec * 1_000_000_000)
        if stamp_ns < 0:
            stamp_ns = 0
        scan.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        scan.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        scan.header.frame_id = self.target_frame or src_frame or msg.header.frame_id
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
