#!/usr/bin/env python3

import argparse
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def quat_norm(q):
    x, y, z, w = q
    return math.sqrt(x * x + y * y + z * z + w * w)


def quat_inverse(q):
    x, y, z, w = q
    n = quat_norm(q)
    if n == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / (n * n)
    return (-x * inv, -y * inv, -z * inv, w * inv)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vec(q, v):
    vx, vy, vz = v
    vq = (vx, vy, vz, 0.0)
    return quat_multiply(quat_multiply(q, vq), quat_inverse(q))[:3]


def yaw_from_quat(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class BaseFootprintTf(Node):
    def __init__(self, odom_frame: str, base_link: str, base_footprint: str, rate_hz: float) -> None:
        super().__init__("go2_base_footprint_tf")
        self._odom_frame = odom_frame
        self._base_link = base_link
        self._base_footprint = base_footprint

        self._broadcaster = TransformBroadcaster(self)

        # Subscribe directly to /tf and mirror odom->base_link stamps.
        # This avoids tf2 buffer lag / deadlocks in rclpy and keeps slam_toolbox happy.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._sub = self.create_subscription(TFMessage, "/tf", self._on_tf, qos)

        self.get_logger().info(
            f"Publishing '{self._base_link}' -> '{self._base_footprint}' (yaw-only footprint), "
            f"driven by TF '{self._odom_frame}' -> '{self._base_link}'"
        )

    def _on_tf(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            if tf.header.frame_id != self._odom_frame or tf.child_frame_id != self._base_link:
                continue

            t = tf.transform.translation
            r = tf.transform.rotation
            q_bl = (float(r.x), float(r.y), float(r.z), float(r.w))
            yaw = yaw_from_quat(q_bl)
            q_yaw = quat_from_yaw(yaw)
            q_rel = quat_multiply(quat_inverse(q_bl), q_yaw)

            # Place footprint on the odom ground plane (z=0) while keeping x/y identical to base_link.
            delta_odom = (0.0, 0.0, -float(t.z))
            t_rel = rotate_vec(quat_inverse(q_bl), delta_odom)

            out = TransformStamped()
            out.header.stamp = tf.header.stamp
            out.header.frame_id = self._base_link
            out.child_frame_id = self._base_footprint
            out.transform.translation.x = float(t_rel[0])
            out.transform.translation.y = float(t_rel[1])
            out.transform.translation.z = float(t_rel[2])
            out.transform.rotation.x = float(q_rel[0])
            out.transform.rotation.y = float(q_rel[1])
            out.transform.rotation.z = float(q_rel[2])
            out.transform.rotation.w = float(q_rel[3])
            self._broadcaster.sendTransform(out)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-link", default="unitree_go2/base_link")
    parser.add_argument("--base-footprint", default="unitree_go2/base_footprint")
    parser.add_argument("--rate", type=float, default=0.0, help="Deprecated (kept for compatibility).")
    args = parser.parse_args()

    rclpy.init()
    node = BaseFootprintTf(args.odom_frame, args.base_link, args.base_footprint, args.rate)
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
