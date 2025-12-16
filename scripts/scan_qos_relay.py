#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanQosRelay(Node):
    def __init__(self, in_topic: str, out_topic: str) -> None:
        super().__init__("go2_scan_qos_relay")

        in_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        out_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(LaserScan, out_topic, out_qos)
        self._sub = self.create_subscription(LaserScan, in_topic, self._on_scan, in_qos)

        self.get_logger().info(
            f"Relaying '{in_topic}' -> '{out_topic}' (in: BEST_EFFORT, out: RELIABLE)"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--in", dest="in_topic", default="/scan_raw")
    parser.add_argument("--out", dest="out_topic", default="/scan")
    args = parser.parse_args()

    rclpy.init()
    node = ScanQosRelay(args.in_topic, args.out_topic)
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
