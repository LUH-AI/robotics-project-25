#!/usr/bin/env python3

import math
import time

import rclpy
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = rclpy.create_node("go2_scan_stats")
    msg = None

    def cb(m):
        nonlocal msg
        msg = m

    sub = node.create_subscription(LaserScan, "/scan", cb, 10)
    end = time.time() + 5.0
    while rclpy.ok() and msg is None and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    if msg is None:
        print("No /scan received")
        return

    finite = [r for r in msg.ranges if math.isfinite(r)]
    print("frame_id:", msg.header.frame_id)
    print("stamp:", msg.header.stamp.sec, msg.header.stamp.nanosec)
    print("count:", len(msg.ranges), "finite:", len(finite))
    if finite:
        print("min:", min(finite), "max:", max(finite))
    else:
        print("all ranges are inf/nan")

    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

