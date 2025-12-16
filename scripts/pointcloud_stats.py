#!/usr/bin/env python3

import time

import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def main():
    rclpy.init()
    node = rclpy.create_node("go2_pointcloud_stats")
    msg = None

    def cb(m):
        nonlocal msg
        msg = m

    sub = node.create_subscription(PointCloud2, "/unitree_go2/lidar/point_cloud", cb, 10)
    end = time.time() + 5.0
    while rclpy.ok() and msg is None and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    if msg is None:
        print("No pointcloud received")
        return

    xs = []
    ys = []
    zs = []
    count = 0
    for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        xs.append(float(x))
        ys.append(float(y))
        zs.append(float(z))
        count += 1
        if count >= 200000:
            break

    print("frame_id:", msg.header.frame_id)
    print("stamp:", msg.header.stamp.sec, msg.header.stamp.nanosec)
    print("points_read:", count)
    if count:
        print("mean[x,y,z]:", sum(xs) / count, sum(ys) / count, sum(zs) / count)
        print("x[min,max]:", min(xs), max(xs))
        print("y[min,max]:", min(ys), max(ys))
        print("z[min,max]:", min(zs), max(zs))

    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
