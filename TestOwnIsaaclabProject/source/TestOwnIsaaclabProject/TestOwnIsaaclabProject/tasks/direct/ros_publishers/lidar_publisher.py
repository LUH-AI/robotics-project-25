import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np


class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__("pointcloud_publisher")

        self.publisher_ = self.create_publisher(
            PointCloud2, "/utlidar/cloud_deskewed", 10
        )

    def publish(self, lidar_data):
        # Create some example points (x, y, z)

        # points = np.array(
        #     [
        #         [0.0, 0.0, 0.0],
        #         [1.0, 0.0, 0.0],
        #         [0.0, 1.0, 0.0],
        #         [0.0, 0.0, 1.0],
        #     ],
        #     dtype=np.float32,
        # )

        points = lidar_data["IsaacExtractRTXSensorPointCloudNoAccumulator"]["data"]

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = "utlidar_lidar"

        cloud_msg = pc2.create_cloud_xyz32(cloud.header, points)

        self.publisher_.publish(cloud_msg)
