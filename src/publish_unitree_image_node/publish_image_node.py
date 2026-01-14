import rclpy
import sys
from rclpy.node import Node

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
import time 

from unitree_sdk2py.go2.video.video_client import VideoClient
from sensor_msgs.msg import Image

import cv2
import numpy as np
from cv_bridge import CvBridge


class UnitreeVideoToRos(Node): 
    def __init__(self):
        super().__init__("unitree_video_to_ros")

        # Initialize VideoClient for Unitree Go2
        self.video_client = VideoClient()
        self.video_client.SetTimeout(10.0)
        self.video_client.Init()
        self.get_logger().info("Unitree Video Client initialized.")

        self.bridge = CvBridge()

        # ROS 2 Publisher for video frames
        self.video_pub = self.create_publisher(Image, "/unitree_go2/front_cam/color_image", 10)

        # Timer to publish video frames at regular intervals
        self.timer = self.create_timer(0.1, self.publish_video_frame)

    def publish_video_frame(self):
        try:
            code, data = self.video_client.GetImageSample()

            if code == 0:
                # Convert frame to ROS Image message and publish
                image_data = np.frombuffer(bytes(data), dtype=np.uint8)
                image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                video_msg = self.bridge.cv2_to_imgmsg(image , encoding="bgr8")
                
                self.video_pub.publish(video_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing video frame: {e}")

def main(args=None):
    ChannelFactoryInitialize(0)

    rclpy.init(args=args)

    node = UnitreeVideoToRos()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()