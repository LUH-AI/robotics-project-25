import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from detector import DepthEstimator


from unitree_sdk2py.core.channel import ChannelFactoryInitialize
import time 

from unitree_sdk2py.go2.video.video_client import VideoClient
from sensor_msgs.msg import Image

class RGBToDepthNode(Node):
    def __init__(self):
        super().__init__("rgb_to_depth_node")

        # Initialize VideoClient for Unitree Go2
        self.video_client = VideoClient()
        self.video_client.SetTimeout(10.0)
        self.video_client.Init()
        self.get_logger().info("Unitree Video Client initialized.")

        # ROS 2 Publisher for video frames
        self.video_pub = self.create_publisher(Image, "/unitree_go2/front_cam/color_image", 10)

        # Timer to publish video frames at regular intervals
        self.timer = self.create_timer(0.08, self.publish_video_frame)

        # Parameters
        model_path = self.declare_parameter("model_path", "./depth_model.pt").value
        self.bridge = CvBridge()

        # Initialize the ObstacleTracker
        self.depth_estimator = DepthEstimator(model_path)

        # ROS 2 Subscribers and Publishers
        # self.rgb_sub = self.create_subscription(
        #     Image, "/unitree_go2/front_cam/color_image", self.rgb_callback, 10
        # )
        self.depth_pub = self.create_publisher(Image, "depth_image", 10)

        print("RGB to Depth Node has been started.")

    def publish_video_frame(self):
        try:
            code, data = self.video_client.GetImageSample()

            if code == 0:
                # Convert frame to ROS Image message and publish
                image_data = np.frombuffer(bytes(data), dtype=np.uint8)
                image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                image = cv2.resize(image, (640, 480))
                video_msg = self.bridge.cv2_to_imgmsg(image , encoding="rgb8")
                video_msg.header.stamp = self.get_clock().now().to_msg()
                video_msg.header.frame_id = "unitree_go2/front_cam"

                self.video_pub.publish(video_msg)

                # Also process for depth estimation
                self.rgb_callback(video_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing video frame: {e}")

    def rgb_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Estimate depth
            depth_map = self.depth_estimator.estimate(rgb_image)

            # Normalize depth map for visualization
            depth_map_normalized = cv2.normalize(
                depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U
            )

            # Convert depth map to ROS Image message
            depth_msg = self.bridge.cv2_to_imgmsg(
                depth_map_normalized, encoding="mono8"
            )

            # Publish the depth image
            self.depth_pub.publish(depth_msg)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):

    ChannelFactoryInitialize(0)


    rclpy.init(args=args)

    node = RGBToDepthNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
