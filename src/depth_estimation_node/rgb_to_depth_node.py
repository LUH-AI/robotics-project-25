import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from detector import DepthEstimator


class RGBToDepthNode(Node):
    def __init__(self):
        super().__init__("rgb_to_depth_node")

        # Parameters
        model_path = self.declare_parameter("model_path", "./depth_model.pt").value
        self.bridge = CvBridge()

        # Initialize the ObstacleTracker
        self.depth_estimator = DepthEstimator(model_path)

        # ROS 2 Subscribers and Publishers
        self.rgb_sub = self.create_subscription(
            Image, "/unitree_go2/front_cam/color_image", self.rgb_callback, 10
        )
        self.depth_pub = self.create_publisher(Image, "depth_image", 10)

        print("RGB to Depth Node has been started.")

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
