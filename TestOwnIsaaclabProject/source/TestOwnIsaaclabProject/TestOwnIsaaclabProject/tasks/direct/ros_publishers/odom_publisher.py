import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__("odometry_publisher")
        self.publisher_ = self.create_publisher(Odometry, "/utlidar/robot_odom", 10)
        self.tf_publisher_ = self.create_publisher(TransformStamped, "/tf", 10)

    def publish_odometry(self, x, y, z, quat_x, quat_y, quat_z, quat_w):
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = y  # x
        msg.pose.pose.position.y = -z  # y
        msg.pose.pose.position.z = x
        msg.pose.pose.orientation.x = quat_y  # quat_x
        msg.pose.pose.orientation.y = -quat_z  # quat_y
        msg.pose.pose.orientation.z = quat_x
        msg.pose.pose.orientation.w = quat_w
        self.publisher_.publish(msg)

        # publish transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = y
        t.transform.translation.y = -z
        t.transform.translation.z = x
        t.transform.rotation.x = quat_y
        t.transform.rotation.y = -quat_z
        t.transform.rotation.z = quat_x
        t.transform.rotation.w = quat_w

        self.tf_publisher_.publish(t)

        # Analog transform: ros2 run tf2_ros static_transform_publisher   0.2 0.0 0.15 0 0 0   base_link utlidar_lidar
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = "base_link"
        static_t.child_frame_id = "utlidar_lidar"
        static_t.transform.translation.x = 0.2
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.15
        static_t.transform.rotation.x = 0.0
        static_t.transform.rotation.y = 0.0
        static_t.transform.rotation.z = 0.0
        static_t.transform.rotation.w = 0.0
        self.tf_publisher_.publish(static_t)
