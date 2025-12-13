import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__("odometry_publisher")
        self.publisher_ = self.create_publisher(Odometry, "/utlidar/robot_odom", 10)

    def publish_odometry(self, x, y, z, quat_x, quat_y, quat_z, quat_w):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = quat_x
        msg.pose.pose.orientation.y = quat_y
        msg.pose.pose.orientation.z = quat_z
        msg.pose.pose.orientation.w = quat_w
        self.publisher_.publish(msg)
