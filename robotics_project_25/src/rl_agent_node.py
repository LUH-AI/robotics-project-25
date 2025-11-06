import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import onnxruntime as ort

class RLAgentNode(Node):
    def __init__(self):
        super().__init__('rl_agent')
        self.policy = ort.InferenceSession("models/policy.onnx")
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("RL agent node started.")

    def odom_cb(self, msg):
        obs = np.array([[msg.pose.pose.position.x,
                         msg.pose.pose.position.y,
                         msg.twist.twist.linear.x]], dtype=np.float32)
        action = self.policy.run(None, {'obs': obs})[0][0]
        twist = Twist()
        twist.linear.x = float(action[0])
        twist.angular.z = float(action[1])
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = RLAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
