import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import (
    ObstaclesAvoidClient,
)
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize


class CmdVelToUnitree(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_unitree")

        # ROS 2 Subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # Initialize SportsClient for Unitree Go2
        # Todo: Maybe use ObstaclesAvoidClient instead?
        self.sports_client = SportClient()
        self.sports_client.SetTimeout(10.0)
        self.sports_client.Init()

        self.get_logger().info("CmdVelToUnitree node initialized.")

    def cmd_vel_callback(self, msg):
        try:
            # Extract linear and angular velocity from Twist message
            linear_x = msg.linear.x
            linear_y = msg.linear.y
            angular_z = msg.angular.z

            # Log the received velocities
            self.get_logger().info(
                f"Received cmd_vel: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}"
            )

            # Send velocity commands to Unitree Go2 robot
            self.sports_client.Move(linear_x, linear_y, angular_z)
        except Exception as e:
            self.get_logger().error(f"Error processing cmd_vel: {e}")


def main(args=None):

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    rclpy.init(args=args)

    node = CmdVelToUnitree()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
