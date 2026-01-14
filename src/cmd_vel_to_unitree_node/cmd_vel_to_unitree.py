import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import (
    ObstaclesAvoidClient,
)
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
import time 

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped


class CmdVelToUnitree(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_unitree")

        # ROS 2 Subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )


        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(
            Odometry, "/utlidar/robot_odom", self.odom_callback, 10
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/unitree_go2/odom", 10
        )

        # Initialize SportsClient for Unitree Go2
        self.obstacle_avoid_client = ObstaclesAvoidClient()
        self.obstacle_avoid_client.SetTimeout(10.0)
        self.obstacle_avoid_client.Init()
        self.obstacle_avoid_client.SwitchSet(True)
        self.obstacle_avoid_client.UseRemoteCommandFromApi(True)
        # Todo: Maybe use ObstaclesAvoidClient instead?
        self.sports_client = SportClient()
        self.sports_client.SetTimeout(10.0)
        self.sports_client.Init()
        self.get_logger().info("CmdVelToUnitree node initialized.")

        time.sleep(1)
        # for _ in range(10):        
        #     # self.sports_client.Move(-0.3, 0, 0)

        #     self.obstacle_avoid_client.Move(0.3, 0, 0)
        #     time.sleep(0.3)
        # print("stop")
        # self.obstacle_avoid_client.Move(0.0, 0, 0)

    def odom_callback(self, msg: Odometry):
        # forward to /unitree_go2/odom
        self.odom_pub.publish(msg)
        
        self.get_logger().info("publishing tf")
        map_base_trans = TransformStamped()
        map_base_trans.header.stamp = self.get_clock().now().to_msg()
        map_base_trans.header.frame_id = "/unitree_go2/odom"
        map_base_trans.child_frame_id = "unitree_go2/base_link"


        map_base_trans.transform.translation.x = 0.0
        map_base_trans.transform.translation.y = 0.0
        map_base_trans.transform.translation.z = 0.0
        map_base_trans.transform.rotation = msg.pose.pose.orientation
        # .x = 0.0
        # map_base_trans.transform.rotation.y = 0.0
        # map_base_trans.transform.rotation.z = 0.0
        # map_base_trans.transform.rotation.w = 0.0
        self.tf_broadcaster.sendTransform(map_base_trans)

        map_base_trans = TransformStamped()
        map_base_trans.header.stamp = self.get_clock().now().to_msg()
        map_base_trans.header.frame_id = "/unitree_go2/odom"
        map_base_trans.child_frame_id = "unitree_go2/base_footprint"
        
        map_base_trans.transform.translation.x = 0.0
        map_base_trans.transform.translation.y = 0.0
        map_base_trans.transform.translation.z = 0.0
        map_base_trans.transform.rotation = msg.pose.pose.orientation #.x = 0.0
        # map_base_trans.transform.rotation.y = 0.0
        # map_base_trans.transform.rotation.z = 0.0
        # map_base_trans.transform.rotation.w = 0.0
        self.tf_broadcaster.sendTransform(map_base_trans)

        


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
            self.obstacle_avoid_client.Move(linear_x, linear_y, angular_z)
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
