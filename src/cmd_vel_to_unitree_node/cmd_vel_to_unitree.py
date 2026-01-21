import rclpy
import sys
import os
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
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from sensor_msgs.msg import PointCloud2


class CmdVelToUnitree(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_unitree")
        # ADDED AT 20.01 tofix lidar frame handling
        self.lidar_frame = os.getenv("GO2_LIDAR_FRAME", "unitree_go2/lidar_frame").strip() or "unitree_go2/lidar_frame"
        self.force_cloud_frame = os.getenv("GO2_FORCE_CLOUD_FRAME", "0") == "1"
        self.force_cloud_stamp = os.getenv("GO2_FORCE_CLOUD_STAMP", "0") == "1"
        self.cloud_stamp_max_age = float(os.getenv("GO2_CLOUD_STAMP_MAX_AGE", "2.0"))
        self._last_stamp_warn = 0.0

        # ROS 2 Subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )


        self.tf_broadcaster = TransformBroadcaster(self)
        self.creat_static_transform()
        self.odom_sub = self.create_subscription(
            Odometry, "/utlidar/robot_odom", self.odom_callback, 10
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/unitree_go2/odom", 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, "/unitree_go2/pose", 10
        )
        # self.pose2_pub = self.create_publisher(
        #     PoseStamped, "/pose", 10
        # )

        self.cloud_sub = self.create_subscription(PointCloud2, "/utlidar/cloud_deskewed", self.cloud_callback, 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/unitree_go2/lidar/point_cloud", 10
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

    def cloud_callback(self, msg: PointCloud2):
        # forward to /unitree_go2/lidar/point_cloud
        # ADDED AT 20.01 tofix lidar frame handling
        if self.force_cloud_frame or not msg.header.frame_id:
            msg.header.frame_id = self.lidar_frame
        now = self.get_clock().now()
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = now.to_msg()
        else:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            age = abs(now.nanoseconds - stamp.nanoseconds) * 1e-9
            if self.force_cloud_stamp or age > self.cloud_stamp_max_age:
                msg.header.stamp = now.to_msg()
                warn_now = time.time()
                if warn_now - self._last_stamp_warn > 2.0:
                    self._last_stamp_warn = warn_now
                    # self.get_logger().warning(
                    #     f"PointCloud2 stamp was out of sync by {age:.2f}s; overriding to wall time."
                    # )
        self.cloud_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        # forward to /unitree_go2/odom
        msg.header.frame_id = "odom"
        msg.child_frame_id = "unitree_go2/base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        # self.get_logger().info(f"ODOM: {msg.twist.twist.linear}")
        self.odom_pub.publish(msg)
        
        # self.get_logger().info("publishing tf")
        map_base_trans = TransformStamped()
        map_base_trans.header.stamp = self.get_clock().now().to_msg()
        map_base_trans.header.frame_id = "odom"
        map_base_trans.child_frame_id = "unitree_go2/base_link"
        
        map_base_trans.transform.translation.x = msg.pose.pose.position.x 
        map_base_trans.transform.translation.y = msg.pose.pose.position.y
        map_base_trans.transform.translation.z = msg.pose.pose.position.z
        map_base_trans.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(map_base_trans)

        # map_base_trans = TransformStamped()
        # map_base_trans.header.stamp = self.get_clock().now().to_msg()
        # map_base_trans.header.frame_id = "/odom"
        # map_base_trans.child_frame_id = "unitree_go2/base_footprint"
    
        # map_base_trans.transform.translation.x = 0.0
        # map_base_trans.transform.translation.y = 0.0
        # map_base_trans.transform.translation.z = 0.0
        # map_base_trans.transform.rotation = msg.pose.pose.orientation #.x = 0.0
        # # map_base_trans.transform.rotation.y = 0.0
        # # map_base_trans.transform.rotation.z = 0.0
        # # map_base_trans.transform.rotation.w = 0.0
        # self.tf_broadcaster.sendTransform(map_base_trans)

        # Also publish PoseStamped to /unitree_go2/pose
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        # self.get_logger().info(f"POSE: {msg.pose.pose}")
        self.pose_pub.publish(pose_msg)
        # self.pose2_pub.publish(pose_msg)
        


    def cmd_vel_callback(self, msg):
        try:
            # Extract linear and angular velocity from Twist message
            linear_x = msg.linear.x / 3.0
            linear_y = msg.linear.y / 3.0
            angular_z = msg.angular.z / 3.0

            # Send velocity commands to Unitree Go2 robot
            # self.sports_client.Move(linear_x, linear_y, angular_z)
            self.obstacle_avoid_client.Move(linear_x, linear_y, angular_z)
        except Exception as e:
            self.get_logger().error(f"Error processing cmd_vel: {e}")


    def creat_static_transform(self):
        zero_stamp = rclpy.time.Time().to_msg()

        lidar_broadcaster = StaticTransformBroadcaster(self)
        base_lidar_transform = TransformStamped()
        base_lidar_transform.header.stamp = zero_stamp
        base_lidar_transform.header.frame_id = "unitree_go2/base_link"
        base_lidar_transform.child_frame_id = self.lidar_frame


        # Translation
        base_lidar_transform.transform.translation.x = 0.2
        base_lidar_transform.transform.translation.y = 0.0
        base_lidar_transform.transform.translation.z = 0.2
        
        # Rotation 
        base_lidar_transform.transform.rotation.x = 0.0
        base_lidar_transform.transform.rotation.y = 0.0
        base_lidar_transform.transform.rotation.z = 0.0
        base_lidar_transform.transform.rotation.w = 1.0
        
        # Publish the transform
        lidar_broadcaster.sendTransform(base_lidar_transform)

        # -------------------------------------------------------------
        # Camera
        # Create and publish the transform
        camera_broadcaster = StaticTransformBroadcaster(self)
        base_cam_transform = TransformStamped()
        base_cam_transform.header.stamp = zero_stamp
        base_cam_transform.header.frame_id = "unitree_go2/base_link"
        base_cam_transform.child_frame_id = "unitree_go2/front_cam"

        # Translation
        base_cam_transform.transform.translation.x = 0.4
        base_cam_transform.transform.translation.y = 0.0
        base_cam_transform.transform.translation.z = 0.2
        
        # Rotation 
        base_cam_transform.transform.rotation.x = -0.5
        base_cam_transform.transform.rotation.y = 0.5
        base_cam_transform.transform.rotation.z = -0.5
        base_cam_transform.transform.rotation.w = 0.5

        # Publish the transform
        camera_broadcaster.sendTransform(base_cam_transform)

def main(args=None):

    print("Initializing Channel Factory...")
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
