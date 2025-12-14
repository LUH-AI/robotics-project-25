from isaacsim import SimulationApp
import sys

CONFIG = {"renderer": "RayTracedLighting", "headless": True}

# Example ROS 2 bridge sample demonstrating the manual loading of stages and manual publishing of images
simulation_app = SimulationApp(CONFIG)

import math, time
import numpy as np
from isaacsim.core.utils import extensions, nucleus


from omni.isaac.core.utils.stage import add_reference_to_stage, open_stage
from omni.isaac.core import World

from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path

from omni.isaac.core.utils.types import ArticulationAction

from pxr import Gf
import omni.replicator.core as rep

from isaacsim.sensors.camera import Camera
from isaacsim.sensors.rtx import LidarRtx

import isaacsim.core.utils.numpy.rotations as rot_utils

# Import our ROS publishing functions
from ros_publishers.publish_pointcloud_from_depth import publish_pointcloud_from_depth
from ros_publishers.lidar_publisher import PointCloudPublisher

from ros_publishers.odom_publisher import OdometryPublisher
from ros_publishers.publish_camera_info import publish_camera_info
from ros_publishers.publish_rgb import publish_rgb
from ros_publishers.publish_camera_tf import publish_camera_tf
from ros_publishers.publish_depth import publish_depth

# Enable ROS 2 bridge extension
extensions.enable_extension("isaacsim.ros2.bridge")

import rclpy

rclpy.init()

simulation_app.update()

world_path = "/home/rlproject25/Desktop/usda_files/World-base.usd"
open_stage(world_path)
# world = SimulationContext(stage_units_in_meters=1.0)
# Create the world from the currently loaded stage
world = World(stage_units_in_meters=1.0)


def main():
    # Locate Isaac Sim assets folder to load environment and robot stages
    assets_root_path = nucleus.get_assets_root_path()
    if assets_root_path is None:
        simulation_app.close()
        sys.exit()

    # Loading the environment
    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError("Could not find Isaac assets root.")

    go2_usd = assets_root + "/Isaac/Robots/Unitree/Go2/go2.usd"
    add_reference_to_stage(usd_path=go2_usd, prim_path="/World/Go2")

    go2 = Robot(prim_path="/World/Go2", name="go2")
    world.scene.add(go2)

    # Create a Camera prim. The Camera class takes the position and orientation in the world axes convention.
    camera = Camera(
        prim_path="/World/Go2/base/base_frame",
        position=np.array([-3.11, -1.87, 1.0]),
        frequency=20,
        resolution=(256, 256),
        orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
    )

    camera.initialize()
    simulation_app.update()
    camera.initialize()

    # Create Lidar
    lidar_attributes = {"omni:sensor:Core:scanRateBaseHz": 20}
    lidar = LidarRtx(
        prim_path="/World/Go2/lidar",
        translation=np.array([0.0, 0.0, 1.0]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        config_file_name="Example_Rotary",
        **lidar_attributes,
    )

    print(lidar)
    lidar.initialize()
    simulation_app.update()

    lidar.attach_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")

    # Reset needed to actually create everything in the world & "start" it
    world.reset()

    controller = go2.get_articulation_controller()
    dof_names = go2.dof_names
    # num_dof = go2.num_dof

    q_stand = np.array(go2.get_joint_positions(), dtype=np.float32)

    # Helper to set by name safely
    def set_if_exists(name, value):
        if name in dof_names:
            q_stand[dof_names.index(name)] = value

    set_if_exists("FL_hip_joint", 0.1)
    set_if_exists("RL_hip_joint", 0.1)
    set_if_exists("FR_hip_joint", -0.1)
    set_if_exists("RR_hip_joint", -0.1)

    set_if_exists("FL_thigh_joint", 0.8)
    set_if_exists("FR_thigh_joint", 0.8)
    set_if_exists("RL_thigh_joint", 1.0)
    set_if_exists("RR_thigh_joint", 1.0)

    set_if_exists("FL_calf_joint", -1.5)
    set_if_exists("FR_calf_joint", -1.5)
    set_if_exists("RL_calf_joint", -1.5)
    set_if_exists("RR_calf_joint", -1.5)

    go2.set_world_pose(position=np.array([0.0, 0.0, 0.4]))
    action = ArticulationAction(joint_positions=q_stand)
    controller.apply_action(action)
    world.step(render=True)

    dt = world.get_physics_dt()
    t = 0.0

    stand_steps = int(2.0 / dt)
    for _ in range(stand_steps):
        action = ArticulationAction(joint_positions=q_stand)
        controller.apply_action(action)
        world.step(render=True)

    q0 = q_stand.copy()

    step_freq = 0.8
    hip_amp = 0.15
    thigh_amp = 0.3

    def leg_angles(phase: float):
        hip = hip_amp * math.sin(phase)
        thigh = thigh_amp * math.sin(phase + math.pi / 2.0)
        return hip, thigh

    idx = {name: dof_names.index(name) for name in dof_names}

    def j(name):
        return idx[name]

    # --------------------------------------------------------------------------------- #
    # --------------------------------------------------------------------------------- #
    # ---------------------- Calling Camera publishing functions ---------------------- #
    # --------------------------------------------------------------------------------- #
    # --------------------------------------------------------------------------------- #
    approx_freq = 30
    publish_camera_tf(camera)
    publish_camera_info(camera, approx_freq, "/visual_slam/camera_info_0")
    publish_rgb(camera, approx_freq, "/visual_slam/image_0")
    publish_depth(camera, approx_freq)
    # publish_pointcloud_from_depth(camera, approx_freq)
    odom_publisher = OdometryPublisher()
    lidar_publisher = PointCloudPublisher()

    # ---------------------------------------------------------------- #
    # ---------------------------------------------------------------- #
    # ---------------------- Initialize Physics ---------------------- #
    # ---------------------------------------------------------------- #
    # ---------------------------------------------------------------- #
    world.initialize_physics()
    world.play()

    # ------------------------------------------------------------ #
    # ------------------------------------------------------------ #
    # ---------------------- Run Simulation ---------------------- #
    # ------------------------------------------------------------ #
    # ------------------------------------------------------------ #

    while simulation_app.is_running():

        pos, rot = go2.get_world_pose()
        # print("Robot position: ", pos, " orientation: ", rot)

        # odom_publisher.publish_odometry(
        #     pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], rot[3]
        # )
        odom_publisher.publish_odometry(
            float(pos[0]),
            float(pos[1]),
            float(pos[2]),
            float(rot[0]),
            float(rot[1]),
            float(rot[2]),
            float(rot[3]),
        )

        data = lidar.get_current_frame()
        print("Lidar point cloud data shape: ", data)
        lidar_publisher.publish(data)

        rclpy.spin_once(odom_publisher, timeout_sec=0)
        rclpy.spin_once(lidar_publisher, timeout_sec=0)

        t += dt
        base_phase = 2.0 * math.pi * step_freq * t

        q_cmd = q0.copy()

        FL_hip, FL_thigh = leg_angles(base_phase)
        RR_hip, RR_thigh = leg_angles(base_phase)
        FR_hip, FR_thigh = leg_angles(base_phase + math.pi)
        RL_hip, RL_thigh = leg_angles(base_phase + math.pi)

        q_cmd[j("FL_hip_joint")] += FL_hip
        q_cmd[j("FL_thigh_joint")] += FL_thigh

        q_cmd[j("FR_hip_joint")] += FR_hip
        q_cmd[j("FR_thigh_joint")] += FR_thigh

        q_cmd[j("RL_hip_joint")] += RL_hip
        q_cmd[j("RL_thigh_joint")] += RL_thigh

        q_cmd[j("RR_hip_joint")] += RR_hip
        q_cmd[j("RR_thigh_joint")] += RR_thigh

        controller.apply_action(ArticulationAction(joint_positions=q_cmd))

        world.step(render=True)

    rclpy.shutdown()
    world.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
