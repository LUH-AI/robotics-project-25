# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.assets import ArticulationCfg
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG


@configclass
class RoboticsProject25EnvCfg(DirectRLEnvCfg):
    # env settings
    decimation = 2
    episode_length_s = 10.0

    # RL interface
    action_space = 12 # joints (for Go2 legs)
    observation_space = 48
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 120,
        render_interval=decimation,
    )

    # robot config
    robot_cfg: ArticulationCfg = UNITREE_GO2_CFG.replace(
        prim_path="/World/envs/env_.*/Go2",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.4),
            rot=(1.0, 0.0, 0.0, 0.0),
            joint_pos={
                "FL_hip_joint": 0.0,
                "FR_hip_joint": 0.0,
                "RL_hip_joint": 0.0,
                "RR_hip_joint": 0.0,
                "FL_thigh_joint": 0.9,
                "FR_thigh_joint": 0.9,
                "RL_thigh_joint": 0.9,
                "RR_thigh_joint": 0.9,
                "FL_calf_joint": -1.8,
                "FR_calf_joint": -1.8,
                "RL_calf_joint": -1.8,
                "RR_calf_joint": -1.8,
            },
        ),
    )

    # Scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        # TODO: Make them as parameters (no hardcoding)
        num_envs=1,
        env_spacing=4.0, # TODO> Make env_spacing larger if walls are big, make env_spacing bigger as we increase num_envs
        replicate_physics=True,
    )
    
    walls_asset = (
            "C:\\Users\\johnn\\Desktop\\IsaacLab\\robotics-project-25\\isaac_sim\\"
            "robotics_project_25\\source\\robotics_project_25\\robotics_project_25\\"
            "tasks\\direct\\robotics_project_25\\assets\\walls.usd"
        )
    table_asset = (
            "C:\\Users\\johnn\\Desktop\\IsaacLab\\robotics-project-25\\isaac_sim\\"
            "robotics_project_25\\source\\robotics_project_25\\robotics_project_25\\"
            "tasks\\direct\\robotics_project_25\\assets\\table.usd"
        )
