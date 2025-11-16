# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane

from .robotics_project_25_env_cfg import RoboticsProject25EnvCfg


class RoboticsProject25Env(DirectRLEnv):
    cfg: RoboticsProject25EnvCfg

    def __init__(self, cfg: RoboticsProject25EnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

    def _setup_scene(self):
        # Ground
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())

        # Walls (static asset)
        # TODO: Please change with your own path
        walls_usd_path = (
            "C:\\Users\\johnn\\Desktop\\IsaacLab\\robotics-project-25\\isaac_sim\\"
            "robotics_project_25\\source\\robotics_project_25\\robotics_project_25\\"
            "tasks\\direct\\robotics_project_25\\assets\\walls.usd"
        )

        walls_cfg = sim_utils.UsdFileCfg(
            usd_path=walls_usd_path,
            visible=True,
            mass_props=sim_utils.MassPropertiesCfg(mass=0.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        )

        template_env = self.scene.env_prim_paths[0] # Generate num_env roomes
        walls_cfg.func(template_env + "/walls", walls_cfg)

        # spawn Unytree Go2 robot
        self.robot = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["robot"] = self.robot

        # Lighting
        light_cfg = sim_utils.DomeLightCfg(
            intensity=4000.0, 
            color=(0.9, 0.9, 0.9)
        )
        light_cfg.func("/World/Light", light_cfg)

        # Clone environments (even if num_envs=1)
        self.scene.clone_environments(copy_from_source=True)

    
    def _pre_physics_step(self, actions: torch.Tensor):
        self.actions = actions.clone()
        self._apply_action()

    def _apply_action(self):
        self.robot.set_joint_effort_target(
            torch.zeros_like(self.robot.data.joint_pos)
        )

    def _get_observations(self):
        # observation
        obs = self.robot.data.root_pos_w.clone()
        return {"policy": obs}

    def _get_rewards(self):
        # No reward
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self):
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        terminated = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        return terminated, time_out

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)

        device = self.device

        env_base = self.scene.env_origins[env_ids]

        # random XY inside a room 
        room_half = 2.0

        rand_xy = (torch.rand(len(env_ids), 2, device=device) * 2 - 1) * room_half
        z = torch.full((len(env_ids), 1), 0.40, device=device)

        local_pos = torch.cat([rand_xy, z], dim=-1)

        world_pos = env_base + local_pos

        quat = self.robot.data.root_quat_w[env_ids]


        pose = torch.cat([world_pos, quat], dim=-1)

        self.robot.write_root_pose_to_sim(pose, env_ids)


