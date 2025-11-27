# Copyright (c) 2022-2025, The Isaac Lab Project Developers
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane

from isaaclab.sim import CuboidCfg, CylinderCfg
from .robotics_project_25_env_cfg import RoboticsProject25EnvCfg
import random

class RoboticsProject25Env(DirectRLEnv):
    cfg: RoboticsProject25EnvCfg

    def __init__(self, cfg: RoboticsProject25EnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

    def _setup_scene(self):
        # Ground
        spawn_ground_plane("/World/ground", cfg=GroundPlaneCfg())

        # Walls (room)
        walls_cfg = sim_utils.UsdFileCfg(
            usd_path=self.cfg.walls_asset,
            visible=True,
            mass_props=sim_utils.MassPropertiesCfg(mass=0.0),  # static
            rigid_props=sim_utils.RigidBodyPropertiesCfg()
        )
        
        # Red sphere obstacle
        cfg_sphere = sim_utils.SphereCfg(
            radius=0.5,
            visible=True,
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 0.0, 0.0),
            ),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        )
        cfg_sphere.func("/World/sphere", cfg_sphere, translation=(1.0, -3.0, 0.7))

        env0 = self.scene.env_prim_paths[0]
        walls_cfg.func(env0 + "/walls", walls_cfg)

        # Obstacles (table)
        self._spawn_obstacles(env0)

        # Robot
        # self.robot = Articulation(self.cfg.robot_cfg)
        self.robot = UNITREE_GO2_CFG.replace(prim_path="World/Go2")
        self.scene.articulations["robot"] = self.robot

        # Light
        light = sim_utils.DomeLightCfg(
            intensity=4000, 
            color=(0.9, 0.9, 0.9)
        )
        light.func("/World/Light", light)

        # clone Envs
        self.scene.clone_environments(copy_from_source=True)



    def _spawn_obstacles(self, env_path):
        robot_pos = self.cfg.robot_cfg.init_state.pos[:2]  
        min_dist = 1.0  

        while True:
            tx = random.uniform(-4.0, +4.0)
            ty = random.uniform(-4.0, +4.0)
            dx = tx - robot_pos[0]
            dy = ty - robot_pos[1]
            if dx*dx + dy*dy >= min_dist*min_dist:
                break

        table_cfg = sim_utils.UsdFileCfg(
            usd_path=self.cfg.table_asset,
            visible=True,
            mass_props=sim_utils.MassPropertiesCfg(mass=0.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg()
        )
        table_cfg.func(
            f"{env_path}/table",
            table_cfg,
            translation=(tx, ty, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0)
        )


    
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
        '''
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

        '''
