import os
import torch
import carb
import gymnasium as gym
from pathlib import Path
from isaaclab.envs import ManagerBasedEnv
from go2.go2_ctrl_cfg import unitree_go2_flat_cfg, unitree_go2_rough_cfg
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, RslRlOnPolicyRunnerCfg
from isaaclab_tasks.utils import get_checkpoint_path
from rsl_rl.runners import OnPolicyRunner

base_vel_cmd_input = None


def _resolve_ckpts_dir() -> str:
    """Resolve checkpoint directory robustly.

    Priority:
    1) GO2_CKPTS_DIR env var
    2) <repo>/src/ckpts
    3) <repo>/ckpts
    4) ./ckpts (cwd)
    """
    env_dir = os.environ.get("GO2_CKPTS_DIR")
    if env_dir:
        return env_dir

    # go2_ctrl.py lives in <repo>/src/go2. Resolve paths relative to that, not CWD.
    src_dir = Path(__file__).resolve().parents[1]
    repo_root = src_dir.parent
    candidates = [str(src_dir / "ckpts"), str(repo_root / "ckpts"), os.path.abspath("ckpts")]
    for c in candidates:
        if os.path.isdir(c):
            return c
    # default to <repo>/src/ckpts so the repo works out-of-the-box
    return candidates[0]


def _resolve_action_dim(env) -> int:
    """Resolve action dimension robustly across wrappers."""
    try:
        unwrapped = getattr(env, "unwrapped", None) or env
        action_manager = getattr(unwrapped, "action_manager", None)
        if action_manager is not None:
            return int(getattr(action_manager, "total_action_dim"))
    except Exception:
        pass

    shape = getattr(getattr(env, "action_space", None), "shape", None)
    if shape is not None:
        try:
            dim = 1
            for s in shape:
                dim *= int(s)
            return int(dim)
        except Exception:
            pass

    return 12

# Initialize base_vel_cmd_input as a tensor when created
def init_base_vel_cmd(num_envs):
    global base_vel_cmd_input
    base_vel_cmd_input = torch.zeros((num_envs, 3), dtype=torch.float32)

# Modify base_vel_cmd to use the tensor directly
def base_vel_cmd(env: ManagerBasedEnv) -> torch.Tensor:
    global base_vel_cmd_input
    return base_vel_cmd_input.clone().to(env.device)

# Update sub_keyboard_event to modify specific rows of the tensor based on key inputs
def sub_keyboard_event(event) -> bool:
    global base_vel_cmd_input
    lin_vel = 1.5
    ang_vel = 1.5
    
    if base_vel_cmd_input is not None:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # Update tensor values for environment 0
            if event.input.name == 'W':
                base_vel_cmd_input[0] = torch.tensor([lin_vel, 0, 0], dtype=torch.float32)
            elif event.input.name == 'S':
                base_vel_cmd_input[0] = torch.tensor([-lin_vel, 0, 0], dtype=torch.float32)
            elif event.input.name == 'A':
                base_vel_cmd_input[0] = torch.tensor([0, lin_vel, 0], dtype=torch.float32)
            elif event.input.name == 'D':
                base_vel_cmd_input[0] = torch.tensor([0, -lin_vel, 0], dtype=torch.float32)
            elif event.input.name == 'Z':
                base_vel_cmd_input[0] = torch.tensor([0, 0, ang_vel], dtype=torch.float32)
            elif event.input.name == 'C':
                base_vel_cmd_input[0] = torch.tensor([0, 0, -ang_vel], dtype=torch.float32)
            
            # If there are multiple environments, handle inputs for env 1
            if base_vel_cmd_input.shape[0] > 1:
                if event.input.name == 'I':
                    base_vel_cmd_input[1] = torch.tensor([lin_vel, 0, 0], dtype=torch.float32)
                elif event.input.name == 'K':
                    base_vel_cmd_input[1] = torch.tensor([-lin_vel, 0, 0], dtype=torch.float32)
                elif event.input.name == 'J':
                    base_vel_cmd_input[1] = torch.tensor([0, lin_vel, 0], dtype=torch.float32)
                elif event.input.name == 'L':
                    base_vel_cmd_input[1] = torch.tensor([0, -lin_vel, 0], dtype=torch.float32)
                elif event.input.name == 'M':
                    base_vel_cmd_input[1] = torch.tensor([0, 0, ang_vel], dtype=torch.float32)
                elif event.input.name == '>':
                    base_vel_cmd_input[1] = torch.tensor([0, 0, -ang_vel], dtype=torch.float32)
        
        # Reset commands to zero on key release
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            base_vel_cmd_input.zero_()
    return True

def get_rsl_flat_policy(cfg):
    cfg.observations.policy.height_scan = None
    env = gym.make("Isaac-Velocity-Flat-Unitree-Go2-v0", cfg=cfg)
    env = RslRlVecEnvWrapper(env)

    # Low level control: rsl control policy
    agent_cfg: RslRlOnPolicyRunnerCfg = unitree_go2_flat_cfg
    ppo_runner = OnPolicyRunner(env, agent_cfg, log_dir=None, device=agent_cfg["device"])
    ckpts_dir = _resolve_ckpts_dir()
    try:
        ckpt_path = get_checkpoint_path(
            log_path=ckpts_dir,
            run_dir=agent_cfg["load_run"],
            checkpoint=agent_cfg["load_checkpoint"],
        )
        ppo_runner.load(ckpt_path)
        policy = ppo_runner.get_inference_policy(device=agent_cfg["device"])
        carb.log_info(f"Loaded Go2 flat policy checkpoint: {ckpt_path}")
    except Exception as e:
        carb.log_warn(
            f"Go2 flat policy checkpoint not found in '{ckpts_dir}'. "
            f"Falling back to zero-action policy. ({e})"
        )

        def policy(obs):
            # obs is a torch tensor from RslRlVecEnvWrapper
            batch = obs.shape[0] if hasattr(obs, "shape") else env.num_envs
            act_dim = _resolve_action_dim(env)
            return torch.zeros((batch, act_dim), device=env.device, dtype=torch.float32)

    return env, policy

def get_rsl_rough_policy(cfg):
    env = gym.make("Isaac-Velocity-Rough-Unitree-Go2-v0", cfg=cfg)
    env = RslRlVecEnvWrapper(env)

    # Low level control: rsl control policy
    agent_cfg: RslRlOnPolicyRunnerCfg = unitree_go2_rough_cfg
    ppo_runner = OnPolicyRunner(env, agent_cfg, log_dir=None, device=agent_cfg["device"])
    ckpts_dir = _resolve_ckpts_dir()
    try:
        ckpt_path = get_checkpoint_path(
            log_path=ckpts_dir,
            run_dir=agent_cfg["load_run"],
            checkpoint=agent_cfg["load_checkpoint"],
        )
        ppo_runner.load(ckpt_path)
        policy = ppo_runner.get_inference_policy(device=agent_cfg["device"])
        carb.log_info(f"Loaded Go2 rough policy checkpoint: {ckpt_path}")
    except Exception as e:
        carb.log_warn(
            f"Go2 rough policy checkpoint not found in '{ckpts_dir}'. "
            f"Falling back to zero-action policy. ({e})"
        )

        def policy(obs):
            batch = obs.shape[0] if hasattr(obs, "shape") else env.num_envs
            act_dim = _resolve_action_dim(env)
            return torch.zeros((batch, act_dim), device=env.device, dtype=torch.float32)

    return env, policy
