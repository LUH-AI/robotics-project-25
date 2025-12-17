import os
import sys
import time
import math
import argparse
from importlib.util import find_spec
from pathlib import Path


# Ensure Isaac Sim's bundled ROS 2 libraries are discoverable even if the outer
# launcher failed to prepend LD_LIBRARY_PATH (e.g., interactive shells that drop
# exports). This is cheap and fixes the repeated "failed to load librmw_*" errors.
BRIDGE_LIB = Path(
    Path.home()
    / "miniconda3/envs/env_isaaclab_py311/lib/python3.11/site-packages/isaacsim/kit/data/Kit/Isaac-Sim/5.1/exts/3/isaacsim.ros2.bridge-4.12.4+107.3.3.lx64/humble/lib"
)
if BRIDGE_LIB.is_dir():
    ld_env = os.environ.get("LD_LIBRARY_PATH", "")
    if str(BRIDGE_LIB) not in ld_env.split(os.pathsep):
        os.environ["LD_LIBRARY_PATH"] = f"{BRIDGE_LIB}{os.pathsep}{ld_env}" if ld_env else str(BRIDGE_LIB)
    path_env = os.environ.get("PATH", "")
    if str(BRIDGE_LIB) not in path_env.split(os.pathsep):
        os.environ["PATH"] = f"{BRIDGE_LIB}{os.pathsep}{path_env}" if path_env else str(BRIDGE_LIB)
    os.environ.setdefault("ROS_DISTRO", "humble")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")


def _preload_libstdcpp() -> None:
    """Ensure a recent libstdc++ (CXXABI >= 1.3.15) is loaded before ROS2 imports.

    Isaac Sim's ROS2 bridge extension pulls in OpenCV / TBB, which require a newer
    libstdc++ than the one bundled inside env_isaaclab.  On systems where we
    cannot tweak LD_PRELOAD easily (ThinLinc), we dlopen a better candidate early.
    """

    override = os.environ.get("GO2_LIBSTDCXX_OVERRIDE", "").strip()
    candidates: list[Path] = []
    if override:
        candidates.append(Path(override))

    home = Path.home()
    candidates.extend(
        [
            home / ".conda/envs/ros2_humble/lib/libstdc++.so.6",
            Path("/home/steffan.wolter/.conda/envs/ros2_humble/lib/libstdc++.so.6"),
            Path("/conda/envs/env_isaaclab_py311/lib/libstdc++.so.6"),
            Path("/conda/envs/ros2_humble/lib/libstdc++.so.6"),
            Path("/opt/ros/humble/lib/libstdc++.so.6"),
        ]
    )

    for candidate in candidates:
        if not candidate.is_file():
            continue
        try:
            import ctypes

            ctypes.CDLL(str(candidate), mode=getattr(ctypes, "RTLD_GLOBAL", 0))
            print(f"[go2_sim] Preloaded libstdc++ from {candidate}")
            return
        except Exception as exc:  # pragma: no cover - best-effort logging
            print(f"[go2_sim] Failed to preload {candidate}: {exc}")
            continue


_preload_libstdcpp()


def _bootstrap_internal_ros2() -> None:
    """Try to load Isaac Sim's bundled ROS2 (humble) if system rclpy isn't available.

    Works for pip installs where Isaac Sim ships a Python-matched ROS2 under:
    <env>/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/
    """
    try:
        import rclpy  # noqa: F401
        return
    except Exception:
        pass

    spec = find_spec("isaacsim")
    if spec is None or spec.origin is None:
        return

    ext_root = Path(spec.origin).parent / "exts" / "isaacsim.ros2.bridge" / "humble"
    py_root = ext_root / "rclpy"
    lib_root = ext_root / "lib"

    if py_root.is_dir():
        sys.path.insert(0, str(py_root))

    if lib_root.is_dir():
        # Windows: DLL search via PATH/add_dll_directory. Linux: also LD_LIBRARY_PATH.
        path_env = os.environ.get("PATH", "")
        os.environ["PATH"] = path_env + os.pathsep + str(lib_root) if path_env else str(lib_root)
        if os.name != "nt":
            ld_env = os.environ.get("LD_LIBRARY_PATH", "")
            os.environ["LD_LIBRARY_PATH"] = ld_env + os.pathsep + str(lib_root) if ld_env else str(lib_root)
        if hasattr(os, "add_dll_directory"):
            try:
                os.add_dll_directory(str(lib_root))
            except Exception:
                pass

        # Preload ROS2 shared libs by absolute path since glibc may not pick up
        # LD_LIBRARY_PATH changes made at runtime. We also preload Isaac Sim's
        # bundled libpython (needed by rclpy's pybind module).
        try:
            import ctypes

            # libpython shipped with Isaac Sim pip install
            kit_plugins = Path(spec.origin).parent / "kit" / "kernel" / "plugins"
            for py_name in ("libpython3.11.so.1.0", "libpython3.11.so"):
                py_lib = kit_plugins / py_name
                if py_lib.exists():
                    try:
                        ctypes.CDLL(str(py_lib), mode=getattr(ctypes, "RTLD_GLOBAL", 0))
                    except Exception:
                        pass

            # Required ROS2 libs for rclpy + common DDS deps.
            # We include second-order dependencies that otherwise fail to load
            # when the dynamic linker can't search humble/lib by name.
            preload = [
                # second-order deps
                "librcl_logging_spdlog.so",
                "libspdlog.so.1",
                "librmw_dds_common.so",
                "librmw_dds_common__rosidl_typesupport_c.so",
                "librmw_dds_common__rosidl_typesupport_cpp.so",
                "librosidl_typesupport_introspection_cpp.so",
                "librosidl_typesupport_introspection_c.so",
                "librosidl_typesupport_cpp.so",
                "librcl_interfaces__rosidl_typesupport_c.so",
                "librcl_interfaces__rosidl_generator_c.so",
                "libbuiltin_interfaces__rosidl_generator_c.so",
                "librosidl_runtime_c.so",
                "liblifecycle_msgs__rosidl_generator_c.so",
                "librosidl_typesupport_fastrtps_c.so",
                "librosidl_typesupport_fastrtps_cpp.so",
                "librclcpp.so",
                "librclcpp_action.so",
                "libtf2.so",
                "libtf2_ros.so",
                "libconsole_bridge.so.1.0",
                "libtinyxml2.so.9",
                "libfastrtps.so.2.6",
                "libyaml.so",
                "librcpputils.so",
                "libament_index_cpp.so",
                "librosidl_typesupport_c.so",
                "librmw_fastrtps_shared_cpp.so",
                "libfastcdr.so.1",
                "libfastcdr.so",
                "libfastrtps.so",
                # first-order deps from _rclpy_pybind11
                "librcutils.so",
                "librmw.so",
                "librmw_implementation.so",
                "librcl_logging_interface.so",
                "librcl_yaml_param_parser.so",
                "librcl.so",
                "librcl_action.so",
                "librcl_lifecycle.so",
                "liblifecycle_msgs__rosidl_typesupport_c.so",
                "librmw_fastrtps_cpp.so",
            ]
            loaded: set[str] = set()
            for _ in range(12):
                progress = False
                for name in preload:
                    if name in loaded:
                        continue
                    candidate = lib_root / name
                    if not candidate.exists():
                        continue
                    try:
                        ctypes.CDLL(str(candidate), mode=getattr(ctypes, "RTLD_GLOBAL", 0))
                        loaded.add(name)
                        progress = True
                    except Exception:
                        pass
                if not progress:
                    break

            # Broad preload of remaining ROS2 libs (message typesupport, etc.)
            # This helps Isaac Sim's ROS2 bridge extension, which has many DT_NEEDED
            # dependencies in humble/lib that won't be found by name otherwise.
            remaining = list(lib_root.glob("*.so*"))
            for _ in range(4):
                progress = False
                for candidate in remaining:
                    if candidate.name in loaded:
                        continue
                    try:
                        ctypes.CDLL(str(candidate), mode=getattr(ctypes, "RTLD_GLOBAL", 0))
                        loaded.add(candidate.name)
                        progress = True
                    except Exception:
                        pass
                if not progress:
                    break
        except Exception:
            pass

    os.environ.setdefault("ROS_DISTRO", "humble")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")


_bootstrap_internal_ros2()


def _bootstrap_isaaclab_extensions() -> None:
    """Expose IsaacLab extension packages shipped inside the pip wheel.

    Pip installs of IsaacLab include extensions under:
    <env>/site-packages/isaaclab/source/<ext_name>/
    We add these roots to sys.path so imports like `isaaclab_assets` work.
    """
    spec = find_spec("isaaclab")
    if spec is None or spec.origin is None:
        return
    src_root = Path(spec.origin).parent / "source"
    # Insert core first so `import isaaclab.*` resolves to the extension version.
    for ext_name in ("isaaclab", "isaaclab_tasks", "isaaclab_assets", "isaaclab_rl"):
        ext_root = src_root / ext_name
        if ext_root.is_dir() and str(ext_root) not in sys.path:
            sys.path.insert(0, str(ext_root))


_bootstrap_isaaclab_extensions()


def _bootstrap_pip_cuda_libs() -> None:
    """Ensure CUDA shared libs from pip-installed NVIDIA wheels are discoverable.

    Torch in Isaac Sim pip env relies on libs shipped in `site-packages/nvidia/*/lib`.
    On some systems these paths are not on the loader search path, leading to
    errors like `libcusparseLt.so.0: cannot open shared object file`.
    """
    if os.name == "nt":
        # Windows uses PATH / add_dll_directory.
        path_var = "PATH"
    else:
        path_var = "LD_LIBRARY_PATH"

    spec = find_spec("nvidia")
    if spec is None or not spec.submodule_search_locations:
        return

    nvidia_root = Path(list(spec.submodule_search_locations)[0])
    lib_dirs: list[str] = []
    for child in nvidia_root.iterdir():
        for sub in ("lib", "lib64"):
            libdir = child / sub
            if libdir.is_dir():
                lib_dirs.append(str(libdir))

    if not lib_dirs:
        return

    # Note: glibc caches LD_LIBRARY_PATH at process start, so updating it
    # here may not help dlopen() by name. We still set it for child processes
    # and additionally pre-load the critical libs by absolute path.
    existing = os.environ.get(path_var, "")
    for libdir in lib_dirs:
        if libdir not in existing.split(os.pathsep):
            existing = libdir + os.pathsep + existing if existing else libdir
            if hasattr(os, "add_dll_directory"):
                try:
                    os.add_dll_directory(libdir)
                except Exception:
                    pass

    os.environ[path_var] = existing
    # Also extend PATH on Linux to help dlopen in some setups.
    if os.name != "nt":
        path_env = os.environ.get("PATH", "")
        for libdir in lib_dirs:
            if libdir not in path_env.split(os.pathsep):
                path_env = libdir + os.pathsep + path_env if path_env else libdir
        os.environ["PATH"] = path_env

    # Preload known CUDA libs that Torch expects but may not find by name.
    try:
        import ctypes
        preload_names = ["libcusparseLt.so.0"]
        for name in preload_names:
            for libdir in lib_dirs:
                candidate = Path(libdir) / name
                if candidate.exists():
                    try:
                        ctypes.CDLL(str(candidate))
                    except Exception:
                        pass
                    break
    except Exception:
        pass


_bootstrap_pip_cuda_libs()

import hydra
import torch
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on running the cartpole RL environment.")
# parser.add_argument('-/persistent/isaac/asset_root/default', type=str, default="/blue/prabhat/tharinduo/isaac_ros2/simulator/Assets/isaacsim_assets/Assets/Isaac/4.5", help='Path to the default asset root directory')

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# Parse AppLauncher args and leave Hydra overrides intact.
# Hydra consumes overrides from sys.argv, so strip argparse-recognized args.
args_cli, hydra_overrides = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_overrides

# Enable streaming/headless defaults.
# Set `GO2_HEADLESS=0` to get a GUI window for WASD teleop.
if os.environ.get("GO2_HEADLESS", "1").lower() in ("0", "false", "no"):
    args_cli.headless = False
else:
    args_cli.headless = True  # keep rendering enabled for streaming
    args_cli.livestream = int(os.environ.get("GO2_LIVESTREAM", "1"))  # 0=off, 1=native, 2=websocket

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
IS_HEADLESS = bool(args_cli.headless)

"""Rest everything follows."""

import torch

import omni

# Some pip/IsaacLab experience files don't auto-load Replicator in headless mode.
# Enable it explicitly since we use Replicator for RTX LiDAR and camera ROS publishers.
try:
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext_id in (
        # Replicator (RTX LiDAR + ROS image publishers)
        "omni.replicator.core",
        "omni.syntheticdata",
        # Sensors (not always loaded in headless experience files)
        "isaacsim.sensors.camera",
        "isaacsim.sensors.rtx",
        "isaacsim.sensors.physx",
    ):
        try:
            ext_manager.set_extension_enabled_immediate(ext_id, True)
        except Exception:
            pass
except Exception:
    pass

from go2.go2_env import Go2RSLEnvCfg, camera_follow
import env.sim_env as sim_env
import go2.go2_sensors as go2_sensors
import carb
import go2.go2_ctrl as go2_ctrl
try:
    import ros2.go2_ros2_bridge as go2_ros2_bridge
except Exception as e:
    go2_ros2_bridge = None
    print(f"ROS2 bridge import failed, disabled: {e}")

FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")
@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):

    env_override = os.environ.get("GO2_ENV_NAME", "").strip()
    if env_override:
        print(f"[go2_sim] Overriding env_name via GO2_ENV_NAME={env_override}")
        cfg.env_name = env_override

    # Go2 Environment setup
    go2_env_cfg = Go2RSLEnvCfg()
    go2_env_cfg.scene.num_envs = cfg.num_envs
    go2_env_cfg.decimation = math.ceil(1./go2_env_cfg.sim.dt/cfg.freq)
    go2_env_cfg.sim.render_interval = go2_env_cfg.decimation
    go2_ctrl.init_base_vel_cmd(cfg.num_envs)
    # env, policy = go2_ctrl.get_rsl_flat_policy(go2_env_cfg)
    env, policy = go2_ctrl.get_rsl_rough_policy(go2_env_cfg)

    # Simulation environment
    if (cfg.env_name == "obstacle-dense"):
        sim_env.create_obstacle_dense_env() # obstacles dense
    elif (cfg.env_name == "obstacle-medium"):
        sim_env.create_obstacle_medium_env() # obstacles medium
    elif (cfg.env_name == "obstacle-sparse"):
        sim_env.create_obstacle_sparse_env() # obstacles sparse
    elif (cfg.env_name == "warehouse"):
        sim_env.create_warehouse_env() # warehouse
    elif (cfg.env_name == "warehouse-forklifts"):
        sim_env.create_warehouse_forklifts_env() # warehouse forklifts
    elif (cfg.env_name == "warehouse-shelves"):
        sim_env.create_warehouse_shelves_env() # warehouse shelves
    elif (cfg.env_name == "full-warehouse"):
        sim_env.create_full_warehouse_env() # full warehouse
    elif (cfg.env_name == "hospital"):
        sim_env.create_hospital_env() # hospital
    elif (cfg.env_name == "grid"):
        sim_env.create_grid_env() # grid
    elif (cfg.env_name == "office"):
        sim_env.create_office_env() # office

    if os.environ.get("GO2_ENABLE_DETECTION_CUBE", "1").lower() not in ("0", "false", "no"):
        try:
            cube_mode = os.environ.get("GO2_CUBE_MODE", "HARD").upper()
            
            if cube_mode == "DEBUG":
                # DEBUG: 1 cube 3m directly in front for testing
                print("[go2_sim] DEBUG MODE: Spawning cube 3m in front of Go2...")
                sim_env.spawn_detection_cube(cfg.num_envs, env_position_m=(3.0, 0.0, 1.5))
                
            elif cube_mode == "EASY":
                # EASY: 1 cube 5m in front (fixed position, easy to find)
                print("[go2_sim] EASY MODE: Spawning cube 5m in front of Go2...")
                sim_env.spawn_detection_cube(cfg.num_envs, env_position_m=(5.0, 0.0, 1.5))
                
            elif cube_mode == "MEDIUM":
                # MEDIUM: 10 cubes random within 15m radius
                print("[go2_sim] MEDIUM MODE: Spawning 10 cubes randomly (15m radius)...")
                for i in range(10):
                    sim_env.spawn_detection_cube_random_near_go2(
                        cfg.num_envs, 
                        radius_m=15.0, 
                        min_dist_m=3.0,
                        size=0.55
                    )
                    if i == 0:  # Only print once
                        print(f"[go2_sim] Spawned cube {i+1}/10")
                        
            else:  # HARD (default)
                # HARD: 1 cube random position within 20m radius
                print("[go2_sim] HARD MODE: Spawning 1 cube randomly (20m radius)...")
                sim_env.spawn_detection_cube_random_near_go2(
                    cfg.num_envs, 
                    radius_m=20.0, 
                    min_dist_m=5.0
                )
                
            print(f"[go2_sim] Detection cube(s) spawned in {cube_mode} mode.")
        except Exception as exc:
            print(f"[WARN] Failed to spawn detection cube: {exc}")

    # Sensor setup
    sm = go2_sensors.SensorManager(cfg.num_envs)
    lidar_annotators = sm.add_rtx_lidar(getattr(cfg.sensor, "lidar_config", None))
    cameras = sm.add_camera(cfg.freq)

    # Keyboard control (GUI only).
    if (not IS_HEADLESS) and os.environ.get("GO2_ENABLE_KEYBOARD", "1").lower() not in ("0", "false", "no"):
        try:
            system_input = carb.input.acquire_input_interface()
            app_window = omni.appwindow.get_default_app_window()
            keyboard = None if app_window is None else app_window.get_keyboard()
            if keyboard is not None:
                system_input.subscribe_to_keyboard_events(keyboard, go2_ctrl.sub_keyboard_event)
        except Exception as e:
            print(f"[WARN] Keyboard teleop disabled: {e}")
    
    # ROS2 Bridge (optional; works with system rclpy or Isaac Sim bundled rclpy)
    dm = None
    rclpy_mod = None
    ros_executor = None
    if go2_ros2_bridge is not None:
        try:
            os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
            os.environ.setdefault("ROS_DOMAIN_ID", "0")
            import rclpy as rclpy_mod
            rclpy_mod.init()
            dm = go2_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, cfg)
            ros_executor = rclpy_mod.executors.SingleThreadedExecutor()
            ros_executor.add_node(dm)
            print(f"[ROS2] Bridge enabled (RMW={os.environ.get('RMW_IMPLEMENTATION')}, DOMAIN={os.environ.get('ROS_DOMAIN_ID')})")
        except Exception as e:
            dm = None
            rclpy_mod = None
            ros_executor = None
            print(f"ROS2 bridge disabled: {e}")
            if os.environ.get("GO2_DEBUG_ROS2", "0") in ("1", "true", "True"):
                import traceback
                traceback.print_exc()

    # Run simulation
    sim_step_dt = float(go2_env_cfg.sim.dt * go2_env_cfg.decimation)
    max_steps_env = os.environ.get("GO2_MAX_STEPS", "").strip()
    max_steps = int(max_steps_env) if max_steps_env else 0
    status_every_env = os.environ.get("GO2_STATUS_EVERY", "").strip()
    status_every = int(status_every_env) if status_every_env else (20 if IS_HEADLESS else 1)
    step_count = 0
    obs, _ = env.reset()
    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():            
            # control joints
            actions = policy(obs)

            # step the environment
            obs, _, _, _ = env.step(actions)

            # ROS2 data
            if dm is not None:
                dm.pub_ros2_data()
                # Keep a persistent executor so subscriptions (e.g., cmd_vel) are serviced reliably.
                if ros_executor is not None:
                    ros_executor.spin_once(timeout_sec=0.0)
                else:
                    rclpy_mod.spin_once(dm, timeout_sec=0.0)

            # Camera follow
            if (cfg.camera_follow) and (not IS_HEADLESS):
                camera_follow(env)

            step_count += 1
            if max_steps and step_count >= max_steps:
                break

            # limit loop time
            elapsed_time = time.time() - start_time
            if elapsed_time < sim_step_dt:
                sleep_duration = sim_step_dt - elapsed_time
                time.sleep(sleep_duration)
        actual_loop_time = time.time() - start_time
        rtf = min(1.0, sim_step_dt/elapsed_time)
        if status_every and (step_count % status_every == 0):
            print(f"\rStep time: {actual_loop_time*1000:.2f}ms, Real Time Factor: {rtf:.2f}", end='', flush=True)
    
    if dm is not None:
        dm.destroy_node()
        rclpy_mod.shutdown()
    simulation_app.close()

if __name__ == "__main__":
    run_simulator()
    
