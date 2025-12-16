import omni
import numpy as np
from pxr import Gf
import omni.replicator.core as rep
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
import os

class SensorManager:
    def __init__(self, num_envs):
        self.num_envs = num_envs

    def add_rtx_lidar(self, lidar_config: str | None = None):
        lidar_annotators = []
        # Prefer a 360° rotary 2D config for Nav2/SLAM, but keep override options.
        lidar_config = (
            lidar_config
            or os.environ.get("GO2_LIDAR_CONFIG")
            or "Example_Rotary_2D"
        )
        for env_idx in range(self.num_envs):
            _, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                # Use a relative prim name; leading '/' can create invalid paths in Replicator.
                path="lidar",
                parent=f"/World/envs/env_{env_idx}/Go2/base",
                # Isaac Sim 5.x config names are uppercase and must match SUPPORTED_LIDAR_CONFIGS.
                config=lidar_config,
                translation=(0.2, 0, 0.2),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
            )

            # Isaac Sim 5.x uses this point-cloud annotator. Keep backward-compatible fallback.
            # Prefer scan-buffer annotators (they reliably produce XYZ returns for RTX LiDAR configs).
            # The "IsaacExtractRTXSensorPointCloudNoAccumulator" annotator may exist but return empty
            # data for some lidar configs, which breaks SLAM/Nav2 (empty /scan).
            annotator_name_candidates = [
                "RtxSensorCpuIsaacCreateRTXLidarScanBuffer",
                "IsaacCreateRTXLidarScanBuffer",
                "IsaacExtractRTXSensorPointCloudNoAccumulator",
            ]
            annotator = None
            for name in annotator_name_candidates:
                try:
                    annotator = rep.AnnotatorRegistry.get_annotator(name)
                    # Helpful in Isaac Sim logs when debugging empty pointclouds.
                    print(f"[go2_sensors] Using RTX LiDAR annotator: {name} (config={lidar_config})")
                    break
                except Exception:
                    continue
            if annotator is None:
                raise RuntimeError(
                    f"Could not find a supported RTX LiDAR annotator. Tried: {annotator_name_candidates}"
                )

            # Attach the annotator to a Replicator render product of the sensor.
            # Avoid overriding render_vars here: some RTX LiDAR annotators expect their own outputs.
            hydra_texture = rep.create.render_product(sensor.GetPath(), [32, 32], name="RtxSensorRenderProduct")
            annotator.attach([hydra_texture.path])
            lidar_annotators.append(annotator)
        return lidar_annotators

    def add_camera(self, freq):
        cameras = []
        for env_idx in range(self.num_envs):
            camera = Camera(
                prim_path=f"/World/envs/env_{env_idx}/Go2/base/front_cam",
                translation=np.array([0.4, 0.0, 0.2]),
                frequency=freq,
                resolution=(640, 480),
                orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
            )
            camera.initialize()
            camera.set_focal_length(1.5)
            cameras.append(camera)
        return cameras
