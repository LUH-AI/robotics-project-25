# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import math
import time

import numpy as np
from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection3d import Detection3DPC
from dimos.types.timestamped import align_timestamped
from dimos.utils.logging_config import setup_logger

try:
    from dimos_lcm.sensor_msgs import CameraInfo
except Exception:  # pragma: no cover - optional dependency
    CameraInfo = object  # type: ignore[assignment]

logger = setup_logger()


class DetectionGoalModule(Module):
    """Convert 2D detections + world pointcloud into stable navigation goals.

    Key idea: "freeze" the observation at the image timestamp by doing TF lookup with
    `time_point=image.ts`, then project the 2D bbox into 3D using a world/map pointcloud.
    """

    color_image: In[Image]
    detection2darray: In[Detection2DArray]
    camera_info: In[CameraInfo]  # dimos_lcm.sensor_msgs.CameraInfo
    global_map: In[PointCloud2]

    # Publish to both planners; only the one wired in the current blueprint will receive it.
    goal_request: Out[PoseStamped]  # ReplanningAStarPlanner
    goal_req: Out[PoseStamped]  # ROSNav

    def __init__(
        self,
        *,
        max_goal_rate_hz: float = 1.5,
        min_goal_change_m: float = 0.3,
        match_tolerance_s: float = 0.5,
        tf_tolerance_s: float = 0.5,
        stand_off_m: float = 1.0,
        flatten_z: bool = True,
        height_filter_m: float = 0.3,
    ) -> None:
        super().__init__()
        self._camera_info: CameraInfo | None = None

        self._max_goal_rate_hz = max_goal_rate_hz
        self._min_goal_change_m = min_goal_change_m
        self._match_tolerance_s = match_tolerance_s
        self._tf_tolerance_s = tf_tolerance_s
        self._stand_off_m = stand_off_m
        self._flatten_z = flatten_z
        self._height_filter_m = height_filter_m

        self._last_goal_ts: float = 0.0
        self._last_goal_pos: Vector3 | None = None

    @rpc
    def start(self) -> None:
        super().start()

        self._disposables.add(
            Disposable(self.camera_info.subscribe(lambda msg: setattr(self, "_camera_info", msg)))
        )

        aligned = align_timestamped(
            self.color_image.observable(),  # type: ignore[no-untyped-call]
            self.detection2darray.observable(),  # type: ignore[no-untyped-call]
            self.global_map.observable(),  # type: ignore[no-untyped-call]
            buffer_size=2.0,
            match_tolerance=self._match_tolerance_s,
        )
        self._disposables.add(aligned.subscribe(self._on_aligned))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_aligned(self, aligned_tuple) -> None:  # type: ignore[no-untyped-def]
        image, det_array, world_pc = aligned_tuple

        if self._camera_info is None:
            return
        if det_array.detections_length == 0:
            return

        now = time.time()
        if self._max_goal_rate_hz > 0 and (now - self._last_goal_ts) < (1.0 / self._max_goal_rate_hz):
            return

        det = self._select_detection(det_array, image=image)
        if det is None:
            return

        world_to_optical = self.tf.get(
            "camera_optical",
            world_pc.frame_id,
            time_point=image.ts,
            time_tolerance=self._tf_tolerance_s,
        )
        if world_to_optical is None:
            logger.debug("DetectionGoalModule: TF lookup failed (camera_optical ↔ world)")
            return

        detection_3d = Detection3DPC.from_2d(
            det=det,
            world_pointcloud=world_pc,
            camera_info=self._camera_info,
            world_to_optical_transform=world_to_optical,
        )
        if detection_3d is None:
            logger.debug("DetectionGoalModule: 3D projection failed")
            return

        robot_tf = self.tf.get(
            world_pc.frame_id,
            "base_link",
            time_point=image.ts,
            time_tolerance=self._tf_tolerance_s,
        )
        if robot_tf is None:
            logger.debug("DetectionGoalModule: TF lookup failed (world ↔ base_link)")
            return

        robot_pos = robot_tf.translation
        target_pos = self._compute_robust_target_position(detection_3d.pointcloud, robot_pos)
        if target_pos is None:
            return

        goal_pos = self._apply_stand_off(robot_pos, target_pos, stand_off_m=self._stand_off_m)
        if self._flatten_z:
            goal_pos = Vector3(goal_pos.x, goal_pos.y, 0.0)

        if not self._should_publish(goal_pos, now_ts=now):
            return

        yaw = math.atan2(target_pos.y - robot_pos.y, target_pos.x - robot_pos.x)
        goal = PoseStamped(
            ts=image.ts,
            frame_id=world_pc.frame_id,
            position=goal_pos,
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
        )

        self._last_goal_ts = now
        self._last_goal_pos = goal_pos

        self.goal_request.publish(goal)
        self.goal_req.publish(goal)

    def _should_publish(self, goal_pos: Vector3, *, now_ts: float) -> bool:
        if self._last_goal_pos is None:
            return True
        dx = goal_pos.x - self._last_goal_pos.x
        dy = goal_pos.y - self._last_goal_pos.y
        dz = goal_pos.z - self._last_goal_pos.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < self._min_goal_change_m:
            return False
        return True

    def _select_detection(self, det_array: Detection2DArray, *, image: Image) -> Detection2DBBox | None:
        best = None
        best_score = -1.0
        for ros_det in det_array.detections:
            det = self._to_detection2d_bbox(ros_det, image=image)
            if not det.is_valid():
                continue

            score = float(det.confidence)
            if score > best_score:
                best_score = score
                best = det

        return best

    def _to_detection2d_bbox(self, ros_det, *, image: Image) -> Detection2DBBox:  # type: ignore[no-untyped-def]
        center_x = ros_det.bbox.center.position.x
        center_y = ros_det.bbox.center.position.y
        width = ros_det.bbox.size_x
        height = ros_det.bbox.size_y

        x1 = float(center_x - width / 2.0)
        y1 = float(center_y - height / 2.0)
        x2 = float(center_x + width / 2.0)
        y2 = float(center_y + height / 2.0)

        class_id_raw = None
        confidence = 0.0
        if getattr(ros_det, "results", None):
            hypothesis = ros_det.results[0].hypothesis
            class_id_raw = getattr(hypothesis, "class_id", None)
            confidence = float(getattr(hypothesis, "score", 0.0))

        try:
            class_id = int(class_id_raw) if class_id_raw is not None else 0
        except Exception:
            class_id = 0

        track_id_raw = getattr(ros_det, "id", "0")
        try:
            track_id = int(track_id_raw) if str(track_id_raw).isdigit() else 0
        except Exception:
            track_id = 0

        name = str(class_id_raw) if class_id_raw is not None else f"class_{class_id}"

        return Detection2DBBox(
            bbox=(x1, y1, x2, y2),
            track_id=track_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            ts=image.ts,
            image=image,
        )

    def _compute_robust_target_position(
        self, pointcloud: PointCloud2, robot_pos: Vector3
    ) -> Vector3 | None:
        points, _ = pointcloud.as_numpy()
        if len(points) < 10:
            return None

        height_mask = points[:, 2] > float(self._height_filter_m)
        filtered = points[height_mask]
        if len(filtered) >= 10:
            points = filtered

        dx = points[:, 0] - robot_pos.x
        dy = points[:, 1] - robot_pos.y
        distances = np.sqrt(dx * dx + dy * dy)
        if len(distances) == 0:
            return None

        distance_threshold = float(np.percentile(distances, 25))
        front_points = points[distances <= distance_threshold]

        if len(front_points) < 3:
            median_dist = float(np.median(distances))
            front_points = points[np.abs(distances - median_dist) < 0.3]
            if len(front_points) < 3:
                return None

        centroid = front_points.mean(axis=0)
        return Vector3(float(centroid[0]), float(centroid[1]), float(centroid[2]))

    def _apply_stand_off(
        self, robot_pos: Vector3, target_pos: Vector3, *, stand_off_m: float
    ) -> Vector3:
        dx = target_pos.x - robot_pos.x
        dy = target_pos.y - robot_pos.y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist <= 1e-6 or stand_off_m <= 0:
            return Vector3(target_pos.x, target_pos.y, target_pos.z)

        if dist <= stand_off_m:
            # Already close enough: keep current target (planner can stop short).
            return Vector3(target_pos.x, target_pos.y, target_pos.z)

        ux = dx / dist
        uy = dy / dist
        return Vector3(
            target_pos.x - ux * stand_off_m,
            target_pos.y - uy * stand_off_m,
            target_pos.z,
        )


detection_goal_module = DetectionGoalModule.blueprint

__all__ = ["DetectionGoalModule", "detection_goal_module"]

