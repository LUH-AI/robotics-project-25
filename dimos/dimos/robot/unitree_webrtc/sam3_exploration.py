#!/usr/bin/env python3

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

import asyncio
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
from queue import Empty, Queue
import threading
import time
from typing import Any
import webbrowser

import cv2
from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
import numpy as np
from reactivex.disposable import Disposable
from sse_starlette.sse import EventSourceResponse
import uvicorn
from unitree_webrtc_connect.constants import RTC_TOPIC

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core import In, Module, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Twist, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.robot.unitree.connection.go2 import GO2Connection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_TEMPLATES_DIR = DIMOS_PROJECT_ROOT / "dimos" / "web" / "templates"
_SAM3_UI_HTML = _TEMPLATES_DIR / "sam3_exploration.html"


@dataclass
class Sam3ExplorationConfig(ModuleConfig):
    host: str = "0.0.0.0"
    port: int = 7788
    open_browser: bool = False
    auto_start_exploration: bool = False

    # SAM3
    sam_model_path: str = "sam3.pt"
    sam_imgsz: int = 644
    sam_conf: float = 0.25
    sam_half: bool = True
    sam_device: str | None = None  # e.g. "0" or "cuda:0"
    require_cuda: bool = True

    # 2D fallback target estimation (when 3D projection fails)
    approx_object_height_m: float = 0.35
    approx_depth_min_m: float = 0.6
    approx_depth_max_m: float = 6.0
    approx_depth_default_m: float = 2.0

    # 3D target refinement (front-most points)
    front_point_min_z_m: float = 0.25
    front_point_percentile: float = 25.0

    # Runtime
    max_inference_fps: float = 10.0
    confirm_frames: int = 2

    # Navigation
    approach_distance_m: float = 0.6
    goal_update_interval_s: float = 0.75
    arrived_distance_m: float = 1.25
    sit_on_arrival: bool = False
    sit_front_angle_deg: float = 25.0
    target_lock_duration_s: float = 2.0
    target_update_min_delta_m: float = 0.2
    target_update_max_jump_m: float = 1.5
    target_update_closer_margin_m: float = 0.15
    target_update_min_interval_s: float = 0.5
    approach_stuck_threshold_m: float = 0.15
    approach_stuck_updates: int = 3
    approach_linear_speed_mps: float = 0.25
    approach_cmd_duration_s: float = 0.6

    # Streaming
    stream_fps: float = 15.0


class Sam3Exploration(Module):
    """Go2 frontier exploration + live SAM3 concept search (no VLM/LLM).

    - Exploration is started/stopped via UI (manual)
    - Runs SAM3 semantic segmentation on the live camera feed using a user-provided text prompt
    - If the prompt is detected with sufficient confidence, stops exploration and drives near the object
    - Serves a web UI with: prompt, annotated live stream, and debug logs
    """

    default_config = Sam3ExplorationConfig
    config: Sam3ExplorationConfig

    rpc_calls: list[str] = [
        "WavefrontFrontierExplorer.explore",
        "WavefrontFrontierExplorer.stop_exploration",
        "WavefrontFrontierExplorer.is_exploration_active",
        "NavigationInterface.set_goal",
        "NavigationInterface.cancel_goal",
        "NavigationInterface.get_state",
        "NavigationInterface.is_goal_reached",
        "GO2Connection.move",
        "GO2Connection.publish_request",
    ]

    color_image: In[Image]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        # Allow env override without requiring blueprint changes.
        model_env = os.environ.get("SAM3_MODEL_PATH") or os.environ.get("DIMOS_SAM3_MODEL")
        if model_env and "sam_model_path" not in kwargs:
            kwargs["sam_model_path"] = model_env

        port_env = os.environ.get("SAM3_UI_PORT")
        if port_env and "port" not in kwargs:
            try:
                kwargs["port"] = int(port_env)
            except ValueError:
                ...

        auto_start_env = os.environ.get("AUTO_START_EXPLORATION")
        if auto_start_env is not None and "auto_start_exploration" not in kwargs:
            kwargs["auto_start_exploration"] = auto_start_env.strip().lower() in {
                "1",
                "true",
                "yes",
                "y",
                "on",
            }

        open_browser_env = os.environ.get("OPEN_BROWSER") or os.environ.get("SAM3_OPEN_BROWSER")
        if open_browser_env is not None and "open_browser" not in kwargs:
            kwargs["open_browser"] = open_browser_env.strip().lower() in {
                "1",
                "true",
                "yes",
                "y",
                "on",
            }

        require_cuda_env = os.environ.get("SAM3_REQUIRE_CUDA") or os.environ.get("REQUIRE_CUDA")
        if require_cuda_env is not None and "require_cuda" not in kwargs:
            kwargs["require_cuda"] = require_cuda_env.strip().lower() in {
                "1",
                "true",
                "yes",
                "y",
                "on",
            }

        sit_env = os.environ.get("SAM3_SIT_ON_ARRIVAL")
        if sit_env is not None and "sit_on_arrival" not in kwargs:
            kwargs["sit_on_arrival"] = sit_env.strip().lower() in {
                "1",
                "true",
                "yes",
                "y",
                "on",
            }

        sit_angle_env = os.environ.get("SAM3_SIT_FRONT_ANGLE_DEG")
        if sit_angle_env is not None and "sit_front_angle_deg" not in kwargs:
            try:
                kwargs["sit_front_angle_deg"] = float(sit_angle_env)
            except ValueError:
                ...

        super().__init__(**kwargs)

        # Shared inputs
        self._input_lock = threading.Lock()
        self._latest_image: Image | None = None
        self._latest_lidar: PointCloud2 | None = None
        self._latest_odom: PoseStamped | None = None

        self._last_frame_ts: float | None = None
        self._last_frame_wall_ts: float | None = None
        self._fps_ema: float | None = None

        # Prompt / state
        self._prompt_lock = threading.Lock()
        self._prompt: str = ""
        self._confirm_counter: int = 0

        self._approach_lock = threading.Lock()
        self._approach_active: bool = False
        self._target_completed: bool = False
        self._target_world: Vector3 | None = None
        self._last_goal_ts: float = 0.0
        self._last_detection_ts: float = 0.0
        self._last_inference_ms: float | None = None
        self._last_goal_dist: float | None = None
        self._goal_stuck_updates: int = 0
        self._target_lock_until: float = 0.0
        self._target_last_update_ts: float = 0.0
        self._target_dirty: bool = False
        self._sit_sent: bool = False

        # Output frame for MJPEG
        self._frame_lock = threading.Lock()
        self._latest_jpeg: bytes | None = None
        self._frame_event = threading.Event()

        # Server + background threads
        self._stop_event = threading.Event()
        self._image_event = threading.Event()
        self._infer_thread: threading.Thread | None = None
        self._uvicorn_thread: threading.Thread | None = None
        self._uvicorn_server: uvicorn.Server | None = None

        # SSE event queue (logs + status)
        self._sse_queue: Queue[dict[str, str]] = Queue(maxsize=500)

        # Camera info (static for Go2)
        self._camera_info = GO2Connection.camera_info_static

        # Lazy-loaded SAM predictor (in inference thread)
        self._sam_predictor = None
        self._sam_init_error: str | None = None
        self._sam_device_used: str | None = None

        # Show something immediately in the UI even before frames arrive.
        self._set_placeholder_frame(
            title="Waiting for camera frames…",
            subtitle="If this persists, check image transport (JPEG/SHM recommended).",
        )

    def _set_placeholder_frame(self, *, title: str, subtitle: str) -> None:
        canvas = np.zeros((720, 1280, 3), dtype=np.uint8)
        canvas[:] = (10, 10, 14)
        cv2.putText(
            canvas,
            title,
            (48, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.05,
            (240, 240, 240),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            subtitle,
            (48, 165),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (180, 180, 190),
            2,
            cv2.LINE_AA,
        )
        ok, buf = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if ok:
            with self._frame_lock:
                self._latest_jpeg = buf.tobytes()
            self._frame_event.set()

    # --------------------
    # Lifecycle
    # --------------------

    @rpc
    def start(self) -> None:
        super().start()

        if not _SAM3_UI_HTML.exists():
            logger.warning(f"SAM3 UI HTML not found at {_SAM3_UI_HTML}")

        self._log("Starting SAM3 exploration module")
        self._log(f"UI: http://localhost:{self.config.port}")

        self._disposables.add(Disposable(self.color_image.subscribe(self._on_image)))
        self._disposables.add(Disposable(self.lidar.subscribe(self._on_lidar)))
        self._disposables.add(Disposable(self.odom.subscribe(self._on_odom)))

        self._start_server()
        self._start_inference()

        if self.config.auto_start_exploration:
            try:
                explore_rpc = self.get_rpc_calls("WavefrontFrontierExplorer.explore")
                started = bool(explore_rpc())
                self._log("Exploration started" if started else "Exploration already running")
            except Exception as e:
                self._log(f"Failed to start exploration: {e!s}")
        else:
            self._log("Exploration is idle. Start it from the UI when ready.")

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        self._image_event.set()
        self._frame_event.set()

        # Stop navigation on shutdown.
        try:
            cancel_goal_rpc = self.get_rpc_calls("NavigationInterface.cancel_goal")
            cancel_goal_rpc()
        except Exception:
            ...

        try:
            stop_explore_rpc = self.get_rpc_calls("WavefrontFrontierExplorer.stop_exploration")
            stop_explore_rpc()
        except Exception:
            ...

        if self._uvicorn_server:
            self._uvicorn_server.should_exit = True

        if self._uvicorn_thread and self._uvicorn_thread.is_alive():
            self._uvicorn_thread.join(timeout=2.0)

        if self._infer_thread and self._infer_thread.is_alive():
            self._infer_thread.join(timeout=2.0)

        super().stop()

    # --------------------
    # Inputs
    # --------------------

    def _on_image(self, img: Image) -> None:
        with self._input_lock:
            self._latest_image = img
        self._image_event.set()

    def _on_lidar(self, pc: PointCloud2) -> None:
        with self._input_lock:
            self._latest_lidar = pc

    def _on_odom(self, odom: PoseStamped) -> None:
        with self._input_lock:
            self._latest_odom = odom

    # --------------------
    # Logging + status
    # --------------------

    def _emit_sse(self, event: str, data: dict[str, Any]) -> None:
        try:
            self._sse_queue.put_nowait({"event": event, "data": json.dumps(data)})
        except Exception:
            # Drop if queue is full.
            ...

    def _log(self, msg: str) -> None:
        ts = time.time()
        logger.info(msg)
        self._emit_sse("log", {"ts": ts, "msg": msg})

    def _get_prompt(self) -> str:
        with self._prompt_lock:
            return self._prompt

    def _set_prompt(self, prompt: str) -> None:
        prompt = prompt.strip()
        with self._prompt_lock:
            self._prompt = prompt
            self._confirm_counter = 0

        with self._approach_lock:
            # New prompt resets target/approach state.
            self._approach_active = False
            self._target_completed = False
            self._target_world = None
            self._last_goal_ts = 0.0
            self._goal_stuck_updates = 0
            self._last_goal_dist = None
            self._target_lock_until = 0.0
            self._target_last_update_ts = 0.0
            self._target_dirty = False
            self._sit_sent = False

        self._log(f"Prompt set to: '{prompt}'" if prompt else "Prompt cleared")
        self._emit_sse("status", self._status_snapshot())

    def _status_snapshot(self) -> dict[str, Any]:
        with self._input_lock:
            odom = self._latest_odom
        with self._approach_lock:
            target_world = self._target_world
            approach_active = self._approach_active
            target_completed = self._target_completed
            last_detection_ts = self._last_detection_ts
            last_inference_ms = self._last_inference_ms
        prompt = self._get_prompt()

        target_dist = None
        if odom and target_world:
            target_dist = (target_world - odom.position).magnitude()

        exploring = None
        try:
            is_exploring_rpc = self.get_rpc_calls("WavefrontFrontierExplorer.is_exploration_active")
            exploring = bool(is_exploring_rpc())
        except Exception:
            exploring = None

        nav_state = None
        try:
            get_state_rpc = self.get_rpc_calls("NavigationInterface.get_state")
            nav_state = str(get_state_rpc())
        except Exception:
            nav_state = None

        return {
            "prompt": prompt,
            "exploration_active": exploring,
            "approach_active": approach_active,
            "target_completed": target_completed,
            "nav_state": nav_state,
            "sam_ready": self._sam_predictor is not None,
            "sam_device": self._sam_device_used,
            "sam_init_error": self._sam_init_error,
            "target_distance_m": target_dist,
            "last_frame_ts": self._last_frame_ts,
            "fps": self._fps_ema,
            "last_detection_ts": last_detection_ts or None,
            "last_inference_ms": last_inference_ms,
        }

    # --------------------
    # Web server
    # --------------------

    def _start_server(self) -> None:
        app = FastAPI()

        @app.get("/")
        async def index() -> Response:  # type: ignore[no-untyped-def]
            if _SAM3_UI_HTML.exists():
                return FileResponse(_SAM3_UI_HTML, media_type="text/html")
            return Response(
                content="sam3_exploration.html not found in dimos/dimos/web/templates",
                media_type="text/plain",
                status_code=500,
            )

        @app.get("/api/status")
        async def api_status() -> JSONResponse:  # type: ignore[no-untyped-def]
            return JSONResponse(self._status_snapshot())

        @app.post("/api/prompt")
        async def api_prompt(request: Request) -> JSONResponse:  # type: ignore[no-untyped-def]
            try:
                payload = await request.json()
            except Exception:
                payload = {}
            prompt = str(payload.get("prompt", ""))
            self._set_prompt(prompt)
            return JSONResponse({"ok": True, "prompt": self._get_prompt()})

        @app.post("/api/exploration/start")
        async def api_start_exploration() -> JSONResponse:  # type: ignore[no-untyped-def]
            try:
                explore_rpc = self.get_rpc_calls("WavefrontFrontierExplorer.explore")
                started = bool(explore_rpc())
                self._log("Exploration started (UI)" if started else "Exploration already running (UI)")
            except Exception as e:
                self._log(f"Failed to start exploration (UI): {e!s}")
            return JSONResponse(self._status_snapshot())

        @app.post("/api/exploration/stop")
        async def api_stop_exploration() -> JSONResponse:  # type: ignore[no-untyped-def]
            try:
                stop_explore_rpc = self.get_rpc_calls("WavefrontFrontierExplorer.stop_exploration")
                cancel_goal_rpc = self.get_rpc_calls("NavigationInterface.cancel_goal")
                cancel_goal_rpc()
                stop_explore_rpc()
                self._log("Exploration stopped (UI)")
            except Exception as e:
                self._log(f"Failed to stop exploration (UI): {e!s}")
            return JSONResponse(self._status_snapshot())

        @app.get("/stream")
        async def stream() -> StreamingResponse:  # type: ignore[no-untyped-def]
            return StreamingResponse(
                self._mjpeg_generator(),
                media_type="multipart/x-mixed-replace; boundary=frame",
                headers={
                    "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
                    "Pragma": "no-cache",
                },
            )

        @app.get("/snapshot.jpg")
        async def snapshot() -> Response:  # type: ignore[no-untyped-def]
            with self._frame_lock:
                frame = self._latest_jpeg
            if frame is None:
                return Response(status_code=503, media_type="image/jpeg", content=b"")
            return Response(
                content=frame,
                media_type="image/jpeg",
                headers={
                    "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
                    "Pragma": "no-cache",
                },
            )


        @app.post("/api/reset")
        async def api_reset() -> JSONResponse:  # type: ignore[no-untyped-def]
            """Reset the demo after a found target: stop nav/exploration, stand up, clear prompt/state."""
            try:
                cancel_goal_rpc = self.get_rpc_calls("NavigationInterface.cancel_goal")
                cancel_goal_rpc()
            except Exception:
                ...

            try:
                stop_explore_rpc = self.get_rpc_calls("WavefrontFrontierExplorer.stop_exploration")
                stop_explore_rpc()
            except Exception:
                ...

            self._stand_up()
            self._set_prompt("")
            self._log("Reset complete (stand up + prompt cleared).")
            return JSONResponse(self._status_snapshot())


        @app.get("/api/events")
        async def events() -> EventSourceResponse:  # type: ignore[no-untyped-def]
            async def event_generator():  # type: ignore[no-untyped-def]
                # Initial status + welcome log
                yield {"event": "status", "data": json.dumps(self._status_snapshot())}
                yield {"event": "log", "data": json.dumps({"ts": time.time(), "msg": "Connected"})}

                last_status_emit = 0.0
                while not self._stop_event.is_set():
                    try:
                        item = self._sse_queue.get(timeout=1.0)
                        yield item
                    except Empty:
                        # Keep connection alive + provide heartbeat status even if no frames arrive.
                        yield {"event": "ping", "data": ""}
                        now = time.time()
                        if (now - last_status_emit) >= 1.0:
                            last_status_emit = now
                            yield {"event": "status", "data": json.dumps(self._status_snapshot())}
                    await asyncio.sleep(0.01)

            return EventSourceResponse(event_generator())

        config = uvicorn.Config(
            app,
            host=self.config.host,
            port=self.config.port,
            log_level="warning",
            access_log=False,
        )
        self._uvicorn_server = uvicorn.Server(config)

        def _run() -> None:
            try:
                asyncio.set_event_loop(asyncio.new_event_loop())
                self._uvicorn_server.run()
            except BaseException as e:
                self._log(f"UI server failed: {e!s}")

        self._uvicorn_thread = threading.Thread(target=_run, daemon=True)
        self._uvicorn_thread.start()

        if self.config.open_browser:
            try:
                webbrowser.open_new_tab(f"http://localhost:{self.config.port}")
            except Exception:
                ...

    def _mjpeg_generator(self):  # type: ignore[no-untyped-def]
        min_period = 1.0 / max(1e-6, float(self.config.stream_fps))
        last_sent = 0.0

        while not self._stop_event.is_set():
            # Rate-limit
            now = time.monotonic()
            sleep_for = (last_sent + min_period) - now
            if sleep_for > 0:
                time.sleep(min(sleep_for, 0.05))

            # Wait for at least one frame.
            if self._latest_jpeg is None:
                self._frame_event.wait(timeout=0.5)
                self._frame_event.clear()
                continue

            with self._frame_lock:
                frame = self._latest_jpeg

            if frame is None:
                continue

            last_sent = time.monotonic()
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: "
                + str(len(frame)).encode("ascii")
                + b"\r\n\r\n"
                + frame
                + b"\r\n"
            )

    # --------------------
    # SAM3 inference + control
    # --------------------

    def _start_inference(self) -> None:
        self._infer_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._infer_thread.start()

    @staticmethod
    def _resolve_label(names: Any, class_id: int, fallback: str) -> str:
        if isinstance(names, dict):
            return str(names.get(class_id, fallback))
        if isinstance(names, (list, tuple)):
            if 0 <= class_id < len(names):
                return str(names[class_id])
            return fallback
        if isinstance(names, str):
            return names
        return fallback

    def _sanitize_imgsz(self, imgsz: int) -> int:
        stride = 14
        if imgsz <= 0:
            return 640
        if imgsz % stride == 0:
            return imgsz
        adjusted = int(math.ceil(imgsz / stride) * stride)
        self._log(f"SAM3 imgsz adjusted {imgsz} -> {adjusted} (stride {stride})")
        return adjusted

    def _estimate_depth_from_bbox(self, bbox: tuple[float, float, float, float], image: Image) -> float:
        x1, y1, x2, y2 = bbox
        width_px = max(1.0, float(x2 - x1))
        height_px = max(1.0, float(y2 - y1))
        size_px = max(width_px, height_px)
        if image.height > 0:
            size_px = min(size_px, float(image.height))
        fy = float(self._camera_info.K[4])
        assumed_size = float(self.config.approx_object_height_m)
        depth = (fy * assumed_size) / size_px if size_px > 0 else float(
            self.config.approx_depth_default_m
        )
        if not math.isfinite(depth):
            depth = float(self.config.approx_depth_default_m)
        return float(
            np.clip(
                depth,
                float(self.config.approx_depth_min_m),
                float(self.config.approx_depth_max_m),
            )
        )

    def _estimate_target_from_bbox(
        self, bbox: tuple[float, float, float, float], image: Image, odom: PoseStamped | None
    ) -> Vector3 | None:
        if odom is None:
            return None

        fx, fy, cx, cy = (
            float(self._camera_info.K[0]),
            float(self._camera_info.K[4]),
            float(self._camera_info.K[2]),
            float(self._camera_info.K[5]),
        )
        x1, y1, x2, y2 = bbox
        px = (float(x1) + float(x2)) * 0.5
        py = (float(y1) + float(y2)) * 0.5
        depth = self._estimate_depth_from_bbox(bbox, image)

        x_norm = (px - cx) / fx
        y_norm = (py - cy) / fy
        point_cam = Vector3(x_norm * depth, y_norm * depth, depth)

        world_frame = odom.frame_id or "world"
        cam_to_world = self.tf.get(
            world_frame,
            "camera_optical",
            time_point=image.ts,
            time_tolerance=5.0,
        )
        if cam_to_world is None:
            return None

        world_point = cam_to_world.rotation.rotate_vector(point_cam) + cam_to_world.translation
        if not math.isfinite(world_point.x) or not math.isfinite(world_point.y):
            return None
        return Vector3(world_point.x, world_point.y, world_point.z)

    def _front_target_from_pointcloud(
        self, pointcloud: PointCloud2, robot_pos: Vector3
    ) -> Vector3 | None:
        points, _ = pointcloud.as_numpy()
        if len(points) < 10:
            return None

        min_z = float(self.config.front_point_min_z_m)
        height_mask = points[:, 2] > min_z
        filtered = points[height_mask]
        if len(filtered) >= 10:
            points = filtered

        dx = points[:, 0] - robot_pos.x
        dy = points[:, 1] - robot_pos.y
        distances = np.sqrt(dx * dx + dy * dy)
        if len(distances) == 0:
            return None

        percentile = float(self.config.front_point_percentile)
        distance_threshold = np.percentile(distances, percentile)
        front_points = points[distances <= distance_threshold]
        if len(front_points) < 3:
            median_dist = np.median(distances)
            close_mask = np.abs(distances - median_dist) < 0.3
            front_points = points[close_mask]
            if len(front_points) < 3:
                return None

        centroid = front_points.mean(axis=0)
        return Vector3(float(centroid[0]), float(centroid[1]), float(centroid[2]))

    def _transform_point_to_world(
        self, point: Vector3, src_frame: str, ts: float, world_frame: str
    ) -> Vector3 | None:
        src_to_world = self.tf.get(world_frame, src_frame, time_point=ts, time_tolerance=5.0)
        if src_to_world is None:
            return None
        world_point = src_to_world.rotation.rotate_vector(point) + src_to_world.translation
        if not math.isfinite(world_point.x) or not math.isfinite(world_point.y):
            return None
        return Vector3(world_point.x, world_point.y, world_point.z)

    def _init_sam(self):  # type: ignore[no-untyped-def]
        from ultralytics.models.sam import SAM3SemanticPredictor

        model_path = Path(self.config.sam_model_path)
        if not model_path.exists():
            raise FileNotFoundError(
                f"SAM3 weights not found at '{model_path}'. Set SAM3_MODEL_PATH or DIMOS_SAM3_MODEL."
            )

        # Force CUDA by default when available (user requested GPU-only execution).
        device_override: str | None = self.config.sam_device
        cuda_available = False
        try:
            import torch

            cuda_available = bool(torch.cuda.is_available())
        except Exception:
            cuda_available = False

        if device_override is None and cuda_available:
            # Ultralytics accepts "0" for cuda:0.
            device_override = "0"

        if bool(self.config.require_cuda) and not cuda_available:
            raise RuntimeError(
                "CUDA is required for SAM3 in this demo but torch.cuda.is_available() is False. "
                "Install a CUDA-enabled PyTorch and ensure an NVIDIA GPU is available."
            )

        # Half precision only makes sense on CUDA.
        use_half = bool(self.config.sam_half) and cuda_available and (device_override not in ("cpu", "-1"))

        imgsz = self._sanitize_imgsz(int(self.config.sam_imgsz))
        overrides: dict[str, Any] = {
            "conf": float(self.config.sam_conf),
            "task": "segment",
            "mode": "predict",
            "model": str(model_path),
            "imgsz": imgsz,
            "half": use_half,
            "verbose": False,
            "save": False,
            "save_txt": False,
            "save_conf": False,
            "save_crop": False,
            "show": False,
        }
        if device_override is not None:
            overrides["device"] = device_override

        predictor = SAM3SemanticPredictor(overrides=overrides)
        predictor.setup_model(verbose=False)
        device_label = str(device_override or "auto")
        if device_label == "0":
            device_label = "cuda:0"
        self._sam_device_used = device_label
        return predictor

    def _inference_loop(self) -> None:
        self._log("SAM3 inference thread started")

        try:
            self._sam_predictor = self._init_sam()
            self._sam_init_error = None
            self._log(f"SAM3 loaded: {self.config.sam_model_path}")
        except Exception as e:
            self._sam_predictor = None
            self._sam_init_error = str(e)
            self._log(f"SAM3 init failed: {e!s}")

        min_period = 1.0 / max(1e-6, float(self.config.max_inference_fps))
        last_run = 0.0

        while not self._stop_event.is_set():
            if not self._image_event.wait(timeout=0.5):
                continue
            self._image_event.clear()

            # Rate-limit inference.
            now = time.monotonic()
            sleep_for = (last_run + min_period) - now
            if sleep_for > 0:
                time.sleep(min(sleep_for, 0.05))
            last_run = time.monotonic()

            with self._input_lock:
                image = self._latest_image
                lidar = self._latest_lidar
                odom = self._latest_odom

            if image is None:
                continue

            # Frame timing / FPS tracking.
            now_wall = time.time()
            if self._last_frame_wall_ts is not None:
                dt = now_wall - self._last_frame_wall_ts
                if dt > 1e-3:
                    inst = 1.0 / dt
                    self._fps_ema = inst if self._fps_ema is None else (0.85 * self._fps_ema + 0.15 * inst)
            self._last_frame_wall_ts = now_wall
            self._last_frame_ts = image.ts

            prompt = self._get_prompt()
            bgr = image.to_bgr().to_opencv()
            annotated = bgr.copy()

            t0 = time.perf_counter()
            best_target_world: Vector3 | None = None
            best_bbox: tuple[float, float, float, float] | None = None
            best_conf = 0.0
            found_any = False

            if prompt and self._sam_predictor is not None:
                try:
                    self._sam_predictor.set_image(bgr)
                    results = self._sam_predictor(text=[prompt])
                    r0 = results[0]

                    if r0.boxes is not None and len(r0.boxes) > 0:
                        confs = r0.boxes.conf.detach().cpu().numpy()
                        best_idx = int(np.argmax(confs))
                        best_conf = float(confs[best_idx])
                        found_any = True
                        best_xyxy = r0.boxes.xyxy[best_idx].detach().cpu().numpy().astype(float)
                        best_bbox = (
                            float(best_xyxy[0]),
                            float(best_xyxy[1]),
                            float(best_xyxy[2]),
                            float(best_xyxy[3]),
                        )

                        # Draw all detections.
                        for i in range(len(confs)):
                            xyxy = r0.boxes.xyxy[i].detach().cpu().numpy().astype(int)
                            cls_i = int(r0.boxes.cls[i].detach().cpu())
                            label = self._resolve_label(getattr(r0, "names", None), cls_i, prompt)
                            conf_i = float(confs[i])
                            color = (64, 255, 160) if i == best_idx else (120, 170, 255)
                            cv2.rectangle(
                                annotated,
                                (int(xyxy[0]), int(xyxy[1])),
                                (int(xyxy[2]), int(xyxy[3])),
                                color,
                                2,
                            )
                            cv2.putText(
                                annotated,
                                f"{label} {conf_i:.2f}",
                                (int(xyxy[0]), max(16, int(xyxy[1]) - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                color,
                                2,
                                cv2.LINE_AA,
                            )

                        # 3D localization for the best detection (using lidar projection).
                        if lidar is not None:
                            try:
                                det2d = Detection2DBBox.from_ultralytics_result(r0, best_idx, image)
                                tf = self.tf.get(
                                    "camera_optical",
                                    lidar.frame_id,
                                    time_point=image.ts,
                                    time_tolerance=5.0,
                                )
                                if tf is not None:
                                    det3d = Detection3DPC.from_2d(
                                        det2d,
                                        world_pointcloud=lidar,
                                        camera_info=self._camera_info,
                                        world_to_optical_transform=tf,
                                    )
                                    if det3d is not None:
                                        if odom is not None:
                                            world_frame = odom.frame_id or "world"
                                            lidar_to_world = self.tf.get(
                                                world_frame,
                                                det3d.pointcloud.frame_id,
                                                time_point=image.ts,
                                                time_tolerance=5.0,
                                            )
                                            if lidar_to_world is not None:
                                                world_pc = det3d.pointcloud.transform(lidar_to_world)
                                                refined = self._front_target_from_pointcloud(
                                                    world_pc, odom.position
                                                )
                                                best_target_world = refined or self._transform_point_to_world(
                                                    det3d.center,
                                                    det3d.pointcloud.frame_id,
                                                    image.ts,
                                                    world_frame,
                                                )
                                            else:
                                                best_target_world = self._transform_point_to_world(
                                                    det3d.center,
                                                    det3d.pointcloud.frame_id,
                                                    image.ts,
                                                    world_frame,
                                                )
                                        else:
                                            best_target_world = None
                            except Exception:
                                # Projection is best-effort; keep the detection overlay anyway.
                                ...
                        if best_target_world is None and best_bbox is not None:
                            best_target_world = self._estimate_target_from_bbox(
                                best_bbox, image, odom
                            )

                except Exception as e:
                    self._log(f"SAM3 inference error: {e!s}")

            infer_ms = (time.perf_counter() - t0) * 1000.0
            with self._approach_lock:
                self._last_inference_ms = infer_ms
                if found_any:
                    self._last_detection_ts = time.time()

            # Decide state transition.
            self._maybe_trigger(
                found_any=found_any, best_conf=best_conf, target_world=best_target_world
            )

            # Render HUD.
            self._draw_hud(
                annotated,
                prompt=prompt,
                found_any=found_any,
                best_conf=best_conf,
                odom=odom,
                target_world=best_target_world,
                infer_ms=infer_ms,
            )

            # Publish MJPEG frame.
            ok, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if ok:
                with self._frame_lock:
                    self._latest_jpeg = buf.tobytes()
                self._frame_event.set()

            # Push status at a modest rate (UI uses it for cards).
            self._emit_sse("status", self._status_snapshot())

    def _maybe_trigger(self, *, found_any: bool, best_conf: float, target_world: Vector3 | None) -> None:
        prompt = self._get_prompt()
        if not prompt:
            return

        update_goal = False
        with self._input_lock:
            odom = self._latest_odom
        with self._approach_lock:
            if self._target_completed:
                return
            if self._approach_active:
                if target_world is not None:
                    now = time.time()
                    if (
                        now >= self._target_lock_until
                        and (now - self._target_last_update_ts)
                        >= float(self.config.target_update_min_interval_s)
                        and self._should_update_target(target_world, odom)
                    ):
                        self._target_world = target_world
                        self._target_last_update_ts = now
                        self._target_dirty = True
                update_goal = True

        if update_goal:
            self._update_goal_if_needed()
            return

        with self._prompt_lock:
            # If we've already reached a target for the current prompt, do not re-trigger.
            if found_any and best_conf >= float(self.config.sam_conf):
                self._confirm_counter += 1
            else:
                self._confirm_counter = 0
            confirmed = self._confirm_counter >= int(self.config.confirm_frames)

        if not confirmed:
            return

        with self._approach_lock:
            self._approach_active = True
            self._target_world = target_world
            # Avoid repeated triggers after the first acquisition.
            self._confirm_counter = 0
            now = time.time()
            self._target_lock_until = now + float(self.config.target_lock_duration_s)
            self._target_last_update_ts = now
            self._target_dirty = True

        self._log(f"Target acquired (conf={best_conf:.2f}). Stopping exploration and approaching.")

        # Hard stop exploration + current navigation.
        try:
            stop_explore_rpc = self.get_rpc_calls("WavefrontFrontierExplorer.stop_exploration")
            cancel_goal_rpc = self.get_rpc_calls("NavigationInterface.cancel_goal")
            cancel_goal_rpc()
            stop_explore_rpc()
            # Reset stuck tracking on fresh acquisition.
            with self._approach_lock:
                self._goal_stuck_updates = 0
                self._last_goal_dist = None
                self._target_lock_until = 0.0
                self._target_last_update_ts = time.time()
                self._target_dirty = True
        except Exception as e:
            self._log(f"Failed to stop exploration/navigation: {e!s}")

        # Send approach goal (best-effort).
        self._update_goal_if_needed(force=True)

    def _should_update_target(self, new_target: Vector3, odom: PoseStamped | None) -> bool:
        current_target = self._target_world
        if current_target is None:
            return True

        delta = (new_target - current_target).magnitude()
        if delta < float(self.config.target_update_min_delta_m):
            return False
        if delta > float(self.config.target_update_max_jump_m):
            return False

        if odom is None:
            return True

        old_dist = (current_target - odom.position).magnitude()
        new_dist = (new_target - odom.position).magnitude()
        if new_dist > (old_dist + float(self.config.target_update_closer_margin_m)):
            return False

        return True

    def _update_goal_if_needed(self, *, force: bool = False) -> None:
        with self._input_lock:
            odom = self._latest_odom

        with self._approach_lock:
            target_world = self._target_world
            last_goal_ts = self._last_goal_ts
            target_dirty = self._target_dirty

        if odom is None:
            return

        now = time.time()
        if not force and (now - last_goal_ts) < float(self.config.goal_update_interval_s):
            return

        if target_world is None:
            # No 3D localization yet: hold position until a 3D estimate is available.
            return

        if not force:
            try:
                nav_state = self.get_rpc_calls("NavigationInterface.get_state")()
                nav_state_str = str(nav_state).lower()
                if ("following" in nav_state_str or "path_following" in nav_state_str) and not target_dirty:
                    return
            except Exception:
                ...

        dist_to_target = (target_world - odom.position).magnitude()
        if dist_to_target <= float(self.config.arrived_distance_m):
            self._log(f"Arrived near target (dist={dist_to_target:.2f}m).")
            self._maybe_sit_on_arrival(odom=odom, target_world=target_world)
            with self._approach_lock:
                self._approach_active = False
                self._target_completed = True
                self._goal_stuck_updates = 0
                self._last_goal_dist = None
                self._target_dirty = False
            return

        delta = target_world - odom.position
        direction = delta.to_2d().normalize()
        goal_pos = target_world - direction * float(self.config.approach_distance_m)

        yaw = math.atan2(delta.y, delta.x)
        goal = PoseStamped(
            position=Vector3(goal_pos.x, goal_pos.y, 0.0),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
            frame_id=odom.frame_id or "world",
            ts=time.time(),
        )

        try:
            set_goal_rpc = self.get_rpc_calls("NavigationInterface.set_goal")
            set_goal_rpc(goal)
            with self._approach_lock:
                self._last_goal_ts = now
                self._target_dirty = False
            self._log(
                f"Approach goal updated: ({goal.position.x:.2f}, {goal.position.y:.2f}) "
                f"dist={dist_to_target:.2f}m"
            )
        except Exception as e:
            self._log(f"Failed to set approach goal: {e!s}")
            self._direct_drive_toward(target_world, odom)

        # Track progress to detect being stuck.
        with self._approach_lock:
            if dist_to_target is not None:
                prev = self._last_goal_dist
                if prev is not None and (prev - dist_to_target) < float(self.config.approach_stuck_threshold_m):
                    self._goal_stuck_updates += 1
                else:
                    self._goal_stuck_updates = 0
                self._last_goal_dist = dist_to_target

            if self._goal_stuck_updates >= int(self.config.approach_stuck_updates):
                self._log("Approach appears stuck; issuing direct forward drive.")
                self._goal_stuck_updates = 0
                self._direct_drive_toward(target_world, odom)

    def _direct_drive_toward(self, target_world: Vector3 | None, odom: PoseStamped) -> None:
        """Best-effort straight drive using Unitree move API."""
        try:
            move_rpc = self.get_rpc_calls("GO2Connection.move")
        except Exception:
            return

        # Drive forward in robot frame (Move skill convention uses x as forward).
        forward = float(self.config.approach_linear_speed_mps)
        twist = Twist(
            linear=Vector3(forward, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.0),
        )
        move_rpc(twist, float(self.config.approach_cmd_duration_s))

    def _maybe_sit_on_arrival(self, *, odom: PoseStamped, target_world: Vector3) -> None:
        if not self.config.sit_on_arrival:
            return

        if not self._target_is_in_front(odom=odom, target_world=target_world):
            return

        with self._approach_lock:
            if self._sit_sent:
                return
            self._sit_sent = True

        try:
            publish_request = self.get_rpc_calls("GO2Connection.publish_request")
        except Exception:
            self._log("Sit command skipped (GO2Connection.publish_request unavailable).")
            return

        try:
            publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": 1009})
            self._log("Sit command issued.")
        except Exception as e:
            self._log(f"Failed to issue Sit command: {e!s}")

    def _target_is_in_front(self, *, odom: PoseStamped, target_world: Vector3) -> bool:
        delta = target_world - odom.position
        if delta.length() <= 1e-6:
            return True

        try:
            local = odom.orientation.inverse().rotate_vector(delta)
        except Exception:
            local = delta

        if local.x <= 0.0:
            return False

        angle = abs(math.atan2(local.y, local.x))
        return angle <= math.radians(float(self.config.sit_front_angle_deg))

    @staticmethod
    def _draw_hud(  # type: ignore[no-untyped-def]
        img_bgr,
        *,
        prompt: str,
        found_any: bool,
        best_conf: float,
        odom: PoseStamped | None,
        target_world: Vector3 | None,
        infer_ms: float,
    ) -> None:
        # HUD panel
        h, w = img_bgr.shape[:2]
        panel_h = 84
        overlay = img_bgr.copy()
        cv2.rectangle(overlay, (0, 0), (w, panel_h), (10, 10, 12), -1)
        cv2.addWeighted(overlay, 0.75, img_bgr, 0.25, 0, img_bgr)

        status = "SEARCHING" if prompt else "IDLE"
        if prompt and found_any:
            status = f"FOUND ({best_conf:.2f})" if best_conf > 0 else "FOUND"

        dist = None
        if odom is not None and target_world is not None:
            dist = (target_world - odom.position).magnitude()

        lines = [
            f"SAM3: {status}   infer={infer_ms:.0f}ms",
            f"prompt: {prompt or '—'}",
            f"target_dist: {dist:.2f}m" if dist is not None else "target_dist: —",
        ]

        y = 26
        for line in lines:
            cv2.putText(
                img_bgr,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (240, 240, 240),
                2,
                cv2.LINE_AA,
            )
            y += 26

    def _issue_sport_api(self, api_id: int) -> bool:
        try:
            publish_request = self.get_rpc_calls("GO2Connection.publish_request")
        except Exception:
            self._log("Sport command skipped (GO2Connection.publish_request unavailable).")
            return False

        try:
            publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": int(api_id)})
            return True
        except Exception as e:
            self._log(f"Failed to issue sport command api_id={api_id}: {e!s}")
            return False

    def _stand_up(self) -> None:
        # Prefer RiseSit when the robot is sitting in front of a found target.
        if self._issue_sport_api(1010):
            self._log("Stand up command issued (RiseSit).")
            return
        if self._issue_sport_api(1004):
            self._log("Stand up command issued (StandUp).")
            return
        self._log("Stand up command failed (no supported sport command available).")




sam3_exploration = Sam3Exploration.blueprint

__all__ = ["Sam3Exploration", "sam3_exploration"]
