"""SAM3 real-time detector wrapper for the Go2 ROS node.

This module adapts the streaming predictor shipped with the
`object-detection/sam3-realtime` fork so that it exposes the same
`DetectionResult` structure used by the Grounded-SAM-2 backend. The goal is to
make the ROS node agnostic to the actual backend while keeping the public
interface small and easy to test.
"""

from __future__ import annotations

import time
import contextlib
from typing import Optional

import numpy as np
import torch

try:  # Reuse the shared dataclass for outputs.
    from go2_grounded_sam2 import DetectionResult
except Exception as exc:  # pragma: no cover - dependency guard
    raise RuntimeError(
        "Failed to import go2_grounded_sam2. Ensure object-detection/ is on PYTHONPATH."
    ) from exc


class Sam3RealtimeDetector:
    """Thin wrapper around the streaming SAM3 predictor.

    The predictor maintains a streaming session. Whenever the prompt changes we
    start a new session so that detections remain consistent from the next
    frame onward.
    """

    def __init__(
        self,
        checkpoint_path: Optional[str] = None,
        device: Optional[str] = None,
        compile: bool = False,
    ) -> None:
        try:
            from sam3.model_builder import build_sam3_stream_predictor
        except Exception as exc:  # pragma: no cover - import guard
            raise RuntimeError(
                "Missing SAM3 dependencies. Install via:\n"
                "  pip install -e object-detection/sam3-realtime\n"
                "and authenticate with HuggingFace to download the checkpoints."
            ) from exc

        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"

        self.predictor = build_sam3_stream_predictor(
            checkpoint_path=checkpoint_path,
            device=device,
            compile=compile,
        )
        # Use fp16 autocast on CUDA to avoid bf16/bias dtype mismatches on some GPUs.
        self._autocast = (
            torch.autocast(device_type="cuda", dtype=torch.float16)
            if device.startswith("cuda")
            else contextlib.nullcontext()
        )
        self.session_id: Optional[str] = None
        self._current_prompt = ""
        self._prompt_is_bound = False
        # Keep placeholders for interface compatibility (ROS params update them).
        self.box_threshold = 0.0
        self.text_threshold = 0.0
        self._restart_session()

    # ------------------------------------------------------------------

    def _restart_session(self) -> None:
        if self.session_id is not None:
            try:
                self.predictor.handle_request(
                    {"type": "close_session", "session_id": self.session_id}
                )
            except Exception:
                pass
        resp = self.predictor.handle_request({"type": "start_session"})
        self.session_id = resp["session_id"]
        self._prompt_is_bound = False

    # ------------------------------------------------------------------

    @staticmethod
    def _normalize_prompt(prompt: str) -> str:
        cleaned = prompt.strip()
        if cleaned and not cleaned.endswith("."):
            cleaned += "."
        return cleaned

    def detect(self, bgr_image: np.ndarray, prompt: str) -> DetectionResult:
        prompt = self._normalize_prompt(prompt)
        if not prompt:
            return DetectionResult(
                boxes_xyxy=np.zeros((0, 4), dtype=np.float32),
                scores=np.zeros((0,), dtype=np.float32),
                labels=[],
                masks=None,
                latency_s=0.0,
            )

        if prompt != self._current_prompt:
            self._current_prompt = prompt
            self._restart_session()

        rgb_image = bgr_image[:, :, ::-1].copy()
        start = time.time()

        with self._autocast, torch.inference_mode():
            self.predictor.handle_request(
                {
                    "type": "add_frame",
                    "session_id": self.session_id,
                    "frame": rgb_image,
                }
            )

            if not self._prompt_is_bound:
                self.predictor.handle_request(
                    {
                        "type": "add_prompt",
                        "session_id": self.session_id,
                        "frame_index": 0,
                        "text": prompt,
                    }
                )
                self._prompt_is_bound = True

            response = self.predictor.handle_request(
                {"type": "run_inference", "session_id": self.session_id}
            )
        outputs = response.get("outputs", {}) or {}
        latency = time.time() - start

        masks = outputs.get("out_binary_masks")
        scores = outputs.get("out_probs")
        boxes_xywh = outputs.get("out_boxes_xywh")

        if (
            masks is None
            or scores is None
            or boxes_xywh is None
            or len(masks) == 0
        ):
            return DetectionResult(
                boxes_xyxy=np.zeros((0, 4), dtype=np.float32),
                scores=np.zeros((0,), dtype=np.float32),
                labels=[],
                masks=None,
                latency_s=latency,
            )

        masks_np = np.asarray(masks).astype(np.uint8)
        scores_np = np.asarray(scores).astype(np.float32)
        boxes_np = np.asarray(boxes_xywh).astype(np.float32)
        height, width = masks_np.shape[1:3]

        boxes_xyxy = np.zeros_like(boxes_np)
        boxes_xyxy[:, 0] = boxes_np[:, 0] * width
        boxes_xyxy[:, 1] = boxes_np[:, 1] * height
        boxes_xyxy[:, 2] = (boxes_np[:, 0] + boxes_np[:, 2]) * width
        boxes_xyxy[:, 3] = (boxes_np[:, 1] + boxes_np[:, 3]) * height

        labels = [prompt] * len(scores_np)

        return DetectionResult(
            boxes_xyxy=boxes_xyxy,
            scores=scores_np,
            labels=labels,
            masks=masks_np,
            latency_s=latency,
        )
