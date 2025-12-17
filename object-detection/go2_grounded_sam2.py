"""Grounded-SAM-2 helper for Go2 camera streams.

Provides a small wrapper around the Grounding DINO HuggingFace model plus
optional SAM 2 mask prediction. The intent is to reuse the same detector in
ROS nodes and offline scripts without duplicating setup code.
"""

from __future__ import annotations

import contextlib
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np
from PIL import Image


@dataclass
class DetectionResult:
    """Container for a single inference pass."""

    boxes_xyxy: np.ndarray  # float32, shape (N, 4) in pixel coords
    scores: np.ndarray  # float32, shape (N,)
    labels: List[str]  # length N, already lowercased
    masks: Optional[np.ndarray]  # bool/uint8, shape (N, H, W) or None
    latency_s: float


class GroundedSAM2Detector:
    """Lightweight detector built from GroundingDINO (HF) + optional SAM 2 masks."""

    def __init__(
        self,
        model_id: str = "IDEA-Research/grounding-dino-tiny",
        device: Optional[str] = None,
        box_threshold: float = 0.3,
        text_threshold: float = 0.25,
        enable_masks: bool = False,
        sam2_config: str = "configs/sam2.1/sam2.1_hiera_l.yaml",
        sam2_checkpoint: str = "./checkpoints/sam2.1_hiera_large.pt",
        verbose: bool = True,
    ) -> None:
        try:
            import torch
            from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
        except Exception as exc:  # pragma: no cover - import guard
            raise RuntimeError(
                "Missing dependencies for Grounded-SAM-2. "
                "Install with: pip install -r object-detection/requirements.txt"
            ) from exc

        self.torch = torch
        self.processor = AutoProcessor.from_pretrained(model_id)

        # Pick device only after torch import to avoid cuda calls when unavailable.
        if device:
            self.device = device
        else:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"

        if verbose:
            print(f"[grounded_sam2] Loading {model_id} on device={self.device}")
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(
            self.device
        )

        self.box_threshold = float(box_threshold)
        self.text_threshold = float(text_threshold)

        # Optional SAM 2 mask predictor.
        self.enable_masks = bool(enable_masks)
        self.sam_predictor = None
        if enable_masks:
            base_dir = Path(__file__).resolve().parent / "Grounded-SAM-2"
            cfg_path = Path(sam2_config)
            ckpt_path = Path(sam2_checkpoint)

            def _resolve(path: Path) -> Path:
                candidates = [path]
                rel = path
                if path.is_absolute():
                    try:
                        rel = path.relative_to(base_dir)
                    except ValueError:
                        rel = path
                candidates.extend(
                    [
                        base_dir / rel,
                        base_dir / "sam2" / rel,
                    ]
                )
                for candidate in candidates:
                    if candidate.exists():
                        return candidate
                return path

            cfg_path = _resolve(cfg_path)
            ckpt_path = _resolve(ckpt_path)
            cfg_name_for_builder = sam2_config
            cfg_name_obj = Path(sam2_config)
            if cfg_name_obj.is_absolute():
                for root in (base_dir, base_dir / "sam2"):
                    try:
                        cfg_name_for_builder = str(cfg_name_obj.relative_to(root))
                        break
                    except ValueError:
                        continue
            if cfg_path.exists() and ckpt_path.exists():
                if verbose:
                    print(f"[grounded_sam2] Loading SAM2 cfg={cfg_path} ckpt={ckpt_path}")
                try:
                    from sam2.build_sam import build_sam2
                    from sam2.sam2_image_predictor import SAM2ImagePredictor

                    sam_model = build_sam2(cfg_name_for_builder, str(ckpt_path), device=self.device)
                    self.sam_predictor = SAM2ImagePredictor(sam_model)
                except Exception as exc:  # pragma: no cover - optional path
                    raise RuntimeError(
                        "Failed to initialize SAM2. "
                        "Verify checkpoints are downloaded via object-detection/Grounded-SAM-2/checkpoints/download_ckpts.sh"
                    ) from exc
            else:
                raise RuntimeError(
                    f"enable_masks=True but SAM2 assets not found (cfg={cfg_path}, ckpt={ckpt_path})."
                )

        # Best-effort mixed precision for GPUs; no-op on CPU.
        self._autocast = contextlib.nullcontext()
        if self.device.startswith("cuda"):
            self._autocast = torch.autocast(device_type="cuda", dtype=torch.bfloat16)
            try:
                props = torch.cuda.get_device_properties(0)
                if props.major >= 8:
                    torch.backends.cuda.matmul.allow_tf32 = True
                    torch.backends.cudnn.allow_tf32 = True
            except Exception:
                pass

    @staticmethod
    def _normalize_prompt(prompt: str) -> str:
        cleaned = prompt.strip().lower()
        if cleaned and not cleaned.endswith("."):
            cleaned += "."
        return cleaned

    def detect(self, bgr_image: np.ndarray, prompt: str) -> DetectionResult:
        """Run text-prompted detection on a BGR image.

        Args:
            bgr_image: uint8 array (H, W, 3) in BGR order.
            prompt: free-form text. Will be lowercased and suffixed with a dot.
        """
        prompt = self._normalize_prompt(prompt)
        if not prompt:
            return DetectionResult(
                boxes_xyxy=np.zeros((0, 4), dtype=np.float32),
                scores=np.zeros((0,), dtype=np.float32),
                labels=[],
                masks=None,
                latency_s=0.0,
            )

        rgb = bgr_image[:, :, ::-1].copy()
        image = Image.fromarray(rgb)
        inputs = self.processor(images=image, text=prompt, return_tensors="pt").to(
            self.device
        )

        start = time.time()
        with self._autocast, self.torch.no_grad():
            outputs = self.model(**inputs)
            results = self.processor.post_process_grounded_object_detection(
                outputs,
                inputs.input_ids,
                threshold=self.box_threshold,
                text_threshold=self.text_threshold,
                target_sizes=[image.size[::-1]],
            )
        latency = time.time() - start

        if not results:
            return DetectionResult(
                boxes_xyxy=np.zeros((0, 4), dtype=np.float32),
                scores=np.zeros((0,), dtype=np.float32),
                labels=[],
                masks=None,
                latency_s=latency,
            )

        det = results[0]
        boxes = det["boxes"].detach().cpu().numpy().astype(np.float32)
        scores = det["scores"].detach().cpu().numpy().astype(np.float32)
        raw_labels = det.get("text_labels", det.get("labels", []))
        labels = [str(lbl) for lbl in raw_labels]

        masks = None
        if self.sam_predictor is not None and boxes.size:
            # SAM2 expects numpy float boxes in XYXY.
            self.sam_predictor.set_image(rgb)
            masks_np, _, _ = self.sam_predictor.predict(
                point_coords=None,
                point_labels=None,
                box=boxes,
                multimask_output=False,
            )
            if masks_np.ndim == 4:  # squeeze [N,1,H,W] -> [N,H,W]
                masks_np = masks_np.squeeze(1)
            masks = masks_np.astype(np.uint8)

        return DetectionResult(
            boxes_xyxy=boxes,
            scores=scores,
            labels=labels,
            masks=masks,
            latency_s=latency,
        )


def draw_detections(
    bgr_image: np.ndarray,
    result: DetectionResult,
    score_threshold: float = 0.0,
) -> np.ndarray:
    """Render bounding boxes + labels on a copy of the image."""
    import cv2

    annotated = bgr_image.copy()
    for box, score, label in zip(result.boxes_xyxy, result.scores, result.labels):
        if score < score_threshold:
            continue
        x0, y0, x1, y1 = box.astype(int)
        cv2.rectangle(annotated, (x0, y0), (x1, y1), (0, 200, 255), 2)
        text = f"{label} {score:.2f}"
        ((tw, th), _) = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(
            annotated,
            (x0, y0 - th - 6),
            (x0 + tw + 4, y0),
            (0, 200, 255),
            -1,
        )
        cv2.putText(
            annotated,
            text,
            (x0 + 2, y0 - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
    return annotated
