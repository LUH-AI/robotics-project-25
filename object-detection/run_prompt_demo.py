"""Quick local sanity check for the Grounded-SAM-2 detector.

Example:
    python3 object-detection/run_prompt_demo.py \
        --prompt "apple pie" \
        --image assets/2025_apple-pie.jpg
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

# Make sure the helper module is importable when run from repo root.
REPO_ROOT = Path(__file__).resolve().parents[1]
GROUNDED_ROOT = REPO_ROOT / "object-detection" / "Grounded-SAM-2"
if str(REPO_ROOT / "object-detection") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "object-detection"))
if str(GROUNDED_ROOT) not in sys.path:
    sys.path.insert(0, str(GROUNDED_ROOT))

from go2_grounded_sam2 import GroundedSAM2Detector, draw_detections


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a one-off Grounded-SAM-2 detection on an image.")
    parser.add_argument(
        "--prompt",
        default="apple pie",
        help="Text prompt to ground objects with.",
    )
    parser.add_argument(
        "--image",
        default=str(REPO_ROOT / "assets/2025_apple-pie.jpg"),
        help="Path to the input image.",
    )
    parser.add_argument(
        "--output",
        default=str(REPO_ROOT / "outputs/object_detection/apple_pie_detection.jpg"),
        help="Where to save the annotated result.",
    )
    parser.add_argument(
        "--device",
        default=None,
        help="Force device (cuda/cpu). Default: auto-detect.",
    )
    parser.add_argument("--box-threshold", type=float, default=0.3)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument(
        "--enable-masks",
        action="store_true",
        help="Enable SAM2 mask prediction (requires downloaded checkpoints).",
    )
    parser.add_argument(
        "--sam2-checkpoint",
        default=str(REPO_ROOT / "object-detection/Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt"),
    )
    parser.add_argument(
        "--sam2-config",
        default=str(GROUNDED_ROOT / "configs/sam2.1/sam2.1_hiera_l.yaml"),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    img_path = Path(args.image)
    if not img_path.exists():
        raise FileNotFoundError(f"Image not found: {img_path}")
    image = cv2.imread(str(img_path))
    if image is None:
        raise RuntimeError(f"Failed to read image: {img_path}")

    detector = GroundedSAM2Detector(
        device=args.device,
        box_threshold=args.box_threshold,
        text_threshold=args.text_threshold,
        enable_masks=args.enable_masks,
        sam2_checkpoint=args.sam2_checkpoint,
        sam2_config=args.sam2_config,
    )
    result = detector.detect(image, args.prompt)
    annotated = draw_detections(image, result)

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), annotated)
    print(
        f"Saved annotated result to {out_path} "
        f"({len(result.boxes_xyxy)} detections, {result.latency_s*1000:.1f} ms)."
    )
    for label, score in zip(result.labels, result.scores):
        print(f" - {label}: {score:.3f}")


if __name__ == "__main__":
    main()
