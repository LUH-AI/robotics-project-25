import numpy as np
from dataclasses import dataclass
from typing import Any, List, Dict, Optional, Union, Tuple
import torch
from PIL import Image
from transformers import AutoModelForMaskGeneration, AutoProcessor, pipeline
import requests
import time

import cv2

# Reuse your existing lightweight HTTP wrapper utils
from server_wrapper import ServerMixin, host_model, send_request, str_to_image

@dataclass
class BoundingBox:
    xmin: int
    ymin: int
    xmax: int
    ymax: int

    @property
    def xyxy(self) -> List[float]:
        return [self.xmin, self.ymin, self.xmax, self.ymax]

@dataclass
class DetectionResult:
    score: float
    label: str
    box: BoundingBox
    mask: Optional[np.array] = None

    @classmethod
    def from_dict(cls, detection_dict: Dict) -> 'DetectionResult':
        return cls(score=detection_dict['score'],
                   label=detection_dict['label'],
                   box=BoundingBox(xmin=detection_dict['box']['xmin'],
                                   ymin=detection_dict['box']['ymin'],
                                   xmax=detection_dict['box']['xmax'],
                                   ymax=detection_dict['box']['ymax']))


class GroundingSAM:

    def __init__(
            self,
            detector_id: Optional[str] = "IDEA-Research/grounding-dino-tiny",
            segmenter_id: Optional[str] = "facebook/sam-vit-base",
            detector_threshold: float =0.3,
            polygon_refinement: bool = False,
        ):

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using device: {self.device}")
        self.object_detector = pipeline(model=detector_id, task="zero-shot-object-detection", device=self.device)
        self.segmentator = AutoModelForMaskGeneration.from_pretrained(segmenter_id).to(self.device)
        self.processor = AutoProcessor.from_pretrained(segmenter_id, use_fast=True)

        self.detector_threshold = detector_threshold
        self.polygon_refinement = polygon_refinement


    def mask_to_polygon(self, mask: np.ndarray) -> List[List[int]]:
        # Find contours in the binary mask
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the contour with the largest area
        largest_contour = max(contours, key=cv2.contourArea)

        # Extract the vertices of the contour
        polygon = largest_contour.reshape(-1, 2).tolist()

        return polygon

    def polygon_to_mask(self, polygon: List[Tuple[int, int]], image_shape: Tuple[int, int]) -> np.ndarray:
        """
        Convert a polygon to a segmentation mask.

        Args:
        - polygon (list): List of (x, y) coordinates representing the vertices of the polygon.
        - image_shape (tuple): Shape of the image (height, width) for the mask.

        Returns:
        - np.ndarray: Segmentation mask with the polygon filled.
        """
        # Create an empty mask
        mask = np.zeros(image_shape, dtype=np.uint8)

        # Convert polygon to an array of points
        pts = np.array(polygon, dtype=np.int32)

        # Fill the polygon with white color (255)
        cv2.fillPoly(mask, [pts], color=(255,))

        return mask

    def get_boxes(self, results: DetectionResult) -> List[List[List[float]]]:
        boxes = []
        for result in results:
            xyxy = result.box.xyxy
            boxes.append(xyxy)

        return [boxes]

    def refine_masks(self, masks: torch.BoolTensor) -> List[np.ndarray]:
        masks = masks.cpu().float()
        masks = masks.permute(0, 2, 3, 1)
        masks = masks.mean(axis=-1)
        masks = (masks > 0).int()
        masks = masks.numpy().astype(np.uint8)
        masks = list(masks)

        if self.polygon_refinement:
            for idx, mask in enumerate(masks):
                shape = mask.shape
                polygon = self.mask_to_polygon(mask)
                mask = self.polygon_to_mask(polygon, shape)
                masks[idx] = mask

        return masks

    def detect(
        self,
        image: Image.Image,
        labels: List[str],
    ) -> List[Dict[str, Any]]:

        """
        Use Grounding DINO to detect a set of labels in an image in a zero-shot fashion.
        """
        # device = "cuda" if torch.cuda.is_available() else "cpu"
        # detector_id = detector_id if detector_id is not None else "IDEA-Research/grounding-dino-tiny"
        # object_detector = pipeline(model=detector_id, task="zero-shot-object-detection", device=device)

        labels = [label if label.endswith(".") else label+"." for label in labels]

        results = self.object_detector(image,  candidate_labels=labels, threshold=self.detector_threshold)
        results = [DetectionResult.from_dict(result) for result in results]

        return results

    def segment(
        self,
        image: Image.Image,
        detection_results: List[Dict[str, Any]],
    ) -> List[DetectionResult]:
        """
        Use Segment Anything (SAM) to generate masks given an image + a set of bounding boxes.
        """

        # device = "cuda" if torch.cuda.is_available() else "cpu"
        # segmenter_id = segmenter_id if segmenter_id is not None else "facebook/sam-vit-base"

        # segmentator = AutoModelForMaskGeneration.from_pretrained(segmenter_id).to(device)
        # processor = AutoProcessor.from_pretrained(segmenter_id)

        boxes = self.get_boxes(detection_results)

        # Critical guard - if no boxes, skip segmentation
        if len(detection_results) == 0:
            return None

        else:
            inputs = self.processor(images=image, input_boxes=boxes, return_tensors="pt").to(self.device)

            outputs = self.segmentator(**inputs)

            masks = self.processor.post_process_masks(
                masks=outputs.pred_masks,
                original_sizes=inputs.original_sizes,
                reshaped_input_sizes=inputs.reshaped_input_sizes
            )[0]

            masks = self.refine_masks(masks)

            for detection_result, mask in zip(detection_results, masks):
                detection_result.mask = mask

            return detection_results

    def grounded_segmentation(
        self,
        image: Union[Image.Image],
        labels: List[str],
    ) -> Tuple[np.ndarray, List[DetectionResult]]:

        with torch.no_grad():
            detections = self.detect(image, labels)
            detections = self.segment(image, detections)

            return np.array(image), detections


def gd_sam_annotate(image: Union[Image.Image, np.ndarray], detection_results: List[DetectionResult]) -> np.ndarray:
    # Convert PIL Image to OpenCV format
    image_cv2 = np.array(image) if isinstance(image, Image.Image) else image
    image_cv2 = cv2.cvtColor(image_cv2, cv2.COLOR_RGB2BGR)

    # Iterate over detections and add bounding boxes and masks
    for detection in detection_results:
        label = detection.label
        score = detection.score
        box = detection.box
        mask = detection.mask

        # Sample a random color for each detection
        color = np.random.randint(0, 256, size=3)

        # Draw bounding box
        cv2.rectangle(image_cv2, (box.xmin, box.ymin), (box.xmax, box.ymax), color.tolist(), 2)
        cv2.putText(image_cv2, f'{label}: {score:.2f}', (box.xmin, box.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color.tolist(), 2)

        # If mask is available, apply it
        if mask is not None:
            # Convert mask to uint8
            mask_uint8 = (mask * 255).astype(np.uint8)
            contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image_cv2, contours, -1, color.tolist(), 2)

    return cv2.cvtColor(image_cv2, cv2.COLOR_BGR2RGB)


if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=12185)
    args = parser.parse_args()

    class GroundingSAMServer(ServerMixin, GroundingSAM):
        def __init__(self, **kwargs):
            GroundingSAM.__init__(self, **kwargs)
            ServerMixin.__init__(self)

        def process_payload(self, payload):
            img = str_to_image(payload["image"])
            img = Image.fromarray(img)
            target_object = [payload["target_prompt"]]
            _, detections = self.grounded_segmentation(img, target_object)

            # Debug detections
            if len(detections) > 0: 
                response ={}
                response["labels"] = []
                response["scores"] = []
                response["boxes"] = []
                for detection in detections:

                    response["labels"].append(detection.label)
                    response["scores"].append(detection.score)
                    response["boxes"].append(detection.box.xyxy)

            print(len(detections))
            print(response)

            return {"response": response}

    server = GroundingSAMServer()
    print("Model loaded!")
    print(f"Hosting on http://localhost:{args.port}")
    host_model(server, "gdsam", port=args.port)

    
    # GROUNDING SAM TEST

    # image_url = "http://images.cocodataset.org/val2017/000000039769.jpg"
    # labels = ["a cat.", "a remote control."]
    # threshold = 0.3

    # detector_id = "IDEA-Research/grounding-dino-tiny"
    # segmenter_id = "facebook/sam-vit-base"

    # image = Image.open(requests.get(image_url, stream=True).raw).convert("RGB")

    # grounding_sam = GroundingSAM()

    # start = time.time()

    # image, detections = grounding_sam.grounded_segmentation(image, labels)

    # end = time.time()

    # print(f"Time taken: {end - start:.2f} seconds")
    # print(f"Found {len(detections)} objects.")
