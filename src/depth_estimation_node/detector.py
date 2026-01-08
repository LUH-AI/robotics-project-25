import torch
import cv2
import numpy as np
from torchvision.transforms import Compose, ToTensor
from PIL import Image
import time
from download import download_model
import matplotlib.pyplot as plt


class DepthEstimator:
    def __init__(
        self,
        model_path,
    ):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")

        self.load_model(model_path, self.device)

    def load_model(self, model_path, device):
        """Load the MiDaS model from the given path."""
        possible_models = [
            "MiDaS",
            "DPT_Large",
            "DPT_Hybrid",
            "MiDaS_small",
        ]
        model_type = possible_models[0]
        model = torch.hub.load("intel-isl/MiDaS", model_type)

        # model.load_state_dict(torch.load(model_path, map_location=device))
        model.to(device)
        model.eval()
        self.model = model

        midas_transform = torch.hub.load("intel-isl/MiDaS", "transforms")
        if model_type in ["DPT_Large", "DPT_Hybrid"]:
            self.transform = midas_transform.dpt_transform
        else:
            self.transform = midas_transform.small_transform

    def estimate(self, image: cv2.Mat) -> np.ndarray:
        """Estimate depth from an image using the MiDaS model."""
        input_batch = self.transform(image).to(self.device)

        with torch.no_grad():
            prediction = self.model(input_batch)

            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=image.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        depth_map = prediction.cpu().numpy()
        return depth_map


def main():
    model_path = "./depth_model.pt"  # MiDaS model file

    # Download MiDaS model if not present
    download_model(
        "https://github.com/intel-isl/MiDaS/releases/download/v2_1/model-f6b98070.pt",
        model_path,
    )

    model = DepthEstimator(model_path)

    start_time = time.time()

    webcam = cv2.VideoCapture(4)
    while True:
        ret, img = webcam.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Convert the image to RGB
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Perform depth estimation
        depth_map = model.estimate(img_rgb)

        # Normalize depth map for visualization
        depth_map_normalized = cv2.normalize(
            depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U
        )

        # Display the original webcam feed
        cv2.imshow("Webcam Feed", img)

        # Display the depth map
        cv2.imshow("Depth Map", depth_map_normalized)

        # Exit the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Release the webcam and close all OpenCV windows
    webcam.release()
    cv2.destroyAllWindows()

    print(f"Time taken: {time.time() - start_time:.2f} seconds")


if __name__ == "__main__":
    main()
