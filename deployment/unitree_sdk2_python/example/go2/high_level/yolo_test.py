import torch

from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
import cv2
import numpy as np
import sys
import pandas as pd


model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
if len(sys.argv)>1:
    ChannelFactoryInitialize(0, sys.argv[1])
else:
    ChannelFactoryInitialize(0)
    
client = VideoClient()  # Create a video client
client.SetTimeout(3.0)
client.Init()

code, data = client.GetImageSample()


while code == 0:
        # Get Image data from Go2 robot
        code, data = client.GetImageSample()

        # Convert to numpy image
        image_data = np.frombuffer(bytes(data), dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        results = model(image)
        cv2.imshow("front_camera", np.squeeze(results.render()))
        # Press ESC to stop
        if cv2.waitKey(20) == 27:
            break  