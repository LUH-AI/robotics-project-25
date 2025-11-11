import torch

# from unitree_sdk2py.go2.video.video_client import VideoClient
# from unitree_sdk2py.core.channel import ChannelFactoryInitialize
import cv2
import numpy as np
import sys
import pandas as pd
from ultralytics import YOLO

model = YOLO("yolov10n.pt")  

# if len(sys.argv)>1:
#     ChannelFactoryInitialize(0, sys.argv[1])
# else:
#     ChannelFactoryInitialize(0)
    
# client = VideoClient()  # Create a video client
# client.SetTimeout(3.0)
# client.Init()

# code, data = client.GetImageSample()

vc = cv2.VideoCapture(0)

while True:
        # Get Image data from Go2 robot
        # code, data = client.GetImageSample()

        ret, frame = vc.read()
        if not ret:
            print("Failed to grab frame")
            continue

        # Convert to numpy image
        # data = cv2.imencode('.jpg', frame)[1].tobytes()
        # image_data = np.frombuffer(bytes(data), dtype=np.uint8)
        # image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        results = model(frame, verbose=False)
        
        for i in results: 
            print(i.names)
            print(i.boxes)

        print("")
        print("")
        cv2.imshow("front_camera", np.squeeze(results[0].plot()))
        # Press ESC to stop
        if cv2.waitKey(20) == 27:
            break  