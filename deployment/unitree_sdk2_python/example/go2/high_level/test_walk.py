import pygame as pg, numpy as np, cv2
import torch, pandas as pd
import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.video.video_client import VideoClient

from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
from unitree_sdk2py.go2.vui.vui_client import VuiClient

from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)


import math
from dataclasses import dataclass
from ultralytics import YOLO

if __name__ == "__main__":
    model = YOLO("yolov8n.pt")
    # model = torch.load()
    # model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)


    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    video_client = VideoClient()  # Create a video client
    video_client.SetTimeout(3.0)
    video_client.Init()

    # ?? Doesnt work
    # obstacle_avoid = ObstaclesAvoidClient()
    # obstacle_avoid.Init()
    # obstacle_avoid.SwitchSet(False)
    # obstacle_avoid.SetTimeout(10.0)

    vui_client = VuiClient()
    vui_client.Init()
    vui_client.SetTimeout(5.0)
    vui_client.SetVolume(3)
    

    code, data = video_client.GetImageSample()
    image_data = np.frombuffer(bytes(data), dtype=np.uint8)
    image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


    sports_client = SportClient()
    sports_client.SetTimeout(10.0)
    sports_client.Init()



    pg.init()
    screen = pg.display.set_mode(image.shape[1::-1])
    pg.display.set_caption("Go2 Sport Mode Test Client")
    clock = pg.time.Clock()

    while True:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                sys.exit()

        code, data = video_client.GetImageSample()
        image_data = np.frombuffer(bytes(data), dtype=np.uint8)
        image_dec = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        if image_dec is None: 
            print("Image decode error!")
            continue

        results = model(image_dec, verbose=False)

        image = cv2.cvtColor(np.squeeze(results[0].plot()), cv2.COLOR_BGR2RGB)

        if image is None: 
            print("Image convert error!")
            continue

        pg_img = pg.image.frombuffer(image.tobytes(), image.shape[1::-1], "RGB")

        
        screen.fill((255, 255, 255))
        screen.blit(pg_img, (0,0))


        pg.display.flip()

        # read wasd input 
        keys = pg.key.get_pressed()
        if keys[pg.K_LCTRL] and keys[pg.K_q]: 
            exit(0) 


        if keys[pg.K_w]:
            sports_client.Move(0.3, 0.0, 0.0)
        elif keys[pg.K_s]:    
            sports_client.Move(-0.3, 0.0, 0.0)
        elif keys[pg.K_a]: 
            sports_client.Move(0.0, 0.3, 0.0)
        elif keys[pg.K_d]: 
            sports_client.Move(0.0, -0.3, 0.0)
        elif keys[pg.K_e]:
            sports_client.Move(0.0, 0.0, -0.75)
        elif keys[pg.K_q]:
            sports_client.Move(0.0, 0.0, 0.75)
        else: 
            sports_client.StopMove()
            

        clock.tick(60)