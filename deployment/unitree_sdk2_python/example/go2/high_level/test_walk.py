import pygame as pg, numpy as np, cv2
import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.video.video_client import VideoClient

from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
import math
from dataclasses import dataclass




if __name__ == "__main__":


    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    video_client = VideoClient()  # Create a video client
    video_client.SetTimeout(3.0)
    video_client.Init()


    code, data = video_client.GetImageSample()
    image_data = np.frombuffer(bytes(data), dtype=np.uint8)
    image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


    client = SportClient()
    client.SetTimeout(10.0)
    client.Init()



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
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pg_img = pg.image.frombuffer(image.tobytes(), image.shape[1::-1], "RGB")

        
        screen.fill((255, 255, 255))
        screen.blit(pg_img, (0,0))


        pg.display.flip()

        # read wasd input 
        keys = pg.key.get_pressed()
        if keys[pg.K_LCTRL] and keys[pg.K_q]: 
            exit(0) 


        if keys[pg.K_w]:
            ret = client.Move(0.3, 0.0, 0.0)
        elif keys[pg.K_s]:    
            client.Move(-0.3, 0.0, 0.0)
        elif keys[pg.K_a]: 
            client.Move(0.0, 0.3, 0.0)
        elif keys[pg.K_d]: 
            client.Move(0.0, -0.3, 0.0)
        elif keys[pg.K_e]:
            client.Move(0.0, 0.0, -0.75)
        elif keys[pg.K_q]:
            client.Move(0.0, 0.0, 0.75)
        else: 
            client.StopMove()

        clock.tick(60)