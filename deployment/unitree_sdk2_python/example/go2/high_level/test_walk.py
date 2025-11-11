import pygame as pg
import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
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

    client = SportClient()
    client.SetTimeout(10.0)
    client.Init()

    pg.init()
    screen = pg.display.set_mode((640, 480))
    pg.display.set_caption("Go2 Sport Mode Test Client")
    clock = pg.time.Clock()

    while True:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                sys.exit()

        screen.fill((255, 255, 255))
        pg.display.flip()

        # read wasd input 
        keys = pg.key.get_pressed()
        if keys[pg.K_w]:
            # client.StopMove()
            ret = client.Move(0.3, 0.0, 0.0)
            # time.sleep(1)

            print(f"Move FW: {ret}")
        elif keys[pg.K_s]:    
            print("Move Backward")
            client.Move(-0.3, 0.0, 0.0)
        elif keys[pg.K_a]: 
            client.Move(0.0, 0.3, 0.0)
        elif keys[pg.K_d]: 
            client.Move(0.0, -0.3, 0.0)

        elif keys[pg.K_e]:
            print("rotate Left")
            client.Move(0.0, 0.0, -0.75)
        elif keys[pg.K_q]:
            print("rotate Right")
            client.Move(0.0, 0.0, 0.75)
        else: 
            client.StopMove()

        clock.tick(60)