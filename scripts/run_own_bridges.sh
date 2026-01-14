#!/bin/bash

source /opt/ros/humble/setup.bash

# $(
#     source /home/rlproject25/ros_py311/install/setup.bash

#     cd /home/rlproject25/Desktop/OPUS_MAX/ISAAC-EXP/src/depth_estimation_node
#     uv run rgb_to_depth_node.py 
# ) &

$(
    cd /home/rlproject25/Desktop/OPUS_MAX/ISAAC-EXP/src/cmd_vel_to_unitree_node
    uv pip install -e /home/rlproject25/robo_project/test_some_stuff/unitree_sdk2_python/
    uv run cmd_vel_to_unitree.py
) &

$(
    cd /home/rlproject25/Desktop/OPUS_MAX/ISAAC-EXP/src/publish_unitree_image_node
    uv pip install -e /home/rlproject25/robo_project/test_some_stuff/unitree_sdk2_python/
    uv run pub_rgb_and_depth_image.py
)