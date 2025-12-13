# Robotics Project 25


## The new run scripts
* Setup env to run our python scripts: `source ./ros_setup.bash`
* Launch Nav2 mapping: `./start_nav2_mapping.bash` 
* Launch Nav2 nav+mapping: `./start_nav2_navigation_and_mapping.bash` 

### **!!! If they do not work, close the terminal and try again.!!!**
If the wrong packages are sourced in the terminal we will get conflicting dependency versions (especially `rclpy` as stated here <https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html#enabling-rclpy-custom-ros-2-packages-and-workspaces-with-python-3-11>)


### Setup Vscode autocompletions
In `.vscode/settings.json` 
```json 
    "python.analysis.extraPaths": [
        "/home/rlproject25/.local/share/ov/pkg/isaac-sim-2023.1.1-rc.8/exts",
        "/home/rlproject25/.local/share/ov/pkg/isaac-sim-2023.1.1-rc.8/kit/exts",
        "/home/rlproject25/robo_project/project-repo-2.0/deps/rclpy/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages",
    ...
```
--- 
Student Robotics Project Winter 2025
Your idea goes here!


## Resources

- Heinrich documentation and incomplete code: https://github.com/LUH-AI/heinrich_template
- Last year's project: https://github.com/LUH-AI/robotics-project-24
- NVIDIA IsaacLab: https://isaac-sim.github.io/IsaacLab/main/index.html
- Unitree Legged Gym: https://github.com/unitreerobotics/unitree_rl_gym/tree/main





ing publishing.
[navToPose-1] [WARN] [1765564538.812832991] [nav_to_pose]: [NavToPose] cmd_vel not received yet, skipping publishing.
[navToPose-1] [WARN] [1765564538.912846344] [nav_to_pose]: [NavToPose] cmd_vel not received yet, skipping publishing.
[rtabmap-2] [WARN] [1765564538.914955918] [rtabmap]: rtabmap: Did not receive data since 5 seconds! Make sure the input topics are published ("$ ros2 topic hz my_topic") and the timestamps in their header are set. If topics are coming from different computers, make sure the clocks of the computers are synchronized ("ntpdate"). Ajusting topic_queue_size (50) and sync_queue_size (50) can also help for better synchronization if framerates and/or delays are different. If topics are not published at the same rate, you could increase "sync_queue_size" and/or "topic_queue_size" parameters (current=50 and 50 respectively).
[rtabmap-2] rtabmap subscribed to (approx sync):
[rtabmap-2]    /utlidar/robot_odom \
[rtabmap-2]    /utlidar/cloud_deskewed
[navToPose-1] [WARN] [1765564539.012819546] [nav_to_pose]: [NavToPose] cmd_vel not received yet, skipping publishing.






[navToPose-1] [WARN] [1765564715.012086590] [nav_to_pose]: [NavToPose] cmd_vel not received yet, skipping publishing.
[rviz2-4] [INFO] [1765564715.036157056] [rviz2]: Setting goal pose: Frame:base_link, Position(0.328515, -3.44019, 0), Orientation(0, 0, -0.651138, 0.758959) = Angle: -1.41817
[navToPose-1] [WARN] [1765564715.112083750] [nav_to_pose]: [NavToPose] cmd_vel not received yet, skipping publishing.

