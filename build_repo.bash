source /opt/ros/humble/setup.bash
cd ./deps/rclpy/IsaacSim-ros_workspaces/
./build_ros.sh -d humble -v 22.04

echo "Follow the setup instructions in https://github.com/Sayantani-Bhattacharya/unitree_go2_nav closely!"
echo "Additionally remember to create the missing (empty) worlds directory according to the error message"
