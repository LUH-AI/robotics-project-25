## Setting up the environment

### Installing ROS via the [Jazzy documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

Follow the installation instructions. Please use the Ubunutu installation. If your working on Microsoft use WSL.


verify you installed ROS globally:

```
echo $ROS_DISTRO
which ros2
```
It should output something like:
```
jazzy
/opt/ros/jazzy/bin/ros2
```

### Sourcing and Building ROS
We want to have ROS available in each Terminal. We therefore need to build it to later source it in each terminal.

Build ROS with colcon
```
colcon build
```
If you don't have colcon install it with 
```
sudo apt get colcon
```


We can do that automatically by adding this at the bottom of our `./bashrc` in the home directory

```
source /opt/ros/jazzy/setup.bash
source ~/<your-project-name>/install/setup.bash
```



### Project Structure

Create Project folder

```
mkdir -p ~/<your-project-name>/src
```

Clone Repo into `/src`:
```
cd ~/<your-project-name>/src

git clone https://github.com/LUH-AI/robotics-project-25.git
```

### Running Isaac Gym in a Container

Isaac Gym is deprecated and can only use old Python versions. To still use it we will containerize it using NVIDIA Container Toolkit