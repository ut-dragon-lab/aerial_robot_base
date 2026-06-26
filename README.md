# The DRAGON repository
### Our repository handles the control of aerial robots - especially for transformable aerial robots such as DRAGON, SPIDAR, and Hydrus.
![uav_intro](images/multilink-all.jpg)

## Setup
!!! Warning: You cannot install ROS 1 and ROS 2 on the same machine. Please use a Docker container or remove ROS 1 beforehand.
### Ubuntu 22.04
Clone repository
```bash
git clone https://github.com/ut-dragon-lab/aerial_robot_base
```
Run our prebuild bash script to install ROS 2 Humble and other dependencies
```bash
source configure.sh
```
Create your workspace
```bash
mkdir -p ~/ros2/aerial_robot_base_ws/src
cd ~/ros2/aerial_robot_base_ws
sudo rosdep init
rosdep update
```
Install depended repositories
```bash
vcs import src < src/aerial_robot_base/aerial_robot_${ROS_DISTRO}.repos
rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
```
Build the workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

### Optional MuJoCo runtime
MuJoCo simulation uses the Python `mujoco` package at runtime. Register the workspace rosdep overlay before installing it:
```bash
cd ~/ros2/aerial_robot_base_ws/src/aerial_robot_base
sudo sh -c "echo yaml file://$(pwd)/dependencies/rosdep/mujoco.yaml > /etc/ros/rosdep/sources.list.d/50-aerial-robot-mujoco.list"
rosdep update
AERIAL_ROBOT_WITH_MUJOCO=true rosdep install -y -r --from-paths aerial_robot_simulation --ignore-src --rosdistro ${ROS_DISTRO} --as-root pip:false
```
If your Python environment is managed separately, the equivalent package is `mujoco`.

Setup pre-commit formatting
```bash
cd ~/ros2/aerial_robot_base_ws/src/aerial_robot_base
pre-commit install
```

For convenience, add the following line to your `~/.bashrc` file
```bash
echo "source ~/ros2/aerial_robot_base_ws/install/setup.bash" >> ~/.bashrc
```

## Build firmware
Please refer to this [repository](https://github.com/ut-dragon-lab/aerial_robot_nerve#) for instructions on the build procedure.

NOTE: Building `micro_ros_agent` for the first time throws an error about not finding the `FindTinyXML2` package. This is a known issue and can be ignored. The build will succeed after the first attempt.

## Docker
For using Docker, here is a convenient Dockerfile provided [https://github.com/johanneskbl/ros2_docker](https://github.com/johanneskbl/ros2_docker).

To authenticate the Docker user on your local machine to access the X (Display) server, run
```bash
xhost +local:root
```

## Run
To run simulation and real-machine, please check the instructions in our [wiki](https://github.com/ut-dragon-lab/aerial_robot_base/wiki).
