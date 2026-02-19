## Setup
### Ubuntu 22.04
#### Install ROS2 Humble from official site
- https://docs.ros.org/en/humble/Installation.html
#### Build ROS system
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo apt update
# Install Python tools
sudo apt install -y python3-vcstool python3-colcon-common-extensions
# Install format tools
sudo apt-get install clang-format
pip install pre-commit
# Create workspace
mkdir -p ~/ros2/aerial_robot_base_ws/src
cd ~/ros2/aerial_robot_base_ws
sudo rosdep init
rosdep update
# install repository
vcs import src --input https://raw.githubusercontent.com/ut-dragon-lab/aerial_robot_base/master/aerial_robot_base.repos
# Install depended repositories
vcs import src < src/aerial_robot_base/aerial_robot_${ROS_DISTRO}.repos
rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
colcon build --symlink-install
# Setup pre-commit formatting
cd ~/ros2/aerial_robot_base_ws/src/aerial_robot_base
pre-commit install
```
#### Build firmware
Please refer [here](https://github.com/ut-dragon-lab/aerial_robot_nerve#) for the build procedure.