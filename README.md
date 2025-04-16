## Setup
### Ubuntu 22.04
#### Install ROS2 Humble from official site
- https://docs.ros.org/en/humble/Installation.html
#### Build
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
vcs import src <<EOF
repositories:
  aerial_robot_base:
    type: git
    url: 'https://github.com/ut-dragon-lab/aerial_robot_base.git'
    version: master
EOF
# Install depended repositories
vcs import src < src/aerial_robot_base/aerial_robot_${ROS_DISTRO}.repos
rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
colcon build --symlink-install
# Setup pre-commit formatting
cd ~/ros2/aerial_robot_base_ws/src/aerial_robot_base
pre-commit install
```