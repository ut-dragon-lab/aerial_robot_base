## Setup
### Ubuntu 22.04
#### Install ROS2 Humble from official site
- https://docs.ros.org/en/humble/Installation.html
#### Build
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo apt install -y python3-vcstool python3-colcon-common-extensions
mkdir -p ~/ros2/aerial_robot_2_ws/src
cd ~/ros2/aerial_robot_2_ws
sudo rosdep init
rosdep update
# install repository
vcs import src <<EOF
repositories:
  aerial_robot_2:
    type: git
    url: 'https://github.com/sugihara-16/aerial_robot_2.git'
    version: master
EOF
# install depended repositories
vcs import src < src/aerial_robot_2/aerial_robot_${ROS_DISTRO}.repos
rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
colcon build --symlink-install
```