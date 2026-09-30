# The DRAGON repository
### Our repository handles the control of aerial robots - especially for transformable aerial robots such as DRAGON, SPIDAR, and Hydrus.
![uav_intro](images/multilink-all.jpg)

## Setup
### Ubuntu 22.04
#### Install ROS2 Humble from official site
- https://docs.ros.org/en/humble/Installation.html
#### Install dependent tools
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo apt update
# install python tools
sudo apt install -y python3-vcstool python3-colcon-common-extensions python3-colcon-clean gdb clang-format
# install format tools
pip install pre-commit black
```
#### Create your workspace
```bash
mkdir -p ~/ros2/aerial_robot_base_ws/src
cd ~/ros2/aerial_robot_base_ws
sudo rosdep init
rosdep update
```
#### Install repositories
```bash
vcs import src --input https://raw.githubusercontent.com/ut-dragon-lab/aerial_robot_base/master/aerial_robot_base.repos
vcs import src < src/aerial_robot_base/aerial_robot_${ROS_DISTRO}.repos
export GZ_VERSION=harmonic
# ros_gz_sim / ros_gz_bridge are supplied by the installed ros-gzharmonic packages.
rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
  --skip-keys="ros_gz_sim ros_gz_bridge"
```
The simulation uses the installed Gazebo Harmonic (`gz-sim8`) and the Humble
`ros-gzharmonic` packages. Build `gz_ros2_control` from the imported source
against the same Gazebo version.

Build the workspace
```bash
export GZ_VERSION=harmonic
colcon build --symlink-install --packages-skip gz_ros2_control_demos ign_ros2_control_demos gz_ros2_control_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Ubuntu 22.04 on VIM4 (arm64)
On VIM4, install ROS 2 Humble runtime packages manually and skip Gazebo / `ros_gz` dependencies during `rosdep`.

#### Install repositories
```bash
cd ~/ros2/aerial_robot_base_ws
vcs import src --input https://raw.githubusercontent.com/ut-dragon-lab/aerial_robot_base/master/aerial_robot_base.repos
vcs import src < src/aerial_robot_base/aerial_robot_${ROS_DISTRO}.repos
```

#### Install dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys="gazebo_ros_pkgs ros_gz_sim gz_ros2_control ignition-gazebo6 ros_gz_bridge"
```

#### Build the workspace
```bash
unset MAKEFLAGS
unset CMAKE_BUILD_PARALLEL_LEVEL
export MAKEFLAGS=-j2
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --executor sequential --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  --symlink-install
source install/setup.bash
```

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
For the ROS 2 DRAGON simulation, takeoff and transformation demo, see
[robots/dragon/README.md](robots/dragon/README.md).

To run simulation and real-machine, please check the instructions in our [wiki](https://github.com/ut-dragon-lab/aerial_robot_base/wiki).
