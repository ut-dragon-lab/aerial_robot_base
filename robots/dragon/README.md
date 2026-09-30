# DRAGON on ROS 2 Humble and Gazebo Harmonic

This package ports the four-link **v1.5 (202601)** DRAGON simulation from
`jsk_aerial_robot/robots/dragon` to `aerial_robot_base`. The original meshes,
masses, inertias, joint angle limits and motor calibration are retained. Simulation
bringup overrides body-joint velocity limits as described below. Six body joints
and eight gimbal joints use separate ROS 2 position controllers.

The model and transformation targets originate from JSK commit
`883009567b503173f90b44f4d79e2df509263c6f`. The ROS 2 implementation reuses the
shared two-axis vectoring PID controller and baselink-attitude navigator;
`dragon_robot_model` checks feasibility with a six-axis vectoring wrench map.
It does not port the legacy NLopt allocation modes, rotor-interference observer,
external-wrench compensation, LQI controller or MCU firmware. Real-machine
bringup is deliberately rejected: this migration is a simulation path.

## Build

Use the installed Gazebo **Harmonic / gz-sim8** and Humble's
`ros-gzharmonic` bridge packages. `aerial_robot_simulation` and the source-built
`gz_ros2_control` must both link to Harmonic. Do not source the earlier local
Fortress overlay (`setup-fortress.bash`).

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2/aerial_robot_base_ws
vcs import src < src/aerial_robot_base/aerial_robot_humble.repos
export GZ_VERSION=harmonic
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
colcon build --symlink-install --parallel-workers 1 --packages-up-to dragon --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

When switching an existing Fortress build, pass `--cmake-clean-cache` when
rebuilding `gz_ros2_control` and `aerial_robot_simulation`.

## Bringup and takeoff

```bash
ros2 launch dragon bringup_launch.py sim:=true real_machine:=false headless:=true
```

The default spawn height is 0.4 m and the takeoff target is 1.5 m at the center
of gravity. `takeoff_height` and `spawn_z` are launch arguments. Wait for
`Ready for Takeoff !!!` and all four controllers to become active.

In a second sourced terminal:

```bash
ros2 run aerial_robot_core keyboard_command.py --ros-args -p robot_ns:=/dragon
```

Press `r` to arm, then `t` to take off. Wait for `Hover !!!` (`flight_state=5`).
`l` lands; `h` immediately stops the simulated motors.

## Transformation

```bash
ros2 run dragon transformation_demo.py --ros-args -p use_sim_time:=true -p mode:=1
```

Original pose modes are retained: `0` seahorse, `1` spiral, `2` M-like.
`reset:=true` returns to the normal pose; `reverse_reset:=true` requests the
reverse normal pose. For example:

```bash
ros2 run dragon transformation_demo.py --ros-args -p use_sim_time:=true -p reset:=true
```

The script requires HOVER and actual joint-state feedback, interpolates body
joints with a smooth cubic trajectory, and returns a nonzero status if HOVER
is lost or joints fail to converge. The default trajectory duration is **5 simulation
seconds**, followed by at least 2 seconds to verify convergence. Override it with
`-p duration:=10.0`; the script no longer silently lengthens this duration.

Simulation bringup sets `body_joint_velocity:=1.0` (rad/s) for the six body
joints. This allows the default mode-0 trajectory's peak speed of 0.936 rad/s.
The standalone URDF retains the original 0.1 rad/s default; to reproduce the
old slow simulation use `body_joint_velocity:=0.1` with `-p duration:=60.0`.
A duration too short for the chosen velocity limit can still cause a convergence
timeout. These are simulation settings, not rated hardware actuator speeds.

The ROS 1 demo publishes a step target to effort-based position servos. This
port uses velocity-based position servos in Gazebo Harmonic, so it keeps a short
smooth trajectory rather than claiming identical ROS 1 actuator dynamics.
Successful script completion confirms measured joint error below 0.03 rad;
flight stability should also be checked from odometry.

## Integration check

With bringup running in another terminal, run:

```bash
python3 src/aerial_robot_base/robots/dragon/test/simulation_smoke.py --output run-dragon/verification
```

The check drives the real keyboard program through a pseudo-terminal: arm,
take off, hold for 30 simulation seconds, transform to mode 0, reset, transform
to mode 1, reset, and land. Every transformation is followed by a 20-second
hold. It checks HOVER continuously, altitude within 0.3 m of 1.5 m, horizontal
position within 0.4 m of the initial CoG, CoG roll/pitch within 0.35 rad, and
final joint error below 0.03 rad, and completion of each default demo within
12 simulation seconds (including process startup and settling). Results and odometry/joint telemetry are saved
as JSON. Mode 2 and reverse reset are available but not covered by this check.

Validated locally on ROS 2 Humble and gz-sim 8.15.0 with the 5-second trajectory
and 1.0 rad/s simulation limit: mode 0, reset, mode 1, and reset each completed
in 8.36–8.41 simulation seconds including process startup and convergence checks.
All four were followed by 20 seconds of stable hover, then landing and ARM_OFF.
CoG altitude during hover/transformation was 1.451–1.559 m; maximum horizontal
displacement was 0.143 m. Results: `run-dragon/speed/trial2/result.json` and
`run-dragon/speed/metrics.json`. The headless test kept at least 12.0 GiB available
RAM and used no swap. These are simulation measurements, not hardware validation.
The PID gains here are tuned for this model and the shared ROS 2 allocator; the
original ROS 1 gains are not interchangeable.

## Interactive visualization

```bash
ros2 launch dragon bringup_launch.py headless:=false
```

This starts both Gazebo and RViz. RViz uses this robot's description topic with
transient-local durability, simulation time, opaque materials, a 30 Hz maximum
frame rate and a camera following the CoG. `rviz:=false` disables only RViz.
Body, gimbals, joints and sensor meshes have explicit matte colors. Lower ambient
light preserves directional shading and shadows without the original white clipping.

The Gazebo GUI has an explicit 60 fps limiter and 2x antialiasing instead of the
upstream 8x default. This limits GPU load independently of physics. It uses the
package's GUI config, without overwriting `~/.gz/sim/8/gui.config`.

On the tested WSLg / Mesa 23.2 laptop, the working configuration is Gazebo's
`ogre` backend on NVIDIA with OpenGL 3.3 / GLSL 330. Advertising OpenGL 4.5
caused poor performance; Intel hardware rendering also produced black output
and a `D3D12: Removing Device` error in RViz before a host freeze. Therefore,
on WSLg RViz defaults to software rendering with two worker threads. Native
Linux retains `ogre2`, hardware RViz rendering and does not override OpenGL versions.
`graphics_adapter` and `render_engine` remain explicit launch options;
`rviz_software_rendering` controls RViz's software fallback separately.

The default `visual_detail:=balanced` uses offline simplified display meshes.
Original CAD meshes are retained and selected with `visual_detail:=full`.
The generated meshes constrain bounding-box differences to 0.5 mm and surface
area differences to 1.5%; these checks do not constitute a pointwise surface-error
bound. Physics geometry, inertia and joint limits are identical between modes.
Generation code and per-mesh metrics are included in `scripts/generate_visual_meshes.py`
and `urdf/v1_5/mesh/visual/metrics.json`; generation dependencies are not needed
to build or run the robot.

Use Release builds for simulation and control performance. The local rebuild
also fixes a strict-aliasing violation in `aerial_robot_nerve`'s inverse-square-root
helper using `memcpy`, preserving its numerical approximation under optimization.

Validated with both windows open: Gazebo sustained about 58 fps (60 fps cap,
stock 1000x845 window), with keyboard arm/takeoff, 10 seconds of hover, landing
and disarm. The bounded 75-second check retained about 12 GB of available WSL
RAM, used no swap, and logged no GPU device removal. RViz software rendering
was around 20 fps in this scene; its 30 Hz setting is an upper limit, not a
promised achieved rate. Tests stop their child processes after completion.
