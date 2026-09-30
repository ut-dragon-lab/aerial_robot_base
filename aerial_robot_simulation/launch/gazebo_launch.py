#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, Shutdown
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
    EnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from ament_index_python.packages import get_package_share_directory

# ---------------------------------------------------------------------------
# Argument declarations  (name, default, description, (optional) choices)
# ---------------------------------------------------------------------------
pkg_share = get_package_share_directory("aerial_robot_simulation")
default_world = os.path.join(pkg_share, "gazebo_model", "world", "empty.world")
is_wslg = os.environ.get("WSL2_GUI_APPS_ENABLED") == "1"
default_qt_qpa_platform = os.environ.get("QT_QPA_PLATFORM", "xcb" if is_wslg else "")
default_mesa_gl_version_override = os.environ.get("MESA_GL_VERSION_OVERRIDE", "3.3" if is_wslg else "")
default_mesa_glsl_version_override = os.environ.get("MESA_GLSL_VERSION_OVERRIDE", "330" if is_wslg else "")
_ARGS = [
    ("robot_ns", "hydrus", "Namespace for all robot nodes"),
    ("robot_model", LaunchConfiguration("robot_ns"), "ROS package containing the robot resources"),
    ("world_sdf_file", default_world, "Gazebo world SDF file path (default: empty world included in this package)"),
    ("headless", "true", "Run without GUI", ["true", "false"]),
    ("graphics_adapter", os.environ.get("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA" if is_wslg else ""),
     "Optional WSLg adapter substring (e.g. Intel or NVIDIA); empty uses Mesa default"),
    ("render_engine", "ogre" if is_wslg else "ogre2", "Gazebo render engine", ["ogre2", "ogre"]),
    ("qt_qpa_platform", default_qt_qpa_platform, "Qt platform plugin used by Gazebo GUI"),
    (
        "mesa_gl_version_override",
        default_mesa_gl_version_override,
        "MESA_GL_VERSION_OVERRIDE for Gazebo; empty leaves it unset",
    ),
    (
        "mesa_glsl_version_override",
        default_mesa_glsl_version_override,
        "MESA_GLSL_VERSION_OVERRIDE for Gazebo; empty leaves it unset",
    ),
    ("sim_param_path", "", "Path to YAML file with simulation parameters"),
    ("spawn_x", "0.0", "Gazebo spawn X position [m] (sim only)"),
    ("spawn_y", "0.0", "Gazebo spawn Y position [m] (sim only)"),
    ("spawn_z", "0.5", "Gazebo spawn Z position [m] (sim only)"),
]


def generate_launch_description():
    # ------------------------------------------------------------------
    # 1.  Declare CLI-overridable arguments
    # ------------------------------------------------------------------
    declared_args = [
        DeclareLaunchArgument(
            name, default_value=default_value, description=description, **({"choices": choices[0]} if choices else {})
        )
        for name, default_value, description, *choices in _ARGS
    ]

    # Resolve / Read at launch time (NOT AT IMPORT TIME)
    robot_ns = LaunchConfiguration("robot_ns")
    world = LaunchConfiguration("world_sdf_file")
    headless = LaunchConfiguration("headless")
    render_engine = LaunchConfiguration("render_engine")
    graphics_adapter = LaunchConfiguration("graphics_adapter")
    qt_qpa_platform = LaunchConfiguration("qt_qpa_platform")
    mesa_gl_version_override = LaunchConfiguration("mesa_gl_version_override")
    mesa_glsl_version_override = LaunchConfiguration("mesa_glsl_version_override")
    sim_param_path = LaunchConfiguration("sim_param_path")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")

    # ------------------------------------------------------------------
    # 2.  Derived paths & environment variables
    # ------------------------------------------------------------------
    robot_share_dir = PathJoinSubstitution([FindPackagePrefix(LaunchConfiguration("robot_model")), "share"])

    # Pass through DISPLAY env var for Gazebo GUI (if not headless)
    set_env = SetEnvironmentVariable(name="DISPLAY", value=os.environ.get("DISPLAY", ""))
    set_xauthority = SetEnvironmentVariable(
        name="XAUTHORITY", value=os.environ.get("XAUTHORITY", os.path.expanduser("~/.Xauthority"))
    )

    set_qt_qpa_platform = SetEnvironmentVariable(
        name="QT_QPA_PLATFORM",
        value=qt_qpa_platform,
        condition=IfCondition(PythonExpression(["'", headless, "' == 'false' and len('", qt_qpa_platform, "') > 0"])),
    )

    set_mesa_d3d12_adapter = SetEnvironmentVariable(
        name="MESA_D3D12_DEFAULT_ADAPTER_NAME",
        value=graphics_adapter,
        condition=IfCondition(PythonExpression(["len('", graphics_adapter, "') > 0"])),
    )

    set_mesa_gl_version_override = SetEnvironmentVariable(
        name="MESA_GL_VERSION_OVERRIDE",
        value=mesa_gl_version_override,
        condition=IfCondition(PythonExpression(["len('", mesa_gl_version_override, "') > 0"])),
    )

    set_mesa_glsl_version_override = SetEnvironmentVariable(
        name="MESA_GLSL_VERSION_OVERRIDE",
        value=mesa_glsl_version_override,
        condition=IfCondition(PythonExpression(["len('", mesa_glsl_version_override, "') > 0"])),
    )

    # Gazebo looks for models, worlds, and plugins in GZ_SIM_RESOURCE_PATH
    set_gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            TextSubstitution(text=":"),
            robot_share_dir,
        ],
    )

    set_gazebo_default_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            PathJoinSubstitution([FindPackagePrefix("gz_ros2_control"), "lib"]),
            TextSubstitution(text=os.pathsep),
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )

    set_fastrtps_profile = SetEnvironmentVariable(
        name="FASTRTPS_DEFAULT_PROFILES_FILE", value=os.path.join(pkg_share, "config", "fastrtps_profiles.xml")
    )

    # ------------------------------------------------------------------
    # 3.  Execute Gazebo processes
    # ------------------------------------------------------------------
    gz_server = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "--render-engine",
            render_engine,
            "-r",
            "-s",
            world,
        ],
        output="screen",
    )

    set_gui_plugin_path = SetEnvironmentVariable(
        name="GZ_GUI_PLUGIN_PATH",
        value=[PathJoinSubstitution([FindPackagePrefix("aerial_robot_simulation"), "lib"]),
               TextSubstitution(text=os.pathsep),
               EnvironmentVariable("GZ_GUI_PLUGIN_PATH", default_value="")],
    )

    gz_client = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-g",
            "--gui-config",
            os.path.join(pkg_share, "config", "gazebo_gui.config"),
            "--render-engine",
            render_engine,
        ],
        condition=UnlessCondition(headless),
        output="screen",
    )

    shutdown_handler = RegisterEventHandler(OnProcessExit(target_action=gz_server, on_exit=[Shutdown()]))

    # ------------------------------------------------------------------
    # 3.  Nodes
    # ------------------------------------------------------------------
    sim_param_server = Node(
        package="aerial_robot_simulation",
        executable="sim_param_server",
        name="sim_param_server",
        namespace=robot_ns,
        parameters=[sim_param_path],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            robot_ns,
            "-topic",
            PythonExpression(["'/' + '", robot_ns, "' + '/robot_description'"]),
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
        ],
        output="screen",
    )

    att_controller_spawner = Node(
        namespace=robot_ns,
        package="controller_manager",
        executable="spawner",
        name="spawn_att_controller",
        output="screen",
        arguments=["attitude_controller", "--param-file", sim_param_path],
        parameters=[{"use_sim_time": True}],
    )

    joint_state_broadcaster_spawner = Node(
        namespace=robot_ns,
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": True}],
    )

    ld = LaunchDescription()

    for arg in declared_args:
        ld.add_action(arg)

    ld.add_action(set_env)
    ld.add_action(set_xauthority)
    ld.add_action(set_qt_qpa_platform)
    ld.add_action(set_mesa_d3d12_adapter)
    ld.add_action(set_mesa_gl_version_override)
    ld.add_action(set_mesa_glsl_version_override)
    ld.add_action(set_gazebo_resource_path)
    ld.add_action(set_gazebo_default_path)
    ld.add_action(set_gui_plugin_path)
    ld.add_action(set_fastrtps_profile)
    ld.add_action(shutdown_handler)
    ld.add_action(sim_param_server)
    ld.add_action(gz_server)
    ld.add_action(gz_client)
    ld.add_action(clock_bridge)
    ld.add_action(spawn_robot)
    ld.add_action(att_controller_spawner)
    ld.add_action(joint_state_broadcaster_spawner)

    return ld
