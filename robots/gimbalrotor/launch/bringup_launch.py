#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
    Command,
    FindExecutable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# ---------------------------------------------------------------------------
# Argument declarations  (name, default, description, (optional) choices)
# ---------------------------------------------------------------------------
# fmt: off
_ARGS = [
    ("robot_model",         "gimbalrotor",      "Name of the robot model ROS package"),
    ("robot_ns",            "gimbalrotor",      "Namespace for all robot nodes"),
    ("airframe",            "quad",             "Gimbalrotor airframe variant", ["quad", "tri", "bi", "tri_omni"]),
    ("real_machine",        "true",             "Use real machine specific bring-up inside model_launch", ["true", "false"]),
    ("main_rate",           "40.0",             "Core node main loop rate [Hz]"),
    ("estimation_mode",     "0",                "Estimator mode on real machine: 0=egomotion, 1=experiment, 2=ground-truth", ["0", "1", "2"]),
    ("model_options",       "",                 "Extra xacro arguments passed verbatim to xacro"),
    ("headless",            "true",             "Run without GUI", ["true", "false"]),
    ("sim",                 "false",            "Launch Gazebo simulation", ["true", "false"]),
    ("sim_estimation_mode", "2",                "Estimator mode in simulation: 0=egomotion, 1=experiment, 2=ground-truth", ["0", "1", "2"]),
    ("launch_spinal",       "true",             "Launch micro-ROS Agent for spinal on real machine", ["true", "false"]),
    ("launch_spinal_bridge", "true",            "Relay root spinal topics to/from robot namespace", ["true", "false"]),
    ("spinal_dev",          "/dev/flight_controller", "Serial device for spinal micro-ROS Agent"),
    ("spinal_baudrate",     "921600",           "Baudrate for spinal micro-ROS Agent"),
    ("spinal_verbosity",    "4",                "Verbosity for spinal micro-ROS Agent"),
    ("launch_mocap",        "true",             "Launch OptiTrack mocap receiver on real machine", ["true", "false"]),
    ("mocap_robot_id",      "1",                "OptiTrack rigid body ID for this robot"),
    ("mocap_multicast_address", "239.255.42.99", "OptiTrack NatNet multicast address"),
    ("mocap_data_port",     "1511",             "OptiTrack NatNet data port"),
    ("mocap_interface_address", "0.0.0.0",      "Local interface address for OptiTrack multicast"),
    ("spawn_x",             "0.0",              "Gazebo spawn X position [m] (sim only)"),
    ("spawn_y",             "0.0",              "Gazebo spawn Y position [m] (sim only)"),
    ("spawn_z",             "0.5",              "Gazebo spawn Z position [m] (sim only)"),
    ("robot_model_rviz",    "rviz_config",      "RViz config filename (resolved inside robot_model pkg/config/)"),
    ("debug_core",          "false",            "Run aerial_robot_core under gdb", ["true", "false"]),
]
# fmt: on


def sanity_check(context, *args, **kwargs):
    # Evaulate AFTER substitutions (e.g., sim and estimation_mode) are resolved, but BEFORE any nodes are launched
    # NOTE: Together with "choices" only real possibility to guardrail arguments
    real_machine = LaunchConfiguration("real_machine").perform(context)
    sim = LaunchConfiguration("sim").perform(context)
    if real_machine.lower() == "true" and sim.lower() == "true":
        raise RuntimeError("real_machine and sim arguments cannot both be true")

    rate = float(LaunchConfiguration("main_rate").perform(context))
    if rate <= 0 or rate > 200:
        raise RuntimeError(f"main_rate={rate} is not as expected. ")


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
    robot_model_pkg = LaunchConfiguration("robot_model")
    robot_ns = LaunchConfiguration("robot_ns")
    airframe = LaunchConfiguration("airframe")
    real_machine = LaunchConfiguration("real_machine")
    main_rate = LaunchConfiguration("main_rate")
    estimation_mode = LaunchConfiguration("estimation_mode")
    model_options = LaunchConfiguration("model_options")
    headless = LaunchConfiguration("headless")
    sim = LaunchConfiguration("sim")
    sim_estimation_mode = LaunchConfiguration("sim_estimation_mode")
    launch_spinal = LaunchConfiguration("launch_spinal")
    launch_spinal_bridge = LaunchConfiguration("launch_spinal_bridge")
    spinal_dev = LaunchConfiguration("spinal_dev")
    spinal_baudrate = LaunchConfiguration("spinal_baudrate")
    spinal_verbosity = LaunchConfiguration("spinal_verbosity")
    launch_mocap = LaunchConfiguration("launch_mocap")
    mocap_robot_id = LaunchConfiguration("mocap_robot_id")
    mocap_multicast_address = LaunchConfiguration("mocap_multicast_address")
    mocap_data_port = LaunchConfiguration("mocap_data_port")
    mocap_interface_address = LaunchConfiguration("mocap_interface_address")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    robot_model_rviz = LaunchConfiguration("robot_model_rviz")
    debug_core = LaunchConfiguration("debug_core")

    active_estimation_mode = PythonExpression(
        ["int('", sim_estimation_mode, "') if '", sim, "' == 'true' else int('", estimation_mode, "')"]
    )
    core_prefix = PythonExpression(["'gdb -ex run --args' if '", debug_core, "' == 'true' else ''"])
    real_machine_only = ["'", real_machine, "' == 'true' and '", sim, "' == 'false'"]

    # ------------------------------------------------------------------
    # 2.  Derived paths
    # ------------------------------------------------------------------
    robot_model_param_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_model_pkg),
            "config",
            "RobotModel.yaml",
        ]
    )

    motor_info_param_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_model_pkg),
            "config",
            airframe,
            "MotorInfo.yaml",
        ]
    )

    state_estimation_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_model_pkg),
            "config",
            "StateEstimation.yaml",
        ]
    )

    navigation_param_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_model_pkg),
            "config",
            "NavigationConfig.yaml",
        ]
    )

    common_control_param_path = PathJoinSubstitution(
        [
            FindPackageShare("aerial_robot_control"),
            "config",
            "PID.yaml",
        ]
    )

    control_config_filename = PythonExpression(
        ["'GimbalrotorControl_sim.yaml' if '", sim, "' == 'true' else 'GimbalrotorControl.yaml'"]
    )

    control_param_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_model_pkg),
            "config",
            airframe,
            control_config_filename,
        ]
    )

    battery_param_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_model_pkg),
            "config",
            "Battery.yaml",
        ]
    )

    servo_param_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_model_pkg),
            "config",
            airframe,
            "Servo.yaml",
        ]
    )

    xacro_filename = PythonExpression(
        ["'gimbalrotor.gazebo.xacro' if '", sim, "' == 'true' else 'gimbalrotor.urdf.xacro'"]
    )

    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_model_pkg),
            "robots",
            airframe,
            xacro_filename,
        ]
    )

    # Build the URDF/xacro string via the xacro CLI.
    # model_options is appended verbatim so callers can inject extra xacro args, e.g.,:
    #   model_options:="prop_num:=6 payload:=true"
    # and wrap in ParameterValue so ROS2 treats it as a string parameter
    robot_description = [
        FindExecutable(name="xacro"),
        TextSubstitution(text=" "),
        xacro_path,
        TextSubstitution(text=" "),
        model_options,
    ]
    robot_description_param = {"robot_description": ParameterValue(Command(robot_description), value_type=str)}

    sim_param_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_model_pkg),
            "config",
            airframe,
            "Simulation.yaml",
        ]
    )

    rviz_config_path = PathJoinSubstitution([FindPackageShare(robot_model_pkg), "config", robot_model_rviz])

    rviz_init_path = PathJoinSubstitution([FindPackageShare(robot_model_pkg), "config", "RvizInit.yaml"])

    # ------------------------------------------------------------------
    # 3.  Nodes
    # ------------------------------------------------------------------
    core_node = Node(
        package="aerial_robot_core",
        executable="aerial_robot_core_node",
        name="aerial_robot_core",
        namespace=robot_ns,
        prefix=core_prefix,
        parameters=[
            {
                "main_rate": main_rate,
                "warn_main_rate": ParameterValue(
                    PythonExpression(["False if '", sim, "' == 'true' else True"]), value_type=bool
                ),
                "main_rate_warn_tolerance": ParameterValue(
                    PythonExpression(["0.5 if '", sim, "' == 'true' else 0.2"]), value_type=float
                ),
                "navigation.require_spinal_ready_for_arm": ParameterValue(
                    PythonExpression(real_machine_only + [" and '", launch_spinal_bridge, "' == 'true'"]),
                    value_type=bool,
                ),
                "flight_navigation_plugin_name": "aerial_robot_navigation/gimbalrotor_navigation",
                "estimation.mode": active_estimation_mode,
                "use_sim_time": sim,
            },
            robot_description_param,
            robot_model_param_path,
            state_estimation_path,
            common_control_param_path,
            control_param_path,
            navigation_param_path,
            motor_info_param_path,
            battery_param_path,
        ],
        output="screen",
    )

    servo_bridge_node = Node(
        package="aerial_robot_model",
        executable="servo_bridge_node",
        name="servo_bridge",
        namespace=robot_ns,
        parameters=[
            robot_description_param,
            servo_param_path,
            {
                "sim": sim,
            },
        ],
        output="screen",
    )

    joint_position_spawner = Node(
        namespace=robot_ns,
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_group_position_controller",
        arguments=[
            "joint_group_position_controller",
            "--param-file",
            sim_param_path,
            "--param-file",
            servo_param_path,
        ],
        condition=IfCondition(sim),
        output="screen",
    )

    spinal_namespace_bridge_node = Node(
        package="spinal",
        executable="spinal_namespace_bridge",
        name="spinal_namespace_bridge",
        parameters=[
            {
                "robot_namespace": robot_ns,
            },
        ],
        condition=IfCondition(PythonExpression(real_machine_only + [" and '", launch_spinal_bridge, "' == 'true'"])),
        output="screen",
    )

    # ------------------------------------------------------------------
    # 4.  Call child launch files
    # ------------------------------------------------------------------
    # Robot model (URDF publisher, RViz, joint-state publisher )
    model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aerial_robot_model"),
                    "launch",
                    "aerial_robot_model_launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_model": robot_model_pkg,
            "robot_ns": robot_ns,
            "real_machine": real_machine,
            "model_options": model_options,
            "headless": headless,
            "rviz_config_path": rviz_config_path,
            "rviz_init_path": rviz_init_path,
            "robot_description": robot_description,
            "sim": sim,
        }.items(),
    )

    # Gazebo simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aerial_robot_simulation"),
                    "launch",
                    "gazebo_launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ns": robot_ns,
            "headless": headless,
            "sim_param_path": sim_param_path,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
        }.items(),
        condition=IfCondition(sim),
    )

    # Real machine: spinal micro-ROS Agent
    spinal_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("spinal"),
                    "launch",
                    "agent_launch.py",
                ]
            )
        ),
        launch_arguments={
            "dev": spinal_dev,
            "baudrate": spinal_baudrate,
            "verbosity": spinal_verbosity,
        }.items(),
        condition=IfCondition(PythonExpression(real_machine_only + [" and '", launch_spinal, "' == 'true'"])),
    )

    # Real machine: OptiTrack mocap receiver
    mocap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aerial_robot_core"),
                    "launch",
                    "external_module",
                    "mocap.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ns": robot_ns,
            "robot_id": mocap_robot_id,
            "multicast_address": mocap_multicast_address,
            "data_port": mocap_data_port,
            "interface_address": mocap_interface_address,
        }.items(),
        condition=IfCondition(PythonExpression(real_machine_only + [" and '", launch_mocap, "' == 'true'"])),
    )

    # ------------------------------------------------------------------
    # 5.  Assemble LaunchDescription
    # ------------------------------------------------------------------
    ld = LaunchDescription()

    for arg in declared_args:
        ld.add_action(arg)

    ld.add_action(OpaqueFunction(function=sanity_check))
    ld.add_action(core_node)
    ld.add_action(servo_bridge_node)
    ld.add_action(joint_position_spawner)
    ld.add_action(spinal_namespace_bridge_node)
    ld.add_action(model_launch)
    ld.add_action(spinal_agent_launch)
    ld.add_action(mocap_launch)
    ld.add_action(sim_launch)

    return ld
