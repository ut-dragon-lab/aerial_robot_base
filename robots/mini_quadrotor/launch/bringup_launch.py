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
    ("robot_model",         "mini_quadrotor",   "Name of the robot model ROS package"),
    ("robot_ns",            "mini_quadrotor",   "Namespace for all robot nodes"),
    ("real_machine",        "true",             "Use real machine specific bring-up inside model_launch", ["true", "false"]),
    ("main_rate",           "100.0",            "Core node main loop rate [Hz]"),
    ("estimation_mode",     "0",                "Estimator mode on real machine: 0=egomotion, 1=experiment, 2=ground-truth", ["0", "1", "2"]),
    ("model_options",       "",                 "Extra xacro arguments passed verbatim to xacro"),
    ("headless",            "true",             "Run without GUI", ["true", "false"]),
    ("sim",                 "false",            "Launch simulation", ["true", "false"]),
    ("simulator",           "gazebo",           "Simulator backend used when sim:=true", ["gazebo", "mujoco"]),
    ("sim_estimation_mode", "2",                "Estimator mode in simulation: 0=egomotion, 1=experiment, 2=ground-truth", ["0", "1", "2"]),
    ("spawn_x",             "0.0",              "Simulation spawn X position [m] (sim only)"),
    ("spawn_y",             "0.0",              "Simulation spawn Y position [m] (sim only)"),
    ("spawn_z",             "0.5",              "Simulation spawn Z position [m] (sim only)"),
    ("mujoco_model",        "",                 "MuJoCo MJCF/XML model path. Empty or missing path generates XML from URDF/Xacro"),
    ("viewer_font_scale",   "100",              "MuJoCo viewer UI font scale percent; set 0 to keep MuJoCo default"),
    ("robot_model_rviz",    "rviz_config.rviz", "RViz config filename (resolved inside robot_model pkg/config/)"),
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
    real_machine = LaunchConfiguration("real_machine")
    main_rate = LaunchConfiguration("main_rate")
    estimation_mode = LaunchConfiguration("estimation_mode")
    model_options = LaunchConfiguration("model_options")
    headless = LaunchConfiguration("headless")
    sim = LaunchConfiguration("sim")
    simulator = LaunchConfiguration("simulator")
    sim_estimation_mode = LaunchConfiguration("sim_estimation_mode")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    mujoco_model = LaunchConfiguration("mujoco_model")
    viewer_font_scale = LaunchConfiguration("viewer_font_scale")
    robot_model_rviz = LaunchConfiguration("robot_model_rviz")
    debug_core = LaunchConfiguration("debug_core")

    sim_is_gazebo = PythonExpression(["('", sim, "' == 'true') and ('", simulator, "' == 'gazebo')"])
    sim_is_mujoco = PythonExpression(["('", sim, "' == 'true') and ('", simulator, "' == 'mujoco')"])

    active_estimation_mode = PythonExpression(
        ["int('", sim_estimation_mode, "') if '", sim, "' == 'true' else int('", estimation_mode, "')"]
    )
    core_prefix = PythonExpression(["'gdb -ex run --args' if '", debug_core, "' == 'true' else ''"])

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

    control_param_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_model_pkg),
            "config",
            "ControllerConfig.yaml",
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
            "Servo.yaml",
        ]
    )

    xacro_filename = PythonExpression(
        ["'robot.gazebo.xacro' if ('", sim, "' == 'true' and '", simulator, "' == 'gazebo') else 'robot.urdf.xacro'"]
    )

    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_model_pkg),
            "urdf",
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
                "estimation.mode": active_estimation_mode,
                "robot_model_fixed": ParameterValue(sim_is_mujoco, value_type=bool),
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
                "use_mujoco": ParameterValue(sim_is_mujoco, value_type=bool),
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
        condition=IfCondition(sim_is_gazebo),
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

    gazebo_launch = IncludeLaunchDescription(
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
        condition=IfCondition(sim_is_gazebo),
    )

    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aerial_robot_simulation"),
                    "launch",
                    "mujoco_launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ns": robot_ns,
            "headless": headless,
            "mujoco_model": mujoco_model,
            "mujoco_urdf_xacro": xacro_path,
            "mujoco_xacro_options": model_options,
            "viewer_font_scale": viewer_font_scale,
            "sim_param_path": sim_param_path,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
        }.items(),
        condition=IfCondition(sim_is_mujoco),
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
    ld.add_action(model_launch)
    ld.add_action(gazebo_launch)
    ld.add_action(mujoco_launch)

    return ld
