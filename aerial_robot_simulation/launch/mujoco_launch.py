#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_ARGS = [
    ("robot_ns", "mini_quadrotor", "Namespace for all robot nodes"),
    ("headless", "true", "Run without GUI", ["true", "false"]),
    ("viewer_font_scale", "100", "MuJoCo viewer UI font scale percent; set 0 to keep MuJoCo default"),
    ("mujoco_model", "", "MuJoCo MJCF/XML model path. Generate from URDF/Xacro when empty or missing"),
    ("mujoco_urdf_xacro", "", "URDF/Xacro path used when MuJoCo XML must be generated"),
    ("mujoco_xacro_options", "", "Extra xacro arguments used for generated MuJoCo XML"),
    ("generated_model_dir", "/tmp/aerial_robot_mujoco", "Directory for generated MuJoCo XML/assets"),
    ("sim_param_path", "", "Path to YAML file with simulation parameters"),
    ("spawn_x", "0.0", "MuJoCo spawn X position [m]"),
    ("spawn_y", "0.0", "MuJoCo spawn Y position [m]"),
    ("spawn_z", "0.5", "MuJoCo spawn Z position [m]"),
]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            name,
            default_value=default_value,
            description=description,
            **({"choices": choices[0]} if choices else {}),
        )
        for name, default_value, description, *choices in _ARGS
    ]

    robot_ns = LaunchConfiguration("robot_ns")
    headless = LaunchConfiguration("headless")
    viewer_font_scale = LaunchConfiguration("viewer_font_scale")
    mujoco_model = LaunchConfiguration("mujoco_model")
    mujoco_urdf_xacro = LaunchConfiguration("mujoco_urdf_xacro")
    mujoco_xacro_options = LaunchConfiguration("mujoco_xacro_options")
    generated_model_dir = LaunchConfiguration("generated_model_dir")
    sim_param_path = LaunchConfiguration("sim_param_path")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")

    sim_param_server = Node(
        package="aerial_robot_simulation",
        executable="sim_param_server",
        name="sim_param_server",
        namespace=robot_ns,
        parameters=[sim_param_path],
        output="screen",
    )

    mujoco_bridge = Node(
        package="aerial_robot_simulation",
        executable="mujoco_bridge.py",
        name="mujoco_bridge",
        namespace=robot_ns,
        parameters=[
            sim_param_path,
            {
                "model_path": mujoco_model,
                "urdf_xacro_path": mujoco_urdf_xacro,
                "xacro_options": mujoco_xacro_options,
                "generated_model_dir": generated_model_dir,
                "headless": ParameterValue(headless, value_type=bool),
                "viewer_font_scale": ParameterValue(viewer_font_scale, value_type=int),
                "spawn_x": ParameterValue(spawn_x, value_type=float),
                "spawn_y": ParameterValue(spawn_y, value_type=float),
                "spawn_z": ParameterValue(spawn_z, value_type=float),
                "use_sim_time": True,
            },
        ],
        output="screen",
    )

    attitude_controller = Node(
        package="aerial_robot_simulation",
        executable="mujoco_attitude_controller",
        name="mujoco_attitude_controller",
        namespace=robot_ns,
        parameters=[
            sim_param_path,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=mujoco_bridge,
            on_exit=[Shutdown()],
        )
    )

    ld = LaunchDescription()
    for arg in declared_args:
        ld.add_action(arg)

    ld.add_action(sim_param_server)
    ld.add_action(attitude_controller)
    ld.add_action(mujoco_bridge)
    ld.add_action(shutdown_handler)
    return ld
