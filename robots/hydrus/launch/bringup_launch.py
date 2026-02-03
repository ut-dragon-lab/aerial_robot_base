#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnExecutionComplete
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    TextSubstitution,
    FindExecutable,
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # --- LaunchConfiguration ---
    headless         = LaunchConfiguration('headless')
    model_options    = LaunchConfiguration('model_options')
    robot_model_pkg  = LaunchConfiguration('robot_model')
    robot_ns         = LaunchConfiguration('robot_ns')
    robot_model_rviz = LaunchConfiguration('robot_model_rviz')
    onboards_model   = LaunchConfiguration('onboards_model')
    sim       = LaunchConfiguration('sim')
    rm       = LaunchConfiguration('rm')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')

    # --- DeclareLaunchArgument ---
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run without GUI (headless mode)'
    )

    model_options_arg = DeclareLaunchArgument(
        'model_options',
        default_value='',
        description='Additional model options (e.g., extra URDF args)'
    )

    robot_model_pkg_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='hydrus',
        description='Name of the robot model package'
    )

    robot_ns_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value='hydrus',
        description='ROS namespace under which to launch the robot'
    )

    robot_model_rviz_arg = DeclareLaunchArgument(
        'robot_model_rviz',
        default_value='rviz_config.rviz',
        description='RViz display configuration for the robot model'
    )

    onboards_model_arg = DeclareLaunchArgument(
        'onboards_model',
        default_value='default_mode_201907',
        description='On-board hardware model configuration'
    )

    simulation_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Run in simulation mode (e.g., use Gazebo clock)'
    )

    realmachine_arg = DeclareLaunchArgument(
        'rm',
        default_value='false',
        description='Run in realmachine mode'
    )

    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='Initial X position'
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Initial Y position'
    )
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.5',
        description='Initial Z position'
    )    

    # --- robot_description parameter  ---
    xacro_type = PythonExpression([
        "'robot.gazebo.xacro' if '", sim, "' == 'true' else 'robot.urdf.xacro'"
    ])
    
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        'robots',
        'quad',
        onboards_model,
        xacro_type
    ])

    robot_description_content = Command([
        'xacro ',
        urdf_xacro,
        model_options
    ])
    
    robot_description = {
        'robot_description':
        ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    # --- servo parameter  ---
    servo_param_file = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        'config',
        'quad',
        onboards_model,
        'Servo.yaml',
    ])

    # --- simulation parameter  ---
    sim_param_file = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        'config',
        'Simulation.yaml',
    ])    

    # --- robot model parameter  TODO: get from yaml file---
    robot_model_plugin_name = {
        'robot_model_plugin_name':
        ParameterValue(
            'multirotor_robot_model',
            value_type=str
        )
    }

    # --- joint state condition parameter---
    need_js = PythonExpression([
            # 'false' if sim=='true' or rm=='true' else 'true'
        "'false' if '", sim, "' == 'true' or '", rm, "' == 'true' else 'true'"
    ])

    # --- rviz config path ---w
    rviz_config = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        'config',
        robot_model_rviz
    ])
    
    # --- nodes ---
    core_node =  Node(
        package='aerial_robot_core',
        executable='aerial_robot_core_node',
        name='aerial_robot_core',
        namespace=robot_ns,
        parameters=[robot_description, robot_model_plugin_name],
        output = 'screen'
    )

    servo_bridge_node = Node(
        package='aerial_robot_model',
        executable='servo_bridge_node',
        name='servo_bridge',
        namespace=robot_ns,
        parameters=[robot_description, servo_param_file,
                    {
                        'sim': sim,
                    }
        ],
    )

    model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('aerial_robot_model'),
                'launch',
                'aerial_robot_model_launch.py'
            ])
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
            'model_options': LaunchConfiguration('model_options'),
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_ns': LaunchConfiguration('robot_ns'),
            'robot_model_rviz': LaunchConfiguration('robot_model_rviz'),
            'robot_description_content': robot_description_content,
            'need_joint_state': need_js,
        }.items(),
    )
    
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('aerial_robot_simulation'),
                'launch',
                'gazebo_launch.py'
            ])
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_ns'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'sim_param_file':   sim_param_file,
            'headless': LaunchConfiguration('headless'),
        }.items(),
        condition=IfCondition(sim),
    )

    joint_state_broadcaster_spawner = Node(
        namespace= robot_ns,
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        condition=IfCondition(sim),
    )

    joint_position_spawner = Node(
        namespace= robot_ns,
        package='controller_manager',
        executable='spawner',
        name='spawn_joint_group_position_controller',
        output='screen',
        arguments=[
            'joint_group_position_controller',
            '--param-file', sim_param_file,
            '--param-file', servo_param_file,
        ],
        condition=IfCondition(sim),
    )

    att_controller_spawner = Node(
        namespace= robot_ns,
        package='controller_manager',
        executable='spawner',
        name='spawn_att_controller',
        output='screen',
        arguments=[
            'attitude_controller',
            '--param-file', sim_param_file
        ],
        condition=IfCondition(sim),
    )

    ld = LaunchDescription()
    ld.add_action(headless_arg)
    ld.add_action(model_options_arg)
    ld.add_action(robot_model_pkg_arg)
    ld.add_action(robot_ns_arg)
    ld.add_action(robot_model_rviz_arg)
    ld.add_action(onboards_model_arg)
    ld.add_action(simulation_arg)
    ld.add_action(realmachine_arg)
    ld.add_action(spawn_x_arg)
    ld.add_action(spawn_y_arg)
    ld.add_action(spawn_z_arg)
    ld.add_action(core_node)
    ld.add_action(servo_bridge_node)
    ld.add_action(model_launch)
    ld.add_action(sim_launch)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_position_spawner)
    ld.add_action(att_controller_spawner)
    return ld
