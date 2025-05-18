#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    TextSubstitution,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # --- launch arguments ---
    arg_defs = [
        ('headless','false', 'Disable RViz if true'),
        ('need_joint_state','true', 'Use joint_state_publisher_gui'),
        ('model_options', '', 'Extra xacro arguments'),
        ('robot_model', 'hydrus', 'Name of robot package (e.g. mi)'),
        ('robot_ns', 'hydrus', 'ROS namespace for this robot'),
        ('onboards_model', 'default_mode_201907', 'robot version'),
        ('simulation', 'true', 'simulation flag'),
        ('robot_model_rviz',
         [ LaunchConfiguration('robot_model'),
           TextSubstitution(text='.rviz') ],
         'RViz config filename under config/'
        ),
    ]

    declare_args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in arg_defs
    ]

    # 3. LaunchConfiguration のまとめ取得
    headless         = LaunchConfiguration('headless')
    need_jsp         = LaunchConfiguration('need_joint_state')
    model_options    = LaunchConfiguration('model_options')
    robot_model_pkg  = LaunchConfiguration('robot_model')
    robot_ns         = LaunchConfiguration('robot_ns')
    robot_model_rviz = LaunchConfiguration('robot_model_rviz')
    onboards_model   = LaunchConfiguration('onboards_model')
    simulation       = LaunchConfiguration('simulation')

    # --- robot_description parameter (xacro → URDF) ---
    xacro_type = None
    if simulation:
        xacro_type = 'robot.gazebo.xacro'
    else:
        xacro_type = 'robot.urdf.xacro'
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        'robots',
        'quad',
        'default_mode_201907',
        xacro_type
    ])

    servo_param_file = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        'config',
        'quad',
        'default_mode_201907',
        'Servo.yaml',
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

    robot_model_plugin_name = {
        'robot_model_plugin_name':
        ParameterValue(
            'multirotor_robot_model',
            value_type=str
        )
    }

    # --- rviz config path: <pkg>/config/<robot_model_rviz> ---
    rviz_config = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        'config',
        robot_model_rviz
    ])

    log_rviz = LogInfo(
        condition=UnlessCondition(headless),
        msg=[
            TextSubstitution(text='Using RViz config: '),
            rviz_config
        ]
    )

    return LaunchDescription([
        *declare_args,
        
        Node(
            package='aerial_robot_core',
            executable='aerial_robot_core_node',
            name='aerial_robot_core',
            namespace=robot_ns,
            condition=IfCondition(need_jsp),
            parameters=[robot_description, robot_model_plugin_name],
            output = 'screen'
        ),

        # servo_bridge
        Node(
            package='aerial_robot_model',
            executable='servo_bridge_node',
            name='servo_bridge',
            namespace=robot_ns,
            condition=IfCondition(need_jsp),
            parameters=[robot_description, servo_param_file],
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_ns,
            parameters=[
                robot_description,
                {'use_sim_time': False, 'tf_prefix': robot_ns}
            ]
        ),
        
        Node(
            package='aerial_robot_model',
            executable='rotor_tf_publisher',
            name='rotor_tf_publisher',
            namespace=robot_ns,
            condition=IfCondition(need_jsp),
            parameters=[
                {'tf_prefix': robot_ns},
                robot_description],
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=robot_ns,
            arguments=['-d', rviz_config],
            condition=UnlessCondition(headless)
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([
        #             FindPackageShare('aerial_robot_simulation'),
        #             'launch',
        #             'gazebo_launch.py'
        #         ])
        #     ),
        #     launch_arguments={
        #         # gazebo_launch.py 側で用意した引数名：渡す LaunchConfiguration／Substitution
        #         # 'world': LaunchConfiguration('world'),
        #         # 'robot_name': LaunchConfiguration('robot_ns'),
        #         'robot_description_content': robot_description_content
        #     }.items()
        # ),
    ])
