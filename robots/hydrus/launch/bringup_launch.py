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
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile

def generate_launch_description():

    # --- launch arguments ---
    headless        = LaunchConfiguration('headless')
    need_jsp        = LaunchConfiguration('need_joint_state')
    model_options   = LaunchConfiguration('model_options')
    robot_model_pkg = LaunchConfiguration('robot_model')
    robot_ns        = LaunchConfiguration('robot_ns')
    robot_model_rviz = LaunchConfiguration('robot_model_rviz')
    onboards_model = LaunchConfiguration('default_mode_201907')

    # --- robot_description parameter (xacro → URDF) ---
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        'robots',
        'quad',
        'default_mode_201907',
        'robot.urdf.xacro'
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
        # arguments
        DeclareLaunchArgument('headless',        default_value='false',
                               description='Disable RViz if true'),
        DeclareLaunchArgument('need_joint_state',default_value='true',
                               description='Use joint_state_publisher_gui'),
        DeclareLaunchArgument('model_options',   default_value='',
                               description='Extra xacro arguments'),
        DeclareLaunchArgument('robot_model',     description='Name of robot package (e.g. mini_quadrotor)'),
        DeclareLaunchArgument('robot_ns',        description='ROS namespace for this robot'),
        DeclareLaunchArgument(
            'robot_model_rviz',
            default_value=[ LaunchConfiguration('robot_model'),
                            TextSubstitution(text='.rviz') ],
            description='RViz config filename under config/'
        ),
        DeclareLaunchArgument(
            'default_mode_201907',
            default_value='',
            description='robot version'
        ),

        # core
        Node(
            package='aerial_robot_core',
            executable='aerial_robot_core_node',
            name='aerial_robot_core',
            namespace=robot_ns,
            condition=IfCondition(need_jsp),
            parameters=[robot_description, robot_model_plugin_name]# ,
            # prefix=['gdb -ex run --args']
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

        # joint_state_publisher_gui (optional)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=robot_ns,
            condition=IfCondition(need_jsp),
            parameters=[robot_description]
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

        # rotor_tf_publisher (fallback)
        Node(
            package='aerial_robot_model',
            executable='rotor_tf_publisher',
            name='rotor_tf_publisher',
            namespace=robot_ns,
            condition=UnlessCondition(need_jsp),
            parameters=[{'tf_prefix': robot_ns}]
        ),

        log_rviz,

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=robot_ns,
            arguments=['-d', rviz_config],
            condition=UnlessCondition(headless)
        ),
    ])
