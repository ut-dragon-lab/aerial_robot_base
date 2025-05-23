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
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # --- LaunchConfiguration ---
    headless         = LaunchConfiguration('headless')
    need_js          = LaunchConfiguration('need_joint_state')
    model_options    = LaunchConfiguration('model_options')
    robot_model_pkg  = LaunchConfiguration('robot_model')
    robot_ns         = LaunchConfiguration('robot_ns')
    robot_model_rviz = LaunchConfiguration('robot_model_rviz')
    robot_description_content = LaunchConfiguration('robot_description_content')

    # --- DeclareLaunchArgument ---
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run without GUI (headless mode)'
    )

    need_joint_state_arg = DeclareLaunchArgument(
        'need_joint_state',
        default_value='true',
        description='Whether to publish joint state information'
    )

    model_options_arg = DeclareLaunchArgument(
        'model_options',
        default_value='',
        description='Additional model options (e.g., extra URDF args)'
    )

    robot_model_pkg_arg = DeclareLaunchArgument(
        'robot_model',
        description='Name of the robot model package'
    )

    robot_ns_arg = DeclareLaunchArgument(
        'robot_ns',
        description='ROS namespace under which to launch the robot'
    )

    robot_model_rviz_arg = DeclareLaunchArgument(
        'robot_model_rviz',
        default_value='rviz_config.rviz',
        description='RViz display configuration for the robot model'
    )

    robot_description_arg = DeclareLaunchArgument(
        'robot_description_content',
        description='Full robot_description (xacro output) from parent'
    )

    # --- robot description parameter ---w
    robot_description = {
            'robot_description':
        ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    # --- rviz config path ---w
    rviz_config = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        'config',
        robot_model_rviz
    ])
    
    # --- nodes ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_ns,
        parameters=[
            robot_description,
            {'use_sim_time': False, 'tf_prefix': robot_ns}
        ]
    )
        
    rotor_tf_publisher_node = Node(
        package='aerial_robot_model',
        executable='rotor_tf_publisher',
        name='rotor_tf_publisher',
        namespace=robot_ns,
        condition=UnlessCondition(need_js),
        parameters=[
            {'tf_prefix': robot_ns},
            robot_description],
    )

    joint_state_pub_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=robot_ns,
        condition=IfCondition(need_js),
        parameters=[robot_description]
    )
            
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_ns,
        arguments=['-d', rviz_config],
        condition=UnlessCondition(headless)
    )

    ld = LaunchDescription()
    ld.add_action(headless_arg)
    ld.add_action(model_options_arg)
    ld.add_action(need_joint_state_arg)
    ld.add_action(robot_model_pkg_arg)
    ld.add_action(robot_ns_arg)
    ld.add_action(robot_model_rviz_arg)
    ld.add_action(robot_description_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rotor_tf_publisher_node)
    ld.add_action(joint_state_pub_node)
    ld.add_action(rviz2_node)
    return ld
