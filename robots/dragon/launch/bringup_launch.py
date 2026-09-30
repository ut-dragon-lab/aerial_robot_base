#!/usr/bin/env python3
"""DRAGON v1.5, four-link ROS 2 / Gazebo Harmonic simulation."""
import os
import math
from pathlib import Path
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def launch_setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    if value('real_machine') != 'false' or value('sim') != 'true':
        raise RuntimeError('This DRAGON port currently supports sim:=true real_machine:=false.')
    body_joint_velocity = float(value('body_joint_velocity'))
    if not math.isfinite(body_joint_velocity) or body_joint_velocity <= 0.0:
        raise ValueError('body_joint_velocity must be finite and positive')
    share = Path(get_package_share_directory('dragon'))
    ns = value('robot_ns')
    description = xacro.process_file(str(share / 'robots/quad/dragon.gazebo.xacro'),
                                     mappings={'robot_name': ns, 'visual_detail': value('visual_detail'),
                                               'body_joint_velocity': value('body_joint_velocity')}).toxml()
    config = lambda name: str(share / 'config' / name)
    params = [{'robot_description': description, 'use_sim_time': True}]
    core_params = params + [yaml.safe_load(Path(config(n)).read_text())['/**/aerial_robot_core']['ros__parameters'] for n in ['RobotModel.yaml', 'StateEstimation.yaml', 'Control.yaml',
                                               'NavigationConfig.yaml', 'MotorInfo.yaml', 'Battery.yaml']]
    core_params += [{'main_rate': 100.0, 'warn_main_rate': False, 'estimation.mode': 2,
                     'flight_navigation_plugin_name': 'aerial_robot_navigation/gimbalrotor_navigation',
                     'navigation.require_spinal_ready_for_arm': False,
                     'navigation.takeoff_height': float(value('takeoff_height'))}]
    nodes = [Node(package='aerial_robot_core', executable='aerial_robot_core_node',
                  name='aerial_robot_core', namespace=ns, parameters=core_params, output='screen'),
             Node(package='robot_state_publisher', executable='robot_state_publisher',
                  namespace=ns, parameters=params, output='screen'),
             Node(package='aerial_robot_model', executable='rotor_tf_publisher', namespace=ns,
                  parameters=params, output='screen'),
             Node(package='aerial_robot_model', executable='servo_bridge_node', name='servo_bridge',
                  namespace=ns, parameters=params + [config('Servo.yaml'), {'sim': True,
                  'controller_manager_timeout': 60.0}], output='screen')]
    for controller in ['joint_group_position_controller', 'gimbal_position_controller']:
        nodes.append(Node(package='controller_manager', executable='spawner', namespace=ns,
                          arguments=[controller, '--param-file', config('Simulation.yaml'),
                                     '--param-file', config('Servo.yaml'), '--controller-manager-timeout', '60'],
                          output='screen'))
    nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('aerial_robot_simulation')) / 'launch/gazebo_launch.py')),
        launch_arguments={'robot_ns': ns, 'robot_model': 'dragon', 'headless': value('headless'),
                          'sim_param_path': config('Simulation.yaml'), 'spawn_z': value('spawn_z'),
                          'graphics_adapter': value('graphics_adapter'), 'render_engine': value('render_engine')}.items()))
    nodes.append(Node(package="rviz2", executable="rviz2", name="rviz2", namespace=ns,
                      arguments=["-d", config("rviz_config")],
                      parameters=[{"use_sim_time": True}],
                      additional_env=({'LIBGL_ALWAYS_SOFTWARE': '1', 'LP_NUM_THREADS': '2'}
                                      if value('rviz_software_rendering') == 'true' else {}),
                      remappings=[("robot_description", f"/{ns.strip('/')}/robot_description")],
                      condition=IfCondition(PythonExpression(["'", LaunchConfiguration("headless"),
                          "' == 'false' and '", LaunchConfiguration("rviz"), "' == 'true'"])), output="screen"))
    return nodes


def generate_launch_description():
    defaults = {'robot_ns': 'dragon', 'sim': 'true', 'real_machine': 'false', 'headless': 'true', 'rviz': 'true',
                'body_joint_velocity': '1.0', 'takeoff_height': '1.5', 'spawn_z': '0.4', 'render_engine': 'ogre' if os.environ.get('WSL2_GUI_APPS_ENABLED') == '1' else 'ogre2', 'visual_detail': 'balanced',
                'graphics_adapter': os.environ.get('MESA_D3D12_DEFAULT_ADAPTER_NAME', 'NVIDIA' if os.environ.get('WSL2_GUI_APPS_ENABLED') == '1' else ''),
                'rviz_software_rendering': 'true' if os.environ.get('WSL2_GUI_APPS_ENABLED') == '1' else 'false'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()]
                             + [OpaqueFunction(function=launch_setup)])
