import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='/',
            description='Namespace for the robot'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'coalesce_interval': 0.025,
            }]
        )
    ])
