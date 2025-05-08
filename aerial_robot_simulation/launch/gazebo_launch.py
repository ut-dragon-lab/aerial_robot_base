import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share   = get_package_share_directory('aerial_robot_simulation')
    default_world = os.path.join(pkg_share, 'gazebo_model', 'world', 'empty.world')

    world_arg      = DeclareLaunchArgument('world',     default_value=default_world,    description='Ignition world file')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='hydrus',       description='Entity name for spawn')

    world        = LaunchConfiguration('world')
    robot_name   = LaunchConfiguration('robot_name')

    robot_prefix = FindPackagePrefix(robot_name)

    robot_share_dir = PathJoinSubstitution([
        robot_prefix,
        'share'
    ])
        
    set_env = SetEnvironmentVariable(
        'DISPLAY', os.environ.get('DISPLAY','')
    )

    set_ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            TextSubstitution(text=':'),
            robot_share_dir
        ]
    )

    # 1) Start Gazebo ignition (without GUI)
    ign_server = ExecuteProcess(
        cmd=[
            'ign', 'gazebo',
            '-r',                              # リプレイなしで即実行
            '-s', 'libgazebo_ros_factory.so',  # ROS 2 プラグイン読み込み
            '-s', 'libgazebo_ros_init.so',
            world                              # world ファイル
        ],
        output='screen'
    )

    # 2) Topic bridge (temporary)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 正しいマッピング形式: topic@gazebo_type@ros2_type
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',

            # IMU センサ
            '/world/.*/link/.*/sensor/imu/imu'
            '@sensor_msgs/msg/Imu'
            '@gz.msgs.IMU',

            # GPS (NavSatFix)
            '/world/.*/link/.*/sensor/navsat/navsat'
            '@sensor_msgs/msg/NavSatFix'
            '@gz.msgs.NavSat',

            # Mocap (モデル全体のPoseStamped)
            '/world/.*/model/.*/pose'
            '@geometry_msgs/msg/PoseStamped'
            '@gz.msgs.Pose_V',
        ],
        output='screen'
    )

    # 3) Spawn robot
    spawn_robot = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name',  robot_name,
                    '-topic', '/hydrus/robot_description'
                ],
                output='screen'
    )

    # 4) Start GUI
    ign_client = ExecuteProcess(
           cmd=[
                'ign', 'gazebo',
                '-g',
            ],
            output='screen'
        )

    return LaunchDescription([
        world_arg,
        robot_name_arg,
        set_env,
        set_ign_resource_path,
        ign_server,
        clock_bridge,
        # gz_sim_launch,
        spawn_robot,
        ign_client
    ])
