import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable, LogInfo, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution, Command
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_share   = get_package_share_directory('aerial_robot_simulation')
    pkg_prefix = get_package_prefix('aerial_robot_simulation')
    ign_plugins_path = os.path.join(pkg_prefix, 'lib')

    default_world = os.path.join(pkg_share, 'gazebo_model', 'world', 'empty.world')
    desc_arg = DeclareLaunchArgument(
        'robot_description_content',
        default_value='',
        description='URDF (xacro result) passed from bringup'
    )
    world_arg      = DeclareLaunchArgument('world_sdf_file',     default_value=default_world,    description='Ignition world file')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='hydrus',       description='Entity name for spawn')

    world        = LaunchConfiguration('world_sdf_file')
    robot_name   = LaunchConfiguration('robot_name')
    robot_description_content = LaunchConfiguration('robot_description_content')

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('aerial_robot_simulation'),
            'config',
            'Gazebo.yaml',
        ]
    )


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

    # set_ign_plugin_path = SetEnvironmentVariable(
    #     name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
    #     value=(ign_plugins_path +
    #            os.pathsep +
    #            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''))
    # )

    set_ign_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            ign_plugins_path,
            os.pathsep,
            EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value='')
        ]
    )

    # set_ign_default_path = SetEnvironmentVariable(
    #     name='GZ_SIM_RESOURCE_PATH',
    #     value=[
    #         EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
    #         TextSubstitution(text=':'),
    #         TextSubstitution(text='$GZ_SIM_SYSTEM_PLUGIN_PATH'),
    #     ]
    # )

    set_ign_default_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join('/opt/ros/humble/lib') + ':' +
              os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    )    


    # 1) Start Gazebo ignition (without GUI)
    ign_server = ExecuteProcess(
        cmd=[
            'ign', 'gazebo',
            '-r',                     
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
            world
        ],
        output='screen'
    )

    # 2) Topic bridge (temporary)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
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

    joint_state_broadcaster_spawner = Node(
        namespace='hydrus',
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # joint_trajectory_controller_spawner = Node(
    #     namespace='hydrus',
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'joint_state_controller',
    #         '--param-file',
    #         robot_controllers,
    #     ],
    # )

    joint_position_spawner = Node(
        namespace='hydrus',
        package='controller_manager',
        executable='spawner',
        name='spawn_joint_group_position_controller',
        output='screen',
        arguments=[
            'joint_group_position_controller',
            '--param-file', robot_controllers
        ]
    )    
    

    return LaunchDescription([
        world_arg,
        robot_name_arg,
        set_env,
        set_ign_resource_path,
        set_ign_default_path,
        set_ign_plugin_path,
        # gz_sim,
        ign_server,
        clock_bridge,
        # gz_sim_launch,
        spawn_robot,
        ign_client,
        joint_state_broadcaster_spawner,
        # joint_trajectory_controller_spawner
        joint_position_spawner,
        # gz_sim,
        # ros2_control_node
    ])
