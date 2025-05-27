import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # --- Load directory path ---
    pkg_share   = get_package_share_directory('aerial_robot_simulation')
    default_world = os.path.join(pkg_share, 'gazebo_model', 'world', 'empty.world')

    # --- LaunchConfiguration ---
    world        = LaunchConfiguration('world_sdf_file')
    robot_name   = LaunchConfiguration('robot_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    servo_param_file = LaunchConfiguration('servo_param_file')
    sim_param_file = LaunchConfiguration('sim_param_file')

    # --- DeclareLaunchArgument ---
    world_arg = DeclareLaunchArgument(
        'world_sdf_file',
        default_value=default_world,
        description='Ignition world file'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Entity name for spawn'
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
    servo_param_file_arg = DeclareLaunchArgument(
        'servo_param_file',
        default_value='',
        description='Path to the servo parameter YAML file'
    )
    sim_param_file_arg = DeclareLaunchArgument(
        'sim_param_file',
        default_value='',
        description='Path to the simulation parameter YAML file'
    )

    # --- environmental setting ---
    robot_prefix = FindPackagePrefix(robot_name)
    robot_share_dir = PathJoinSubstitution([robot_prefix, 'share'])
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

    set_ign_default_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            TextSubstitution(text='/opt/ros/humble/lib'),
            TextSubstitution(text=os.pathsep),
            EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value=''),
        ]
    )

    set_fastrtps_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(pkg_share, 'config', 'fastrtps_profiles.xml')
    )

    # --- Gazebo ---
    ign_server = ExecuteProcess(
        cmd=[
            'ign', 'gazebo',
            '-r',                     
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
            world
        ],
        output='screen',
    )

    ign_client = ExecuteProcess(
        cmd=[
            'ign', 'gazebo',
            '-g',
        ],
        output='screen',
    )

    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=ign_server,
            on_exit=[Shutdown()]
        )
    )

    # --- Nodes ---
    sim_param_server = Node(
        package='aerial_robot_simulation',
        executable='sim_param_server',
        name='sim_param_server',
        namespace= robot_name,
        parameters=[sim_param_file,
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )
    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  robot_name,
            '-topic', PythonExpression(["'/' + '", robot_name, "' + '/robot_description'"]),
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        namespace= robot_name,
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    joint_position_spawner = Node(
        namespace= robot_name,
        package='controller_manager',
        executable='spawner',
        name='spawn_joint_group_position_controller',
        output='screen',
        arguments=[
            'joint_group_position_controller',
            '--param-file', sim_param_file,
            '--param-file', servo_param_file,
        ]
    )

    att_controller_spawner = Node(
        namespace= robot_name,
        package='controller_manager',
        executable='spawner',
        name='spawn_att_controller',
        output='screen',
        arguments=[
            'attitude_controller',
            '--param-file', sim_param_file
        ]
    )

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(spawn_x_arg)
    ld.add_action(spawn_y_arg)
    ld.add_action(spawn_z_arg)
    ld.add_action(servo_param_file_arg)
    ld.add_action(sim_param_file_arg)
    ld.add_action(set_fastrtps_profile)
    ld.add_action(set_env)
    ld.add_action(set_ign_resource_path)
    ld.add_action(set_ign_default_path)
    ld.add_action(sim_param_server)
    ld.add_action(ign_server)
    ld.add_action(shutdown_handler)
    ld.add_action(ign_client)
    ld.add_action(clock_bridge)
    ld.add_action(spawn_robot)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_position_spawner)
    ld.add_action(att_controller_spawner)    

    return ld
