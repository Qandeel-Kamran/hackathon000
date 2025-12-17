from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_gazebo = LaunchConfiguration('use_gazebo')
    world_file = LaunchConfiguration('world_file')
    robot_model = LaunchConfiguration('robot_model')
    headless = LaunchConfiguration('headless')

    # Declare launch arguments
    declare_use_gazebo_cmd = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Use Gazebo simulation'
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='empty.world',
        description='Choose one of the world files from `/gazebo_ros_pkgs/gazebo_ros_worlds/worlds` or provide your own'
    )

    declare_robot_model_cmd = DeclareLaunchArgument(
        'robot_model',
        default_value='humanoid_robot',
        description='Robot model to spawn in Gazebo'
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without GUI'
    )

    # Start Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_gazebo),
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    # Start Gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(use_gazebo),
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_gazebo}
        ],
        output='screen'
    )

    # Joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': use_gazebo}
        ],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_model,
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_robot_model_cmd)
    ld.add_action(declare_headless_cmd)

    # Add the commands and nodes
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_robot_node)

    return ld