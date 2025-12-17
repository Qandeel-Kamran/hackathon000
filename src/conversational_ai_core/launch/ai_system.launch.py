from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch argument for robot model
        DeclareLaunchArgument(
            'robot_model',
            default_value='default_robot',
            description='Robot model to use for the simulation'
        ),

        # Launch argument for simulation mode
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Launch argument for Gazebo
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='true',
            description='Use Gazebo simulation'
        ),

        # Conversational AI Node
        Node(
            package='conversational_ai_core',
            executable='conversational_ai_node',
            name='conversational_ai_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_model': LaunchConfiguration('robot_model')}
            ],
            output='screen'
        ),

        # Robot Control Bridge Node
        Node(
            package='conversational_ai_core',
            executable='robot_control_bridge_node',
            name='robot_control_bridge_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_model': LaunchConfiguration('robot_model')}
            ],
            output='screen'
        ),

        # Sensor Fusion Node
        Node(
            package='conversational_ai_core',
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Simulation Bridge Node
        Node(
            package='conversational_ai_core',
            executable='simulation_bridge_node',
            name='simulation_bridge_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Educational Content Server
        Node(
            package='educational_content',
            executable='lesson_content_server',
            name='lesson_content_server',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        )
    ])