from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    debug_mode = LaunchConfiguration('debug_mode')

    return LaunchDescription([
        DeclareLaunchArgument('debug_mode', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('spot_micro_motion_cmd'),
                    'launch',
                    'motion_cmd.launch.py',
                ])
            ),
            launch_arguments={'debug_mode': debug_mode}.items(),
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        Node(
            package='spot_micro_joy',
            executable='spot_micro_joy_node',
            name='spot_micro_joy_node',
            output='screen',
        ),
    ])
