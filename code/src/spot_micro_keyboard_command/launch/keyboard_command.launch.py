from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    run_plot = LaunchConfiguration('run_plot')
    run_rviz = LaunchConfiguration('run_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('run_plot', default_value='false'),
        DeclareLaunchArgument('run_rviz', default_value='false'),

        # Node(
        #     package='spot_micro_keyboard_command',
        #     executable='spot_micro_keyboard_command_node',
        #     name='spot_micro_keyboard_command_node',
        #     output='screen',
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('spot_micro_plot'),
                    'launch',
                    'start_plotting.launch.py',
                ])
            ),
            condition=IfCondition(run_plot),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('spot_micro_rviz'),
                    'launch',
                    'show_model.launch.py',
                ])
            ),
            condition=IfCondition(run_rviz),
        ),
    ])
