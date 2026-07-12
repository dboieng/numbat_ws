from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_slam = LaunchConfiguration('rviz_slam')

    return LaunchDescription([
        DeclareLaunchArgument('rviz_slam', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('spot_micro_keyboard_command'),
                    'launch',
                    'keyboard_command.launch.py',
                ])
            )
        ),

        GroupAction(
            condition=IfCondition(rviz_slam),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('spot_micro_rviz'),
                            'launch',
                            'slam.launch.py',
                        ])
                    )
                )
            ],
        ),

        GroupAction(
            condition=UnlessCondition(rviz_slam),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('spot_micro_rviz'),
                            'launch',
                            'show_model.launch.py',
                        ])
                    )
                )
            ],
        ),
    ])
