from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration('model')
    rvizconfig = LaunchConfiguration('rvizconfig')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([
                FindPackageShare('spot_micro_rviz'),
                'urdf',
                'spot_micro.urdf.xacro'
            ]),
        ),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('spot_micro_rviz'),
                'rviz',
                'spot_micro.rviz'
            ]),
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_gui': gui}],
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', model]),
                    value_type=str
                ),
            }],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rvizconfig],
            output='screen',
        ),

        Node(
            package='spot_micro_keyboard_command',
            executable='spot_micro_keyboard_command_node',
            name='spot_micro_keyboard_command_node',
            output='screen',
        ),
    ])