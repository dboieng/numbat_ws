import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('smov_config'),
        'my_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='i2c_pwm_board',
            executable='node',
            arguments=['1'],
            parameters=[config],
            output='screen',
        ),
        Node(
            package='smov_config',
            executable='servos',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='smov_states',
            executable='node',
            parameters=[config],
            prefix="bash -c 'sleep 2.0; exec $0 $@'",
            output='screen',
        ),
    ])