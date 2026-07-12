from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lcd_monitor',
            executable='lcd_monitor_node',
            name='lcd_monitor_node',
            output='screen',
        )
    ])
