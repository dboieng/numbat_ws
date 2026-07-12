from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_i2c_pwmboard = LaunchConfiguration('run_i2c_pwmboard')

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_i2c_pwmboard',
            default_value='false',
            description='Start the i2c_pwm_board node if true'
        ),

        Node(
            package='servo_move_keyboard',
            executable='servo_move_keyboard_node',
            name='servo_move_keyboard_node',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            condition=IfCondition(run_i2c_pwmboard),
            package='i2c_pwm_board',
            executable='node',
            name='i2c_pwm_board',
            arguments=['1'],
            output='screen',
        ),
    ])