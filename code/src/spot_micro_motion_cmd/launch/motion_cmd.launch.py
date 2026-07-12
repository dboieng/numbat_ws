import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # === Launch arguments ===
    run_standalone = LaunchConfiguration('run_standalone')
    debug_mode = LaunchConfiguration('debug_mode')
    run_lcd = LaunchConfiguration('run_lcd')

    # === Package directories ===
    spot_micro_motion_cmd_share_dir = get_package_share_directory(
        'spot_micro_motion_cmd'
    )

    lcd_monitor_share_dir = get_package_share_directory(
        'lcd_monitor'
    )

    # === Paths ===
    motion_cmd_config_file = os.path.join(
        spot_micro_motion_cmd_share_dir,
        'config',
        'spot_micro_motion_cmd.yaml'
    )

    lcd_monitor_launch_file = os.path.join(
        lcd_monitor_share_dir,
        'launch',
        'lcd_monitor.launch.py'
    )


    # === Included launch files ===
    lcd_monitor_launch = GroupAction(
        condition=IfCondition(run_lcd),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lcd_monitor_launch_file)
            )
        ],
    )

    # === Nodes ===
    i2c_pwm_board_node = Node(
        condition=UnlessCondition(run_standalone),
        package='i2c_pwm_board',
        executable='node',
        name='i2c_pwm_board',
        arguments=['1'],
        output='screen',
    )

    spot_micro_motion_cmd_node = Node(
        package='spot_micro_motion_cmd',
        executable='spot_micro_motion_cmd_node',
        name='spot_micro_motion_cmd_node',
        output='screen',
        parameters=[
            motion_cmd_config_file,
            {
                'run_standalone': run_standalone,
                'debug_mode': debug_mode,
            }
        ],
    )

    # === Launch Description ===
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'run_standalone',
        default_value='false'
    ))

    ld.add_action(DeclareLaunchArgument(
        'debug_mode',
        default_value='true'
    ))

    ld.add_action(DeclareLaunchArgument(
        'run_lcd',
        default_value='false'
    ))

    ld.add_action(i2c_pwm_board_node)
    ld.add_action(spot_micro_motion_cmd_node)
    #ld.add_action(lcd_monitor_launch)

    return ld