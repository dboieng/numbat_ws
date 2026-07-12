from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    run_post_proc = LaunchConfiguration('run_post_proc')

    return LaunchDescription([
        DeclareLaunchArgument('run_post_proc', default_value='false'),

        GroupAction(
            condition=UnlessCondition(run_post_proc),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('spot_micro_motion_cmd'),
                            'launch',
                            'motion_cmd.launch.py',
                        ])
                    )
                ),
                Node(
                    package='rplidar_ros',
                    executable='rplidarNode',
                    name='rplidarNode',
                    output='screen',
                    parameters=[{
                        'serial_port': '/dev/ttyUSB0',
                        'serial_baudrate': 115200,
                        'frame_id': 'lidar_link',
                        'inverted': False,
                        'angle_compensate': True,
                    }],
                ),
            ],
        ),

        Node(
            package='hector_mapping',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'use_tf_scan_transformation': True,
                'use_tf_pose_start_estimate': False,
                'pub_map_odom_transform': True,
                'map_resolution': 0.050,
                'map_size': 2048,
                'map_start_x': 0.5,
                'map_start_y': 0.5,
                'map_multi_res_levels': 2,
                'update_factor_free': 0.4,
                'update_factor_occupied': 0.9,
                'map_update_distance_thresh': 0.4,
                'map_update_angle_thresh': 0.06,
                'laser_z_min_value': -1.0,
                'laser_z_max_value': 1.0,
                'advertise_map_service': True,
                'scan_subscriber_queue_size': 5,
                'scan_topic': 'scan',
                'tf_map_scanmatch_transform_frame_name': 'scanmatcher_frame',
            }],
        ),
    ])
