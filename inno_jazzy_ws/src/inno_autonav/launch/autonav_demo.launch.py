"""Launch the custom autonomous navigation pipeline without localization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('inno_autonav')
    drive_share = get_package_share_directory('inno_drive_bridge')
    config_file = os.path.join(package_share, 'config', 'autonav_params.yaml')
    drive_config = os.path.join(drive_share, 'config', 'drive_params.yaml')
    use_serial = LaunchConfiguration('use_serial')
    serial_port = LaunchConfiguration('serial_port')
    use_wheel_odom_tf = LaunchConfiguration('use_wheel_odom_tf')
    map_yaml = LaunchConfiguration('map_yaml')
    semantic_yaml = LaunchConfiguration('semantic_yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_serial', default_value='false'),
            DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('use_wheel_odom_tf', default_value='false'),
            DeclareLaunchArgument(
                'map_yaml',
                default_value='/home/gosunwoo/fire_robot_rpi/maps/inno_map_nav.yaml',
            ),
            DeclareLaunchArgument(
                'semantic_yaml',
                default_value=(
                    '/home/gosunwoo/fire_robot_rpi/inno_jazzy_ws/'
                    'src/inno_autonav/config/semantic_points.yaml'
                ),
            ),
            Node(
                package='inno_autonav',
                executable='planning_grid_publisher',
                name='planning_grid_publisher',
                parameters=[config_file, {'map_yaml': map_yaml}],
                output='screen',
            ),
            Node(
                package='inno_autonav',
                executable='dynamic_obstacle_layer',
                name='dynamic_obstacle_layer',
                parameters=[config_file],
                output='screen',
            ),
            Node(
                package='inno_autonav',
                executable='astar_replanner',
                name='astar_replanner',
                parameters=[config_file],
                output='screen',
            ),
            Node(
                package='inno_autonav',
                executable='skid_path_follower',
                name='skid_path_follower',
                parameters=[config_file],
                output='screen',
            ),
            Node(
                package='inno_autonav',
                executable='mission_commander',
                name='mission_commander',
                parameters=[config_file, {'semantic_yaml': semantic_yaml}],
                output='screen',
            ),
            Node(
                package='inno_drive_bridge',
                executable='cmdvel_to_esp32_serial',
                name='cmdvel_to_esp32_serial',
                parameters=[drive_config, {'serial_port': serial_port}],
                output='screen',
                emulate_tty=True,
                condition=IfCondition(use_serial),
            ),
            Node(
                package='inno_drive_bridge',
                executable='step_count_to_odom',
                name='step_count_to_odom',
                parameters=[
                    drive_config,
                    {
                        'publish_tf': True,
                        'odom_frame': 'odom',
                        'base_frame': 'base_link',
                    },
                ],
                output='screen',
                condition=IfCondition(use_wheel_odom_tf),
            ),
        ]
    )
