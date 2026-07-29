import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('inno_bringup')
    config_path = os.path.join(package_share, 'config', 'lidar_mount.yaml')
    with open(config_path, encoding='utf-8') as config_file:
        mount = yaml.safe_load(config_file)

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'lidar_only.launch.py')
        ),
        launch_arguments={'frame_id': str(mount['laser_frame'])}.items(),
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_transform_publisher',
        arguments=[
            '--x', str(mount['lidar_x']),
            '--y', str(mount['lidar_y']),
            '--z', str(mount['lidar_z']),
            '--roll', str(mount['lidar_roll']),
            '--pitch', str(mount['lidar_pitch']),
            '--yaw', str(mount['lidar_yaw']),
            '--frame-id', str(mount['base_frame']),
            '--child-frame-id', str(mount['laser_frame']),
        ],
        output='screen',
    )

    return LaunchDescription([lidar, static_tf])
