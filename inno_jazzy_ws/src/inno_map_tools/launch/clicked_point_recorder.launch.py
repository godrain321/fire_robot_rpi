"""Launch the RViz clicked-point YAML recorder."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from inno_map_tools.project_paths import project_path


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('inno_map_tools')
    default_config = os.path.join(package_share, 'config', 'map_tools_params.yaml')
    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file', default_value=default_config),
            DeclareLaunchArgument(
                'output_file',
                default_value=project_path('maps', 'clicked_points_debug.yaml'),
            ),
            Node(
                package='inno_map_tools',
                executable='save_clicked_points',
                name='map_tools',
                output='screen',
                parameters=[
                    LaunchConfiguration('config_file'),
                    {'output_file': LaunchConfiguration('output_file')},
                ],
            ),
        ]
    )
