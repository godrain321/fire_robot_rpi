"""Motor-disabled RViz preview for Mode 5 with EXIT1 blocked."""

from ament_index_python.packages import get_package_share_directory
from inno_robot_bringup.project_paths import project_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration as L

from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('inno_robot_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=project_path('maps', 'inno_map_nav.yaml'),
        ),
        DeclareLaunchArgument(
            'waypoint_file',
            default_value=project_path(
                'docs', 'full_map_waypoints_1m_numbered.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'semantic_file',
            default_value=project_path(
                'inno_jazzy_ws', 'src', 'inno_autonav', 'config',
                'semantic_points.yaml'
            ),
        ),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        Node(
            package='inno_autonav',
            executable='mode5_route_preview',
            name='mode5_route_preview',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'map_yaml': L('map_yaml'),
                'waypoint_file': L('waypoint_file'),
                'semantic_file': L('semantic_file'),
                'blocked_exit_ids': ['EXIT1'],
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='mode5_route_preview_rviz',
            output='screen',
            arguments=[
                '-d', bringup + '/rviz/inno_slam.rviz',
                '--ros-args', '-p', 'use_sim_time:=false',
            ],
        ),
    ])
