"""Capture one RViz Publish Point as an industrial machinery landmark."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'semantic_file',
                description='semantic waypoint/landmark YAML file',
            ),
            DeclareLaunchArgument(
                'machinery_name',
                default_value='INDUSTRIAL_MACHINERY',
                description='Name using letters, numbers, underscores, or hyphens',
            ),
            DeclareLaunchArgument(
                'description',
                default_value='factory_machinery',
                description='Human-readable machinery description',
            ),
            DeclareLaunchArgument(
                'timeout',
                default_value='300',
                description='Seconds to wait for one RViz Publish Point click',
            ),
            Node(
                package='inno_semantic_nav',
                executable='capture_landmark',
                name='capture_industrial_machinery',
                output='screen',
                emulate_tty=True,
                arguments=[
                    LaunchConfiguration('machinery_name'),
                    '--semantic-file',
                    LaunchConfiguration('semantic_file'),
                    '--category',
                    'industrial_machinery',
                    '--description',
                    LaunchConfiguration('description'),
                    '--timeout',
                    LaunchConfiguration('timeout'),
                ],
            ),
        ]
    )
