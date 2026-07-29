from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bags_dir = os.path.expanduser('~/bags')
    os.makedirs(bags_dir, exist_ok=True)
    default_output = os.path.join(
        bags_dir,
        f"rpi_lidar_mount_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    output = LaunchConfiguration('output')

    recorder = Node(
        package='rosbag2_transport',
        executable='record',
        name='lidar_bag_recorder',
        arguments=['-o', output, '/scan', '/tf', '/tf_static'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('output', default_value=default_output),
        recorder,
    ])
