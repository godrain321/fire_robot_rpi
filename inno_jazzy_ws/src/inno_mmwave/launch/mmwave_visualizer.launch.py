"""Standalone C4001/SEN0610 visualization demo."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('inno_mmwave')
    args = [
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('configure_sensor', default_value='true'),
        DeclareLaunchArgument('assist_check_sec', default_value='10.0'),
        DeclareLaunchArgument('start_driver', default_value='true'),
    ]

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            share + '/launch/mmwave_bringup.launch.py'
        ),
        condition=IfCondition(L('start_driver')),
        launch_arguments={
            'serial_port': L('serial_port'),
            'configure_sensor': L('configure_sensor'),
            'assist_check_sec': L('assist_check_sec'),
        }.items(),
    )
    gui = Node(
        package='inno_mmwave',
        executable='mmwave_gui',
        name='mmwave_visualizer',
        output='screen',
        emulate_tty=True,
    )
    return LaunchDescription(args + [backend, gui])
