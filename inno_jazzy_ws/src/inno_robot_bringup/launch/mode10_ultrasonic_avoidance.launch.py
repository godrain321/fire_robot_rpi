"""Standalone Mode 10: sonar trigger, LiDAR-checked turn, ESP32 motor output."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('inno_robot_bringup')
    drive = get_package_share_directory('inno_drive_bridge')
    arguments = [
        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('use_serial', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('cruise_speed_mps', default_value='0.06'),
        DeclareLaunchArgument('turn_speed_radps', default_value='0.35'),
    ]
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup + '/launch/lidar_only.launch.py'),
        launch_arguments={
            'start_lidar': L('start_lidar'),
            'serial_port': L('lidar_port'),
            'scan_topic': '/scan',
            'base_frame': 'base_link',
            'laser_frame': 'laser',
        }.items(),
    )
    serial = Node(
        package='inno_drive_bridge', executable='cmdvel_to_esp32_serial',
        name='cmdvel_to_esp32_serial', output='screen',
        parameters=[drive + '/config/drive_params.yaml',
                    {'serial_port': L('esp32_port')}],
        condition=IfCondition(L('use_serial')),
    )
    avoidance = Node(
        package='inno_drive_bridge', executable='mode10_ultrasonic_avoidance',
        name='mode10_ultrasonic_avoidance', output='screen',
        parameters=[{
            'auto_start': ParameterValue(L('auto_start'), value_type=bool),
            'cruise_speed_mps': ParameterValue(
                L('cruise_speed_mps'), value_type=float),
            'turn_speed_radps': ParameterValue(
                L('turn_speed_radps'), value_type=float),
        }],
    )
    return LaunchDescription(arguments + [lidar, serial, avoidance])
