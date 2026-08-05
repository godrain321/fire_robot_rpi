import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('inno_drive_bridge')
    config_file = os.path.join(package_share, 'config', 'drive_params.yaml')
    serial_port = LaunchConfiguration('serial_port')
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='ESP32 USB serial device, for example /dev/ttyUSB0 or /dev/ttyACM0',
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.08',
            description='Keyboard forward/reverse speed in m/s',
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.35',
            description='Keyboard turn speed in rad/s',
        ),
        Node(
            package='inno_drive_bridge',
            executable='keyboard_cmdvel_demo',
            name='keyboard_cmdvel_demo',
            parameters=[
                config_file,
                {
                    'linear_speed': ParameterValue(
                        linear_speed,
                        value_type=float,
                    ),
                    'angular_speed': ParameterValue(
                        angular_speed,
                        value_type=float,
                    ),
                },
            ],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='inno_drive_bridge',
            executable='cmd_vel_mode_mux',
            name='cmd_vel_mode_mux',
            parameters=[config_file],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='inno_drive_bridge',
            executable='cmdvel_to_esp32_serial',
            name='cmdvel_to_esp32_serial',
            parameters=[config_file, {'serial_port': serial_port}],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='inno_drive_bridge',
            executable='step_count_to_odom',
            name='step_count_to_odom',
            parameters=[config_file],
            output='screen',
            emulate_tty=True,
        ),
    ])
