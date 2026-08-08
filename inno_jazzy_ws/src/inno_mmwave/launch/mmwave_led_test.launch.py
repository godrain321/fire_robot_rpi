"""Run the real C4001 driver and a latched ten-LED bank."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('inno_mmwave')
    args = [
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('configure_sensor', default_value='true'),
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('gpio_chip', default_value='4'),
    ]
    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            share + '/launch/mmwave_bringup.launch.py'
        ),
        condition=IfCondition(L('start_driver')),
        launch_arguments={
            'serial_port': L('serial_port'),
            'configure_sensor': L('configure_sensor'),
        }.items(),
    )
    led = Node(
        package='inno_mmwave',
        executable='mmwave_presence_led',
        name='mmwave_presence_led',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'gpio_chip': ParameterValue(L('gpio_chip'), value_type=int),
            'gpio_lines': [17, 27, 22, 23, 24, 25, 5, 6, 16, 26],
            'active_high': True,
        }],
    )
    return LaunchDescription(args + [backend, led])
