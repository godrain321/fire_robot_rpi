"""C4001 UART driver and conservative mobility classifier."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('inno_mmwave')
    args = [
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('configure_sensor', default_value='true'),
        DeclareLaunchArgument('assist_check_sec', default_value='10.0'),
    ]

    driver = Node(
        package='inno_mmwave',
        executable='c4001_node',
        name='c4001_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            share + '/config/c4001.yaml',
            {
                'serial_port': L('serial_port'),
                'configure_on_start': ParameterValue(
                    L('configure_sensor'), value_type=bool
                ),
            },
        ],
    )
    mobility = Node(
        package='inno_mmwave',
        executable='mmwave_mobility',
        name='mmwave_mobility',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'assist_check_sec': ParameterValue(
                    L('assist_check_sec'), value_type=float
                ),
            },
        ],
    )
    return LaunchDescription(args + [driver, mobility])
