from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    scan_mode = LaunchConfiguration('scan_mode')
    frame_id = LaunchConfiguration('frame_id')

    c1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('sllidar_ros2'), '/launch/sllidar_c1_launch.py']
        ),
        launch_arguments={
            'serial_port': serial_port,
            'scan_mode': scan_mode,
            'frame_id': frame_id,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('scan_mode', default_value='Standard'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        c1_launch,
    ])
