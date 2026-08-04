"""LiDAR-only localization: RF2O odom plus slam_toolbox pose-graph correction."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('inno_robot_bringup')
    defaults = {
        'serial_port': '/dev/ttyUSB0', 'serial_baudrate': '460800',
        'posegraph': '/home/gosunwoo/fire_robot_rpi/inno_jazzy_ws/maps/inno_posegraph_20260717_112806',
        'localization_params': share + '/config/slam_toolbox_localization.yaml',
        'use_rviz': 'true',
    }
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share + '/launch/lidar_only.launch.py'),
        launch_arguments={
            'start_lidar': 'true', 'serial_port': L('serial_port'),
            'serial_baudrate': L('serial_baudrate'), 'publish_static_tf': 'true',
            'scan_topic': '/scan', 'base_frame': 'base_link', 'laser_frame': 'laser',
        }.items(),
    )
    rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry', output='screen',
        parameters=[share + '/config/rf2o.yaml'],
    )
    localization = Node(
        package='slam_toolbox', executable='localization_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[L('localization_params'), {'map_file_name': L('posegraph')}],
    )
    trail = Node(package='inno_robot_bringup', executable='tf_to_path',
                 name='lidar_path', output='screen')
    return LaunchDescription(args + [lidar, rf2o, localization, trail])
