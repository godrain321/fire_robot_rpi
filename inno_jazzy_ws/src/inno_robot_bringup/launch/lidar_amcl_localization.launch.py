"""Saved PGM/YAML localization using LiDAR RF2O odometry and AMCL."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('inno_robot_bringup')
    args = [
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('serial_baudrate', default_value='460800'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value='/home/gosunwoo/fire_robot_rpi/maps/inno_map_raw.yaml',
        ),
        DeclareLaunchArgument(
            'amcl_params', default_value=share + '/config/amcl_lidar_only.yaml'
        ),
    ]
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
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[{'yaml_filename': L('map_yaml')}],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[L('amcl_params')],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}],
    )
    trail = Node(package='inno_robot_bringup', executable='tf_to_path',
                 name='lidar_path', output='screen')
    tf_bridge = Node(
        package='inno_robot_bringup', executable='amcl_pose_tf_bridge',
        name='amcl_pose_tf_bridge', output='screen',
    )
    return LaunchDescription(
        args + [lidar, rf2o, map_server, amcl, lifecycle, tf_bridge, trail]
    )
