"""Saved PGM/YAML localization using LiDAR RF2O odometry and AMCL."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from inno_robot_bringup.project_paths import project_path


def generate_launch_description():
    share = get_package_share_directory('inno_robot_bringup')
    args = [
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('serial_baudrate', default_value='460800'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=project_path('maps', 'inno_map_raw.yaml'),
        ),
        DeclareLaunchArgument(
            'amcl_params', default_value=share + '/config/amcl_lidar_only.yaml'
        ),
        DeclareLaunchArgument('set_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
    ]
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share + '/launch/lidar_only.launch.py'),
        launch_arguments={
            'start_lidar': L('start_lidar'), 'serial_port': L('serial_port'),
            'serial_baudrate': L('serial_baudrate'),
            'publish_static_tf': L('start_lidar'),
            'scan_topic': '/scan', 'base_frame': 'base_link', 'laser_frame': 'laser',
        }.items(),
    )
    rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry', output='screen',
        parameters=[share + '/config/rf2o.yaml'],
        arguments=['--ros-args', '--log-level', 'warn'],
        condition=IfCondition(L('start_lidar')),
    )
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[{'yaml_filename': L('map_yaml')}],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[
            L('amcl_params'),
            {
                'set_initial_pose': ParameterValue(
                    L('set_initial_pose'), value_type=bool
                ),
                'initial_pose.x': ParameterValue(
                    L('initial_pose_x'), value_type=float
                ),
                'initial_pose.y': ParameterValue(
                    L('initial_pose_y'), value_type=float
                ),
                'initial_pose.yaw': ParameterValue(
                    L('initial_pose_yaw'), value_type=float
                ),
            },
        ],
    )
    lifecycle = Node(
        package='inno_robot_bringup', executable='lifecycle_autostart',
        name='lifecycle_autostart_localization', output='screen',
        parameters=[{'node_names': ['map_server', 'amcl']}],
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
