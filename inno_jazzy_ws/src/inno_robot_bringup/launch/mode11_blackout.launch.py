"""Humble Mode 11: Mode 1 keyboard drive plus switchable RF2O/encoder-IMU pose."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from inno_robot_bringup.project_paths import project_path


def generate_launch_description():
    bringup = get_package_share_directory('inno_robot_bringup')
    drive = get_package_share_directory('inno_drive_bridge')
    args = [
        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data_raw'),
        DeclareLaunchArgument('encoder_topic', default_value='/wheel_encoder_ticks'),
        DeclareLaunchArgument('allow_virtual_step_counts', default_value='false'),
        DeclareLaunchArgument('left_encoder_sign', default_value='1'),
        DeclareLaunchArgument('right_encoder_sign', default_value='1'),
        DeclareLaunchArgument('imu_yaw_sign', default_value='1'),
        DeclareLaunchArgument('wheel_radius_m', default_value='0.04'),
        DeclareLaunchArgument('encoder_counts_per_rev', default_value='16384'),
        DeclareLaunchArgument('maximum_encoder_counts_per_sec', default_value='12000.0'),
        DeclareLaunchArgument('map_yaml', default_value=project_path(
            'maps', 'inno_map_raw.yaml')),
        DeclareLaunchArgument('use_serial', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_map', default_value='true'),
        DeclareLaunchArgument('auto_localization', default_value='true'),
        DeclareLaunchArgument('set_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
    ]
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup + '/launch/lidar_only.launch.py'),
        launch_arguments={
            'serial_port': L('lidar_port'),
            'scan_topic': '/scan',
            'base_frame': 'base_link',
            'laser_frame': 'laser',
        }.items(),
    )
    # Both RF2O and AMCL see only the gated scan. /scan remains visible in RViz.
    rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry', output='screen',
        parameters=[bringup + '/config/rf2o.yaml', {
            'laser_scan_topic': '/mode11/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
        }],
    )
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[{'yaml_filename': L('map_yaml')}],
        condition=IfCondition(L('use_map')),
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        output='screen',
        parameters=[bringup + '/config/amcl_lidar_only.yaml', {
            'scan_topic': '/mode11/scan',
            'tf_broadcast': False,
            'set_initial_pose': ParameterValue(
                L('set_initial_pose'), value_type=bool),
            'initial_pose.x': ParameterValue(
                L('initial_pose_x'), value_type=float),
            'initial_pose.y': ParameterValue(
                L('initial_pose_y'), value_type=float),
            'initial_pose.yaw': ParameterValue(
                L('initial_pose_yaw'), value_type=float),
        }],
        remappings=[('scan', '/mode11/scan')],
        condition=IfCondition(L('use_map')),
    )
    lifecycle = Node(
        package='inno_robot_bringup', executable='lifecycle_autostart',
        name='lifecycle_autostart_mode11', output='screen',
        parameters=[{'node_names': ['map_server', 'amcl']}],
        condition=IfCondition(L('use_map')),
    )
    auto_localization = Node(
        package='inno_robot_bringup', executable='auto_localization_supervisor',
        name='auto_localization_supervisor_mode11', output='screen',
        parameters=[{'scan_topic': '/mode11/scan'}],
        condition=IfCondition(PythonExpression([
            "'", L('use_map'), "'.lower() == 'true' and '",
            L('auto_localization'), "'.lower() == 'true'",
        ])),
    )
    selector = Node(
        package='inno_robot_bringup', executable='mode11_localization',
        name='mode11_localization', output='screen',
        parameters=[{'imu_topic': L('imu_topic'),
                     'encoder_topic': L('encoder_topic'),
                     'allow_virtual_step_counts': ParameterValue(
                         L('allow_virtual_step_counts'), value_type=bool),
                     'require_map_alignment': ParameterValue(
                         L('use_map'), value_type=bool),
                     'imu_yaw_sign': ParameterValue(
                         L('imu_yaw_sign'), value_type=int),
                     'left_sign': ParameterValue(
                         L('left_encoder_sign'), value_type=int),
                     'right_sign': ParameterValue(
                         L('right_encoder_sign'), value_type=int),
                     'wheel_radius': ParameterValue(
                         L('wheel_radius_m'), value_type=float),
                     'motor_full_steps_per_rev': ParameterValue(
                         L('encoder_counts_per_rev'), value_type=int),
                     'microsteps': 1,
                     'maximum_steps_per_sec': ParameterValue(
                         L('maximum_encoder_counts_per_sec'), value_type=float)}],
    )
    map_path = Node(
        package='inno_robot_bringup', executable='tf_to_path',
        name='mode11_tf_to_path', output='screen',
        parameters=[{'fixed_frame': 'map', 'base_frame': 'base_link',
                     'path_topic': '/mode11/path', 'max_points': 10000}],
        condition=IfCondition(L('use_map')),
    )
    odom_path = Node(
        package='inno_robot_bringup', executable='tf_to_path',
        name='mode11_tf_to_path', output='screen',
        parameters=[{'fixed_frame': 'odom', 'base_frame': 'base_link',
                     'path_topic': '/mode11/path', 'max_points': 10000}],
        condition=UnlessCondition(L('use_map')),
    )
    imu_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_imu_tf', output='screen',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
    )
    keyboard = Node(
        package='inno_drive_bridge', executable='keyboard_cmdvel_demo',
        name='keyboard_cmdvel_demo', output='screen', emulate_tty=True,
        parameters=[drive + '/config/drive_params.yaml', {
            'mode11_blackout_key': True,
        }],
    )
    mux = Node(
        package='inno_drive_bridge', executable='cmd_vel_mode_mux',
        name='cmd_vel_mode_mux', output='screen',
    )
    serial = Node(
        package='inno_drive_bridge', executable='cmdvel_to_esp32_serial',
        name='cmdvel_to_esp32_serial', output='screen',
        parameters=[drive + '/config/drive_params.yaml',
                    {'serial_port': L('esp32_port')}],
        condition=IfCondition(L('use_serial')),
    )
    map_rviz = Node(
        package='rviz2', executable='rviz2', name='mode11_rviz',
        output='screen', arguments=['-d', bringup + '/rviz/mode11.rviz'],
        condition=IfCondition(PythonExpression([
            "'", L('use_rviz'), "'.lower() == 'true' and '",
            L('use_map'), "'.lower() == 'true'",
        ])),
    )
    odom_rviz = Node(
        package='rviz2', executable='rviz2', name='mode11_rviz',
        output='screen', arguments=['-d', bringup + '/rviz/mode11_odom.rviz'],
        condition=IfCondition(PythonExpression([
            "'", L('use_rviz'), "'.lower() == 'true' and '",
            L('use_map'), "'.lower() != 'true'",
        ])),
    )
    return LaunchDescription(args + [
        lidar, rf2o, map_server, amcl, lifecycle, auto_localization,
        selector, map_path, odom_path, imu_tf,
        keyboard, mux, serial, map_rviz, odom_rviz,
    ])
