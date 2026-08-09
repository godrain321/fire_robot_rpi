"""One-command field test: LiDAR localization, path, keyboard, A*, follower, ESP32."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from inno_robot_bringup.project_paths import project_path


def generate_launch_description():
    bringup = get_package_share_directory('inno_robot_bringup')
    drive = get_package_share_directory('inno_drive_bridge')
    autonav = get_package_share_directory('inno_autonav')
    mmwave = get_package_share_directory('inno_mmwave')
    thermal_dir = project_path(
        'mlx90640', 'demo codes', 'mlx90640', 'python'
    )
    args = [
        DeclareLaunchArgument(
            'esp32_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                'de2033aed827f0119bb79ad8346f00fe-if00-port0'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                '4a5b9018526eef11bff6e0c2c169b110-if00-port0'
            ),
        ),
        DeclareLaunchArgument('mmwave_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('mmwave_configure_sensor', default_value='true'),
        DeclareLaunchArgument('assist_check_sec', default_value='10.0'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=project_path('maps', 'inno_map_raw.yaml'),
        ),
        DeclareLaunchArgument(
            'planning_map_yaml',
            default_value=project_path('maps', 'inno_map_nav.yaml'),
        ),
        DeclareLaunchArgument(
            'waypoint_file',
            default_value=project_path('maps', 'waypoint_queue_latest.yaml'),
        ),
        # One speed pair applies identically to MODE 1, MODE 2, and MODE 3.
        DeclareLaunchArgument('drive_speed', default_value='0.20'),
        DeclareLaunchArgument('turn_speed', default_value='0.35'),
        DeclareLaunchArgument('use_dynamic_obstacles', default_value='true'),
        DeclareLaunchArgument('start_thermal_viewer', default_value='true'),
    ]
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup + '/launch/lidar_amcl_localization.launch.py'),
        launch_arguments={
            'serial_port': L('lidar_port'), 'map_yaml': L('map_yaml'),
            'node_output': 'log',
        }.items(),
    )
    mmwave_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mmwave + '/launch/mmwave_bringup.launch.py'),
        launch_arguments={
            'serial_port': L('mmwave_port'),
            'configure_sensor': L('mmwave_configure_sensor'),
            'assist_check_sec': L('assist_check_sec'),
            'node_output': 'log',
        }.items(),
    )
    status_console = Node(
        package='inno_mmwave', executable='mmwave_status_console',
        name='mmwave_status_console', output='screen', emulate_tty=True,
        output_format='{line}',
    )
    rescue_led = Node(
        package='inno_mmwave', executable='mmwave_presence_led',
        name='rescuee_led_bank', output='log', emulate_tty=True,
        parameters=[{
            'gpio_chip': 4,
            'gpio_lines': [17, 27, 22, 23, 24],
            'active_high': True,
            'trigger_topic': '/victim_detected',
            'reset_on_false': True,
        }],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(autonav + '/launch/autonav_demo.launch.py'),
        launch_arguments={
            'use_serial': 'false', 'use_wheel_odom_tf': 'false',
            'map_yaml': L('planning_map_yaml'),
            'max_linear_speed': L('drive_speed'),
            'max_angular_speed': L('turn_speed'),
            'use_dynamic_obstacles': L('use_dynamic_obstacles'),
            'node_output': 'log',
        }.items(),
    )
    keyboard = Node(
        package='inno_drive_bridge', executable='keyboard_cmdvel_demo',
        name='keyboard_cmdvel_demo', output='log', emulate_tty=True,
        parameters=[
            drive + '/config/drive_params.yaml',
            {
                'linear_speed': ParameterValue(
                    L('drive_speed'), value_type=float
                ),
                'angular_speed': ParameterValue(
                    L('turn_speed'), value_type=float
                ),
            },
        ],
    )
    mux = Node(package='inno_drive_bridge', executable='cmd_vel_mode_mux',
               name='cmd_vel_mode_mux', output='log')
    serial = Node(
        package='inno_drive_bridge', executable='cmdvel_to_esp32_serial',
        name='cmdvel_to_esp32_serial', output='log',
        parameters=[drive + '/config/drive_params.yaml', {'serial_port': L('esp32_port')}],
    )
    waypoint_queue = Node(
        package='inno_autonav', executable='waypoint_queue',
        name='waypoint_queue', output='log',
        parameters=[{
            'load_file': L('waypoint_file'),
            'save_file': L('waypoint_file'),
        }],
    )
    robot_heading = Node(
        package='inno_robot_bringup', executable='tf_heading_marker',
        name='tf_heading_marker', output='log',
        parameters=[{
            'fixed_frame': 'map',
            'base_frame': 'base_link',
            'arrow_length_m': 0.55,
            'arrow_width_m': 0.12,
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', bringup + '/rviz/inno_slam.rviz'],
    )
    thermal_viewer = ExecuteProcess(
        cmd=['python3', thermal_dir + '/mlx90640.py'],
        cwd=thermal_dir,
        name='mlx90640_viewer',
        output='log',
        condition=IfCondition(L('start_thermal_viewer')),
    )
    return LaunchDescription(
        args + [
            localization, mmwave_bringup, status_console, rescue_led,
            navigation, keyboard,
            mux, serial, waypoint_queue, robot_heading, rviz, thermal_viewer,
        ]
    )
