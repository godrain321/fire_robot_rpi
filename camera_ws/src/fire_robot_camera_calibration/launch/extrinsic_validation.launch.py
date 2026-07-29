"""Launch calibrated streams, static TF, and projection validation."""

from pathlib import Path

from fire_robot_camera_calibration.launch_helpers import camera_ros_node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the extrinsic validation workflow."""
    default_camera_info = str(
        Path.home() / '.ros' / 'camera_info' / 'camera.yaml'
    )
    default_extrinsic = str(
        Path.home()
        / 'fire_robot_calibration'
        / 'lidar_camera_extrinsic.yaml'
    )
    default_screenshots = str(
        Path.home()
        / 'fire_robot_calibration'
        / 'validation'
    )

    arguments = [
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('camera_namespace', default_value=''),
        DeclareLaunchArgument('camera', default_value='0'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_optical_frame',
        ),
        DeclareLaunchArgument(
            'camera_info_path',
            default_value=default_camera_info,
        ),
        DeclareLaunchArgument(
            'raw_image_topic',
            default_value='/camera/image_raw',
        ),
        DeclareLaunchArgument(
            'input_transport',
            default_value='raw',
        ),
        DeclareLaunchArgument(
            'rectified_image_topic',
            default_value='/camera/image_rect',
        ),
        DeclareLaunchArgument(
            'rectified_info_topic',
            default_value='/camera/camera_info_rect',
        ),
        DeclareLaunchArgument('balance', default_value='0.3'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='laser_frame',
        ),
        DeclareLaunchArgument(
            'extrinsic_path',
            default_value=default_extrinsic,
        ),
        DeclareLaunchArgument(
            'screenshot_dir',
            default_value=default_screenshots,
        ),
        DeclareLaunchArgument(
            'start_lidar',
            default_value='true',
            description='Start the RPLIDAR driver in this launch',
        ),
        DeclareLaunchArgument(
            'lidar_launch_package',
            default_value='sllidar_ros2',
        ),
        DeclareLaunchArgument(
            'lidar_launch_file',
            default_value='sllidar_a1_launch.py',
        ),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument(
            'lidar_serial_baudrate',
            default_value='115200',
            description='RPLIDAR A1: 115200; change for another model',
        ),
    ]

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(
                        LaunchConfiguration('lidar_launch_package')
                    ),
                    'launch',
                    LaunchConfiguration('lidar_launch_file'),
                ]
            )
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate': LaunchConfiguration(
                'lidar_serial_baudrate'
            ),
            'frame_id': LaunchConfiguration('lidar_frame'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_lidar')),
    )

    rectifier = Node(
        package='fire_robot_camera_calibration',
        executable='rectify_camera',
        output='screen',
        parameters=[
            {
                'input_topic': LaunchConfiguration('raw_image_topic'),
                'input_transport': LaunchConfiguration(
                    'input_transport'
                ),
                'output_image_topic': LaunchConfiguration(
                    'rectified_image_topic'
                ),
                'output_camera_info_topic': LaunchConfiguration(
                    'rectified_info_topic'
                ),
                'camera_info_path': LaunchConfiguration(
                    'camera_info_path'
                ),
                'frame_id': LaunchConfiguration('camera_frame'),
                'balance': ParameterValue(
                    LaunchConfiguration('balance'),
                    value_type=float,
                ),
            }
        ],
    )

    static_transform = Node(
        package='fire_robot_camera_calibration',
        executable='static_tf_publisher',
        output='screen',
        parameters=[
            {
                'extrinsic_path': LaunchConfiguration(
                    'extrinsic_path'
                ),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'lidar_frame': LaunchConfiguration('lidar_frame'),
            }
        ],
    )

    overlay = Node(
        package='fire_robot_camera_calibration',
        executable='tf_overlay',
        output='screen',
        parameters=[
            {
                'image_topic': LaunchConfiguration(
                    'rectified_image_topic'
                ),
                'camera_info_topic': LaunchConfiguration(
                    'rectified_info_topic'
                ),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'lidar_frame': LaunchConfiguration('lidar_frame'),
                'screenshot_dir': LaunchConfiguration(
                    'screenshot_dir'
                ),
            }
        ],
    )

    return LaunchDescription(
        arguments
        + [
            camera_ros_node(),
            lidar,
            rectifier,
            static_transform,
            overlay,
        ]
    )
