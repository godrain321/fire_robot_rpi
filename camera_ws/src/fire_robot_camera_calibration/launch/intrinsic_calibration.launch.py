"""Launch camera_ros and the ROS 2 monocular calibration GUI."""

from pathlib import Path

from fire_robot_camera_calibration.launch_helpers import camera_ros_node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Create the live intrinsic calibration workflow."""
    default_camera_info = str(
        Path.home() / '.ros' / 'camera_info' / 'camera.yaml'
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
            'image_topic',
            default_value='/camera/image_raw',
        ),
        DeclareLaunchArgument(
            'camera_service_namespace',
            default_value='/camera',
        ),
        DeclareLaunchArgument(
            'board_size',
            default_value='8x9',
            description='Checkerboard inner corners, not square count',
        ),
        DeclareLaunchArgument(
            'square_size',
            default_value='0.07',
            description='Checkerboard square edge length in metres',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='imx708_wide__base_axi_pcie_120000_rp1_i2c_80000_imx708_1a_1280x720',
        ),
        DeclareLaunchArgument(
            'use_fisheye_flags',
            default_value='true',
            description=(
                'Adds stable fisheye optimizer flags. Select fisheye with '
                'the GUI camera-type slider.'
            ),
        ),
    ]

    calibrator = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='cameracalibrator',
        output='screen',
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'GALLIUM_DRIVER': 'llvmpipe',
        },
        arguments=[
            '--size',
            LaunchConfiguration('board_size'),
            '--square',
            LaunchConfiguration('square_size'),
            '--camera_name',
            LaunchConfiguration('camera_name'),
            '--no-service-check',
        ],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            (
                'camera',
                LaunchConfiguration('camera_service_namespace'),
            ),
        ],
        condition=UnlessCondition(
            LaunchConfiguration('use_fisheye_flags')
        ),
    )
    fisheye_hint = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='cameracalibrator_fisheye',
        output='screen',
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'GALLIUM_DRIVER': 'llvmpipe',
        },
        arguments=[
            '--size',
            LaunchConfiguration('board_size'),
            '--square',
            LaunchConfiguration('square_size'),
            '--camera_name',
            LaunchConfiguration('camera_name'),
            '--no-service-check',
            '--fisheye-recompute-extrinsicsts',
            '--fisheye-fix-skew',
        ],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            (
                'camera',
                LaunchConfiguration('camera_service_namespace'),
            ),
        ],
        condition=IfCondition(
            LaunchConfiguration('use_fisheye_flags')
        ),
    )
    return LaunchDescription(
        arguments + [camera_ros_node(), calibrator, fisheye_hint]
    )
