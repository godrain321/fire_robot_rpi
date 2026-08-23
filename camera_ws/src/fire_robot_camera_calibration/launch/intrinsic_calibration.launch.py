"""Launch camera_ros and fixed-model monocular calibration GUI."""

from pathlib import Path

from fire_robot_camera_calibration.launch_helpers import camera_ros_node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            default_value='imx708_wide_1280x720',
        ),
        DeclareLaunchArgument(
            'camera_model',
            default_value='fisheye',
            description=(
                'Locked calibration model: fisheye for IMX708 Wide, or '
                'pinhole for a standard field-of-view lens.'
            ),
        ),
        DeclareLaunchArgument(
            'max_chessboard_speed',
            default_value='2.0',
            description='Reject checkerboard samples moving faster (px/frame)',
        ),
    ]

    calibrator = Node(
        package='fire_robot_camera_calibration',
        executable='fixed_model_cameracalibrator',
        name='fixed_model_cameracalibrator',
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
            '--camera-name',
            LaunchConfiguration('camera_name'),
            '--camera-model',
            LaunchConfiguration('camera_model'),
            '--max-chessboard-speed',
            LaunchConfiguration('max_chessboard_speed'),
        ],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            (
                'camera',
                LaunchConfiguration('camera_service_namespace'),
            ),
        ],
    )
    return LaunchDescription(arguments + [camera_ros_node(), calibrator])
