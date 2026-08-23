"""Launch guided raw-image capture for the offline Rational workflow."""

from pathlib import Path

from fire_robot_camera_calibration.launch_helpers import camera_ros_node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create camera and guided checkerboard capture nodes."""
    default_camera_info = str(
        Path.home() / '.ros' / 'camera_info' / 'camera.yaml'
    )
    default_output = str(
        Path.home()
        / 'fire_robot_calibration'
        / 'intrinsic_images'
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
            'transport',
            default_value='raw',
            description='raw or compressed',
        ),
        DeclareLaunchArgument('board_cols', default_value='8'),
        DeclareLaunchArgument('board_rows', default_value='9'),
        DeclareLaunchArgument('max_images', default_value='80'),
        DeclareLaunchArgument('preview', default_value='true'),
        DeclareLaunchArgument('auto_save', default_value='true'),
        DeclareLaunchArgument(
            'blur_threshold',
            default_value='35.0',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value=default_output,
        ),
    ]

    capture = Node(
        package='fire_robot_camera_calibration',
        executable='guided_capture',
        name='guided_checkerboard_capture',
        output='screen',
        parameters=[
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'transport': LaunchConfiguration('transport'),
                'output_dir': LaunchConfiguration('output_dir'),
                'board_cols': ParameterValue(
                    LaunchConfiguration('board_cols'),
                    value_type=int,
                ),
                'board_rows': ParameterValue(
                    LaunchConfiguration('board_rows'),
                    value_type=int,
                ),
                'max_images': ParameterValue(
                    LaunchConfiguration('max_images'),
                    value_type=int,
                ),
                'preview': ParameterValue(
                    LaunchConfiguration('preview'),
                    value_type=bool,
                ),
                'auto_save': ParameterValue(
                    LaunchConfiguration('auto_save'),
                    value_type=bool,
                ),
                'blur_threshold': ParameterValue(
                    LaunchConfiguration('blur_threshold'),
                    value_type=float,
                ),
            }
        ],
    )

    shutdown_when_capture_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=capture,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason='guided intrinsic capture exited',
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        arguments
        + [shutdown_when_capture_exits, camera_ros_node(), capture]
    )
