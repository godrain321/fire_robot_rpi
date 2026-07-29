"""Launch helpers shared by the calibration workflows."""

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def camera_ros_node():
    """Build the optional libcamera-based Raspberry Pi camera node."""
    camera_info_url = ParameterValue(
        [
            TextSubstitution(text='file://'),
            LaunchConfiguration('camera_info_path'),
        ],
        value_type=str,
    )
    return Node(
        package='camera_ros',
        executable='camera_node',
        namespace=LaunchConfiguration('camera_namespace'),
        name='camera',
        output='screen',
        parameters=[
            {
                # Leave this unforced: camera_ros accepts either index (0)
                # or a full libcamera device name (string).
                'camera': LaunchConfiguration('camera'),
                'width': ParameterValue(
                    LaunchConfiguration('width'),
                    value_type=int,
                ),
                'height': ParameterValue(
                    LaunchConfiguration('height'),
                    value_type=int,
                ),
                'frame_id': LaunchConfiguration('camera_frame'),
                'camera_info_url': camera_info_url,
            }
        ],
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )
