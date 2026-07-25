from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config = (
        get_package_share_directory("inno_camera_tools")
        + "/config/camera_module_3.yaml"
    )

    camera = LaunchConfiguration("camera")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    pixel_format = LaunchConfiguration("format")
    frame_id = LaunchConfiguration("frame_id")
    rectify = LaunchConfiguration("rectify")

    components = [
        ComposableNode(
            package="camera_ros",
            plugin="camera::CameraNode",
            name="camera",
            namespace="camera",
            parameters=[
                config,
                {
                    "camera": camera,
                    "width": width,
                    "height": height,
                    "format": pixel_format,
                    "frame_id": frame_id,
                },
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        ComposableNode(
            package="image_proc",
            plugin="image_proc::RectifyNode",
            name="rectify",
            namespace="camera",
            condition=IfCondition(rectify),
            remappings=[
                ("image", "image_raw"),
                ("camera_info", "camera_info"),
                ("image_rect", "image_rect"),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
    ]

    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=components,
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera", default_value="0"),
            DeclareLaunchArgument("width", default_value="1280"),
            DeclareLaunchArgument("height", default_value="720"),
            DeclareLaunchArgument("format", default_value="RGB888"),
            DeclareLaunchArgument(
                "frame_id", default_value="camera_optical_frame"
            ),
            DeclareLaunchArgument(
                "rectify",
                default_value="false",
                description="Enable after intrinsic camera calibration.",
            ),
            container,
        ]
    )
