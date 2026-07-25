from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def static_tf(name, parent, child, prefix):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", LaunchConfiguration(prefix + "_x"),
            "--y", LaunchConfiguration(prefix + "_y"),
            "--z", LaunchConfiguration(prefix + "_z"),
            "--roll", LaunchConfiguration(prefix + "_roll"),
            "--pitch", LaunchConfiguration(prefix + "_pitch"),
            "--yaw", LaunchConfiguration(prefix + "_yaw"),
            "--frame-id", parent,
            "--child-frame-id", child,
        ],
    )


def generate_launch_description():
    bringup_share = get_package_share_directory("inno_robot_bringup")
    camera_share = get_package_share_directory("inno_camera_tools")
    sensor_config = bringup_share + "/config/sensors.yaml"

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            camera_share + "/launch/camera_module_3.launch.py"
        ),
        condition=IfCondition(LaunchConfiguration("use_camera")),
    )

    lidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        parameters=[
            sensor_config,
            {"serial_port": LaunchConfiguration("lidar_port")},
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_lidar")),
    )

    rf2o = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        parameters=[sensor_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rf2o")),
    )

    transform_defaults = {
        "laser_x": "0.0",
        "laser_y": "0.0",
        "laser_z": "0.30",
        "laser_roll": "0.0",
        "laser_pitch": "0.0",
        "laser_yaw": "0.0",
        "camera_x": "0.10",
        "camera_y": "0.0",
        "camera_z": "0.20",
        "camera_roll": "0.0",
        "camera_pitch": "0.0",
        "camera_yaw": "0.0",
    }

    args = [
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_rf2o", default_value="true"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
    ]
    for prefix in ("laser", "camera"):
        for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
            name = prefix + "_" + axis
            args.append(
                DeclareLaunchArgument(
                    name, default_value=transform_defaults[name]
                )
            )

    return LaunchDescription(
        args
        + [
            LogInfo(
                msg=(
                    "WARNING: camera/laser extrinsics are uncalibrated. "
                    "Using provisional transforms: laser=(0, 0, 0.30 m), "
                    "camera=(0.10, 0, 0.20 m)."
                )
            ),
            static_tf(
                "base_to_laser_tf", "base_link", "laser_frame", "laser"
            ),
            static_tf(
                "base_to_camera_tf", "base_link", "camera_link", "camera"
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_to_optical_tf",
                arguments=[
                    "--roll", "-1.57079632679",
                    "--pitch", "0.0",
                    "--yaw", "-1.57079632679",
                    "--frame-id", "camera_link",
                    "--child-frame-id", "camera_optical_frame",
                ],
            ),
            camera_launch,
            lidar,
            rf2o,
        ]
    )
