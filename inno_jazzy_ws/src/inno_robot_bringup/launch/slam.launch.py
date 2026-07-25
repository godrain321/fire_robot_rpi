from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("inno_robot_bringup")
    slam_share = get_package_share_directory("slam_toolbox")

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            bringup_share + "/launch/sensors.launch.py"
        ),
        launch_arguments={
            "use_camera": LaunchConfiguration("use_camera"),
            "lidar_port": LaunchConfiguration("lidar_port"),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            slam_share + "/launch/online_async_launch.py"
        ),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": bringup_share
            + "/config/slam_toolbox.yaml",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", bringup_share + "/rviz/slam.rviz"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("use_camera", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            sensors,
            slam,
            rviz,
        ]
    )
