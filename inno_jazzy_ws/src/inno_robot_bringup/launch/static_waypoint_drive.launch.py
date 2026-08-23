"""LiDAR/AMCL plus static-map waypoint driving, with all hazard layers off."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node

from inno_robot_bringup.project_paths import project_path


def generate_launch_description():
    bringup = get_package_share_directory("inno_robot_bringup")
    autonav = get_package_share_directory("inno_autonav")
    args = [
        DeclareLaunchArgument("use_serial", default_value="false"),
        DeclareLaunchArgument("esp32_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument(
            "map_yaml", default_value=project_path("maps", "inno_map_raw.yaml")
        ),
        DeclareLaunchArgument(
            "planning_map_yaml",
            default_value=project_path("maps", "inno_map_nav.yaml"),
        ),
        DeclareLaunchArgument(
            "waypoint_file",
            default_value=project_path("maps", "waypoint_queue_latest.yaml"),
        ),
        DeclareLaunchArgument(
            "semantic_yaml",
            default_value=autonav + "/config/semantic_points.yaml",
        ),
        DeclareLaunchArgument("max_linear_speed", default_value="0.06"),
        DeclareLaunchArgument("max_angular_speed", default_value="0.45"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ]
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            bringup + "/launch/lidar_amcl_localization.launch.py"
        ),
        launch_arguments={
            "serial_port": L("lidar_port"),
            "map_yaml": L("map_yaml"),
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(autonav + "/launch/autonav_demo.launch.py"),
        launch_arguments={
            "use_serial": L("use_serial"),
            "serial_port": L("esp32_port"),
            "use_wheel_odom_tf": "false",
            "map_yaml": L("planning_map_yaml"),
            "semantic_yaml": L("semantic_yaml"),
            "waypoint_file": L("waypoint_file"),
            "max_linear_speed": L("max_linear_speed"),
            "max_angular_speed": L("max_angular_speed"),
            "use_dynamic_obstacles": "false",
            "require_thermal_grid": "false",
            "require_thermal_active": "false",
            "hazard_belief_enabled": "false",
            "exit_evaluator_enabled": "false",
            "evacuation_manager_enabled": "false",
            "event_replanning_enabled": "false",
            "exit_switching_enabled": "false",
            "waypoint_planning_enabled": "true",
            "waypoint_accept_direct_goal": "true",
            "astar_accept_goal_pose": "false",
            "astar_periodic_replanning_enabled": "false",
        }.items(),
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="static_waypoint_drive_rviz",
        arguments=["-d", bringup + "/rviz/inno_slam.rviz"],
        output="screen", condition=IfCondition(L("use_rviz")),
    )
    return LaunchDescription(args + [localization, navigation, rviz])
