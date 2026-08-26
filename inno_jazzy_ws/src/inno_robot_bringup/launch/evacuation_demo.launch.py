"""Mode 5: one-command autonomous evacuation demo on the real robot."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from inno_robot_bringup.project_paths import project_path


def generate_launch_description():
    bringup = get_package_share_directory("inno_robot_bringup")
    thermal = get_package_share_directory("inno_thermal")
    arguments = [
        DeclareLaunchArgument("esp32_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument(
            "discovery_range",
            default_value="LOCALHOST",
            description=(
                "ROS discovery scope. LOCALHOST is reliable for the single-Pi "
                "robot; use SUBNET only when remote ROS hosts are required."
            ),
        ),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("mmwave_port", default_value="/dev/ttyAMA0"),
        DeclareLaunchArgument("mmwave_configure_sensor", default_value="true"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_mmwave", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("assist_check_sec", default_value="10.0"),
        # inno_autonav/config/semantic_points.yaml의 init. 실제 시연 시작점이 바뀌면
        # 명령행 인자로 덮어써야 하며 경로 자체는 이 값에 고정되지 않는다.
        DeclareLaunchArgument(
            "initial_pose_x", default_value="4.817799091339111"
        ),
        DeclareLaunchArgument(
            "initial_pose_y", default_value="-9.854209899902344"
        ),
        DeclareLaunchArgument(
            "initial_pose_yaw", default_value="-0.4439678115329046"
        ),
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
        DeclareLaunchArgument("drive_speed", default_value="0.06"),
        DeclareLaunchArgument("turn_speed", default_value="0.35"),
        DeclareLaunchArgument("use_serial", default_value="true"),
        DeclareLaunchArgument("use_thermal_sensor", default_value="true"),
        DeclareLaunchArgument("thermal_x", default_value="0.10"),
        DeclareLaunchArgument("thermal_y", default_value="0.0"),
        DeclareLaunchArgument("thermal_z", default_value="0.20"),
        DeclareLaunchArgument("thermal_roll", default_value="0.0"),
        DeclareLaunchArgument("thermal_pitch", default_value="0.0"),
        DeclareLaunchArgument("thermal_yaw", default_value="0.0"),
        DeclareLaunchArgument("event_replanning_enabled", default_value="true"),
        DeclareLaunchArgument("exit_switching_enabled", default_value="true"),
        DeclareLaunchArgument("waypoint_planning_enabled", default_value="true"),
        DeclareLaunchArgument("evacuation_demo_auto_start", default_value="true"),
        DeclareLaunchArgument("use_camera_mode4", default_value="true"),
        DeclareLaunchArgument(
            "yolo_model_path",
            default_value=project_path(
                "models", "yolov8n_best_opencv_640.onnx"
            ),
        ),
        DeclareLaunchArgument("yolo_confidence", default_value="0.50"),
    ]

    thermal_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            thermal + "/launch/thermal_sensor.launch.py"
        ),
        launch_arguments={"enable_cost_layer": "true"}.items(),
        condition=IfCondition(L("use_thermal_sensor")),
    )
    thermal_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_thermal_camera_tf",
        arguments=[
            "--x", L("thermal_x"),
            "--y", L("thermal_y"),
            "--z", L("thermal_z"),
            "--roll", L("thermal_roll"),
            "--pitch", L("thermal_pitch"),
            "--yaw", L("thermal_yaw"),
            "--frame-id", "base_link",
            "--child-frame-id", "thermal_camera_link",
        ],
        condition=IfCondition(L("use_thermal_sensor")),
    )
    field_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            bringup + "/launch/field_waypoint_test.launch.py"
        ),
        launch_arguments={
            "esp32_port": L("esp32_port"),
            "lidar_port": L("lidar_port"),
            "mmwave_port": L("mmwave_port"),
            "mmwave_configure_sensor": L("mmwave_configure_sensor"),
            "use_lidar": L("use_lidar"),
            "use_mmwave": L("use_mmwave"),
            "use_rviz": L("use_rviz"),
            "assist_check_sec": L("assist_check_sec"),
            "set_initial_pose": "true",
            "initial_pose_x": L("initial_pose_x"),
            "initial_pose_y": L("initial_pose_y"),
            "initial_pose_yaw": L("initial_pose_yaw"),
            "map_yaml": L("map_yaml"),
            "planning_map_yaml": L("planning_map_yaml"),
            "waypoint_file": L("waypoint_file"),
            "drive_speed": L("drive_speed"),
            "turn_speed": L("turn_speed"),
            "use_serial": L("use_serial"),
            "use_thermal_sensor": L("use_thermal_sensor"),
            "mode5_enabled": "true",
            "use_dynamic_obstacles": "true",
            "hazard_belief_enabled": "true",
            "exit_evaluator_enabled": "true",
            "evacuation_manager_enabled": "true",
            "evacuation_activate_selected_route": "true",
            "event_replanning_enabled": L("event_replanning_enabled"),
            "exit_switching_enabled": L("exit_switching_enabled"),
            "waypoint_planning_enabled": L("waypoint_planning_enabled"),
            "waypoint_accept_direct_goal": "false",
            "astar_accept_goal_pose": PythonExpression([
                "'false' if '", L("waypoint_planning_enabled"),
                "' == 'true' else 'true'"
            ]),
            "mode3_standoff_distance_m": "2.0",
            "mode3_publish_canonical_plan": "true",
            "mode4_standoff_distance_m": "2.0",
            "mode4_publish_canonical_plan": "true",
            "use_camera_mode4": L("use_camera_mode4"),
            "yolo_model_path": L("yolo_model_path"),
            "yolo_confidence": L("yolo_confidence"),
            "start_thermal_viewer": "false",
        }.items(),
    )
    orchestrator = Node(
        package="inno_autonav",
        executable="evacuation_demo_orchestrator",
        name="evacuation_demo_orchestrator",
        output="screen",
        parameters=[{
            "enabled": True,
            "auto_start": ParameterValue(
                L("evacuation_demo_auto_start"), value_type=bool
            ),
            "moving_survivor_enabled": ParameterValue(
                L("use_camera_mode4"), value_type=bool
            ),
        }],
    )
    return LaunchDescription(
        arguments + [
            SetEnvironmentVariable(
                "ROS_AUTOMATIC_DISCOVERY_RANGE", L("discovery_range")
            ),
            thermal_transform, thermal_bringup, field_bringup, orchestrator
        ]
    )
