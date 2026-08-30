"""Mode 7: thermal-aware autonomous evacuation, gas + person detection OFF.

The existing Mode 5 exit-decision path (exit_evaluator_node ->
evacuation_manager_node) auto-selects the destination exit and activates the
route; the unchanged waypoint + weighted-A* + Stage 6 event-replanning pipeline
drives there on a hazard belief that carries the MLX90640 thermal cost but not
the gas layer. Removed vs Mode 5: the evacuation_demo_orchestrator victim state
machine, mmWave / RGB camera / YOLO, Mode 3/4 person inspection, Mode 3 audio,
and the MQ-135 / CO cost layer.

Structure: this is ``field_waypoint_test.launch.py`` with the thermal sensor
stack + base->thermal static TF added and the exit/evacuation nodes enabled.
The thin ``mode7_mission_coordinator`` waits for localization, map TF, thermal
hazard, evaluator, and manager readiness before calling the existing
``/plan_evacuation`` service.  It owns no exit-selection or path algorithm.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from inno_robot_bringup.project_paths import project_path


def generate_launch_description():
    bringup = get_package_share_directory("inno_robot_bringup")
    thermal = get_package_share_directory("inno_thermal")

    args = [
        DeclareLaunchArgument("esp32_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("use_serial", default_value="true"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("auto_localization", default_value="true"),
        DeclareLaunchArgument("mode7_auto_start", default_value="false"),
        DeclareLaunchArgument("set_initial_pose", default_value="false"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
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
        DeclareLaunchArgument("use_dynamic_obstacles", default_value="true"),
        DeclareLaunchArgument("event_replanning_enabled", default_value="true"),
        DeclareLaunchArgument("waypoint_planning_enabled", default_value="true"),
        DeclareLaunchArgument("exit_switching_enabled", default_value="true"),
        # base_link -> thermal_camera_link mounting offset (same names as Mode 5/6).
        DeclareLaunchArgument("thermal_x", default_value="0.10"),
        DeclareLaunchArgument("thermal_y", default_value="0.0"),
        DeclareLaunchArgument("thermal_z", default_value="0.20"),
        DeclareLaunchArgument("thermal_roll", default_value="0.0"),
        DeclareLaunchArgument("thermal_pitch", default_value="0.0"),
        DeclareLaunchArgument("thermal_yaw", default_value="0.0"),
    ]

    thermal_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            thermal + "/launch/thermal_sensor.launch.py"
        ),
        launch_arguments={"enable_cost_layer": "true"}.items(),
    )
    thermal_transform = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_to_thermal_camera_tf",
        arguments=[
            "--x", L("thermal_x"), "--y", L("thermal_y"), "--z", L("thermal_z"),
            "--roll", L("thermal_roll"), "--pitch", L("thermal_pitch"),
            "--yaw", L("thermal_yaw"),
            "--frame-id", "base_link", "--child-frame-id", "thermal_camera_link",
        ],
    )

    field_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            bringup + "/launch/field_waypoint_test.launch.py"
        ),
        launch_arguments={
            "esp32_port": L("esp32_port"),
            "lidar_port": L("lidar_port"),
            "use_serial": L("use_serial"),
            "use_lidar": L("use_lidar"),
            "use_rviz": "false",           # Mode 7 supplies its own RViz below
            "auto_localization": L("auto_localization"),
            "set_initial_pose": L("set_initial_pose"),
            "initial_pose_x": L("initial_pose_x"),
            "initial_pose_y": L("initial_pose_y"),
            "initial_pose_yaw": L("initial_pose_yaw"),
            "map_yaml": L("map_yaml"),
            "planning_map_yaml": L("planning_map_yaml"),
            "waypoint_file": L("waypoint_file"),
            "drive_speed": L("drive_speed"),
            "turn_speed": L("turn_speed"),
            "use_dynamic_obstacles": L("use_dynamic_obstacles"),
            "person_inspection_enabled": "false",
            # thermal-only hazard belief: thermal ON, gas/CO OFF
            "hazard_belief_enabled": "true",
            "hazard_thermal_enabled": "true",
            "hazard_co_enabled": "false",
            "use_thermal_sensor": "true",
            "require_thermal_grid": "false",
            "require_thermal_active": "false",
            "waypoint_planning_enabled": L("waypoint_planning_enabled"),
            "event_replanning_enabled": L("event_replanning_enabled"),
            "waypoint_accept_direct_goal": "false",
            "astar_accept_goal_pose": "true",
            # mission / exit decision ON (existing Mode 5 nodes, victim-independent)
            "exit_evaluator_enabled": "true",
            "evacuation_manager_enabled": "true",
            "evacuation_activate_selected_route": "true",
            "exit_switching_enabled": L("exit_switching_enabled"),
            # person / victim detection OFF; orchestrator not started here
            "mode5_enabled": "false",
            "use_mmwave": "false",
            "use_camera_mode4": "false",
            "use_mode3_audio": "false",
            "start_thermal_viewer": "false",
        }.items(),
    )

    mission_coordinator = Node(
        package="inno_autonav",
        executable="mode7_mission_coordinator",
        name="mode7_mission_coordinator",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "enabled": True,
            "auto_start": ParameterValue(L("mode7_auto_start"), value_type=bool),
        }],
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="mode7_thermal_drive_rviz",
        arguments=["-d", bringup + "/rviz/mode7_thermal_drive.rviz"],
        output="screen", condition=IfCondition(L("use_rviz")),
    )

    return LaunchDescription(args + [
        thermal_transform, thermal_bringup, field_bringup, mission_coordinator, rviz,
    ])
