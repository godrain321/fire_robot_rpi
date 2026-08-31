"""Mode 8: full Mode 5 evacuation mission + MLX90640 thermal costmap.

NOT a new navigation stack. A thin wrapper that runs the existing Mode 5 launch
(``evacuation_demo.launch.py``) with ``use_thermal_sensor:=true``. Mode 5 already
conditionally starts the thermal sensor stack, the base->thermal_camera_link
static TF, and wires ``hazard_thermal_enabled`` to that same flag, so Mode 8 only:

  * forces ``use_thermal_sensor:=true`` (Mode 5's default stays ``false``),
  * substitutes a thermal-aware RViz config, suppressing Mode 5's own RViz with
    ``use_rviz:=false`` so nothing is launched twice.

Every mission behaviour (Mode 5 state machine, exit evaluator, evacuation
manager, exit switching, victim search, waypoint queue, weighted A*,
SafePathSimplifier, event replanning, skid_path_follower, cmd_vel_mode_mux,
ESP32 serial) comes from the unchanged Mode 5 tree.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node

from inno_robot_bringup.project_paths import project_path

# name -> default, kept identical to evacuation_demo.launch.py so an unspecified
# Mode 8 arg reproduces Mode 5 exactly. ``use_thermal_sensor`` and ``use_rviz``
# are driven by Mode 8 itself and are not in this list.
_FORWARD = {
    "esp32_port": "/dev/ttyUSB0",
    "lidar_port": "/dev/ttyUSB1",
    "mmwave_port": "/dev/ttyAMA0",
    "mmwave_configure_sensor": "true",
    "use_lidar": "true",
    "use_mmwave": "true",
    "use_serial": "true",
    "use_camera_mode4": "false",
    "use_mode3_audio": "true",
    "discovery_range": "LOCALHOST",
    "assist_check_sec": "10.0",
    "initial_pose_x": "4.817799091339111",
    "initial_pose_y": "-9.854209899902344",
    "initial_pose_yaw": "-0.4439678115329046",
    "drive_speed": "0.06",
    "turn_speed": "0.35",
    "event_replanning_enabled": "true",
    "exit_switching_enabled": "true",
    "waypoint_planning_enabled": "true",
    "evacuation_demo_auto_start": "false",
    "use_gas_sensor": "false",
    "gas_input_mode": "legacy_ppm",
    "gas_safe_adc": "0.0",
    "gas_blocked_adc": "4096.0",
    "thermal_x": "0.085",
    "thermal_y": "0.0",
    "thermal_z": "0.20",
    "thermal_roll": "0.0",
    "thermal_pitch": "0.0",
    "thermal_yaw": "0.0",
    "yolo_confidence": "0.40",
}
_FORWARD_PATHS = {
    "map_yaml": project_path("maps", "inno_map_raw.yaml"),
    "planning_map_yaml": project_path("maps", "inno_map_nav.yaml"),
    "waypoint_file": project_path("maps", "waypoint_queue_latest.yaml"),
    "yolo_model_path": project_path("models", "yolov8n_best_opencv_640.onnx"),
}


def generate_launch_description():
    bringup = get_package_share_directory("inno_robot_bringup")

    args = [DeclareLaunchArgument("use_rviz", default_value="true")]
    args += [DeclareLaunchArgument(n, default_value=v) for n, v in _FORWARD.items()]
    args += [DeclareLaunchArgument(n, default_value=v)
             for n, v in _FORWARD_PATHS.items()]

    forwarded = {n: L(n) for n in (*_FORWARD, *_FORWARD_PATHS)}
    # Keep the nested Mode 5 launch arguments local. Without this scope its
    # use_rviz=false launch configuration leaks back into this wrapper and
    # disables Mode 8's dedicated thermal RViz action below.
    mode5 = GroupAction(
        scoped=True,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                bringup + "/launch/evacuation_demo.launch.py"
            ),
            launch_arguments={
                **forwarded,
                "use_thermal_sensor": "true",  # Mode 8 = Mode 5 + thermal
                # Keep soft costs below 50 C, but hard-block at >= 50 C.
                "temperature_cost_scale_max_c": "60.0",
                "temperature_blocked_c": "50.0",
                "use_rviz": "false",          # Mode 8 owns RViz below
            }.items(),
        )],
    )

    rviz = Node(
        package="rviz2", executable="rviz2",
        name="mode8_evacuation_thermal_rviz",
        arguments=["-d", bringup + "/rviz/mode8_evacuation_thermal.rviz"],
        output="screen", condition=IfCondition(L("use_rviz")),
    )

    return LaunchDescription(args + [mode5, rviz])
