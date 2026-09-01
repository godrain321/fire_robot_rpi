"""Mode 9 profile: the complete existing Mode 8 plus periodic voice guidance."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L

from inno_robot_bringup.project_paths import project_path


# Public Mode 8 arguments and defaults are mirrored only for forwarding into
# its launch. Mode 8 remains the sole owner of the evacuation node hierarchy.
_MODE8_ARGUMENTS = {
    "use_rviz": "true",
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
_MODE8_PATH_ARGUMENTS = {
    "map_yaml": project_path("maps", "inno_map_raw.yaml"),
    "planning_map_yaml": project_path("maps", "inno_map_nav.yaml"),
    "waypoint_file": project_path("maps", "waypoint_queue_latest.yaml"),
    "yolo_model_path": project_path("models", "yolov8n_best_opencv_640.onnx"),
}
_VOICE_ARGUMENTS = {
    "voice_enabled": "true",
    "voice_interval_sec": "7.0",
    "voice_play_immediately": "true",
    "voice_audio_directory": "~/fire_robot_audio",
    "voice_audio_file": "evacuation_guide.wav",
    "voice_audio_device": "auto",
    "voice_player_executable": "aplay",
    "voice_playback_volume_percent": "100",
}


def generate_launch_description():
    bringup = get_package_share_directory("inno_robot_bringup")
    voice = get_package_share_directory("inno_evacuation_voice")
    arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in {
            **_MODE8_ARGUMENTS,
            **_MODE8_PATH_ARGUMENTS,
            **_VOICE_ARGUMENTS,
        }.items()
    ]

    mode8 = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            bringup + "/launch/mode8_evacuation_thermal.launch.py"
        ),
        launch_arguments={
            name: L(name) for name in (*_MODE8_ARGUMENTS, *_MODE8_PATH_ARGUMENTS)
        }.items(),
    )])
    periodic_voice = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            voice + "/launch/periodic_evacuation_voice.launch.py"
        ),
        launch_arguments={
            "enabled": L("voice_enabled"),
            "interval_sec": L("voice_interval_sec"),
            "play_immediately": L("voice_play_immediately"),
            "audio_directory": L("voice_audio_directory"),
            "audio_file": L("voice_audio_file"),
            "audio_device": L("voice_audio_device"),
            "player_executable": L("voice_player_executable"),
            "playback_volume_percent": L("voice_playback_volume_percent"),
            "active_drive_mode": "5",
            "activation_mode": "drive_mode",
        }.items(),
    )])
    return LaunchDescription(arguments + [mode8, periodic_voice])
