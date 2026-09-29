"""Final operator profile: Mode 1 manual, Mode 2 full mission, Mode 3 greeting."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from inno_robot_bringup.project_paths import project_path


_MODE9_ARGUMENTS = {
    'use_rviz': 'true',
    'esp32_port': '/dev/ttyUSB0',
    'lidar_port': '/dev/ttyUSB1',
    'mmwave_port': '/dev/ttyAMA0',
    'mmwave_configure_sensor': 'true',
    'use_lidar': 'true',
    'use_mmwave': 'true',
    'use_serial': 'true',
    'use_camera_mode4': 'true',
    'use_mode3_audio': 'true',
    'mode3_audio_directory': '~/fire_robot_audio',
    'mode3_audio_device': 'auto',
    'mode3_audio_volume_percent': '100',
    'use_mode3_demo': 'true',
    'mode3_intro_audio_file': 'mode3_intro.wav',
    'mode3_spin_speed_radps': '1.0',
    'discovery_range': 'LOCALHOST',
    'assist_check_sec': '10.0',
    'initial_pose_x': '4.817799091339111',
    'initial_pose_y': '-9.854209899902344',
    'initial_pose_yaw': '-0.4439678115329046',
    'drive_speed': '0.06',
    'turn_speed': '0.35',
    'event_replanning_enabled': 'true',
    'exit_switching_enabled': 'true',
    'waypoint_planning_enabled': 'true',
    'evacuation_demo_auto_start': 'false',
    'use_gas_sensor': 'true',
    'gas_input_mode': 'legacy_ppm',
    'gas_safe_adc': '0.0',
    'gas_blocked_adc': '4096.0',
    'thermal_x': '0.085',
    'thermal_y': '0.0',
    'thermal_z': '0.20',
    'thermal_roll': '0.0',
    'thermal_pitch': '0.0',
    'thermal_yaw': '0.0',
    'yolo_confidence': '0.40',
    'localization_scan_topic': '/mode11/scan',
    'rf2o_publish_tf': 'false',
    'use_amcl_tf_bridge': 'false',
    'use_localization_path': 'false',
    'cmd_vel_output_topic': '/cmd_vel_before_ultrasonic',
    'mode11_enabled': 'true',
    'imu_topic': '/imu/data_raw',
    'encoder_topic': '/wheel_encoder_ticks',
    'allow_virtual_step_counts': 'false',
    'left_encoder_sign': '1',
    'right_encoder_sign': '1',
    'imu_yaw_sign': '1',
    'wheel_radius_m': '0.04',
    'encoder_counts_per_rev': '16384',
    'maximum_encoder_counts_per_sec': '12000.0',
    'voice_enabled': 'true',
    'voice_interval_sec': '7.0',
    'voice_play_immediately': 'true',
    'voice_audio_directory': '~/fire_robot_audio',
    'voice_audio_file': 'evacuation_guide.wav',
    'voice_audio_device': 'auto',
    'voice_player_executable': 'aplay',
    'voice_playback_volume_percent': '100',
}
_PATH_ARGUMENTS = {
    'map_yaml': project_path('maps', 'inno_map_raw.yaml'),
    'planning_map_yaml': project_path('maps', 'inno_map_nav.yaml'),
    'waypoint_file': project_path('maps', 'waypoint_queue_latest.yaml'),
    'yolo_model_path': project_path('models', 'yolov8n_best_opencv_640.onnx'),
}
_ULTRASONIC_ARGUMENTS = {
    'ultrasonic_trigger_distance_m': '0.50',
    'ultrasonic_clear_distance_m': '0.70',
    'ultrasonic_hold_sec': '1.0',
    'ultrasonic_side_clearance_m': '0.70',
    'ultrasonic_front_clearance_m': '0.70',
}


def generate_launch_description():
    bringup = get_package_share_directory('inno_robot_bringup')
    arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in {
            **_MODE9_ARGUMENTS,
            **_PATH_ARGUMENTS,
            **_ULTRASONIC_ARGUMENTS,
        }.items()
    ]

    complete_mission = GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                bringup + '/launch/mode9_full_evacuation.launch.py'
            ),
            launch_arguments={
                name: L(name) for name in (*_MODE9_ARGUMENTS, *_PATH_ARGUMENTS)
            }.items(),
        )
    ])
    ultrasonic_safety = Node(
        package='inno_drive_bridge',
        executable='mode10_ultrasonic_avoidance',
        name='integrated_ultrasonic_safety',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'input_cmd_vel_topic': '/cmd_vel_before_ultrasonic',
            'output_cmd_vel_topic': '/cmd_vel',
            'active_operator_modes': [2],
            'auto_start': True,
            'trigger_distance_m': ParameterValue(
                L('ultrasonic_trigger_distance_m'), value_type=float
            ),
            'clear_distance_m': ParameterValue(
                L('ultrasonic_clear_distance_m'), value_type=float
            ),
            'hold_sec': ParameterValue(
                L('ultrasonic_hold_sec'), value_type=float
            ),
            'turn_speed_radps': ParameterValue(
                L('turn_speed'), value_type=float
            ),
            'side_clearance_m': ParameterValue(
                L('ultrasonic_side_clearance_m'), value_type=float
            ),
            'front_clearance_m': ParameterValue(
                L('ultrasonic_front_clearance_m'), value_type=float
            ),
        }],
    )
    return LaunchDescription(arguments + [complete_mission, ultrasonic_safety])
