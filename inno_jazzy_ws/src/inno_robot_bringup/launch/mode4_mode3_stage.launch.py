"""Run the unchanged Mode 3 greeting/turn with only its drive support nodes."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    drive = get_package_share_directory('inno_drive_bridge')
    drive_params = drive + '/config/drive_params.yaml'
    arguments = [
        DeclareLaunchArgument('use_serial', default_value='true'),
        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument(
            'mode3_audio_directory', default_value='~/fire_robot_audio'
        ),
        DeclareLaunchArgument('mode3_audio_file', default_value='mode3_intro.wav'),
        DeclareLaunchArgument('mode3_audio_device', default_value='auto'),
        DeclareLaunchArgument('mode3_audio_volume_percent', default_value='100'),
        DeclareLaunchArgument('mode3_spin_speed_radps', default_value='1.0'),
    ]
    mode3 = Node(
        package='inno_robot_bringup',
        executable='mode3_greeting_spin',
        name='mode3_greeting_spin',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'angular_speed_radps': ParameterValue(
                L('mode3_spin_speed_radps'), value_type=float
            ),
            'audio_directory': L('mode3_audio_directory'),
            'audio_file': L('mode3_audio_file'),
            'audio_device': L('mode3_audio_device'),
            'playback_volume_percent': ParameterValue(
                L('mode3_audio_volume_percent'), value_type=int
            ),
        }],
    )
    mux = Node(
        package='inno_drive_bridge',
        executable='cmd_vel_mode_mux',
        name='cmd_vel_mode_mux',
        output='log',
    )
    serial = Node(
        package='inno_drive_bridge',
        executable='cmdvel_to_esp32_serial',
        name='cmdvel_to_esp32_serial',
        output='log',
        parameters=[drive_params, {'serial_port': L('esp32_port')}],
        condition=IfCondition(L('use_serial')),
    )
    stop_if_mode3_exits = RegisterEventHandler(OnProcessExit(
        target_action=mode3,
        on_exit=[EmitEvent(event=Shutdown(reason='Mode 3 node exited'))],
    ))
    stop_if_mux_exits = RegisterEventHandler(OnProcessExit(
        target_action=mux,
        on_exit=[EmitEvent(event=Shutdown(reason='Mode 3 mux exited'))],
    ))
    stop_if_serial_exits = RegisterEventHandler(OnProcessExit(
        target_action=serial,
        on_exit=[EmitEvent(event=Shutdown(reason='Mode 3 serial exited'))],
    ))
    return LaunchDescription(arguments + [
        mode3,
        mux,
        serial,
        stop_if_mode3_exits,
        stop_if_mux_exits,
        stop_if_serial_exits,
    ])
