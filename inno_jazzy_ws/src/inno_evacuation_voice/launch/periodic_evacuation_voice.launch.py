"""Launch the independent periodic evacuation voice guide."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('inno_evacuation_voice')
    config = os.path.join(share, 'config', 'evacuation_voice_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('interval_sec', default_value='7.0'),
        DeclareLaunchArgument('play_immediately', default_value='true'),
        DeclareLaunchArgument('active_drive_mode', default_value='5'),
        DeclareLaunchArgument('audio_directory', default_value='~/fire_robot_audio'),
        DeclareLaunchArgument('audio_file', default_value='evacuation_guide.wav'),
        DeclareLaunchArgument('audio_device', default_value='auto'),
        DeclareLaunchArgument('player_executable', default_value='aplay'),
        DeclareLaunchArgument('playback_volume_percent', default_value='100'),
        DeclareLaunchArgument('activation_mode', default_value='drive_mode'),
        Node(
            package='inno_evacuation_voice',
            executable='periodic_evacuation_voice_node',
            name='periodic_evacuation_voice_node',
            output='screen',
            emulate_tty=True,
            parameters=[config, {
                'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
                'interval_sec': ParameterValue(
                    LaunchConfiguration('interval_sec'), value_type=float
                ),
                'play_immediately': ParameterValue(
                    LaunchConfiguration('play_immediately'), value_type=bool
                ),
                'active_drive_mode': ParameterValue(
                    LaunchConfiguration('active_drive_mode'), value_type=int
                ),
                'audio_directory': LaunchConfiguration('audio_directory'),
                'audio_file': LaunchConfiguration('audio_file'),
                'audio_device': LaunchConfiguration('audio_device'),
                'player_executable': LaunchConfiguration('player_executable'),
                'playback_volume_percent': ParameterValue(
                    LaunchConfiguration('playback_volume_percent'), value_type=int
                ),
                'activation_mode': LaunchConfiguration('activation_mode'),
            }],
        ),
    ])
