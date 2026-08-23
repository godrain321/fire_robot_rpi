"""Launch Camera Module 3 and the distance-aware FOV viewer."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('inno_camera_tools')
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            share + '/launch/camera_module_3.launch.py'
        ),
        launch_arguments={
            'camera': L('camera'),
            'width': L('width'),
            'height': L('height'),
            'format': L('format'),
            'rectify': 'false',
        }.items(),
        condition=IfCondition(L('start_camera')),
    )
    viewer = Node(
        package='inno_camera_tools',
        executable='camera_fov_viewer',
        name='camera_fov_viewer',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'image_topic': L('image_topic'),
            'camera_info_topic': L('camera_info_topic'),
            'calibration_file': L('calibration_file'),
            'output_dir': L('output_dir'),
            'target_distance_m': ParameterValue(
                L('target_distance_m'), value_type=float
            ),
            'display_scale': ParameterValue(
                L('display_scale'), value_type=float
            ),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('camera', default_value='0'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('format', default_value='RGB888'),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/image_raw'
        ),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera_info'
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=share + '/config/pi_camera3_wide_camera_info.yaml',
        ),
        DeclareLaunchArgument(
            'output_dir', default_value='~/fire_robot_fov_check'
        ),
        DeclareLaunchArgument('target_distance_m', default_value='2.0'),
        DeclareLaunchArgument('display_scale', default_value='0.85'),
        camera,
        viewer,
    ])
