"""Run Camera Module 3, YOLO person inference, and an annotated preview."""

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
    detector = Node(
        package='inno_camera_tools',
        executable='camera_person_detector',
        name='camera_person_detector_check',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'image_topic': L('image_topic'),
            'detection_topic': L('detection_topic'),
            'annotated_image_topic': L('annotated_image_topic'),
            'model_path': L('model_path'),
            'confidence_threshold': ParameterValue(
                L('confidence_threshold'), value_type=float
            ),
            'inference_rate_hz': ParameterValue(
                L('inference_rate_hz'), value_type=float
            ),
            'inference_image_size': ParameterValue(
                L('inference_image_size'), value_type=int
            ),
            'only_during_mode4_observation': False,
        }],
    )
    viewer = Node(
        package='image_view',
        executable='image_view',
        name='camera_person_detection_view',
        output='screen',
        remappings=[('image', L('annotated_image_topic'))],
        parameters=[{'autosize': True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('camera', default_value='0'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('format', default_value='RGB888'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument(
            'detection_topic', default_value='/camera/person_detections'
        ),
        DeclareLaunchArgument(
            'annotated_image_topic',
            default_value='/camera/person_detection_image',
        ),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('confidence_threshold', default_value='0.50'),
        DeclareLaunchArgument('inference_rate_hz', default_value='3.0'),
        DeclareLaunchArgument('inference_image_size', default_value='640'),
        camera,
        detector,
        viewer,
    ])
