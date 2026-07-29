"""Launch a static map, semantic markers, and an RViz editing session."""

import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _expanded_path(raw_path: str, label: str) -> Path:
    if not raw_path.strip():
        raise RuntimeError(f'{label} launch 인자가 비어 있습니다.')
    expanded = os.path.expandvars(os.path.expanduser(raw_path))
    return Path(expanded).resolve(strict=False)


def _validate_map(map_path: Path) -> None:
    if not map_path.exists():
        raise RuntimeError(f'map YAML 파일이 없습니다: {map_path}')
    if not map_path.is_file():
        raise RuntimeError(f'map 경로가 일반 파일이 아닙니다: {map_path}')
    try:
        with map_path.open('r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f'map YAML을 읽을 수 없습니다 ({map_path}): {exc}') from exc
    if not isinstance(config, dict) or not config.get('image'):
        raise RuntimeError(f'map YAML에 image 항목이 없습니다: {map_path}')
    image_value = os.path.expandvars(os.path.expanduser(str(config['image'])))
    image_path = Path(image_value)
    if not image_path.is_absolute():
        image_path = map_path.parent / image_path
    image_path = image_path.resolve(strict=False)
    if not image_path.exists():
        raise RuntimeError(f'map YAML이 가리키는 이미지 파일이 없습니다: {image_path}')
    if not image_path.is_file():
        raise RuntimeError(f'map 이미지 경로가 일반 파일이 아닙니다: {image_path}')


def _launch_setup(context):
    map_path = _expanded_path(LaunchConfiguration('map').perform(context), 'map')
    semantic_path = _expanded_path(
        LaunchConfiguration('semantic_file').perform(context), 'semantic_file'
    )
    rviz_path = _expanded_path(
        LaunchConfiguration('rviz_config').perform(context), 'rviz_config'
    )
    _validate_map(map_path)

    if semantic_path.exists() and not semantic_path.is_file():
        raise RuntimeError(f'semantic_file 경로가 일반 파일이 아닙니다: {semantic_path}')
    if not semantic_path.parent.exists():
        raise RuntimeError(
            f'semantic_file의 상위 디렉터리가 없습니다: {semantic_path.parent}'
        )
    if not semantic_path.parent.is_dir():
        raise RuntimeError(
            f'semantic_file의 상위 경로가 디렉터리가 아닙니다: {semantic_path.parent}'
        )
    if not rviz_path.exists() or not rviz_path.is_file():
        raise RuntimeError(f'RViz 설정 파일이 없습니다: {rviz_path}')

    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': str(map_path)},
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server']},
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='inno_semantic_nav',
            executable='semantic_marker_node',
            name='semantic_marker_node',
            output='screen',
            parameters=[
                {'semantic_file': str(semantic_path)},
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='semantic_map_editor_rviz',
            output='screen',
            arguments=['-d', str(rviz_path)],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    default_rviz = os.path.join(
        get_package_share_directory('inno_semantic_nav'),
        'rviz',
        'semantic_map_editor.rviz',
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'map',
                default_value='',
                description='nav2_map_server가 읽을 map YAML 경로',
            ),
            DeclareLaunchArgument(
                'semantic_file',
                default_value='semantic_points.yaml',
                description='named poses와 landmarks를 저장할 YAML 경로',
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('rviz_config', default_value=default_rviz),
            DeclareLaunchArgument('start_rviz', default_value='true'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
