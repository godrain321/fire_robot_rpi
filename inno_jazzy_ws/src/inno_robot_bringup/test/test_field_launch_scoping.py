import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def _load_field_launch_module():
    launch_file = (
        Path(__file__).resolve().parents[1]
        / 'launch/field_waypoint_test.launch.py'
    )
    spec = importlib.util.spec_from_file_location(
        'field_waypoint_test_launch', launch_file
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_autonav_include_is_scoped_away_from_esp32_serial_flag():
    description = _load_field_launch_module().generate_launch_description()
    entities = list(description.entities)

    navigation_groups = [
        entity for entity in entities if isinstance(entity, GroupAction)
    ]
    assert len(navigation_groups) == 1
    assert any(
        isinstance(entity, IncludeLaunchDescription)
        for entity in navigation_groups[0].get_sub_entities()
    )
    assert any(
        isinstance(entity, Node)
        and entity.node_executable == 'cmdvel_to_esp32_serial'
        for entity in entities
    )


def test_planner_reference_file_is_separate_from_rviz_queue_file():
    description = _load_field_launch_module().generate_launch_description()
    declared = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    context = LaunchContext()
    queue_default = perform_substitutions(
        context, declared['waypoint_file'].default_value
    )
    planner_default = perform_substitutions(
        context, declared['planner_waypoint_file'].default_value
    )
    assert queue_default.endswith('maps/waypoint_queue_latest.yaml')
    assert planner_default.endswith(
        'docs/full_map_waypoints_1m_numbered.yaml'
    )
    assert queue_default != planner_default


def test_mode4_camera_defaults_keep_inference_warm_on_pi():
    description = _load_field_launch_module().generate_launch_description()
    declared = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    context = LaunchContext()

    def default(name):
        return perform_substitutions(context, declared[name].default_value)

    assert default('camera_width') == '1280'
    assert default('camera_height') == '720'
    assert default('yolo_confidence') == '0.40'
    assert default('yolo_inference_rate_hz') == '3.0'
    assert default('yolo_only_during_mode4_observation') == 'false'
    assert default('yolo_model_path').endswith(
        'models/yolov8n_best_opencv_640.onnx'
    )
