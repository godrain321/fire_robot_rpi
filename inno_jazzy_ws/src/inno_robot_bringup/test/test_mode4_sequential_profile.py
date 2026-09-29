"""Static contracts for Mode 4's Mode 3 -> Mode 6 + YOLO sequence."""

from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[2]
BRINGUP = SOURCE_ROOT / 'inno_robot_bringup'
REPOSITORY = SOURCE_ROOT.parents[1]
MODE3 = BRINGUP / 'inno_robot_bringup' / 'mode3_greeting_spin.py'
GATE = BRINGUP / 'inno_robot_bringup' / 'mode4_mode3_gate.py'
STAGE = BRINGUP / 'launch' / 'mode4_mode3_stage.launch.py'
RUN_MODE4 = REPOSITORY / 'run_mode4.sh'
RUN_MODES = REPOSITORY / 'run_modes.sh'
CAMERA_LAUNCH = (
    SOURCE_ROOT / 'inno_camera_tools' / 'launch'
    / 'camera_inference_check.launch.py'
)
PERSON_DETECTOR = (
    SOURCE_ROOT / 'inno_camera_tools' / 'inno_camera_tools'
    / 'person_detector.py'
)


def source(path: Path) -> str:
    return path.read_text(encoding='utf-8')


def test_existing_mode3_completion_contract_is_reused_without_changes():
    mode3 = source(MODE3)
    gate = source(GATE)
    assert "self._publish_status('COMPLETE:ONE_TURN')" in mode3
    assert "MODE3_COMPLETE = 'COMPLETE:ONE_TURN'" in gate
    assert "'/mode3_demo/status'" in gate
    assert "'/mode3/demo_request'" in gate


def test_mode3_stage_reuses_existing_node_and_drive_support_only():
    stage = source(STAGE)
    assert "executable='mode3_greeting_spin'" in stage
    assert "executable='cmd_vel_mode_mux'" in stage
    assert "executable='cmdvel_to_esp32_serial'" in stage
    assert 'camera' not in stage.lower()
    assert 'yolo' not in stage.lower()
    assert stage.count('OnProcessExit(') == 3


def test_followup_processes_start_only_after_gate_success():
    script = source(RUN_MODE4)
    gate_wait = script.index('wait -n -p first_stage_pid')
    failure_guard = script.index('if (( mode3_status != 0 ))')
    mode6_start = script.index('"${robot_root}/run_mode6.sh"')
    yolo_start = script.index('"${robot_root}/run_camera_inference_check.sh"')
    assert gate_wait < failure_guard < mode6_start < yolo_start
    assert 'wait -n -p first_stage_pid' in script
    assert 'wait -n -p completed_pid "${mode6_pid}" "${yolo_pid}"' in script


def test_mode4_enables_mode6_automatic_initial_pose():
    script = source(RUN_MODE4)
    assert 'set_initial_pose:=true \\\n' in script
    assert 'set_initial_pose:=false' not in script


def test_mode4_owns_child_process_groups_and_cleanup():
    script = source(RUN_MODE4)
    assert 'trap cleanup EXIT' in script
    assert "trap 'exit 130' INT TERM" in script
    assert script.count('\nsetsid ') == 4
    assert 'kill -INT -- "-${process_pid}"' in script
    assert 'kill -TERM -- "-${process_pid}"' in script
    assert 'kill -KILL -- "-${process_pid}"' in script


def test_yolo_launch_has_one_camera_and_visible_annotated_output():
    camera = source(CAMERA_LAUNCH)
    assert camera.count("camera_module_3.launch.py") == 1
    assert "executable='camera_person_detector'" in camera
    assert "executable='image_view'" in camera
    assert "condition=IfCondition(L('use_image_view'))" in camera
    assert "('image', L('annotated_image_topic'))" in camera


def test_mode4_routes_annotated_yolo_image_to_rviz_without_image_view():
    script = source(RUN_MODE4)
    rviz = source(BRINGUP / 'rviz' / 'mode4_thermal_camera.rviz')
    assert 'use_image_view:=false' in script
    assert 'mode4_thermal_camera.rviz' in script
    assert 'Name: YOLO Camera' in rviz
    assert 'Value: /camera/person_detection_image' in rviz
    assert 'Enabled: true' in rviz
    assert 'Hide Left Dock: true' in rviz
    assert 'Hide Right Dock: true' in rviz
    assert 'QMainWindow State:' in rviz
    thermal_arc = rviz.split('Name: Thermal Arc Points', 1)[1].split(
        '- Class:', 1
    )[0]
    assert 'Enabled: false' in thermal_arc


def test_camera_runner_uses_the_available_dynamic_model_and_local_runtime():
    runner = source(REPOSITORY / 'run_camera_inference_check.sh')
    assert 'models/yolov8n_best.onnx' in runner
    assert 'models/yolov8n_best_opencv_640.onnx' not in runner
    assert 'python_runtime="${CAMERA_PYTHON_RUNTIME:-${runtime_root}/python}"' in runner
    assert 'export PYTHONPATH=' in runner


def test_annotated_yolo_image_is_published_without_discovery_race():
    detector = source(PERSON_DETECTOR)
    assert 'get_subscription_count() == 0' not in detector
    assert 'self.annotated_publisher.publish(message)' in detector


def test_run_modes_routes_explicit_mode4_without_changing_interactive_modes():
    runner = source(RUN_MODES)
    assert "${1:-}" in runner
    assert 'exec "${robot_root}/run_mode4.sh" "$@"' in runner
    assert 'mode2_integrated_fire_evacuation.launch.py' in runner
