import ast
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[2]
BRINGUP = SOURCE_ROOT / 'inno_robot_bringup'
LAUNCH = BRINGUP / 'launch' / 'mode2_integrated_fire_evacuation.launch.py'
FIELD = BRINGUP / 'launch' / 'field_waypoint_test.launch.py'
LOCALIZATION = BRINGUP / 'launch' / 'lidar_amcl_localization.launch.py'
RUN = SOURCE_ROOT.parents[1] / 'run_modes.sh'


def source(path):
    return path.read_text(encoding='utf-8')


def assigned_dict(path, name):
    tree = ast.parse(source(path))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name
            for target in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError(f'{name} not found')


def test_integrated_profile_enables_old_mode_2_through_11_capabilities():
    values = assigned_dict(LAUNCH, '_MODE9_ARGUMENTS')
    assert values['use_camera_mode4'] == 'true'
    assert values['use_gas_sensor'] == 'true'
    assert values['mode11_enabled'] == 'true'
    assert values['localization_scan_topic'] == '/mode11/scan'
    assert values['rf2o_publish_tf'] == 'false'
    assert values['use_amcl_tf_bridge'] == 'false'
    assert values['cmd_vel_output_topic'] == '/cmd_vel_before_ultrasonic'
    assert values['voice_enabled'] == 'true'


def test_integrated_profile_wraps_mode9_and_adds_one_final_safety_filter():
    text = source(LAUNCH)
    assert text.count('mode9_full_evacuation.launch.py') == 1
    assert text.count("executable='mode10_ultrasonic_avoidance'") == 1
    assert "'active_operator_modes': [2]" in text
    assert "'output_cmd_vel_topic': '/cmd_vel'" in text


def test_field_profile_uses_one_mode11_tf_selector_and_mode3_demo():
    text = source(FIELD)
    assert text.count("executable='mode11_localization'") == 1
    assert text.count("executable='mode3_greeting_spin'") == 1
    assert "'path_topic': '/lidar_path'" in text
    assert "'output_topic': L('cmd_vel_output_topic')" in text


def test_localization_tf_owners_are_launch_configurable():
    text = source(LOCALIZATION)
    assert "DeclareLaunchArgument('rf2o_publish_tf'" in text
    assert "DeclareLaunchArgument('use_amcl_tf_bridge'" in text
    assert "'publish_tf': ParameterValue(L('rf2o_publish_tf')" in text


def test_run_modes_uses_final_integrated_profile():
    text = source(RUN)
    assert 'mode2_integrated_fire_evacuation.launch.py' in text
    assert 'use_camera_mode4:=true' in text
    assert 'use_gas_sensor:=true' in text
    assert RUN.stat().st_mode & 0o111
