from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_setup_installs_audio_launch_and_config():
    source = (ROOT / 'setup.py').read_text(encoding='utf-8')
    assert "glob('audio/*.wav')" in source
    assert "glob('launch/*.launch.py')" in source
    assert "glob('config/*.yaml')" in source
    assert 'periodic_evacuation_voice_node' in source


def test_ros_interface_and_nonblocking_player_contract():
    source = (
        ROOT / 'inno_evacuation_voice' / 'periodic_evacuation_voice_node.py'
    ).read_text(encoding='utf-8')
    for interface in ('/drive_mode', '/autonomy_cancel', '/evacuation_voice/status'):
        assert interface in source
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in source
    assert 'ReliabilityPolicy.RELIABLE' in source
    assert 'subprocess.Popen' in source
    assert 'shell=False' in source
    assert "plughw:CARD={card},DEV=0" in source


def test_default_configuration_contract():
    config = (ROOT / 'config' / 'evacuation_voice_params.yaml').read_text(
        encoding='utf-8'
    )
    for setting in (
        'interval_sec: 7.0', 'play_immediately: true',
        'active_drive_mode: 5', 'activation_mode: "drive_mode"',
        'audio_file: "evacuation_guide.wav"',
    ):
        assert setting in config


def test_package_contains_only_its_independent_dependencies():
    package = (ROOT / 'package.xml').read_text(encoding='utf-8')
    assert '<name>inno_evacuation_voice</name>' in package
    assert 'inno_robot_bringup' not in package
