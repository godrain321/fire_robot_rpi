from pathlib import Path

from std_msgs.msg import Int32, String

from inno_robot_bringup.mode3_audio_guide import (
    Mode3AudioGuide,
    MODE4_SURVIVOR_CONFIRMED_STATUS,
    SURVIVOR_CONFIRMED_STATUS,
    alsa_control_path,
    build_amixer_command,
    build_aplay_command,
    discover_usb_alsa_device,
    expand_audio_directory,
    select_audio_directory,
)


def test_usb_alsa_card_is_selected_by_stable_card_id():
    cards = '''
 0 [vc4hdmi0      ]: vc4-hdmi - vc4-hdmi-0
                      vc4-hdmi-0
 2 [Device        ]: USB-Audio - USB Audio Device
                      USB Audio Device at usb-xhci-hcd.1, full speed
'''

    assert discover_usb_alsa_device(cards) == 'plughw:CARD=Device,DEV=0'


def test_no_usb_audio_card_does_not_fall_back_to_hdmi():
    cards = '''
 0 [vc4hdmi0      ]: vc4-hdmi - vc4-hdmi-0
                      vc4-hdmi-0
'''

    assert discover_usb_alsa_device(cards) is None


def test_stable_card_id_maps_to_kernel_control_device():
    cards = """
 0 [vc4hdmi0      ]: vc4-hdmi - vc4-hdmi-0
 1 [Device         ]: USB-Audio - USB Audio Device
"""

    assert alsa_control_path(
        'plughw:CARD=Device,DEV=0', cards
    ) == Path('/dev/snd/controlC1')


def test_audio_directory_expands_home(monkeypatch):
    monkeypatch.setenv('HOME', '/tmp/robot-user')

    assert expand_audio_directory('~/fire_robot_audio') == Path(
        '/tmp/robot-user/fire_robot_audio'
    )


def test_packaged_audio_is_fallback_for_a_fresh_clone(tmp_path):
    preferred = tmp_path / 'missing-operator-audio'
    packaged = tmp_path / 'packaged-audio'
    packaged.mkdir()
    for name in (
        'survivor_detected.wav',
        'follow_me.wav',
        'arrived_exit.wav',
        'emergency_stop.wav',
    ):
        (packaged / name).touch()

    assert select_audio_directory(preferred, packaged) == packaged


def test_aplay_command_never_uses_a_shell():
    command = build_aplay_command(
        '/usr/bin/aplay', 'plughw:CARD=Device,DEV=0', Path('/tmp/guide.wav')
    )

    assert command == [
        '/usr/bin/aplay',
        '--quiet',
        '-D',
        'plughw:CARD=Device,DEV=0',
        '/tmp/guide.wav',
    ]


def test_amixer_command_targets_only_the_selected_usb_card():
    command = build_amixer_command(
        '/usr/bin/amixer', 'plughw:CARD=Device,DEV=0', 80
    )

    assert command == [
        '/usr/bin/amixer',
        '--quiet',
        '--card',
        'Device',
        'sset',
        'Speaker',
        '80%',
        'unmute',
    ]


def _bare_audio_node():
    node = object.__new__(Mode3AudioGuide)
    node.drive_mode = 3
    node.enabled = True
    node.last_mode3_status = ''
    node.last_mode4_status = ''
    node.armed_for_mode3 = False
    node.armed_for_mode4 = False
    node.get_logger = lambda: type(
        'Logger', (), {'debug': lambda _self, _message: None}
    )()
    node.plays = []
    node._play_survivor_guide = lambda: node.plays.append(True)
    return node


def test_live_blue_marker_transition_plays_exactly_once():
    node = _bare_audio_node()
    node._on_drive_mode(Int32(data=3))
    node._on_mode3_status(String(data='MODE3_CAMERA_OBSERVING'))

    node._on_mode3_status(String(data=SURVIVOR_CONFIRMED_STATUS))
    node._on_mode3_status(String(data=SURVIVOR_CONFIRMED_STATUS))

    assert node.plays == [True]


def test_transient_blue_status_from_before_startup_is_not_replayed():
    node = _bare_audio_node()

    node._on_mode3_status(String(data=SURVIVOR_CONFIRMED_STATUS))

    assert node.plays == []


def test_new_inspection_rearms_one_more_live_trigger():
    node = _bare_audio_node()
    node._on_mode3_status(String(data='MODE3_READY:PRESS_SPACE'))
    node._on_mode3_status(String(data=SURVIVOR_CONFIRMED_STATUS))
    node._on_mode3_status(String(data='MODE3_WAITING_FOR_DYNAMIC_OBSTACLE'))
    node._on_mode3_status(String(data=SURVIVOR_CONFIRMED_STATUS))

    assert node.plays == [True, True]


def test_mode4_live_blue_marker_transition_plays_exactly_once():
    node = _bare_audio_node()
    node._on_drive_mode(Int32(data=4))
    node._on_mode4_status(String(data='MODE4_CAMERA_YOLO_OBSERVING'))

    node._on_mode4_status(String(data=MODE4_SURVIVOR_CONFIRMED_STATUS))
    node._on_mode4_status(String(data=MODE4_SURVIVOR_CONFIRMED_STATUS))

    assert node.plays == [True]


def test_mode4_transient_blue_status_from_before_startup_is_not_replayed():
    node = _bare_audio_node()
    node._on_drive_mode(Int32(data=4))

    node._on_mode4_status(String(data=MODE4_SURVIVOR_CONFIRMED_STATUS))

    assert node.plays == []
