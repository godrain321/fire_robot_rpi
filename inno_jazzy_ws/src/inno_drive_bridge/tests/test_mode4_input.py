import pytest

from inno_drive_bridge.mode4_input import (
    command_source_for_drive_mode,
    parse_mode4_waypoints,
)


@pytest.mark.parametrize(
    'text, expected',
    [
        ('w1,w5', ['w1', 'w5']),
        ('W1, W5, w159', ['w1', 'w5', 'w159']),
        ('w01 w2', ['w1', 'w2']),
        (' w1,\tw5  w6 ', ['w1', 'w5', 'w6']),
    ],
)
def test_parse_mode4_waypoints(text, expected):
    assert parse_mode4_waypoints(text) == expected


@pytest.mark.parametrize('text', ['', 'w1', 'w0,w2', 'w1,exit2', 'w1;w2'])
def test_reject_invalid_mode4_waypoints(text):
    with pytest.raises(ValueError):
        parse_mode4_waypoints(text)


def test_mode4_uses_autonomous_cmd_vel_source():
    assert command_source_for_drive_mode(1) == 1
    assert command_source_for_drive_mode(2) == 2
    assert command_source_for_drive_mode(4) == 2
    with pytest.raises(ValueError):
        command_source_for_drive_mode(3)
