import pytest

from inno_autonav.waypoint_selection import (
    resolve_mode4_waypoints,
    waypoint_names_from_document,
)


def test_named_waypoints_keep_yaml_names_for_mode4():
    document = {
        'frame_id': 'map',
        'poses': {
            'w1': {'x': 1.0, 'y': 2.0},
            'w5': {'x': 3.0, 'y': 4.0},
            'w6': {'x': 5.0, 'y': 6.0},
        },
    }
    names = waypoint_names_from_document(document)
    selected_names, indices = resolve_mode4_waypoints('W1,w5,w6', names)
    assert selected_names == ['w1', 'w5', 'w6']
    assert indices == [0, 1, 2]


def test_positional_snapshot_gets_stable_w_names():
    assert waypoint_names_from_document({'poses': [{}, {}]}) == ['w1', 'w2']


@pytest.mark.parametrize('selection', ['w1', 'w1,w999', 'exit1,w5'])
def test_invalid_mode4_selection_is_rejected(selection):
    with pytest.raises(ValueError):
        resolve_mode4_waypoints(selection, ['w1', 'w5', 'w6'])
