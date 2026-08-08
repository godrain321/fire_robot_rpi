import math

import pytest

from inno_autonav.waypoint_file import WaypointFileError, validated_pose_values


def document(x=1.0, frame='map'):
    return {
        'header': {'frame_id': frame},
        'poses': [{
            'header': {'frame_id': frame},
            'pose': {
                'position': {'x': x, 'y': 2.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            },
        }],
    }


def test_valid_document_preserves_values():
    assert validated_pose_values(document(), 'map') == (
        (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    )


def test_empty_queue_is_rejected_before_go():
    with pytest.raises(WaypointFileError, match='no poses'):
        validated_pose_values({'header': {'frame_id': 'map'}, 'poses': []}, 'map')


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_position_is_rejected(value):
    with pytest.raises(WaypointFileError, match='finite'):
        validated_pose_values(document(value), 'map')


def test_wrong_frame_and_bad_quaternion_are_rejected():
    with pytest.raises(WaypointFileError, match='does not match'):
        validated_pose_values(document(frame='odom'), 'map')
    bad = document()
    bad['poses'][0]['pose']['orientation']['w'] = 2.0
    with pytest.raises(WaypointFileError, match='quaternion norm'):
        validated_pose_values(bad, 'map')
