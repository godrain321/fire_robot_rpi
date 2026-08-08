import math

import pytest
from geometry_msgs.msg import TransformStamped

from inno_robot_bringup.tf_heading_marker import (
    delete_heading_marker,
    marker_from_transform,
)
from visualization_msgs.msg import Marker


def test_marker_tracks_tf_position_and_heading_with_one_stable_id():
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'
    transform.transform.translation.x = 2.5
    transform.transform.translation.y = -1.25
    transform.transform.rotation.z = math.sin(0.5 * 1.2)
    transform.transform.rotation.w = math.cos(0.5 * 1.2)

    marker = marker_from_transform(
        transform, length_m=0.45, width_m=0.10
    )

    assert marker.header.frame_id == 'map'
    assert marker.ns == 'robot_heading_tf'
    assert marker.id == 0
    assert marker.type == Marker.ARROW
    assert marker.pose.position.x == 2.5
    assert marker.pose.position.y == -1.25
    assert marker.pose.orientation.z == transform.transform.rotation.z
    assert marker.pose.orientation.w == transform.transform.rotation.w
    assert marker.scale.x == 0.45
    assert marker.scale.y == marker.scale.z == 0.10


def test_delete_marker_uses_same_identity_outside_mode_three():
    transform = TransformStamped()
    marker = delete_heading_marker('map', transform.header.stamp)
    assert marker.header.frame_id == 'map'
    assert marker.ns == 'robot_heading_tf'
    assert marker.id == 0
    assert marker.action == Marker.DELETE


def test_marker_rejects_invalid_dimensions_and_tf_values():
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    with pytest.raises(ValueError, match='positive'):
        marker_from_transform(transform, length_m=0.0)

    transform.transform.translation.x = float('nan')
    with pytest.raises(ValueError, match='finite'):
        marker_from_transform(transform)
