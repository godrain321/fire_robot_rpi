from geometry_msgs.msg import Point
import math
import pytest
from visualization_msgs.msg import Marker, MarkerArray

from inno_autonav.victim_fusion import (
    LidarCluster,
    RescueeTracker,
    cluster_points,
    filtered_dynamic_markers,
    follow_victim_positions,
    select_unique_range_match,
    valid_mmwave_match_distance,
)


def selection(points, distance):
    return select_unique_range_match(
        points,
        robot_x=0.0,
        robot_y=0.0,
        sensor_yaw_rad=0.0,
        horizontal_fov_rad=math.radians(100.0),
        mmwave_distance_m=distance,
        cluster_radius_m=0.22,
        max_cluster_points=3,
        absolute_tolerance_m=0.8,
        relative_tolerance=0.25,
        maximum_tolerance_m=1.5,
        ambiguity_margin_m=0.35,
    )


def test_one_to_three_nearby_lidar_points_form_one_person_cluster():
    groups = cluster_points(
        [(2.0, 0.0), (2.05, 0.03), (1.98, -0.02), (4.0, 1.0)],
        connection_radius_m=0.22,
    )
    assert sorted(len(group) for group in groups) == [1, 3]

    selected = selection(
        [(2.0, 0.0), (2.05, 0.03), (1.98, -0.02), (4.0, 1.0)],
        distance=2.2,
    )
    assert selected is not None
    assert selected.point_count == 3
    assert selected.range_m == pytest.approx(2.01, abs=0.05)


def test_rescue_zone_accepts_zero_to_four_metres_only():
    assert valid_mmwave_match_distance(0.01, 4.0)
    assert valid_mmwave_match_distance(4.0, 4.0)
    assert not valid_mmwave_match_distance(0.0, 4.0)
    assert not valid_mmwave_match_distance(4.01, 4.0)


def test_uncertain_four_metre_mmwave_can_match_nearby_front_lidar():
    selected = selection([(3.2, 0.0), (3.25, 0.02)], distance=4.0)
    assert selected is not None
    assert selected.point_count == 2
    assert selected.range_m == pytest.approx(3.225, abs=0.03)


def test_similar_range_clusters_are_left_ambiguous():
    selected = selection([(1.8, 0.0), (2.2, 0.0)], distance=2.0)
    assert selected is None


def test_cluster_outside_forward_sensor_fov_is_not_a_person_candidate():
    assert selection([(-2.0, 0.0)], distance=2.0) is None
    outside_angle = math.radians(51.0)
    assert selection([
        (2.0 * math.cos(outside_angle), 2.0 * math.sin(outside_angle))
    ], distance=2.0) is None
    inside_angle = math.radians(45.0)
    assert selection([
        (2.0 * math.cos(inside_angle), 2.0 * math.sin(inside_angle))
    ], distance=2.0) is not None


def test_four_point_cluster_is_not_a_person_candidate():
    assert selection(
        [(2.0, 0.0), (2.03, 0.0), (2.06, 0.0), (2.09, 0.0)],
        distance=2.0,
    ) is None


def test_moving_rescuee_keeps_its_classification_and_updates_position():
    original = [(2.0, 1.0)]
    moved = follow_victim_positions(
        original,
        [(2.35, 1.0), (2.38, 1.02)],
        cluster_radius_m=0.22,
        max_cluster_points=3,
        follow_radius_m=0.65,
        ambiguity_margin_m=0.15,
    )
    assert moved[0][0] == pytest.approx(2.365)
    assert moved[0][1] == pytest.approx(1.01)
    assert follow_victim_positions(
        moved,
        [],
        cluster_radius_m=0.22,
        max_cluster_points=3,
        follow_radius_m=0.65,
        ambiguity_margin_m=0.15,
    ) == moved


    four_point_cluster = [
        (2.1, 1.0), (2.13, 1.0), (2.16, 1.0), (2.19, 1.0)
    ]
    assert follow_victim_positions(
        original,
        four_point_cluster,
        cluster_radius_m=0.22,
        max_cluster_points=3,
        follow_radius_m=0.65,
        ambiguity_margin_m=0.15,
    ) == original


def test_tracker_requires_repeated_matches_then_latches_until_clear():
    tracker = RescueeTracker(
        confirm_hits=3,
        confirm_sec=0.5,
        evidence_timeout_sec=1.5,
        track_radius_m=0.35,
        merge_radius_m=0.55,
    )
    cluster = LidarCluster(2.0, 1.0, 2, 2.2, 0.1)
    assert tracker.observe(cluster, 0.0) is None
    assert tracker.observe(cluster, 0.3) is None
    victim = tracker.observe(cluster, 0.6)
    assert victim is not None
    assert tracker.victims == [victim]

    tracker.expire(100.0)
    assert tracker.victims == [victim]
    assert tracker.observe(cluster, 101.0) is None
    tracker.clear()
    assert tracker.victims == []


def test_latched_victim_is_removed_from_red_display_only():
    clear = Marker(action=Marker.DELETEALL)
    red = Marker()
    red.type = Marker.SPHERE_LIST
    red.action = Marker.ADD
    red.points = [Point(x=2.0, y=1.0), Point(x=4.0, y=1.0)]
    original = MarkerArray(markers=[clear, red])

    filtered = filtered_dynamic_markers(
        original, victims=[(2.1, 1.0)], suppression_radius_m=0.5
    )
    assert len(original.markers[1].points) == 2
    assert len(filtered.markers[1].points) == 1
    assert filtered.markers[1].points[0].x == 4.0
