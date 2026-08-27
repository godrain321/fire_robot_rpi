from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped
import numpy as np

from inno_autonav.dynamic_obstacle_layer import (
    DynamicObstacleLayer,
    build_wall_exclusion_mask,
    cluster_obstacle_indices,
    inflate_sparse_obstacle_indices,
    is_in_forward_avoidance_window,
    match_people_to_clusters,
)


def test_nearby_scan_cells_form_one_physical_obstacle():
    clusters = cluster_obstacle_indices(
        [10 * 100 + 10, 10 * 100 + 11, 11 * 100 + 12],
        width=100,
        resolution=0.05,
        radius_m=0.15,
    )
    assert len(clusters) == 1
    assert len(clusters[0]) == 3


def test_separated_scan_cells_remain_separate_obstacles():
    clusters = cluster_obstacle_indices(
        [10 * 100 + 10, 10 * 100 + 30],
        width=100,
        resolution=0.05,
        radius_m=0.20,
    )
    assert len(clusters) == 2


def test_sparse_inflation_matches_integer_disk_geometry():
    data = inflate_sparse_obstacle_indices(
        [2 * 7 + 3], width=7, height=5, radius_cells=2
    )

    expected = {
        (x, y)
        for y in range(5)
        for x in range(7)
        if (x - 3) ** 2 + (y - 2) ** 2 <= 4
    }
    actual = {
        (int(x), int(y)) for y, x in np.argwhere(data >= 100)
    }
    assert actual == expected


def test_person_classification_matches_only_nearby_obstacle():
    layer = object.__new__(DynamicObstacleLayer)
    layer.person_match_radius = 0.75
    layer.classified_people = [(2.0, 3.0, 0.0)]

    assert layer._is_person(2.2, 3.1) is True
    assert layer._is_person(4.0, 3.0) is False


def test_person_marker_changes_from_red_to_blue():
    layer = object.__new__(DynamicObstacleLayer)
    layer.map_frame = 'map'
    layer.inflation_radius = 0.30
    layer.person_match_radius = 0.75
    layer.classified_people = []
    published = []
    layer.marker_publisher = type(
        'Publisher', (), {'publish': lambda self, message: published.append(message)}
    )()
    clusters = [({101}, 2.0, 3.0)]

    layer._publish_markers(Time(), clusters)
    assert len(published[-1].markers[1].points) == 1
    assert len(published[-1].markers[2].points) == 0

    layer.classified_people = [(2.0, 3.0, 0.0)]
    layer._publish_markers(Time(), clusters)
    assert len(published[-1].markers[1].points) == 0
    assert len(published[-1].markers[2].points) == 1


def test_one_person_changes_only_nearest_of_two_close_obstacles():
    clusters = [({101}, 2.0, 3.0), ({102}, 2.6, 3.0)]
    people = [(2.55, 3.0, 0.0)]

    assert match_people_to_clusters(clusters, people, 0.75) == {1}


def test_blue_person_marker_persists_without_current_lidar_cluster():
    layer = object.__new__(DynamicObstacleLayer)
    layer.map_frame = 'map'
    layer.inflation_radius = 0.30
    layer.person_match_radius = 0.75
    layer.classified_people = [(2.0, 3.0, 0.0)]
    published = []
    layer.marker_publisher = type(
        'Publisher', (), {'publish': lambda self, message: published.append(message)}
    )()

    layer._publish_markers(Time(), [])

    assert len(published[-1].markers[1].points) == 0
    assert len(published[-1].markers[2].points) == 1


def test_survivor_track_update_moves_existing_blue_marker_instead_of_adding_trail():
    layer = object.__new__(DynamicObstacleLayer)
    layer.map_frame = 'map'
    layer.classified_people = [(2.0, 3.0, 0.0)]
    layer.get_logger = lambda: type(
        'Logger', (), {'warning': lambda self, message: None}
    )()
    point = PointStamped()
    point.header.frame_id = 'map'
    point.point.x = 2.8
    point.point.y = 3.0

    layer._update_person(point, match_radius=2.5)

    assert len(layer.classified_people) == 1
    assert layer.classified_people[0][:2] == (2.8, 3.0)


def test_avoidance_window_is_front_facing_and_limited_to_one_metre():
    robot = (0.0, 0.0, 0.0)
    half_angle = np.deg2rad(45.0)
    assert is_in_forward_avoidance_window(robot, (0.8, 0.2), 1.0, half_angle)
    assert not is_in_forward_avoidance_window(robot, (1.1, 0.0), 1.0, half_angle)
    assert not is_in_forward_avoidance_window(robot, (-0.5, 0.0), 1.0, half_angle)


def test_wall_exclusion_radius_expands_saved_wall_buffer():
    data = np.zeros((21, 21), dtype=np.int8)
    data[10, 10] = 100
    mask = build_wall_exclusion_mask(data, resolution=0.05, radius_m=0.40)
    assert mask[10, 18]
    assert not mask[10, 19]


def test_mode3_blue_track_follows_current_lidar_cluster():
    import time

    layer = object.__new__(DynamicObstacleLayer)
    now = time.monotonic()
    layer.classified_people = [(2.0, 3.0, now)]
    layer.person_track_ids = [7]
    layer.person_track_velocities = [(0.0, 0.0)]
    layer.next_person_track_id = 8
    layer.person_track_stale = 1.5
    layer.person_track_match_radius = 1.0
    layer.person_track_max_speed = 1.8

    layer._track_people_from_current_clusters([({101, 102, 103}, 2.25, 3.0)])

    assert layer.person_track_ids == [7]
    assert layer.classified_people[0][0] == 2.25
    assert layer.classified_people[0][1] == 3.0
