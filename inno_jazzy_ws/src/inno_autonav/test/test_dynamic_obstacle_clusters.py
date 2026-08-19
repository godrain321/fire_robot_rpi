from builtin_interfaces.msg import Time

from inno_autonav.dynamic_obstacle_layer import (
    DynamicObstacleLayer,
    cluster_obstacle_indices,
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
