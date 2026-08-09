import numpy as np

from inno_autonav.dynamic_obstacle_layer import (
    LargeObstacleTracker,
    build_wall_exclusion_mask,
    cluster_scan_points,
    dynamic_avoidance_enabled,
    is_clear_dynamic_candidate,
    minimum_cluster_points_for_mode,
    scan_range_membership,
)
from inno_autonav.grid_utils import MapGrid


def make_grid(data, resolution=0.05):
    values = np.asarray(data, dtype=np.int8)
    return MapGrid(
        width=values.shape[1],
        height=values.shape[0],
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        frame_id='map',
        data=values,
    )


def test_saved_wall_returns_and_nearby_free_cells_are_rejected():
    data = np.zeros((15, 15), dtype=np.int8)
    data[7, 7] = 100
    grid = make_grid(data)
    mask = build_wall_exclusion_mask(
        grid.data, grid.resolution, radius_m=0.25
    )

    assert not is_clear_dynamic_candidate(grid, mask, 7, 7)
    assert not is_clear_dynamic_candidate(grid, mask, 11, 7)
    assert is_clear_dynamic_candidate(grid, mask, 13, 7)


def test_unknown_map_boundary_gets_the_same_clearance_buffer():
    data = np.zeros((15, 15), dtype=np.int8)
    data[:, 0] = -1
    grid = make_grid(data)
    mask = build_wall_exclusion_mask(
        grid.data, grid.resolution, radius_m=0.15
    )

    assert not is_clear_dynamic_candidate(grid, mask, 2, 7)
    assert is_clear_dynamic_candidate(grid, mask, 5, 7)


def test_zero_clearance_preserves_known_free_cells_only():
    data = np.zeros((5, 5), dtype=np.int8)
    data[2, 2] = 100
    grid = make_grid(data)
    mask = build_wall_exclusion_mask(
        grid.data, grid.resolution, radius_m=0.0
    )

    assert not is_clear_dynamic_candidate(grid, mask, 2, 2)
    assert is_clear_dynamic_candidate(grid, mask, 3, 2)


def test_large_obstacle_requires_at_least_five_clustered_scan_points():
    four = [(0.03 * index, 0.0) for index in range(4)]
    five = [(0.03 * index, 0.0) for index in range(5)]

    assert cluster_scan_points(four, 0.18, min_points=5) == []
    clusters = cluster_scan_points(five, 0.18, min_points=5)
    assert len(clusters) == 1
    assert set(clusters[0]) == set(five)


def test_mode_two_uses_three_points_and_mode_three_uses_fifteen():
    assert minimum_cluster_points_for_mode(1, 3, 15) is None
    assert minimum_cluster_points_for_mode(2, 3, 15) == 3
    assert minimum_cluster_points_for_mode(3, 3, 15) == 15
    assert not dynamic_avoidance_enabled(2)
    assert dynamic_avoidance_enabled(3)

    fourteen = [(0.01 * index, 0.0) for index in range(14)]
    fifteen = [(0.01 * index, 0.0) for index in range(15)]
    assert len(cluster_scan_points(fourteen, 0.18, min_points=3)) == 1
    assert cluster_scan_points(fourteen, 0.18, min_points=15) == []
    assert len(cluster_scan_points(fifteen, 0.18, min_points=15)) == 1


def test_rescuee_and_avoidance_scan_ranges_both_stop_at_four_metres():
    common = dict(
        sensor_min=0.05,
        sensor_max=12.0,
        configured_min=0.15,
        avoidance_max=4.0,
        observation_max=4.0,
    )
    assert scan_range_membership(5.0, **common) == (False, False)
    assert scan_range_membership(4.0, **common) == (True, True)
    assert scan_range_membership(4.01, **common) == (False, False)


def test_large_obstacle_requires_three_consecutive_spatial_matches():
    tracker = LargeObstacleTracker(
        confirm_scans=3, match_radius_m=0.35, max_gap_sec=0.35
    )
    first = [[(1.0 + 0.03 * index, 2.0) for index in range(5)]]
    moved = [[(1.05 + 0.03 * index, 2.0) for index in range(5)]]

    assert tracker.update(first, 0.0) == []
    assert tracker.update(moved, 0.1) == []
    assert tracker.update(moved, 0.2) == moved

    assert tracker.update([], 0.3) == []
    assert tracker.update(first, 0.4) == []
    assert tracker.update(first, 0.5) == []
    assert tracker.update(first, 0.6) == first
