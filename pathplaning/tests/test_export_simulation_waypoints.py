import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from pathplaning.export_simulation_waypoints import (
    ExportError,
    OccupancyMap,
    Transform2D,
    build_waypoint_document,
    load_simulation_path,
    remove_duplicate_points,
    simplify_path,
    validate_waypoint_document,
    yaw_values,
)


def free_map(size=20):
    return OccupancyMap(1.0, (0.0, 0.0, 0.0), size, size,
                        np.zeros((size, size), dtype=np.int8))


def test_reference_waypoint_yaml_is_compatible():
    root = Path(__file__).resolve().parents[2]
    document = yaml.safe_load((root / "pathplaning/waypoint_queue_latest.yaml").read_text())
    assert validate_waypoint_document(document) == 3


def test_factory_transform_matches_all_measured_control_points():
    transform = Transform2D("factory_v3_world_xy_m", "map",
                            13.00189199, -29.41813371, 60.29982582450894)
    pairs = (
        ((9.347600752389777, 18.20457252873464),
         (1.8202285766601562, -12.278865814208984)),
        ((21.09187540024168, 9.461507399786615),
         (15.233551025390625, -6.409286975860596)),
        ((19.0071266789979, 26.14927174869926),
         (-0.29485416412353516, 0.04797935485839844)),
        ((12.93890985585197, 16.802116035062053),
         (4.817799091339111, -9.854209899902344)),
    )
    assert max(math.dist(transform.apply(source), target)
               for source, target in pairs) < 1e-6


@pytest.mark.parametrize(("points", "expected"), [
    (((0, 0), (1, 0)), (0.0, 0.0)),
    (((0, 0), (0, 1)), (math.pi / 2, math.pi / 2)),
    (((0, 0), (-1, 0)), (math.pi, math.pi)),
    (((0, 0), (0, -1)), (-math.pi / 2, -math.pi / 2)),
])
def test_yaw_cardinal_directions(points, expected):
    assert yaw_values(points) == pytest.approx(expected)


def test_quaternion_is_normalized_and_loader_shape_matches():
    document = build_waypoint_document(((1.0, 2.0), (2.0, 3.0)))
    assert validate_waypoint_document(document) == 2
    for entry in document["poses"]:
        q = entry["pose"]["orientation"]
        assert math.sqrt(sum(float(q[k]) ** 2 for k in "xyzw")) == pytest.approx(1.0)


def test_duplicates_removed_but_endpoints_and_corner_remain():
    grid = free_map()
    points = ((1, 1), (1, 1), (2, 1), (3, 1), (3, 2), (3, 3))
    assert remove_duplicate_points(points) == ((1, 1), (2, 1), (3, 1), (3, 2), (3, 3))
    result = simplify_path(points, grid, minimum_spacing_m=10,
                           direction_change_deg=5, allow_unknown=False)
    assert result == ((1.0, 1.0), (3.0, 1.0), (3.0, 3.0))


def test_long_straight_path_does_not_emit_spacing_waypoints():
    grid = free_map()
    points = tuple((1.0 + index * 0.25, 2.0) for index in range(40))
    result = simplify_path(points, grid, minimum_spacing_m=0.30,
                           direction_change_deg=8.0, allow_unknown=False)
    assert result == (points[0], points[-1])


def test_maximum_spacing_preserves_gradual_curve_shape():
    grid = free_map(size=40)
    points = tuple(
        (10.0 + 5.0 * math.sin(index * 0.02),
         10.0 + 5.0 * (1.0 - math.cos(index * 0.02)))
        for index in range(30)
    )
    result = simplify_path(
        points, grid, minimum_spacing_m=0.30, maximum_spacing_m=0.80,
        direction_change_deg=8.0, allow_unknown=False,
    )
    assert len(result) > 2
    assert result[0] == points[0]
    assert result[-1] == points[-1]


def test_shortcut_through_obstacle_is_not_accepted():
    grid = free_map()
    grid.occupancy[2, 2] = 100
    points = ((1, 1), (1, 3), (3, 3))
    result = simplify_path(points, grid, minimum_spacing_m=10,
                           direction_change_deg=5, allow_unknown=False)
    assert (1.0, 3.0) in result


def test_occupied_and_unknown_waypoints_are_rejected():
    grid = free_map()
    grid.occupancy[2, 2] = 100
    with pytest.raises(ExportError, match="occupied"):
        grid.check_cell((2, 2), False)
    grid.occupancy[2, 2] = -1
    with pytest.raises(ExportError, match="unknown"):
        grid.check_cell((2, 2), False)
    grid.check_cell((2, 2), True)


def test_bad_or_short_simulation_paths_are_rejected(tmp_path):
    missing_frame = tmp_path / "missing.yaml"
    missing_frame.write_text("points: [[0, 0], [1, 1]]\n")
    with pytest.raises(ExportError, match="coordinate_frame"):
        load_simulation_path(missing_frame)
    short = tmp_path / "short.yaml"
    short.write_text("coordinate_frame: simulation\npoints: [[0, 0]]\n")
    with pytest.raises(ExportError, match="at least two"):
        load_simulation_path(short)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_coordinates_are_rejected(bad):
    with pytest.raises(ExportError, match="finite"):
        Transform2D("a", "b", 0, 0, 0).apply((bad, 0))


def test_map_origin_resolution_and_rotated_origin():
    grid = OccupancyMap(0.5, (10.0, 20.0, math.pi / 2), 4, 4,
                        np.zeros((4, 4), dtype=np.int8))
    assert grid.world_to_grid((9.75, 20.75)) == (1, 0)


def test_waypoint_order_is_preserved():
    points = ((1.0, 1.0), (2.0, 3.0), (4.0, 2.0))
    document = build_waypoint_document(points)
    restored = tuple((entry["pose"]["position"]["x"],
                      entry["pose"]["position"]["y"])
                     for entry in document["poses"])
    assert restored == points
