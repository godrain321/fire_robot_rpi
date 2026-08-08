from pathlib import Path
import math

import numpy as np
from PIL import Image
import pytest
from sensor_msgs.msg import LaserScan
import yaml

from inno_autonav.astar_replanner import (
    astar_search,
    footprint_clearance_radius,
    simplify_path,
)
from inno_autonav.skid_path_follower import nearest_scan_clearances
from inno_autonav.grid_utils import (
    MapGrid,
    grid_to_world,
    inflate_occupied_cells,
    load_pgm_as_occupancy,
    world_to_grid,
)


def test_pgm_y_axis_is_flipped(tmp_path: Path):
    pixels = np.full((3, 4), 254, dtype=np.uint8)
    pixels[0, 1] = 0  # top PGM row must become highest OccupancyGrid y row.
    Image.fromarray(pixels, mode='L').save(tmp_path / 'map.pgm', format='PPM')
    metadata = {
        'image': 'map.pgm', 'resolution': 0.5, 'origin': [-1.0, -2.0, 0.0],
        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }
    (tmp_path / 'map.yaml').write_text(
        yaml.safe_dump(metadata), encoding='utf-8'
    )
    grid = load_pgm_as_occupancy(tmp_path / 'map.yaml')
    assert grid.data[2, 1] == 100
    assert grid.data[0, 1] == 0


def test_world_grid_round_trip_uses_cell_center():
    grid = MapGrid(10, 10, 0.1, -1.0, -2.0, 0.0, 'map', np.zeros((10, 10)))
    assert world_to_grid(-0.75, -1.65, grid) == (2, 3)
    assert grid_to_world(2, 3, grid) == pytest.approx((-0.75, -1.65))


def test_inflation_and_astar_avoid_wall_gap():
    data = np.zeros((12, 12), dtype=np.int8)
    data[:, 6] = 100
    data[7, 6] = 0
    path = astar_search(data, (1, 1), (10, 10), True, True)
    assert path
    assert (6, 7) in path
    simplified = simplify_path(path, data, True)
    assert simplified[0] == (1, 1)
    assert simplified[-1] == (10, 10)
    inflated = inflate_occupied_cells(data, 1)
    assert inflated[7, 6] == 100
    assert not astar_search(inflated, (1, 1), (10, 10), True, True)


def test_unknown_is_occupied_switch():
    data = np.zeros((3, 5), dtype=np.int8)
    data[:, 2] = -1
    assert not astar_search(data, (0, 1), (4, 1), True, False)
    assert astar_search(data, (0, 1), (4, 1), False, False)


def test_skid_footprint_clearance_uses_half_diagonal_plus_margin():
    radius = footprint_clearance_radius(0.39, 0.20, 0.10)
    assert radius == pytest.approx(0.3192, abs=0.0002)


def test_single_scan_point_still_triggers_immediate_safety_ranges():
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = 0.1
    scan.range_min = 0.05
    scan.range_max = 12.0
    scan.ranges = [0.25]

    front, all_around = nearest_scan_clearances(scan, 0.61)
    assert front == pytest.approx(0.25)
    assert all_around == pytest.approx(0.25)


def test_side_scan_point_blocks_rotation_but_not_front_emergency():
    scan = LaserScan()
    scan.angle_min = 1.0
    scan.angle_increment = 0.1
    scan.range_min = 0.05
    scan.range_max = 12.0
    scan.ranges = [0.25]

    front, all_around = nearest_scan_clearances(scan, 0.61)
    assert math.isinf(front)
    assert all_around == pytest.approx(0.25)
