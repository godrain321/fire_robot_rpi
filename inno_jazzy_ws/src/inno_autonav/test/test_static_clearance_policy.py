"""Static-clearance contract shared by waypoint and cell-A* paths."""

import math
from pathlib import Path

import numpy as np
import yaml

from inno_autonav.grid_utils import inflate_occupied_cells
from inno_autonav.safe_path_simplifier import expanded_path, segment_is_safe
from inno_autonav.weighted_planner import (
    weighted_a_star_with_escape,
    weighted_astar_search,
)


RESOLUTION_M = 0.05
CLEARANCE_M = 0.80
CLEARANCE_CELLS = math.ceil(CLEARANCE_M / RESOLUTION_M)


def test_runtime_yaml_sets_common_static_clearance_to_eight_tenths():
    config_path = Path(__file__).parents[1] / "config" / "autonav_params.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert config["astar_replanner"]["ros__parameters"][
        "path_block_check_radius"
    ] == CLEARANCE_M


def inflated_map(size=81, obstacle=(40, 40)):
    source = np.zeros((size, size), dtype=np.int8)
    source[obstacle[1], obstacle[0]] = 100
    return inflate_occupied_cells(source, CLEARANCE_CELLS)


def test_occupied_cells_are_inflated_to_eight_tenths_of_a_metre():
    costs = inflated_map()
    assert costs[40, 40 + CLEARANCE_CELLS] == 100
    assert costs[40, 40 + CLEARANCE_CELLS + 1] == 0
    assert costs[40 + 12, 40 + 12] == 0  # 0.849 m from the obstacle centre


def test_astar_path_excludes_cells_inside_static_clearance():
    costs = inflated_map()
    result = weighted_astar_search(costs, (2, 40), (78, 40))
    assert result.path
    assert all(costs[y, x] < 100 for x, y in result.path)
    assert min(
        math.hypot(x - 40, y - 40) * RESOLUTION_M
        for x, y in result.path
    ) > CLEARANCE_M


def test_path_farther_than_static_clearance_remains_available():
    costs = inflated_map()
    result = weighted_astar_search(costs, (2, 70), (78, 70))
    assert result.path
    assert all(y == 70 for _, y in result.path)


def test_diagonal_corner_cut_is_rejected_for_waypoint_and_astar_paths():
    costs = np.zeros((4, 4), dtype=np.int8)
    costs[1, 2] = 100
    costs[2, 1] = 100
    assert not segment_is_safe((1, 1), (2, 2), costs, True)
    astar_path = weighted_astar_search(costs, (1, 1), (2, 2)).path
    assert astar_path
    assert astar_path != ((1, 1), (2, 2))


def test_waypoint_segment_and_astar_use_the_same_inflated_cells():
    costs = inflated_map()
    waypoint_segment = ((2, 40), (78, 40))
    assert not segment_is_safe(*waypoint_segment, costs, True)
    detour = weighted_astar_search(costs, *waypoint_segment)
    assert detour.path
    assert all(costs[y, x] < 100 for x, y in expanded_path(detour.path))


def test_start_escape_remains_allowed_but_an_inflated_goal_is_rejected():
    static = np.zeros((31, 31), dtype=np.int8)
    static[15, 15] = 100
    costs = inflate_occupied_cells(static, 4)
    escaped = weighted_a_star_with_escape(
        costs, (18, 15), (25, 15), static >= 100
    )
    assert escaped.path
    assert escaped.escape_path
    blocked_goal = weighted_a_star_with_escape(
        costs, (25, 15), (18, 15), static >= 100
    )
    assert not blocked_goal.path
