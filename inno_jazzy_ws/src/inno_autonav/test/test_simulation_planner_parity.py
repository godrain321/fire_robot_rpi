"""Same-fixture parity checks against the sibling factory_v5 checkout."""

import math
from pathlib import Path
import sys

import numpy as np
import pytest

from inno_autonav.weighted_planner import (
    traversal_multiplier,
    weighted_a_star_with_escape,
    weighted_astar_search,
)


SIMULATION_ROOT = (
    Path(__file__).resolve().parents[5] / "fire_robot" / "simulator" / "factory_v5"
)
SIMULATION_AVAILABLE = (
    SIMULATION_ROOT / "planner" / "a_star.py"
).is_file()
pytestmark = pytest.mark.skipif(
    not SIMULATION_AVAILABLE,
    reason="sibling factory_v5 checkout is unavailable",
)
if SIMULATION_AVAILABLE:
    sys.path.insert(0, str(SIMULATION_ROOT))
    from planner.a_star import (  # noqa: E402
        weighted_a_star as simulation_weighted_a_star,
        weighted_a_star_with_escape as simulation_weighted_a_star_with_escape,
    )


def _simulation_costmap(encoded: np.ndarray) -> np.ndarray:
    return np.asarray([
        [traversal_multiplier(value, 24.0, 1.5) for value in row]
        for row in encoded
    ])


def test_weighted_astar_same_fixture_matches_factory_v5_cost_and_path():
    encoded = np.zeros((7, 9), dtype=float)
    encoded[3, 2:7] = 90.0
    start, goal = (0, 3), (8, 3)
    ros = weighted_astar_search(encoded, start, goal)
    simulation = simulation_weighted_a_star(
        _simulation_costmap(encoded), start, goal
    )
    assert ros.path == tuple(simulation.path)
    assert ros.total_cost == pytest.approx(simulation.total_cost)


def test_escape_same_fixture_matches_factory_v5_metadata_and_cost():
    encoded = np.zeros((5, 6), dtype=float)
    encoded[1:4, 0:2] = 100.0
    static = np.zeros_like(encoded, dtype=bool)
    start, goal = (0, 2), (5, 2)
    simulation_costs = _simulation_costmap(encoded)
    simulation_costs[encoded >= 100.0] = math.inf
    ros = weighted_a_star_with_escape(encoded, start, goal, static)
    simulation = simulation_weighted_a_star_with_escape(
        simulation_costs, start, goal, static
    )
    assert ros.path == tuple(simulation.path)
    assert ros.escape_path == simulation.escape_path
    assert ros.replan_start == simulation.replan_start
    assert ros.total_cost == pytest.approx(simulation.total_cost)
