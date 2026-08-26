"""Numeric parity fixture against factory_v5 planner.exit_evaluator."""

from pathlib import Path
import sys

import numpy as np
import pytest

from inno_autonav.exit_evaluator import (
    ExitEvaluationConfig as RosConfig,
    ExitEvaluator as RosEvaluator,
    ExitHazardSnapshot,
    ExitItem,
)
from inno_autonav.weighted_planner import weighted_astar_search
from inno_hazard.hazard_belief import HazardGridGeometry


SIMULATION = (
    Path(__file__).resolve().parents[5]
    / "fire_robot" / "simulator" / "factory_v5"
)
SIMULATION_AVAILABLE = (
    SIMULATION / "planner" / "exit_evaluator.py"
).is_file()
pytestmark = pytest.mark.skipif(
    not SIMULATION_AVAILABLE,
    reason="sibling factory_v5 checkout is unavailable",
)
if SIMULATION_AVAILABLE:
    sys.path.insert(0, str(SIMULATION))
    from planner.exit_evaluator import (  # noqa: E402
        ExitEvaluationConfig as SimConfig,
        ExitEvaluator as SimEvaluator,
    )
    from world.entities import Exit as SimExit  # noqa: E402
    from world.fire_maps import EstimatedFireMap, MapMetadata  # noqa: E402


def test_same_fixture_matches_simulation_exit_metrics_and_reasons():
    size = 7
    sim_geometry = MapMetadata(0, 6, 0, 6, 1, size, size, (0, 0))
    sim_fire = EstimatedFireMap(
        sim_geometry, temperature_blocked_c=60, co_blocked_ppm=1600
    )
    temperature = np.full((size, size), 20.0)
    co = np.zeros((size, size))
    observed = np.ones((size, size), dtype=bool)
    sim_fire.replace_layers(
        temperature, co, observed, np.zeros((size, size))
    )
    costs = np.ones((size, size))
    costs[0, 1:4] = 2.5
    static = np.zeros((size, size), dtype=bool)
    dynamic = np.zeros_like(static)
    sim = SimEvaluator(
        sim_geometry, SimConfig(), temperature_blocked_c=60,
        co_blocked_ppm=1600, base_cost=1,
    ).evaluate(
        SimExit("EXIT1", (4, 0), (4, 0)), (0, 0), cost_map=costs,
        static_obstacle_map=static, dynamic_obstacle_map=dynamic,
        estimated_fire_map=sim_fire, evaluated_at=5,
    )

    geometry = HazardGridGeometry(size, size, 1.0)
    ros_snapshot = ExitHazardSnapshot(
        geometry, costs, temperature, co, observed, observed, observed,
        np.zeros((size, size)), static, dynamic, static, 11, 60, 1600, 1,
    )

    def planner(view, start, goal):
        return weighted_astar_search(
            view.final_cost, start, goal, costs_are_traversal=True,
            use_traversal_cost=True,
        )

    ros = RosEvaluator(RosConfig(), path_planner=planner).evaluate(
        ExitItem("EXIT1", (4.5, 0.5), (4.5, 0.5)), (0.5, 0.5),
        snapshot=ros_snapshot, evaluated_at=5,
    )
    assert ros.reachable == sim.reachable
    assert ros.accepted == sim.accepted
    assert ros.approach_position_grid == sim.approach_position_grid
    assert ros.path_length_m == pytest.approx(sim.path_length_m)
    assert ros.accumulated_risk_cost == pytest.approx(sim.accumulated_risk_cost)
    assert ros.max_path_temperature_c == sim.max_path_temperature_c
    assert ros.max_path_co_ppm == sim.max_path_co_ppm
    assert ros.exit_temperature_c == sim.exit_temperature_c
    assert ros.exit_co_ppm == sim.exit_co_ppm
    assert ros.unknown_ratio == sim.unknown_ratio
    assert tuple(item.value for item in ros.rejection_reasons) == tuple(
        item.value for item in sim.rejection_reasons
    )
