"""Same-fixture parity checks against the sibling factory_v5 checkout.

Mirrors the sibling-checkout pattern used by test_simulation_planner_parity.py /
test_simulation_evacuation_planner_parity.py: skip entirely if the fire_robot
simulator checkout is not present next to fire_robot_rpi.
"""

from pathlib import Path
import sys

import numpy as np
import pytest

from inno_autonav.event_replanning import (
    EventReplanningConfig as RosConfig,
    EventReplanningPolicy as RosPolicy,
)


SIMULATION_ROOT = (
    Path(__file__).resolve().parents[5] / "fire_robot" / "simulator" / "factory_v5"
)
SIMULATION_AVAILABLE = (
    SIMULATION_ROOT / "navigation" / "event_replanning.py"
).is_file()
pytestmark = pytest.mark.skipif(
    not SIMULATION_AVAILABLE,
    reason="sibling factory_v5 checkout is unavailable",
)
if SIMULATION_AVAILABLE:
    sys.path.insert(0, str(SIMULATION_ROOT))
    from navigation.event_replanning import (  # noqa: E402
        EventReplanningConfig as SimConfig,
        EventReplanningPolicy as SimPolicy,
    )


PATH = ((1, 1), (2, 1), (3, 1), (4, 1))


def costmap(size=6, value=1.0):
    return np.full((size, size), value, dtype=float)


def make_pair(**overrides):
    return RosPolicy(RosConfig(**overrides)), SimPolicy(SimConfig(**overrides))


def assert_matches(ros_decision, sim_decision):
    assert ros_decision.required == sim_decision.required
    assert ros_decision.immediate_stop == sim_decision.immediate_stop
    assert ros_decision.invalidate_current_path == sim_decision.invalidate_current_path
    assert ros_decision.reason.value == sim_decision.reason.value
    assert int(ros_decision.priority) == int(sim_decision.priority)
    assert ros_decision.affected_cell_grid == sim_decision.affected_cell_grid


def evaluate_both(ros, sim, **kwargs):
    return ros.evaluate(**kwargs), sim.evaluate(**kwargs)


def test_safe_path_matches():
    ros, sim = make_pair()
    assert_matches(*evaluate_both(
        ros, sim, current_path=PATH, current_costmap=costmap(),
        costmap_revision=1, robot_pose=(1.0, 1.0), elapsed_time=0.0,
    ))


def test_dynamic_obstacle_matches():
    ros, sim = make_pair()
    dynamic = np.zeros((6, 6), dtype=bool)
    dynamic[1, 3] = True
    assert_matches(*evaluate_both(
        ros, sim, current_path=PATH, current_costmap=costmap(),
        costmap_revision=1, dynamic_obstacle_map=dynamic,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    ))


def test_blocked_cost_cell_matches():
    ros, sim = make_pair()
    grid = costmap()
    grid[1, 2] = np.inf
    assert_matches(*evaluate_both(
        ros, sim, current_path=PATH, current_costmap=grid,
        costmap_revision=1, robot_pose=(1.0, 1.0), elapsed_time=0.0,
    ))


def test_out_of_map_matches():
    ros, sim = make_pair()
    path = ((1, 1), (2, 1), (99, 99))
    assert_matches(*evaluate_both(
        ros, sim, current_path=path, current_costmap=costmap(),
        costmap_revision=1, robot_pose=(1.0, 1.0), elapsed_time=0.0,
    ))


def test_thermal_hard_block_matches():
    ros, sim = make_pair()
    temperature = np.zeros((6, 6))
    temperature[1, 2] = 61.0
    observed = np.zeros((6, 6), dtype=bool)
    observed[1, 2] = True
    assert_matches(*evaluate_both(
        ros, sim, current_path=PATH, current_costmap=costmap(),
        costmap_revision=1, temperature_map=temperature,
        temperature_observed_mask=observed,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    ))


def test_co_hard_block_matches():
    ros, sim = make_pair()
    co = np.zeros((6, 6))
    co[1, 3] = 1601.0
    observed = np.zeros((6, 6), dtype=bool)
    observed[1, 3] = True
    assert_matches(*evaluate_both(
        ros, sim, current_path=PATH, current_costmap=costmap(),
        costmap_revision=1, co_map=co, co_observed_mask=observed,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    ))


def test_periodic_and_distance_sequences_match():
    ros, sim = make_pair(periodic_interval_s=5.0, periodic_travel_distance_m=2.0)
    assert_matches(*evaluate_both(
        ros, sim, current_path=PATH, current_costmap=costmap(),
        costmap_revision=1, robot_pose=(0.0, 0.0), elapsed_time=0.0,
    ))
    assert_matches(*evaluate_both(
        ros, sim, current_path=PATH, current_costmap=costmap(),
        costmap_revision=1, robot_pose=(3.0, 0.0), elapsed_time=1.0,
    ))
    assert_matches(*evaluate_both(
        ros, sim, current_path=PATH, current_costmap=costmap(),
        costmap_revision=1, robot_pose=(3.0, 0.0), elapsed_time=6.0,
    ))


def test_hysteresis_and_duplicate_suppression_sequence_matches():
    ros, sim = make_pair()
    temperature = np.zeros((6, 6))
    temperature[1, 2] = 61.0
    observed = np.zeros((6, 6), dtype=bool)
    observed[1, 2] = True
    kwargs = dict(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        temperature_map=temperature, temperature_observed_mask=observed,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    ros_decision, sim_decision = evaluate_both(ros, sim, **kwargs)
    assert_matches(ros_decision, sim_decision)
    ros.mark_processed(
        ros_decision, costmap_revision=1, elapsed_time=0.0,
        robot_pose=(1.0, 1.0), selected_exit_id="EXIT1",
    )
    sim.mark_processed(
        sim_decision, costmap_revision=1, elapsed_time=0.0,
        robot_pose=(1.0, 1.0), selected_exit_id="EXIT1",
    )
    # Same reason + same revision immediately after -> both suppress.
    assert_matches(*evaluate_both(ros, sim, **{**kwargs, "elapsed_time": 0.05}))
    # Readings between release (55) and block (60) keep the latch on both sides.
    for step, value in enumerate((59.0, 58.0, 56.0), start=1):
        temperature = np.zeros((6, 6))
        temperature[1, 2] = value
        step_kwargs = {
            **kwargs, "temperature_map": temperature,
            "costmap_revision": 1 + step, "elapsed_time": float(step),
        }
        ros_decision, sim_decision = evaluate_both(ros, sim, **step_kwargs)
        assert_matches(ros_decision, sim_decision)
        ros.mark_processed(
            ros_decision, costmap_revision=1 + step, elapsed_time=float(step),
            robot_pose=(1.0, 1.0), selected_exit_id="EXIT1",
        )
        sim.mark_processed(
            sim_decision, costmap_revision=1 + step, elapsed_time=float(step),
            robot_pose=(1.0, 1.0), selected_exit_id="EXIT1",
        )
