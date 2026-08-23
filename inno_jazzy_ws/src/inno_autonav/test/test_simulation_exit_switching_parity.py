"""Same-fixture parity checks against the sibling factory_v5 checkout.

Mirrors the sibling-checkout pattern used by the other test_simulation_*_parity.py
files: skip entirely if the fire_robot simulator checkout is not present next to
fire_robot_rpi. The ROS port's RouteTemperatureTrendMonitor takes an extra
temperature_observed_mask parameter (see exit_switching.py's docstring for why);
passing an all-True mask reproduces the simulation's own "any finite value counts"
semantics exactly, so this is a fair same-fixture comparison.
"""

from pathlib import Path
import sys

import numpy as np
import pytest

from inno_autonav.exit_switching import (
    DelayedCostSwitch as RosDelayedCostSwitch,
    RouteTemperatureTrendMonitor as RosMonitor,
    current_direction_world as ros_current_direction_world,
    is_opposite_direction as ros_is_opposite_direction,
)


SIMULATION_ROOT = (
    Path(__file__).resolve().parents[5] / "fire_robot" / "simulator" / "factory_v5"
)
if not (SIMULATION_ROOT / "navigation" / "exit_switching.py").is_file():
    pytest.skip("sibling factory_v5 checkout is unavailable", allow_module_level=True)
sys.path.insert(0, str(SIMULATION_ROOT))

from navigation.exit_switching import (  # noqa: E402
    DelayedCostSwitch as SimDelayedCostSwitch,
    RouteTemperatureTrendMonitor as SimMonitor,
    current_direction_world as sim_current_direction_world,
    is_opposite_direction as sim_is_opposite_direction,
)


PATH = ((0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0))


def test_route_temperature_trend_monitor_sequence_matches():
    ros = RosMonitor(evaluation_window=6, minimum_temperature_c=40.0)
    sim = SimMonitor(evaluation_window=6, minimum_temperature_c=40.0)
    observed = np.ones((6, 6), dtype=bool)
    for revision, cost in enumerate((2.0, 2.2, 2.4, 2.2, 2.6, 2.8, 3.0, 3.2), start=1):
        cost_map = np.full((6, 6), cost)
        temperature = np.full((6, 6), 41.0)
        ros_decision = ros.record(
            PATH, cost_map, temperature, observed,
            revision=revision, evaluated_at=float(revision),
        )
        sim_decision = sim.record(
            PATH, cost_map, temperature, revision=revision, evaluated_at=float(revision),
        )
        assert ros_decision.switch_required == sim_decision.switch_required
        assert ros_decision.consecutive_increases == sim_decision.consecutive_increases
        assert ros_decision.baseline_average_cost == sim_decision.baseline_average_cost
        assert ros_decision.current_average_cost == sim_decision.current_average_cost


def test_delayed_cost_switch_matches():
    ros = RosDelayedCostSwitch(1.0)
    sim = SimDelayedCostSwitch(1.0)
    ros.arm("EXIT2", "reason", 5.0)
    sim.arm("EXIT2", "reason", 5.0)
    for distance in (5.2, 5.9, 6.0, 6.1):
        assert ros.ready(distance) == sim.ready(distance)


def test_current_direction_world_matches():
    cases = [
        ((0.0, 0.0), (1.0, 0.0), [(0.0, 0.0), (0.0, 1.0)], 3.14),
        ((0.0, 0.0), None, [(0.0, 0.0), (0.0, 1.0)], 3.14),
        ((0.0, 0.0), None, [], 1.5707963267948966),
    ]
    for robot, waypoint, recent, yaw in cases:
        assert ros_current_direction_world(robot, waypoint, recent, yaw) == (
            sim_current_direction_world(robot, waypoint, recent, yaw)
        )


def test_is_opposite_direction_matches():
    direction = (1.0, 0.0)
    robot = (0.0, 0.0)
    for target in ((-5.0, 0.0), (0.0, 5.0), (5.0, 0.0), (-1.0, -1.0)):
        assert ros_is_opposite_direction(
            direction, robot, target, minimum_difference_deg=90.0
        ) == sim_is_opposite_direction(
            direction, robot, target, minimum_difference_deg=90.0
        )
