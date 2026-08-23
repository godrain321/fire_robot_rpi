"""Numeric parity with the sibling factory_v5 PartialFireCostmap."""

from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

from inno_hazard.hazard_belief import (
    HazardBelief,
    HazardBeliefConfig,
    HazardGridGeometry,
)


SIM_ROOT = Path(__file__).resolve().parents[5] / "fire_robot" / "simulator" / "factory_v5"
if not (SIM_ROOT / "mapping" / "partial_costmap.py").is_file():
    pytest.skip("sibling factory_v5 checkout unavailable", allow_module_level=True)
sys.path.insert(0, str(SIM_ROOT))

from mapping.partial_costmap import PartialCostmapConfig, PartialFireCostmap  # noqa: E402


class SimulationGrid:
    width = 7
    height = 7
    resolution = 1.0

    @staticmethod
    def world_to_grid(x, y):
        return int(round(x)), int(round(y))

    @staticmethod
    def in_bounds(cell):
        return 0 <= cell[0] < 7 and 0 <= cell[1] < 7


def test_same_observations_produce_same_partial_cost_layers_and_revision():
    static = np.zeros((7, 7), dtype=bool)
    sim_config = PartialCostmapConfig(
        grid_resolution=1.0, use_inflation=False,
        temperature_safe=40.0, temperature_blocked=60.0,
        temperature_weight=24.0, temperature_power=1.5,
        co_safe=100.0, co_blocked=1600.0,
        co_weight=8.0, co_power=2.0,
        gas_update_radius=0.0,
    )
    ros_config = HazardBeliefConfig(
        temperature_safe_c=40.0, temperature_blocked_c=60.0,
        temperature_weight=24.0, temperature_power=1.5,
        co_enabled=True, co_safe_ppm=100.0, co_blocked_ppm=1600.0,
        co_weight=8.0, co_power=2.0, gas_update_radius_m=0.0,
    )
    simulation = PartialFireCostmap(SimulationGrid(), static, sim_config)
    ros = HazardBelief(HazardGridGeometry(7, 7, 1.0), static, ros_config)

    ray = SimpleNamespace(
        valid=True, hit_world_position=(2.0, 2.0, 0.0), row=0, col=0
    )
    simulation.update_thermal_observations(np.array([[50.0]]), (ray,), 1.0)
    ros.update_temperature_observations((((2, 2), 50.0),), 1.0)
    simulation.update_co_observation(3.0, 3.0, 850.0, 1.0)
    # HazardGridGeometry uses ROS cell areas; 3.5 is the centre of cell (3,3).
    ros.update_co_observation(3.5, 3.5, 850.0, 1.0)
    probability = np.zeros((7, 7))
    probability[4, 4] = 0.6
    simulation.update_estimated_fire_probability(
        probability, cost_weight=50.0, minimum_probability=0.4
    )
    ros.update_estimated_fire_probability(
        probability, cost_weight=50.0, minimum_probability=0.4
    )
    simulation.advance_time(16.0)
    ros.advance_time(16.0)

    np.testing.assert_array_equal(
        ros.temperature_observed_mask, simulation.temperature_observed_mask
    )
    np.testing.assert_array_equal(ros.co_observed_mask, simulation.co_observed_mask)
    np.testing.assert_allclose(
        ros.temperature_belief_map, simulation.temperature_belief_map,
        equal_nan=True,
    )
    np.testing.assert_allclose(ros.co_belief_map, simulation.co_belief_map, equal_nan=True)
    np.testing.assert_allclose(ros.temperature_cost_map, simulation.temperature_cost_map)
    np.testing.assert_allclose(ros.co_cost_map, simulation.co_cost_map)
    np.testing.assert_allclose(
        ros.estimated_fire_cost_map, simulation.estimated_fire_cost_map
    )
    np.testing.assert_allclose(
        ros.stale_observation_cost_map, simulation.stale_observation_cost_map
    )
    np.testing.assert_array_equal(ros.blocked_mask, simulation.blocked_mask)
    np.testing.assert_allclose(ros.final_cost_map, simulation.final_cost_map)
    assert ros.revision == simulation.revision
