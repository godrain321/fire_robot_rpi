import math
from pathlib import Path

import numpy as np
import pytest

from inno_hazard.hazard_belief import (
    HazardBelief,
    HazardBeliefConfig,
    HazardGridGeometry,
)


def test_hazard_node_keeps_tf_callbacks_on_a_second_executor_thread():
    source_path = (
        Path(__file__).resolve().parents[1]
        / "inno_hazard"
        / "hazard_belief_node.py"
    )
    source = source_path.read_text(encoding="utf-8")
    assert "MultiThreadedExecutor(num_threads=2)" in source
    assert "executor.add_node(node)" in source
    assert "rclpy.spin(node)" not in source
    assert '"thermal_stream_timeout_s": 3.0' in source
    assert '"latest_tf_fallback_tolerance_sec": 1.0' in source
    assert "if self.last_thermal_ns is None:" in source
    assert "self.last_thermal_ns = observation_ns" in source
    assert "self.last_thermal_ns = self.get_clock().now().nanoseconds" not in source


def test_hazard_thermal_tf_uses_bounded_latest_fallback():
    source_path = (
        Path(__file__).resolve().parents[1]
        / "inno_hazard"
        / "hazard_belief_node.py"
    )
    source = source_path.read_text(encoding="utf-8")
    body = source.split("def _thermal(", 1)[1].split("\n    def ", 1)[0]
    stamped_lookup, fallback = body.split(
        "except TransformException as stamped_error:", 1
    )
    assert "Time.from_msg(message.header.stamp)" in stamped_lookup
    assert "Time()," in fallback
    assert "latest_transform_is_fresh(" in fallback
    assert "self.latest_tf_fallback_tolerance_sec" in fallback
    assert "return" in fallback


def belief(*, resolution=1.0, width=7, height=7, static=None, **changes):
    geometry = HazardGridGeometry(width, height, resolution)
    static = np.zeros((height, width), bool) if static is None else static
    return HazardBelief(geometry, static, HazardBeliefConfig(**changes))


def test_initial_unknown_cells_have_base_cost_without_fake_observations():
    item = belief()
    assert not item.temperature_observed_mask.any()
    assert not item.co_observed_mask.any()
    assert np.isnan(item.temperature_belief_map).all()
    assert np.all(item.final_cost_map == 1.0)
    assert not item.blocked_mask.any()


def test_temperature_update_duplicate_max_and_latest_scan_replacement():
    item = belief()
    item.update_temperature_observations(
        [((2, 2), 35), ((2, 2), 42), ((2, 2), 39)], 1.0
    )
    assert item.temperature_belief_map[2, 2] == 42
    assert item.temperature_observed_mask[2, 2]
    assert item.temperature_cost_map[2, 2] > 0
    item.update_temperature_observations([((2, 2), 45)], 2.0)
    assert item.temperature_belief_map[2, 2] == 45
    item.update_temperature_observations([((2, 2), 41)], 3.0)
    assert item.temperature_belief_map[2, 2] == 41


def test_temperature_threshold_and_soft_cost_formula():
    item = belief()
    item.update_temperature_observations(
        [((1, 1), 40), ((2, 1), 45), ((3, 1), 50),
         ((4, 1), 55), ((5, 1), 59), ((6, 1), 60)], 1.0
    )
    expected = [24.0 * ((value - 40.0) / 20.0) ** 1.5
                for value in (40, 45, 50, 55, 59)]
    np.testing.assert_allclose(item.temperature_cost_map[1, 1:6], expected)
    assert np.isfinite(item.final_cost_map[1, 5])
    assert np.isinf(item.final_cost_map[1, 6])


def test_mode8_temperature_hard_block_preserves_legacy_soft_scale():
    temperatures = (0.0, 20.0, 30.0, 40.0, 49.0, 49.9, 50.0, 50.1, 60.0)
    observations = [((index, 0), value) for index, value in enumerate(temperatures)]
    legacy = belief(width=len(temperatures), height=1)
    mode8 = belief(
        width=len(temperatures), height=1,
        temperature_cost_scale_max_c=60.0,
        temperature_blocked_c=50.0,
    )
    legacy.update_temperature_observations(observations, 1.0)
    mode8.update_temperature_observations(observations, 1.0)

    below = np.asarray(temperatures) < 50.0
    np.testing.assert_allclose(
        mode8.temperature_cost_map[0, below],
        legacy.temperature_cost_map[0, below],
    )
    assert not mode8.blocked_mask[0, below].any()
    assert np.isfinite(mode8.final_cost_map[0, below]).all()
    assert mode8.blocked_mask[0, ~below].all()
    assert np.isinf(mode8.final_cost_map[0, ~below]).all()
    assert mode8.temperature_cost_map[0, 4] == pytest.approx(
        24.0 * ((49.0 - 40.0) / (60.0 - 40.0)) ** 1.5
    )


def test_co_disabled_does_not_invent_measurements():
    item = belief(co_enabled=False)
    update = item.update_co_observation(2.5, 2.5, 800, 1.0)
    assert not update.changed_cells
    assert not item.co_observed_mask.any()
    assert not item.co_cost_map.any()


def test_co_belief_confidence_soft_cost_and_hard_block():
    item = belief(
        co_enabled=True, gas_update_radius_m=1.0,
        gas_gaussian_sigma_m=0.5,
    )
    item.update_co_observation(3.5, 3.5, 850, 1.0)
    assert item.co_belief_map[3, 3] == 850
    assert item.co_confidence_map[3, 3] == 1.0
    assert item.co_confidence_map[3, 2] == pytest.approx(math.exp(-2.0))
    assert item.co_cost_map[3, 3] == pytest.approx(8.0 * 0.5 ** 2)
    item.update_co_observation(3.5, 3.5, 1600, 2.0)
    assert np.isinf(item.final_cost_map[3, 3])


def test_stale_cost_grace_growth_cap_and_revision():
    item = belief()
    update = item.update_temperature_observations([((2, 2), 45)], 1.0)
    first_revision = item.revision
    assert update.changed_cells == frozenset({(2, 2)})
    item.advance_time(6.0)
    assert item.stale_observation_cost_map[2, 2] == 0
    item.advance_time(16.0)
    assert item.stale_observation_cost_map[2, 2] == pytest.approx(0.5)
    assert item.revision > first_revision
    item.advance_time(1000.0)
    assert item.stale_observation_cost_map[2, 2] == 2.0
    assert not item.blocked_mask[2, 2]


def test_identical_temperature_with_new_timestamp_does_not_increment_revision():
    item = belief()
    item.update_temperature_observations([((2, 2), 45)], 1.0)
    revision = item.revision
    item.update_temperature_observations([((2, 2), 45)], 2.0)
    assert item.revision == revision
    assert item.last_observed_time_map[2, 2] == 2.0


def test_temperature_cost_or_blocking_change_increments_revision():
    item = belief(temperature_blocked_c=50.0)
    item.update_temperature_observations([((2, 2), 45.0)], 1.0)
    first_revision = item.revision

    item.update_temperature_observations([((2, 2), 46.0)], 2.0)
    assert item.revision == first_revision + 1
    assert not item.blocked_mask[2, 2]

    item.update_temperature_observations([((2, 2), 50.0)], 3.0)
    assert item.revision == first_revision + 2
    assert item.blocked_mask[2, 2]


def test_already_inflated_dynamic_grid_is_not_inflated_twice():
    item = belief()
    incoming = np.zeros(item.shape, bool)
    incoming[3, 2:5] = True
    item.update_dynamic_obstacles(incoming, already_inflated=True)
    np.testing.assert_array_equal(item.dynamic_inflated_obstacle_map, incoming)
    assert item.blocked_mask[3, 2:5].all()
    assert not item.blocked_mask[2, 3]


def test_fire_probability_is_soft_only_and_respects_minimum():
    item = belief()
    probabilities = np.zeros(item.shape)
    probabilities[2, 1:4] = (0.2, 0.6, 0.9)
    item.update_estimated_fire_probability(
        probabilities, cost_weight=50.0, minimum_probability=0.4
    )
    np.testing.assert_allclose(item.estimated_fire_cost_map[2, 1:4], (0, 30, 45))
    assert np.isfinite(item.final_cost_map[2, 3])
    assert not item.blocked_mask[2, 3]


@pytest.mark.parametrize("resolution", [0.20, 0.05])
def test_meter_gas_radius_is_resolution_independent(resolution):
    extent = 2.0
    size = int(extent / resolution)
    item = belief(
        resolution=resolution, width=size, height=size,
        co_enabled=True, gas_update_radius_m=0.5,
        gas_gaussian_sigma_m=0.5,
    )
    item.update_co_observation(1.0, 1.0, 500, 1.0)
    center = item.geometry.world_to_grid(1.0, 1.0)
    observed = np.argwhere(item.co_observed_mask)
    farthest = max(math.hypot(col - center[0], row - center[1]) * resolution
                   for row, col in observed)
    assert farthest <= 0.5 + 1e-12
    assert farthest >= 0.5 - resolution


def test_final_cost_is_exact_sum_of_all_soft_layers():
    item = belief(co_enabled=True)
    item.update_temperature_observations([((2, 2), 50)], 1.0)
    x, y = item.geometry.grid_to_world(2, 2)
    item.update_co_observation(x, y, 850, 1.0)
    probabilities = np.zeros(item.shape)
    probabilities[2, 2] = 0.6
    item.update_estimated_fire_probability(
        probabilities, cost_weight=50, minimum_probability=0.4
    )
    item.advance_time(16.0)
    expected = (
        item.config.base_cost + item.temperature_cost_map[2, 2]
        + item.co_cost_map[2, 2] + item.unknown_cost_map[2, 2]
        + item.estimated_fire_cost_map[2, 2]
        + item.stale_observation_cost_map[2, 2]
    )
    assert item.final_cost_map[2, 2] == pytest.approx(expected)


def test_initial_route_cost_ignores_temperature_but_keeps_obstacles():
    item = belief(temperature_blocked_c=50.0)
    item.update_temperature_observations([((2, 2), 60.0)], 1.0)
    dynamic = np.zeros(item.shape, dtype=bool)
    dynamic[3, 3] = True
    item.update_dynamic_obstacles(dynamic, already_inflated=True)

    initial = item.cost_without_temperature()

    assert np.isinf(item.final_cost_map[2, 2])
    assert initial[2, 2] == pytest.approx(item.config.base_cost)
    assert np.isinf(initial[3, 3])
    assert np.isinf(initial[item.static_obstacle_map]).all()
