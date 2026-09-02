import numpy as np

from inno_hazard.hazard_belief import (
    HazardBelief, HazardBeliefConfig, HazardGridGeometry,
)
from inno_hazard.hazard_snapshot import (
    decode_hazard_snapshot_message, hazard_snapshot_message,
)


def test_snapshot_keeps_all_layers_and_revision_coherent():
    geometry = HazardGridGeometry(3, 2, 0.05, -1.0, 2.0, 0.2, "map")
    belief = HazardBelief(
        geometry, np.zeros((2, 3), dtype=bool),
        HazardBeliefConfig(co_enabled=True),
    )
    belief.update_temperature_observations([((1, 0), 45.0)], 2.0)
    belief.update_co_observation(-0.925, 2.025, 150.0, 2.0)
    probability = np.zeros((2, 3))
    probability[1, 2] = 0.6
    message = hazard_snapshot_message(
        belief, probability, status="ACTIVE"
    )
    metadata, layers = decode_hazard_snapshot_message(message)
    assert metadata["revision"] == belief.revision
    assert metadata["status"] == "ACTIVE"
    assert metadata["resolution"] == 0.05
    np.testing.assert_allclose(
        layers["final_cost"], belief.final_cost_map, equal_nan=True
    )
    np.testing.assert_allclose(
        layers["temperature_c"], belief.temperature_belief_map,
        equal_nan=True,
    )
    np.testing.assert_array_equal(
        layers["observed"].astype(bool), belief.observed_mask
    )
    assert layers["fire_probability"][1, 2] == np.float32(0.6)


def test_snapshot_exposes_mode8_50c_block_threshold_and_blocked_layer():
    geometry = HazardGridGeometry(1, 1, 0.05)
    belief = HazardBelief(
        geometry, np.zeros((1, 1), dtype=bool),
        HazardBeliefConfig(
            temperature_cost_scale_max_c=60.0,
            temperature_blocked_c=50.0,
        ),
    )
    belief.update_temperature_observations([((0, 0), 50.0)], 1.0)
    message = hazard_snapshot_message(
        belief, np.zeros((1, 1)), status="ACTIVE_THERMAL_ONLY"
    )
    metadata, layers = decode_hazard_snapshot_message(message)
    assert metadata["temperature_blocked_c"] == 50.0
    assert bool(layers["blocked"][0, 0]) is True
    assert np.isinf(layers["final_cost"][0, 0])


def test_compact_mode8_snapshot_sends_only_five_required_layers():
    geometry = HazardGridGeometry(3, 2, 0.05)
    belief = HazardBelief(
        geometry, np.zeros((2, 3), dtype=bool), HazardBeliefConfig()
    )
    message = hazard_snapshot_message(
        belief, np.zeros((2, 3)), status="ACTIVE_THERMAL_ONLY",
        include_temperature=True, include_co=False, include_fire=False,
    )
    metadata, layers = decode_hazard_snapshot_message(message)
    assert message.layout.dim[0].size == 5
    assert metadata["channels"] == [
        "final_cost", "static_obstacle", "dynamic_obstacle",
        "temperature_c", "temperature_observed",
    ]
    assert set(layers) == {
        "final_cost", "static_obstacle", "dynamic_obstacle",
        "temperature_c", "temperature_observed", "co_ppm",
        "fire_probability", "co_observed", "observed", "blocked",
    }
    assert np.isnan(layers["co_ppm"]).all()


def test_initial_route_snapshot_is_three_layers_and_uses_override():
    geometry = HazardGridGeometry(1, 1, 0.05)
    belief = HazardBelief(
        geometry, np.zeros((1, 1), dtype=bool),
        HazardBeliefConfig(temperature_blocked_c=50.0),
    )
    belief.update_temperature_observations([((0, 0), 60.0)], 1.0)
    initial_cost = belief.cost_without_temperature()
    message = hazard_snapshot_message(
        belief, np.zeros((1, 1)),
        status="ACTIVE_INITIAL_STATIC_DYNAMIC_ONLY",
        final_cost=initial_cost, include_temperature=False,
        include_co=False, include_fire=False,
    )
    metadata, layers = decode_hazard_snapshot_message(message)
    assert message.layout.dim[0].size == 3
    assert metadata["channels"] == [
        "final_cost", "static_obstacle", "dynamic_obstacle",
    ]
    assert np.isfinite(layers["final_cost"][0, 0])
    assert not bool(layers["blocked"][0, 0])
