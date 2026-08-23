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
