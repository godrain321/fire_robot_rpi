"""Stage 3: MQ-135 /mq135/filtered_adc -> existing hazard belief gas pipeline.

Stage 3 only wires the sensor input; it changes no hazard algorithm. These
tests pin the wiring contract (config + node source) and confirm the existing
``update_co_observation`` path records one observation at the robot cell while
leaving thermal/static/dynamic cost untouched until a real sample arrives.
"""

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from inno_hazard.hazard_belief import (
    HazardBelief,
    HazardBeliefConfig,
    HazardGridGeometry,
)

_PKG = Path(__file__).resolve().parents[1]
_PARAMS = _PKG / "config" / "hazard_params.yaml"
_NODE_SRC = (_PKG / "inno_hazard" / "hazard_belief_node.py").read_text()


def _hazard_params():
    doc = yaml.safe_load(_PARAMS.read_text())
    return doc["hazard_belief_node"]["ros__parameters"]


# --- A. gas input topic is the Stage 2 topic --------------------------------
def test_co_topic_points_at_mq135_filtered_adc():
    assert _hazard_params()["co_topic"] == "/mq135/filtered_adc"


def test_co_disabled_by_default_so_existing_profiles_are_unchanged():
    assert _hazard_params()["co_enabled"] is False


def test_gas_update_radius_is_single_cell():
    assert _hazard_params()["gas_update_radius_m"] == 0.0


# --- B. subscription message type is std_msgs/Float32 ----------------------
def test_node_subscribes_float32_on_co_topic():
    assert "create_subscription(Float32, self.co_topic, self._co" in _NODE_SRC
    assert "from std_msgs.msg import" in _NODE_SRC and "Float32" in _NODE_SRC


# --- C. callback -> TF map->base_link -> update_co_observation(x, y, value) -
def test_co_callback_uses_map_to_base_link_tf_then_updates_observation():
    body = _NODE_SRC.split("def _co(", 1)[1].split("\n    def ", 1)[0]
    assert "lookup_transform(" in body
    assert "self.frame_id, self.base_frame" in body
    assert "self.belief.update_co_observation(" in body
    assert "transform.translation.x" in body
    assert "transform.translation.y" in body
    assert "message.data" in body


# --- D. TF failure just skips the sample, never crashes the node ----------
def test_co_callback_swallows_transform_exception():
    body = _NODE_SRC.split("def _co(", 1)[1].split("\n    def ", 1)[0]
    assert "except TransformException:" in body
    # the except branch returns instead of re-raising / propagating
    tail = body.split("except TransformException:", 1)[1]
    assert "return" in tail.split("def ", 1)[0]
    assert "raise" not in tail.split("return", 1)[0]


# --- C (behaviour). one Float32 sample -> exactly the robot cell records it -
def test_single_sample_records_only_the_robot_cell():
    geometry = HazardGridGeometry(width=7, height=7, resolution=1.0)
    item = HazardBelief(
        geometry, np.zeros((7, 7), bool),
        HazardBeliefConfig(co_enabled=True, gas_update_radius_m=0.0),
    )
    robot_x, robot_y, sample = 2.0, 3.0, 1817.3
    item.update_co_observation(robot_x, robot_y, sample, 1.0)

    col, row = geometry.world_to_grid(robot_x, robot_y)
    assert item.co_observed_mask[row, col]
    assert math.isclose(item.co_belief_map[row, col], sample)
    assert item.co_observed_mask.sum() == 1  # no spreading in Stage 3


# --- E. regression: enabling gas with no sample does not move any cost -----
def test_enabling_gas_without_samples_leaves_cost_identical():
    static = np.zeros((7, 7), bool)
    geometry = HazardGridGeometry(width=7, height=7, resolution=1.0)
    off = HazardBelief(geometry, static, HazardBeliefConfig(co_enabled=False))
    on = HazardBelief(geometry, static, HazardBeliefConfig(co_enabled=True))

    # a thermal observation must fuse the same way whether or not gas is armed
    for grid in (off, on):
        grid.update_temperature_observations([((2, 2), 52.0)], 1.0)

    np.testing.assert_array_equal(off.final_cost_map, on.final_cost_map)
    np.testing.assert_array_equal(off.blocked_mask, on.blocked_mask)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
