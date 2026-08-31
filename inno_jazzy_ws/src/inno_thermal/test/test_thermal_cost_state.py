from pathlib import Path

import numpy as np
import pytest

from inno_thermal.thermal_cost_geometry import (
    GridGeometry,
    ThermalCostState,
    thermal_stream_is_stale,
)


def geometry(width=5, height=4, resolution=0.1, origin_x=0.0):
    return GridGeometry(width, height, resolution, origin_x, 0.0, frame_id="map")


def test_timeout_removes_old_cells_but_not_fresh_cells():
    state = ThermalCostState(2.0, 0.0)
    state.set_geometry(geometry())
    state.apply_frame({(1, 1): 80}, 1_000_000_000)
    assert state.expire(3_000_000_000) == 0
    assert state.expire(3_000_000_001) == 1
    assert state.costs[1, 1] == 0


def test_safe_observation_replaces_prior_high_cost():
    state = ThermalCostState(2.0, 0.0)
    state.set_geometry(geometry())
    state.apply_frame({(1, 1): 90}, 1)
    state.apply_frame({(1, 1): 0}, 2)
    assert state.costs[1, 1] == 0
    assert (1, 1) not in state.last_observed_ns


def test_persistent_observation_does_not_expire_and_latest_scan_replaces_it():
    state = ThermalCostState(2.0, 0.0, persistent_observations=True)
    state.set_geometry(geometry())
    state.apply_frame({(1, 1): 90}, 1_000_000_000)
    assert state.expire(100_000_000_000) == 0
    assert state.costs[1, 1] == 90
    state.apply_frame({(1, 1): 15}, 101_000_000_000)
    assert state.costs[1, 1] == 15
    state.apply_frame({(1, 1): 0}, 102_000_000_000)
    assert state.costs[1, 1] == 0


def test_geometry_change_resets_state_and_data_shape():
    state = ThermalCostState(2.0, 0.0)
    first = geometry()
    assert state.set_geometry(first)
    state.apply_frame({(1, 1): 90}, 1)
    assert not state.set_geometry(first)
    second = geometry(width=7, height=3, origin_x=1.0)
    assert state.set_geometry(second)
    assert np.count_nonzero(state.costs) == 0
    assert state.costs.shape == (3, 7)
    assert len(state.flattened()) == 21


def test_clear_reports_nonzero_cells_and_empties_timestamps():
    state = ThermalCostState(2.0, 0.0)
    state.set_geometry(geometry())
    state.apply_frame({(1, 1): 50, (2, 2): 70}, 1)
    assert state.clear() == 2
    assert np.count_nonzero(state.costs) == 0
    assert not state.last_observed_ns


def test_zero_timeout_clears_unobserved_cells_on_next_frame():
    state = ThermalCostState(0.0, 0.0)
    state.set_geometry(geometry())
    state.apply_frame({(1, 1): 70}, 1)
    state.apply_frame({(2, 2): 80}, 2)
    assert state.costs[1, 1] == 0
    assert state.costs[2, 2] == 80


def test_zero_timeout_clears_on_publish_expiry_cycle():
    state = ThermalCostState(0.0, 0.0)
    state.set_geometry(geometry())
    state.apply_frame({(1, 1): 70}, 1)
    assert state.expire(2) == 1
    assert state.costs[1, 1] == 0


def test_ros_clock_jump_backwards_expires_future_timestamps():
    state = ThermalCostState(2.0, 0.0)
    state.set_geometry(geometry())
    state.apply_frame({(1, 1): 70}, 10_000_000_000)
    assert state.expire(1_000_000_000) == 1
    assert state.costs[1, 1] == 0


def test_thermal_stream_becomes_stale_only_after_timeout():
    last = 1_000_000_000
    assert not thermal_stream_is_stale(last, 2_000_000_000, 1.0)
    assert thermal_stream_is_stale(last, 2_000_000_001, 1.0)


def test_thermal_stream_clock_jump_is_fail_safe():
    assert thermal_stream_is_stale(10_000_000_000, 1_000_000_000, 1.0)


@pytest.mark.parametrize("timeout", [-0.1, float("nan"), float("inf")])
def test_thermal_stream_timeout_must_be_valid(timeout):
    with pytest.raises(ValueError, match="timeout"):
        thermal_stream_is_stale(0, 0, timeout)


def test_thermal_cost_node_keeps_tf_callbacks_on_a_second_executor_thread():
    source_path = (
        Path(__file__).resolve().parents[1]
        / "inno_thermal"
        / "thermal_cost_layer.py"
    )
    source = source_path.read_text(encoding="utf-8")
    assert "MultiThreadedExecutor(num_threads=2)" in source
    assert "executor.add_node(node)" in source
    assert "rclpy.spin(node)" not in source
    assert '"thermal_data_timeout_sec": 3.0' in source
    assert "if not self._has_valid_arc:" in source
    assert "self._has_valid_arc = True" in source
    assert "self._last_arc_received_ns = now_ns" in source
