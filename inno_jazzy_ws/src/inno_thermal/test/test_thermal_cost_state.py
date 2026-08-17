import numpy as np

from inno_thermal.thermal_cost_geometry import GridGeometry, ThermalCostState


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
