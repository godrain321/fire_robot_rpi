import math

import numpy as np
import pytest

from inno_thermal.thermal_cost_geometry import (
    GridGeometry,
    aggregate_cell_costs,
    inflate_cell_costs,
    quaternion_to_yaw,
    temperature_to_cost,
    transform_point,
    world_to_grid,
)


def geometry(**overrides):
    values = dict(width=20, height=20, resolution=0.5, origin_x=0.0, origin_y=0.0)
    values.update(overrides)
    return GridGeometry(**values)


def test_temperature_thresholds_and_midrange():
    assert temperature_to_cost(10.0, 20.0, 60.0, 2.0) == 0
    assert temperature_to_cost(20.0, 20.0, 60.0, 2.0) == 0
    assert temperature_to_cost(60.0, 20.0, 60.0, 2.0) == 100
    assert temperature_to_cost(80.0, 20.0, 60.0, 2.0) == 100
    assert 1 <= temperature_to_cost(40.0, 20.0, 60.0, 2.0) <= 99


def test_temperature_power_is_applied():
    linear = temperature_to_cost(40.0, 20.0, 60.0, 1.0)
    quadratic = temperature_to_cost(40.0, 20.0, 60.0, 2.0)
    assert linear == 50
    assert quadratic == 25


def test_duplicate_cells_keep_maximum_cost():
    assert aggregate_cell_costs([((2, 3), 10), ((2, 3), 70), ((2, 3), 20)]) == {
        (2, 3): 70
    }


def test_world_to_grid_handles_translation_and_outside():
    grid = geometry(origin_x=10.0, origin_y=-5.0)
    assert world_to_grid(10.75, -3.75, grid) == (1, 2)
    assert world_to_grid(9.99, -5.0, grid) is None


def test_world_to_grid_handles_origin_yaw():
    yaw = math.pi / 2.0
    grid = geometry(origin_qz=math.sin(yaw / 2), origin_qw=math.cos(yaw / 2))
    # Local grid point (1.25, 0.75) rotated +90 degrees into world.
    assert world_to_grid(-0.75, 1.25, grid) == (2, 1)
    assert quaternion_to_yaw(0.0, 0.0, grid.origin_qz, grid.origin_qw) == pytest.approx(yaw)


def test_transform_point_identity_and_rotation_translation():
    assert transform_point((1, 2, 3), (0, 0, 0), (0, 0, 0, 1)) == pytest.approx((1, 2, 3))
    yaw = math.pi / 2.0
    result = transform_point(
        (1, 0, 0), (2, 3, 4), (0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
    )
    assert result == pytest.approx((2, 4, 4), abs=1e-9)


def test_zero_inflation_changes_only_original_cell():
    result = inflate_cell_costs({(5, 5): 100}, geometry(), 0.0)
    assert result == {(5, 5): 100}


def test_inflation_decreases_with_distance_and_keeps_center():
    result = inflate_cell_costs({(5, 5): 100}, geometry(resolution=0.1), 0.25)
    assert result[(5, 5)] == 100
    assert 100 > result[(6, 5)] > result[(7, 5)] > 0
    assert all(0 <= cost <= 100 for cost in result.values())


@pytest.mark.parametrize(
    "arguments,match",
    [
        ((30.0, 60.0, 20.0, 2.0), "must exceed"),
        ((30.0, 20.0, 60.0, 0.0), "must be positive"),
        ((math.nan, 20.0, 60.0, 2.0), "must be finite"),
    ],
)
def test_invalid_temperature_parameters_are_rejected(arguments, match):
    with pytest.raises(ValueError, match=match):
        temperature_to_cost(*arguments)
