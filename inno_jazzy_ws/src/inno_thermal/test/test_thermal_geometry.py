import math

import numpy as np
import pytest

from inno_thermal.thermal_geometry import (
    apply_orientation,
    compute_column_angles,
    compute_column_max,
    project_columns_to_arc,
)


def test_column_max_has_32_correct_values():
    temperatures = np.arange(24 * 32, dtype=np.float32).reshape(24, 32)
    result = compute_column_max(temperatures)
    assert result.shape == (32,)
    np.testing.assert_array_equal(result, temperatures[-1, :])


def test_arc_direction_radius_and_temperature_are_preserved():
    column_max = np.linspace(20.0, 51.0, 32, dtype=np.float32)
    points = project_columns_to_arc(column_max, 110.0, 0.15)
    assert points.shape == (32, 4)
    assert points[0, 1] > 0.0
    assert points[-1, 1] < 0.0
    # Even-width images have two centre pixels straddling zero.
    assert abs(points[15, 1]) < 0.01
    assert abs(points[16, 1]) < 0.01
    np.testing.assert_allclose(
        np.hypot(points[:, 0], points[:, 1]), 0.15, rtol=0.0, atol=1e-6
    )
    np.testing.assert_array_equal(points[:, 3], column_max)


def test_column_angle_formula_has_expected_signs():
    angles = compute_column_angles(32, 110.0)
    assert angles[0] > 0.0
    assert angles[-1] < 0.0
    assert math.isclose(angles[15], -angles[16], abs_tol=1e-12)


@pytest.mark.parametrize("shape", [(32, 24), (768,), (24, 31), (1, 24, 32)])
def test_wrong_temperature_shape_is_rejected(shape):
    with pytest.raises(ValueError, match="shape must be"):
        compute_column_max(np.zeros(shape, dtype=np.float32))


def test_partial_nan_column_uses_remaining_finite_maximum():
    temperatures = np.arange(24 * 32, dtype=np.float32).reshape(24, 32)
    temperatures[-1, 4] = np.nan
    result = compute_column_max(temperatures)
    assert result[4] == temperatures[-2, 4]


def test_all_nan_column_invalidates_geometry_frame():
    temperatures = np.zeros((24, 32), dtype=np.float32)
    temperatures[:, 7] = np.nan
    with pytest.raises(ValueError, match="only NaN"):
        compute_column_max(temperatures)


def test_default_orientation_is_unchanged_and_contiguous():
    temperatures = np.arange(24 * 32, dtype=np.float32).reshape(24, 32)
    result = apply_orientation(temperatures)
    np.testing.assert_array_equal(result, temperatures)
    assert result.flags.c_contiguous


def test_orientation_controls_are_isolated_and_shape_preserving():
    temperatures = np.arange(24 * 32, dtype=np.float32).reshape(24, 32)
    assert apply_orientation(temperatures, flip_horizontal=True)[0, 0] == temperatures[0, -1]
    assert apply_orientation(temperatures, flip_vertical=True)[0, 0] == temperatures[-1, 0]
    rotated = apply_orientation(temperatures, rotate_180=True)
    assert rotated.shape == (24, 32)
    assert rotated[0, 0] == temperatures[-1, -1]
