import math

import numpy as np

from inno_hazard.thermal_adapter import localized_temperature_cells
from inno_thermal.thermal_cost_geometry import GridGeometry


def geometry(resolution=1.0):
    return GridGeometry(5, 5, resolution, 0, 0)


def test_adapter_transforms_aggregates_and_filters_invalid_points():
    static = np.zeros((5, 5), bool)
    static[2, 2] = True
    points = (
        (0.2, 0.2, 0, 35), (0.3, 0.3, 0, 42),
        (1.2, 1.2, 0, 39), (2.2, 2.2, 0, 80),
        (math.nan, 0, 0, 90), (20, 20, 0, 90),
    )
    result = dict(localized_temperature_cells(points, geometry(), static))
    assert result[(0, 0)] == 42
    assert result[(1, 1)] == 39
    assert (2, 2) not in result


def test_adapter_reuses_full_3d_transform_geometry():
    static = np.zeros((5, 5), bool)
    transform = ((1, 2, 0), (0, 0, 0, 1))
    result = dict(localized_temperature_cells(
        ((0.2, 0.2, 0, 45),), geometry(), static, transform
    ))
    assert result == {(1, 2): 45}
