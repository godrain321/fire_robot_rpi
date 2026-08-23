"""Pure thermal point aggregation using inno_thermal geometry helpers."""

import math

from inno_thermal.thermal_cost_geometry import transform_point, world_to_grid


def localized_temperature_cells(points, geometry, static_obstacles, transform=None):
    """Return latest-frame maximum Celsius for each valid free map cell."""
    output = {}
    for point in points:
        x, y, z, temperature = (float(value) for value in point)
        if not all(math.isfinite(value) for value in (x, y, z, temperature)):
            continue
        if transform is not None:
            translation, quaternion = transform
            x, y, _ = transform_point((x, y, z), translation, quaternion)
        cell = world_to_grid(x, y, geometry)
        if cell is None or static_obstacles[cell[1], cell[0]]:
            continue
        output[cell] = max(output.get(cell, -math.inf), temperature)
    return tuple(sorted(output.items()))
