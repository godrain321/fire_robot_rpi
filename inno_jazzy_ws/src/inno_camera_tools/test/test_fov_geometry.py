import math

import pytest

from inno_camera_tools.fov_geometry import (
    field_of_view_deg,
    object_size_px,
    plane_coverage_m,
    scaled_focal_lengths,
)


def test_calibrated_camera_geometry_at_two_metres():
    horizontal, vertical = field_of_view_deg(
        1280, 720, 825.5795842632374, 824.1957896996265
    )
    width, height = plane_coverage_m(2.0, horizontal, vertical)

    assert horizontal == pytest.approx(75.56, abs=0.02)
    assert vertical == pytest.approx(47.21, abs=0.02)
    assert width == pytest.approx(3.101, abs=0.005)
    assert height == pytest.approx(1.747, abs=0.005)


def test_person_projection_and_resolution_scaling():
    focal_x, focal_y = scaled_focal_lengths(
        825.0, 824.0, 1280, 720, 640, 360
    )
    width, height = object_size_px(2.0, 0.5, 1.7, focal_x, focal_y)

    assert focal_x == pytest.approx(412.5)
    assert focal_y == pytest.approx(412.0)
    assert width == pytest.approx(103.125)
    assert height == pytest.approx(350.2)


@pytest.mark.parametrize(
    'call',
    [
        lambda: field_of_view_deg(0, 720, 800, 800),
        lambda: plane_coverage_m(-1, 75, 47),
        lambda: plane_coverage_m(1, 180, 47),
        lambda: object_size_px(math.nan, 0.5, 1.7, 800, 800),
        lambda: scaled_focal_lengths(800, 800, 0, 720, 1280, 720),
    ],
)
def test_invalid_geometry_is_rejected(call):
    with pytest.raises(ValueError):
        call()
