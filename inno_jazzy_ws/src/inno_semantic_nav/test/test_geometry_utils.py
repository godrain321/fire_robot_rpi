import math

import pytest

from inno_semantic_nav.geometry_utils import (
    normalize_yaw,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


def test_yaw_zero_to_quaternion():
    assert quaternion_from_yaw(0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_yaw_positive_half_pi_to_quaternion():
    expected = math.sqrt(0.5)
    assert quaternion_from_yaw(math.pi / 2.0) == pytest.approx(
        (0.0, 0.0, expected, expected)
    )


def test_yaw_negative_half_pi_to_quaternion():
    expected = math.sqrt(0.5)
    assert quaternion_from_yaw(-math.pi / 2.0) == pytest.approx(
        (0.0, 0.0, -expected, expected)
    )


@pytest.mark.parametrize(
    'yaw',
    [-math.pi, -math.pi / 2.0, -0.2, 0.0, 0.8, math.pi / 2.0, math.pi],
)
def test_quaternion_yaw_round_trip(yaw):
    quaternion = quaternion_from_yaw(yaw)
    result = yaw_from_quaternion(*quaternion)
    assert math.sin(result) == pytest.approx(math.sin(yaw), abs=1.0e-9)
    assert math.cos(result) == pytest.approx(math.cos(yaw), abs=1.0e-9)


@pytest.mark.parametrize(
    ('angle', 'expected'),
    [
        (0.0, 0.0),
        (2.0 * math.pi, 0.0),
        (5.0 * math.pi / 2.0, math.pi / 2.0),
        (-5.0 * math.pi / 2.0, -math.pi / 2.0),
        (3.0 * math.pi, math.pi),
    ],
)
def test_yaw_normalization(angle, expected):
    result = normalize_yaw(angle)
    assert -math.pi <= result <= math.pi
    assert result == pytest.approx(expected, abs=1.0e-9)


def test_quaternion_is_normalized():
    quaternion = quaternion_from_yaw(123.45)
    assert math.sqrt(sum(value * value for value in quaternion)) == pytest.approx(1.0)


def test_zero_length_quaternion_is_rejected():
    with pytest.raises(ValueError):
        yaw_from_quaternion(0.0, 0.0, 0.0, 0.0)
