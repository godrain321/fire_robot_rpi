import argparse

from camera_calibration.calibrator import CAMERA_MODEL

from fire_robot_camera_calibration.fixed_model_calibrator import (
    camera_model_from_name,
    parse_board_size,
)
import pytest


def test_camera_models_are_explicit():
    assert camera_model_from_name('fisheye') == CAMERA_MODEL.FISHEYE
    assert camera_model_from_name('PINHOLE') == CAMERA_MODEL.PINHOLE


def test_unknown_camera_model_is_rejected():
    with pytest.raises(ValueError):
        camera_model_from_name('auto')


def test_board_size_parses_inner_corner_counts():
    assert parse_board_size('8x9') == (8, 9)


@pytest.mark.parametrize('value', ['8X9', '8x', '1x9', '0x9', 'eightx9'])
def test_invalid_board_size_is_rejected(value):
    with pytest.raises(argparse.ArgumentTypeError):
        parse_board_size(value)
