"""Tests for frame conversions used by the extrinsic workflow."""

from fire_robot_camera_calibration.calibration_math import (
    camera_lidar_transform,
    lidar_to_camera_rotation,
    quaternion_from_rotation,
    rotation_from_quaternion,
)

import numpy as np


def test_rep_103_lidar_axes_map_to_optical_axes():
    rotation = lidar_to_camera_rotation(0.0, 0.0, 0.0)

    lidar_forward = np.array([1.0, 0.0, 0.0])
    lidar_left = np.array([0.0, 1.0, 0.0])
    lidar_up = np.array([0.0, 0.0, 1.0])

    assert np.allclose(
        rotation @ lidar_forward,
        [0.0, 0.0, 1.0],
    )
    assert np.allclose(
        rotation @ lidar_left,
        [-1.0, 0.0, 0.0],
    )
    assert np.allclose(
        rotation @ lidar_up,
        [0.0, -1.0, 0.0],
    )


def test_camera_position_becomes_camera_origin():
    camera_position = np.array([0.12, -0.03, 0.18])
    rotation, translation = camera_lidar_transform(
        camera_position,
        roll_deg=1.0,
        pitch_deg=-2.0,
        yaw_deg=3.0,
    )

    assert np.allclose(
        rotation @ camera_position + translation,
        np.zeros(3),
        atol=1e-12,
    )


def test_rotation_quaternion_round_trip():
    rotation = lidar_to_camera_rotation(2.5, -3.0, 4.0)
    quaternion = quaternion_from_rotation(rotation)
    reconstructed = rotation_from_quaternion(*quaternion)

    assert np.isclose(np.linalg.norm(quaternion), 1.0)
    assert np.allclose(reconstructed, rotation, atol=1e-12)
