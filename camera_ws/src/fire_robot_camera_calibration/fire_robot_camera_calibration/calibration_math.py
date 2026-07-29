"""Coordinate conversion helpers shared by the calibration nodes."""

import math

import numpy as np


def rotation_x(angle_rad):
    """Return a right-handed X-axis rotation matrix."""
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.array(
        [[1.0, 0.0, 0.0],
         [0.0, cosine, -sine],
         [0.0, sine, cosine]],
        dtype=np.float64,
    )


def rotation_y(angle_rad):
    """Return a right-handed Y-axis rotation matrix."""
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.array(
        [[cosine, 0.0, sine],
         [0.0, 1.0, 0.0],
         [-sine, 0.0, cosine]],
        dtype=np.float64,
    )


def rotation_z(angle_rad):
    """Return a right-handed Z-axis rotation matrix."""
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.array(
        [[cosine, -sine, 0.0],
         [sine, cosine, 0.0],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def lidar_to_camera_rotation(roll_deg, pitch_deg, yaw_deg):
    """Map laser REP-103 axes into optical axes, then apply corrections."""
    base_rotation = np.array(
        [[0.0, -1.0, 0.0],
         [0.0, 0.0, -1.0],
         [1.0, 0.0, 0.0]],
        dtype=np.float64,
    )
    correction = (
        rotation_z(math.radians(yaw_deg))
        @ rotation_y(math.radians(pitch_deg))
        @ rotation_x(math.radians(roll_deg))
    )
    return correction @ base_rotation


def camera_lidar_transform(
    camera_position_in_lidar,
    roll_deg=0.0,
    pitch_deg=0.0,
    yaw_deg=0.0,
):
    """Return R and t for p_camera = R * p_lidar + t."""
    rotation = lidar_to_camera_rotation(roll_deg, pitch_deg, yaw_deg)
    camera_position = np.asarray(
        camera_position_in_lidar,
        dtype=np.float64,
    ).reshape(3)
    translation = -rotation @ camera_position
    return rotation, translation


def quaternion_from_rotation(rotation):
    """Convert a 3x3 rotation matrix to a normalized xyzw quaternion."""
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = np.trace(matrix)

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
        ) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0, 1] + matrix[1, 0]) / scale
        qz = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
        ) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / scale
        qx = (matrix[0, 1] + matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(
            1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
        ) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / scale
        qx = (matrix[0, 2] + matrix[2, 0]) / scale
        qy = (matrix[1, 2] + matrix[2, 1]) / scale
        qz = 0.25 * scale

    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm <= np.finfo(np.float64).eps:
        raise ValueError('Rotation produced a zero-length quaternion')
    return quaternion / norm


def rotation_from_quaternion(qx, qy, qz, qw):
    """Convert an xyzw quaternion to a 3x3 rotation matrix."""
    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm <= np.finfo(np.float64).eps:
        raise ValueError('Quaternion has zero length')
    qx, qy, qz, qw = quaternion / norm
    return np.array(
        [
            [
                1.0 - 2.0 * qy * qy - 2.0 * qz * qz,
                2.0 * qx * qy - 2.0 * qz * qw,
                2.0 * qx * qz + 2.0 * qy * qw,
            ],
            [
                2.0 * qx * qy + 2.0 * qz * qw,
                1.0 - 2.0 * qx * qx - 2.0 * qz * qz,
                2.0 * qy * qz - 2.0 * qx * qw,
            ],
            [
                2.0 * qx * qz - 2.0 * qy * qw,
                2.0 * qy * qz + 2.0 * qx * qw,
                1.0 - 2.0 * qx * qx - 2.0 * qy * qy,
            ],
        ],
        dtype=np.float64,
    )
