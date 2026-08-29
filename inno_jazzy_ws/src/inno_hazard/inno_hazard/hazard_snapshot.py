"""Pack a coherent HazardBelief revision into one Float32MultiArray payload."""

from __future__ import annotations

import json

import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


CHANNELS = (
    "final_cost", "temperature_c", "co_ppm", "fire_probability",
    "observed", "temperature_observed", "co_observed", "blocked",
    "static_obstacle", "dynamic_obstacle",
)


def hazard_snapshot_message(belief, fire_probability, *, status):
    probability = np.asarray(fire_probability, dtype=float)
    if probability.shape != belief.shape:
        raise ValueError("fire probability geometry differs from belief")
    metadata = {
        "revision": int(belief.revision),
        "status": str(status),
        "frame_id": belief.geometry.frame_id,
        "resolution": float(belief.geometry.resolution),
        "origin_x": float(belief.geometry.origin_x),
        "origin_y": float(belief.geometry.origin_y),
        "origin_yaw": float(belief.geometry.origin_yaw),
        "base_cost": float(belief.config.base_cost),
        "temperature_blocked_c": float(belief.config.temperature_blocked_c),
        # Stage 6: the *effective* gas blocked threshold actually used to mark
        # cells lethal (Stage 4). In legacy_ppm mode this is co_blocked_ppm
        # unchanged; in adc mode it is gas_blocked_adc, so every /hazard/snapshot
        # consumer's `co_ppm >= co_blocked_ppm` check stays consistent with the
        # `blocked` mask and the inf cells in `final_cost`.
        "co_blocked_ppm": float(belief.config.gas_blocked_threshold),
    }
    layers = np.stack((
        belief.final_cost_map,
        belief.temperature_belief_map,
        belief.co_belief_map,
        probability,
        belief.observed_mask,
        belief.temperature_observed_mask,
        belief.co_observed_mask,
        belief.blocked_mask,
        belief.static_obstacle_map,
        belief.dynamic_inflated_obstacle_map,
    )).astype(np.float32)
    message = Float32MultiArray()
    message.layout.dim = [
        MultiArrayDimension(
            label=json.dumps(metadata, separators=(",", ":")),
            size=len(CHANNELS), stride=layers.size,
        ),
        MultiArrayDimension(
            label="height", size=layers.shape[1],
            stride=layers.shape[1] * layers.shape[2],
        ),
        MultiArrayDimension(
            label="width", size=layers.shape[2], stride=layers.shape[2],
        ),
    ]
    message.data = layers.reshape(-1).tolist()
    return message


def decode_hazard_snapshot_message(message):
    if len(message.layout.dim) != 3:
        raise ValueError("hazard snapshot requires channel/height/width dimensions")
    channels, height, width = (int(item.size) for item in message.layout.dim)
    if channels != len(CHANNELS) or height <= 0 or width <= 0:
        raise ValueError("hazard snapshot dimensions are invalid")
    try:
        metadata = json.loads(message.layout.dim[0].label)
    except (TypeError, ValueError) as exc:
        raise ValueError("hazard snapshot metadata is invalid") from exc
    required = {
        "revision", "status", "frame_id", "resolution", "origin_x",
        "origin_y", "origin_yaw", "base_cost", "temperature_blocked_c",
        "co_blocked_ppm",
    }
    if required - set(metadata):
        raise ValueError("hazard snapshot metadata is incomplete")
    values = np.asarray(message.data, dtype=float)
    if values.size != channels * height * width:
        raise ValueError("hazard snapshot payload length is invalid")
    layers = values.reshape(channels, height, width)
    return metadata, {name: layers[index].copy() for index, name in enumerate(CHANNELS)}
