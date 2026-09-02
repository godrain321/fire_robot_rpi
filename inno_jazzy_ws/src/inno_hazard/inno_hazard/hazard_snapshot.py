"""Pack a coherent HazardBelief revision into one Float32MultiArray payload."""

from __future__ import annotations

import json

import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


LEGACY_CHANNELS = (
    "final_cost", "temperature_c", "co_ppm", "fire_probability",
    "observed", "temperature_observed", "co_observed", "blocked",
    "static_obstacle", "dynamic_obstacle",
)
# Public compatibility name used by older tests/tools.
CHANNELS = LEGACY_CHANNELS

REQUIRED_CHANNELS = (
    "final_cost", "static_obstacle", "dynamic_obstacle",
)


def hazard_snapshot_message(
    belief, fire_probability, *, status, final_cost=None,
    include_temperature=True, include_co=True, include_fire=True,
):
    probability = np.asarray(fire_probability, dtype=float)
    if probability.shape != belief.shape:
        raise ValueError("fire probability geometry differs from belief")
    cost = belief.final_cost_map if final_cost is None else np.asarray(
        final_cost, dtype=float
    )
    if cost.shape != belief.shape:
        raise ValueError("final cost geometry differs from belief")

    channel_layers = [
        ("final_cost", cost),
        ("static_obstacle", belief.static_obstacle_map),
        ("dynamic_obstacle", belief.dynamic_inflated_obstacle_map),
    ]
    if include_temperature:
        channel_layers.extend((
            ("temperature_c", belief.temperature_belief_map),
            ("temperature_observed", belief.temperature_observed_mask),
        ))
    if include_co:
        channel_layers.extend((
            ("co_ppm", belief.co_belief_map),
            ("co_observed", belief.co_observed_mask),
        ))
    if include_fire:
        channel_layers.append(("fire_probability", probability))
    channels = tuple(name for name, _layer in channel_layers)
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
        "channels": channels,
    }
    layers = np.stack(tuple(layer for _name, layer in channel_layers)).astype(
        np.float32
    )
    message = Float32MultiArray()
    message.layout.dim = [
        MultiArrayDimension(
            label=json.dumps(metadata, separators=(",", ":")),
            size=len(channels), stride=layers.size,
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
    channel_count, height, width = (
        int(item.size) for item in message.layout.dim
    )
    if channel_count <= 0 or height <= 0 or width <= 0:
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
    if "channels" in metadata:
        channels = tuple(metadata["channels"])
        if (
            len(channels) != channel_count
            or len(set(channels)) != len(channels)
            or set(REQUIRED_CHANNELS) - set(channels)
            or set(channels) - set(LEGACY_CHANNELS)
        ):
            raise ValueError("hazard snapshot channels are invalid")
    else:
        # Backward compatibility for already-built/recorded 10-layer data.
        if channel_count != len(LEGACY_CHANNELS):
            raise ValueError("legacy hazard snapshot channels are invalid")
        channels = LEGACY_CHANNELS
    values = np.asarray(message.data, dtype=float)
    if values.size != channel_count * height * width:
        raise ValueError("hazard snapshot payload length is invalid")
    encoded = values.reshape(channel_count, height, width)
    decoded = {
        name: encoded[index].copy() for index, name in enumerate(channels)
    }
    shape = (height, width)
    decoded.setdefault("temperature_c", np.full(shape, np.nan))
    decoded.setdefault("co_ppm", np.full(shape, np.nan))
    decoded.setdefault("fire_probability", np.zeros(shape))
    decoded.setdefault("temperature_observed", np.zeros(shape))
    decoded.setdefault("co_observed", np.zeros(shape))
    decoded.setdefault(
        "observed",
        np.logical_or(
            decoded["temperature_observed"], decoded["co_observed"]
        ).astype(float),
    )
    decoded.setdefault(
        "blocked",
        np.logical_not(np.isfinite(decoded["final_cost"])).astype(float),
    )
    return metadata, decoded
