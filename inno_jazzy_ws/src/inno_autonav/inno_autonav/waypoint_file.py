"""Pure-Python validation for saved waypoint queue documents."""

from __future__ import annotations

import math
from typing import Mapping, Sequence


class WaypointFileError(ValueError):
    pass


def _number(value, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise WaypointFileError(f'{label} must be numeric') from exc
    if not math.isfinite(result):
        raise WaypointFileError(f'{label} must be finite')
    return result


def validated_pose_values(document: Mapping, map_frame: str):
    """Return validated numeric pose tuples without importing ROS messages."""
    if not isinstance(document, Mapping):
        raise WaypointFileError('waypoint YAML root must be a mapping')
    header = document.get('header', {})
    if not isinstance(header, Mapping):
        raise WaypointFileError('waypoint YAML header must be a mapping')
    frame = header.get('frame_id', map_frame)
    if frame and frame != map_frame:
        raise WaypointFileError(
            f'waypoint frame {frame!r} does not match {map_frame!r}'
        )
    entries = document.get('poses')
    if not isinstance(entries, Sequence) or isinstance(entries, (str, bytes)):
        raise WaypointFileError('waypoint YAML poses must be a sequence')
    if not entries:
        raise WaypointFileError('waypoint YAML contains no poses')
    result = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, Mapping):
            raise WaypointFileError(f'waypoint {index} must be a mapping')
        entry_header = entry.get('header', {})
        if not isinstance(entry_header, Mapping):
            raise WaypointFileError(f'waypoint {index} header must be a mapping')
        entry_frame = entry_header.get('frame_id', map_frame)
        if entry_frame and entry_frame != map_frame:
            raise WaypointFileError(
                f'waypoint {index} frame {entry_frame!r} does not match {map_frame!r}'
            )
        pose = entry.get('pose')
        if not isinstance(pose, Mapping):
            raise WaypointFileError(f'waypoint {index} is missing pose')
        position = pose.get('position')
        orientation = pose.get('orientation')
        if not isinstance(position, Mapping) or not isinstance(orientation, Mapping):
            raise WaypointFileError(f'waypoint {index} position/orientation is invalid')
        xyz = tuple(_number(position.get(name), f'waypoint {index} position.{name}')
                    for name in ('x', 'y', 'z'))
        xyzw = tuple(_number(orientation.get(name),
                             f'waypoint {index} orientation.{name}')
                     for name in ('x', 'y', 'z', 'w'))
        norm = math.sqrt(sum(value * value for value in xyzw))
        if not math.isclose(norm, 1.0, abs_tol=1e-6):
            raise WaypointFileError(
                f'waypoint {index} quaternion norm is {norm:.6f}, expected 1'
            )
        result.append(xyz + xyzw)
    return tuple(result)
