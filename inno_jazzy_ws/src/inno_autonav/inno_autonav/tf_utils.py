"""Small 2D TF helpers with throttled failure logging."""

from __future__ import annotations

import math
import time
from typing import Dict, Optional, Tuple

from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .grid_utils import yaw_from_quaternion


class TfHelper:
    def __init__(self, node, cache_time_sec: float = 10.0) -> None:
        self.node = node
        self.buffer = Buffer(cache_time=Duration(seconds=cache_time_sec))
        self.listener = TransformListener(self.buffer, node)
        self._last_log: Dict[str, float] = {}

    def _warn_throttled(self, key: str, message: str, period: float = 5.0) -> None:
        now = time.monotonic()
        if now - self._last_log.get(key, -period) >= period:
            self._last_log[key] = now
            self.node.get_logger().warning(message)

    def lookup_transform(self, target_frame: str, source_frame: str):
        try:
            return self.buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.10),
            )
        except TransformException as exc:
            self._warn_throttled(
                f'{target_frame}<-{source_frame}',
                f'TF {target_frame} <- {source_frame} 대기 중: {exc}',
            )
            return None

    def lookup_pose_2d(
        self, map_frame: str, base_frame: str
    ) -> Optional[Tuple[float, float, float]]:
        transform = self.lookup_transform(map_frame, base_frame)
        if transform is None:
            return None
        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        return float(translation.x), float(translation.y), yaw

    @staticmethod
    def transform_point_2d(transform, x: float, y: float) -> Tuple[float, float]:
        yaw = yaw_from_quaternion(transform.transform.rotation)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        translation = transform.transform.translation
        return (
            float(translation.x) + cosine * x - sine * y,
            float(translation.y) + sine * x + cosine * y,
        )
