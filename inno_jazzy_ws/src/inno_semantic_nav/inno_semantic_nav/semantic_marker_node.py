"""Continuously visualize semantic poses and landmarks as RViz markers."""

import math
from pathlib import Path
from typing import Any, Dict, Iterable, Tuple
import zlib

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from .geometry_utils import normalize_yaw, quaternion_from_yaw
from .semantic_store import SemanticStore, SemanticStoreError


CATEGORY_COLORS: Dict[str, Tuple[float, float, float, float]] = {
    'exit': (0.90, 0.16, 0.12, 1.0),
    'machine': (0.95, 0.55, 0.10, 1.0),
    'industrial_machinery': (0.95, 0.35, 0.05, 1.0),
    'pillar': (0.55, 0.55, 0.60, 1.0),
}
DEFAULT_POSE_COLOR = (0.10, 0.65, 0.95, 1.0)
DEFAULT_LANDMARK_COLOR = (0.20, 0.80, 0.35, 1.0)
TEXT_COLOR = (1.0, 1.0, 1.0, 1.0)


def _stable_id(*parts: str) -> int:
    key = '\x1f'.join(parts).encode('utf-8')
    return zlib.crc32(key) & 0x7FFFFFFF


def _finite(entry: Dict[str, Any], field: str, owner: str) -> float:
    try:
        value = float(entry[field])
    except (KeyError, TypeError, ValueError) as exc:
        raise SemanticStoreError(f'{owner}의 {field} 값이 없거나 숫자가 아닙니다.') from exc
    if not math.isfinite(value):
        raise SemanticStoreError(f'{owner}의 {field} 값은 유한한 숫자여야 합니다.')
    return value


def _set_color(marker: Marker, color: Iterable[float]) -> None:
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color


def _base_marker(node: Node, frame_id: str, namespace: str, marker_id: int) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = namespace
    marker.id = marker_id
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


class SemanticMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('semantic_marker_node')
        self.declare_parameter('semantic_file', '')
        self.declare_parameter('reload_period', 0.5)
        semantic_file = self.get_parameter('semantic_file').get_parameter_value().string_value
        if not semantic_file:
            raise SemanticStoreError('semantic_file 파라미터가 비어 있습니다.')
        self._store = SemanticStore(semantic_file)
        self._path: Path = self._store.path

        reload_period = self.get_parameter('reload_period').value
        if not isinstance(reload_period, (int, float)) or reload_period <= 0.0:
            raise SemanticStoreError('reload_period 파라미터는 0보다 커야 합니다.')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(MarkerArray, '/semantic_markers', qos)
        self._never_seen = object()
        self._last_seen_signature: Any = self._never_seen
        self._timer = self.create_timer(float(reload_period), self._reload_if_changed)
        self._reload_if_changed()

    def _signature(self):
        try:
            stat = self._path.stat()
        except FileNotFoundError:
            return None
        except OSError as exc:
            return ('error', str(exc))
        return (stat.st_mtime_ns, stat.st_size, stat.st_ino)

    def _reload_if_changed(self) -> None:
        signature = self._signature()
        if signature == self._last_seen_signature:
            return
        self._last_seen_signature = signature
        try:
            document = self._store.load(create_if_missing=True)
            markers = self._build_markers(document)
        except (SemanticStoreError, ValueError, TypeError) as exc:
            self.get_logger().error(
                f'semantic 파일을 다시 읽지 못했습니다. 기존 Marker를 유지합니다: {exc}'
            )
            return

        self._publisher.publish(markers)
        self.get_logger().info(
            f'semantic Marker 갱신: poses={len(document["poses"])}, '
            f'landmarks={len(document["landmarks"])} ({self._path})'
        )

    def _build_markers(self, document: Dict[str, Any]) -> MarkerArray:
        frame_id = document.get('frame_id', 'map')
        if not isinstance(frame_id, str) or not frame_id:
            raise SemanticStoreError('frame_id는 비어 있지 않은 문자열이어야 합니다.')

        clear = Marker()
        clear.header.frame_id = frame_id
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers = [clear]

        for name, entry in sorted(document['poses'].items()):
            if not isinstance(entry, dict):
                raise SemanticStoreError(f'pose {name!r}의 값은 mapping이어야 합니다.')
            x = _finite(entry, 'x', f'pose {name!r}')
            y = _finite(entry, 'y', f'pose {name!r}')
            yaw = normalize_yaw(_finite(entry, 'yaw', f'pose {name!r}'))
            category = str(entry.get('category', ''))
            color = CATEGORY_COLORS.get(category, DEFAULT_POSE_COLOR)

            body = _base_marker(
                self, frame_id, 'semantic_pose_body', _stable_id('pose', name, 'body')
            )
            body.type = Marker.CYLINDER
            body.pose.position.x = x
            body.pose.position.y = y
            body.pose.position.z = 0.10
            body.scale.x = 0.24
            body.scale.y = 0.24
            body.scale.z = 0.20
            _set_color(body, color)
            markers.append(body)

            arrow = _base_marker(
                self, frame_id, 'semantic_pose_yaw', _stable_id('pose', name, 'yaw')
            )
            arrow.type = Marker.ARROW
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.22
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            arrow.pose.orientation.x = qx
            arrow.pose.orientation.y = qy
            arrow.pose.orientation.z = qz
            arrow.pose.orientation.w = qw
            arrow.scale.x = 0.55
            arrow.scale.y = 0.08
            arrow.scale.z = 0.12
            _set_color(arrow, color)
            markers.append(arrow)

            text = _base_marker(
                self, frame_id, 'semantic_pose_text', _stable_id('pose', name, 'text')
            )
            text.type = Marker.TEXT_VIEW_FACING
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.55
            text.scale.z = 0.22
            text.text = f'{name} [{category}]' if category else str(name)
            _set_color(text, TEXT_COLOR)
            markers.append(text)

        for name, entry in sorted(document['landmarks'].items()):
            if not isinstance(entry, dict):
                raise SemanticStoreError(f'landmark {name!r}의 값은 mapping이어야 합니다.')
            x = _finite(entry, 'x', f'landmark {name!r}')
            y = _finite(entry, 'y', f'landmark {name!r}')
            category = str(entry.get('category', ''))
            color = CATEGORY_COLORS.get(category, DEFAULT_LANDMARK_COLOR)

            body = _base_marker(
                self, frame_id, 'semantic_landmark_body', _stable_id('landmark', name, 'body')
            )
            body.type = Marker.CUBE
            body.pose.position.x = x
            body.pose.position.y = y
            body.pose.position.z = 0.13
            body.scale.x = 0.26
            body.scale.y = 0.26
            body.scale.z = 0.26
            _set_color(body, color)
            markers.append(body)

            text = _base_marker(
                self, frame_id, 'semantic_landmark_text', _stable_id('landmark', name, 'text')
            )
            text.type = Marker.TEXT_VIEW_FACING
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.50
            text.scale.z = 0.20
            text.text = f'{name} [{category}]' if category else str(name)
            _set_color(text, TEXT_COLOR)
            markers.append(text)

        return MarkerArray(markers=markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticMarkerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SemanticStoreError as exc:
        print(f'오류: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
