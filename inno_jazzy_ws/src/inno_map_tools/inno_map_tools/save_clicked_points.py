"""Append RViz Publish Point clicks to a debug YAML file."""

from pathlib import Path
from typing import Any, Dict

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node

from .map_utils import MapToolsError, atomic_save_yaml, load_yaml


DEFAULT_OUTPUT = '/home/gosunwoo/fire_robot_rpi/maps/clicked_points_debug.yaml'


class ClickedPointRecorder(Node):
    def __init__(self) -> None:
        super().__init__('map_tools')
        self.declare_parameter('output_file', DEFAULT_OUTPUT)
        self.declare_parameter('topic', '/clicked_point')
        self.declare_parameter('expected_frame', 'map')

        self.output_file = Path(
            str(self.get_parameter('output_file').value)
        ).expanduser().resolve(strict=False)
        self.topic = str(self.get_parameter('topic').value)
        self.expected_frame = str(self.get_parameter('expected_frame').value)
        if not self.output_file.parent.is_dir():
            raise MapToolsError(
                f'출력 상위 디렉터리가 없습니다: {self.output_file.parent}'
            )

        self.subscription = self.create_subscription(
            PointStamped, self.topic, self._on_point, 10
        )
        self.get_logger().info(
            f'{self.topic} 기록 시작: frame={self.expected_frame}, '
            f'output={self.output_file}'
        )

    def _load_document(self) -> Dict[str, Any]:
        if not self.output_file.exists():
            return {'frame_id': self.expected_frame, 'clicked_points': []}
        _, document = load_yaml(self.output_file, 'clicked points')
        points = document.get('clicked_points')
        if not isinstance(points, list):
            raise MapToolsError('clicked_points 항목은 list여야 합니다.')
        frame_id = document.get('frame_id', self.expected_frame)
        if frame_id != self.expected_frame:
            raise MapToolsError(
                f'기존 파일 frame_id={frame_id!r}, '
                f'expected_frame={self.expected_frame!r}가 다릅니다.'
            )
        document['frame_id'] = frame_id
        return document

    def _on_point(self, message: PointStamped) -> None:
        if message.header.frame_id != self.expected_frame:
            self.get_logger().error(
                f'frame_id={message.header.frame_id!r} 클릭을 거부했습니다. '
                f'{self.expected_frame!r} frame에서 클릭하십시오.'
            )
            return
        try:
            document = self._load_document()
            entry = {
                'x': float(message.point.x),
                'y': float(message.point.y),
                'z': float(message.point.z),
                'timestamp': {
                    'sec': int(message.header.stamp.sec),
                    'nanosec': int(message.header.stamp.nanosec),
                },
            }
            document['clicked_points'].append(entry)
            atomic_save_yaml(self.output_file, document)
            index = len(document['clicked_points'])
            self.get_logger().info(
                f'point #{index} 저장: x={entry["x"]:.6f}, '
                f'y={entry["y"]:.6f}, z={entry["z"]:.6f}'
            )
        except MapToolsError as exc:
            self.get_logger().error(str(exc))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ClickedPointRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except MapToolsError as exc:
        print(f'오류: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
