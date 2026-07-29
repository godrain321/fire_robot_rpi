"""Capture one RViz Publish Point click and store it as a landmark."""

import argparse
import sys
import time
from typing import Optional

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from .semantic_store import (
    DuplicateNameError,
    SemanticStore,
    SemanticStoreError,
    default_document,
    validate_name,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='RViz Publish Point를 landmark로 저장합니다.')
    parser.add_argument('name', help='저장할 landmark 이름')
    parser.add_argument('--semantic-file', required=True, help='semantic_points.yaml 경로')
    parser.add_argument('--category', default='', help='분류(예: machine)')
    parser.add_argument('--description', default='', help='설명')
    parser.add_argument('--timeout', type=float, default=120.0, help='수신 대기시간(초)')
    parser.add_argument('--overwrite', action='store_true', help='같은 이름의 landmark를 덮어쓰기')
    return parser


def _wait_for_point(node: Node, timeout: float) -> Optional[PointStamped]:
    received = []

    def callback(message: PointStamped) -> None:
        if not received:
            received.append(message)

    subscription = node.create_subscription(PointStamped, '/clicked_point', callback, 10)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and not received:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            rclpy.spin_once(node, timeout_sec=min(0.2, remaining))
        return received[0] if received else None
    finally:
        node.destroy_subscription(subscription)


def main(args=None) -> None:
    raw_args = sys.argv if args is None else args
    cli_args = remove_ros_args(args=raw_args)[1:]
    parsed = _parser().parse_args(cli_args)
    if parsed.timeout <= 0.0:
        _parser().error('--timeout은 0보다 커야 합니다.')

    try:
        validate_name(parsed.name)
        store = SemanticStore(parsed.semantic_file)
        existing = (
            store.load(create_if_missing=False) if store.path.exists() else default_document()
        )
        if parsed.name in existing['poses']:
            raise DuplicateNameError(
                f'{parsed.name!r}은(는) 이미 pose 이름으로 사용 중입니다.'
            )
        if parsed.name in existing['landmarks'] and not parsed.overwrite:
            raise DuplicateNameError(
                f'{parsed.name!r}이(가) 이미 존재합니다. 덮어쓰려면 --overwrite를 사용하십시오.'
            )
    except SemanticStoreError as exc:
        print(f'오류: {exc}', file=sys.stderr)
        return

    rclpy.init(args=raw_args)
    node = Node('capture_landmark')
    try:
        print('RViz에서 Publish Point를 선택한 뒤 저장할 위치를 클릭하십시오.')
        message = _wait_for_point(node, parsed.timeout)
        if message is None:
            print(f'오류: /clicked_point를 {parsed.timeout:g}초 안에 받지 못했습니다.', file=sys.stderr)
            return

        expected_frame = existing.get('frame_id', 'map')
        if message.header.frame_id != expected_frame:
            print(
                f'오류: frame_id가 {message.header.frame_id!r}입니다. '
                f'{expected_frame!r} frame의 점만 저장할 수 있습니다.',
                file=sys.stderr,
            )
            return

        store.add_landmark(
            parsed.name,
            message.point.x,
            message.point.y,
            category=parsed.category,
            description=parsed.description,
            overwrite=parsed.overwrite,
        )
        print(
            f'{parsed.name} 저장 완료: x={message.point.x:.6f}, '
            f'y={message.point.y:.6f}\n파일: {store.path}'
        )
    except KeyboardInterrupt:
        print('\n취소되었습니다. semantic 파일은 수정하지 않았습니다.')
    except SemanticStoreError as exc:
        print(f'오류: {exc}', file=sys.stderr)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
