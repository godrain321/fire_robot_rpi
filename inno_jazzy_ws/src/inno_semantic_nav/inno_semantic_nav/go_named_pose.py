"""Resolve a named pose and optionally send a Nav2 NavigateToPose goal."""

import argparse
import math
import sys
import time
from typing import Any, Dict, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args

from .geometry_utils import normalize_yaw, quaternion_from_yaw
from .semantic_store import SemanticStore, SemanticStoreError, validate_name


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='이름으로 Nav2 NavigateToPose 목표를 생성합니다.')
    parser.add_argument('name', help='이동할 pose 이름')
    parser.add_argument('--semantic-file', required=True, help='semantic_points.yaml 경로')
    parser.add_argument('--dry-run', action='store_true', help='액션을 보내지 않고 pose만 발행')
    parser.add_argument('--action-name', default='/navigate_to_pose', help='NavigateToPose 액션 이름')
    parser.add_argument(
        '--server-timeout', type=float, default=5.0, help='액션 서버 대기시간(초)'
    )
    return parser


def _finite_pose_value(pose: Dict[str, Any], field: str) -> float:
    try:
        value = float(pose[field])
    except (KeyError, TypeError, ValueError) as exc:
        raise SemanticStoreError(f'pose의 {field} 값이 없거나 숫자가 아닙니다.') from exc
    if not math.isfinite(value):
        raise SemanticStoreError(f'pose의 {field} 값은 유한한 숫자여야 합니다.')
    return value


def _make_pose(node: Node, frame_id: str, entry: Dict[str, Any]) -> PoseStamped:
    x = _finite_pose_value(entry, 'x')
    y = _finite_pose_value(entry, 'y')
    yaw = normalize_yaw(_finite_pose_value(entry, 'yaw'))
    qx, qy, qz, qw = quaternion_from_yaw(yaw)

    message = PoseStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = frame_id
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.position.z = 0.0
    message.pose.orientation.x = qx
    message.pose.orientation.y = qy
    message.pose.orientation.z = qz
    message.pose.orientation.w = qw
    return message


def _print_pose(name: str, pose: PoseStamped, yaw: float) -> None:
    stamp = pose.header.stamp
    position = pose.pose.position
    orientation = pose.pose.orientation
    print(
        f'Named Pose: {name}\n'
        f'  header.frame_id: {pose.header.frame_id}\n'
        f'  header.stamp: {stamp.sec}.{stamp.nanosec:09d}\n'
        f'  position: x={position.x:.6f}, y={position.y:.6f}, z={position.z:.6f}\n'
        f'  orientation: x={orientation.x:.6f}, y={orientation.y:.6f}, '
        f'z={orientation.z:.6f}, w={orientation.w:.6f}\n'
        f'  yaw: {yaw:.6f} rad'
    )


def _duration_text(duration_message: Any) -> str:
    if hasattr(duration_message, 'sec') and hasattr(duration_message, 'nanosec'):
        return f'{duration_message.sec + duration_message.nanosec / 1.0e9:.1f}s'
    return str(duration_message)


def _feedback_callback(node: Node, feedback_message: Any) -> None:
    feedback = getattr(feedback_message, 'feedback', feedback_message)
    fields = []
    if hasattr(feedback, 'current_pose'):
        position = feedback.current_pose.pose.position
        fields.append(f'current_pose=({position.x:.2f}, {position.y:.2f})')
    if hasattr(feedback, 'distance_remaining'):
        fields.append(f'distance_remaining={feedback.distance_remaining:.2f}m')
    if hasattr(feedback, 'navigation_time'):
        fields.append(f'navigation_time={_duration_text(feedback.navigation_time)}')
    if hasattr(feedback, 'estimated_time_remaining'):
        fields.append(
            f'estimated_time_remaining={_duration_text(feedback.estimated_time_remaining)}'
        )
    if fields:
        node.get_logger().info('Nav2 feedback: ' + ', '.join(fields))


def _briefly_deliver_pose(node: Node) -> None:
    deadline = time.monotonic() + 0.3
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def _print_result_details(wrapped_result: Any) -> None:
    """Print the Jazzy NavigateToPose result fields when they are available."""
    result = getattr(wrapped_result, 'result', None)
    if result is None or not hasattr(result, 'error_code'):
        return

    error_code = result.error_code
    error_message = str(getattr(result, 'error_msg', '')).strip()
    detail = f'Nav2 결과: error_code={error_code}'
    if error_message:
        detail += f', error_msg={error_message}'
    print(detail)


def main(args=None) -> None:
    raw_args = sys.argv if args is None else args
    cli_args = remove_ros_args(args=raw_args)[1:]
    parsed = _parser().parse_args(cli_args)
    if parsed.server_timeout <= 0.0:
        _parser().error('--server-timeout은 0보다 커야 합니다.')

    try:
        validate_name(parsed.name)
        store = SemanticStore(parsed.semantic_file)
        document = store.load(create_if_missing=True)
        if parsed.name not in document['poses']:
            if parsed.name in document['landmarks']:
                print(
                    f'오류: {parsed.name!r}은(는) landmark이므로 이동 목표로 사용할 수 없습니다.',
                    file=sys.stderr,
                )
            else:
                available = ', '.join(sorted(document['poses'])) or '(없음)'
                print(
                    f'오류: pose {parsed.name!r}을(를) 찾을 수 없습니다. '
                    f'사용 가능한 pose: {available}',
                    file=sys.stderr,
                )
            return
        entry = document['poses'][parsed.name]
        if not isinstance(entry, dict):
            raise SemanticStoreError(f'pose {parsed.name!r}의 값은 mapping이어야 합니다.')
        frame_id = document.get('frame_id', 'map')
    except SemanticStoreError as exc:
        print(f'오류: {exc}', file=sys.stderr)
        return

    rclpy.init(args=raw_args)
    node = Node('go_named_pose')
    goal_handle: Optional[Any] = None
    action_client: Optional[ActionClient] = None
    try:
        pose = _make_pose(node, frame_id, entry)
        yaw = normalize_yaw(_finite_pose_value(entry, 'yaw'))
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        publisher = node.create_publisher(PoseStamped, '/named_goal_pose', qos)
        publisher.publish(pose)
        _print_pose(parsed.name, pose, yaw)

        if parsed.dry_run:
            print('dry-run: /named_goal_pose 발행 완료. Nav2 액션은 전송하지 않았습니다.')
            _briefly_deliver_pose(node)
            return

        action_client = ActionClient(node, NavigateToPose, parsed.action_name)
        if not action_client.wait_for_server(timeout_sec=parsed.server_timeout):
            print(
                '오류: Nav2 NavigateToPose action server가 실행 중이지 않습니다.\n'
                '지도·좌표 변환만 확인하려면 --dry-run을 사용하십시오.',
                file=sys.stderr,
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = ''
        send_future = action_client.send_goal_async(
            goal, feedback_callback=lambda message: _feedback_callback(node, message)
        )
        rclpy.spin_until_future_complete(node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print('오류: NavigateToPose 목표가 거절되었습니다.', file=sys.stderr)
            return

        print(f'NavigateToPose 목표가 수락되었습니다: {parsed.name}')
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not result_future.done():
            return
        wrapped_result = result_future.result()
        status = getattr(wrapped_result, 'status', GoalStatus.STATUS_UNKNOWN)
        print(f'NavigateToPose 최종 상태: {STATUS_NAMES.get(status, f"STATUS_{status}")}')
        _print_result_details(wrapped_result)
    except KeyboardInterrupt:
        print('\n중단 요청을 받았습니다. 진행 중인 목표 취소를 시도합니다.')
        if goal_handle is not None and goal_handle.accepted:
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=2.0)
                print('목표 취소 요청을 전송했습니다.')
            except Exception as exc:  # ROS shutdown races must not hide the user interrupt.
                print(f'경고: 목표 취소 요청 중 오류: {exc}', file=sys.stderr)
    except (SemanticStoreError, ValueError) as exc:
        print(f'오류: {exc}', file=sys.stderr)
    finally:
        if action_client is not None:
            action_client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
