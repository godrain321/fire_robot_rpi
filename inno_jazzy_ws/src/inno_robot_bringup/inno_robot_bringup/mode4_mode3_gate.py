"""Start the existing Mode 3 demo and exit after its real completion status."""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Int32, String


MODE3_READY = 'READY'
MODE3_RUNNING_PREFIX = 'RUNNING:'
MODE3_COMPLETE = 'COMPLETE:ONE_TURN'


class Mode4Mode3Gate(Node):
    def __init__(self) -> None:
        super().__init__('mode4_mode3_gate')
        self.declare_parameter('startup_timeout_sec', 15.0)
        self.declare_parameter('completion_timeout_sec', 30.0)
        self.startup_timeout_sec = float(
            self.get_parameter('startup_timeout_sec').value
        )
        self.completion_timeout_sec = float(
            self.get_parameter('completion_timeout_sec').value
        )
        if self.startup_timeout_sec <= 0.0 or self.completion_timeout_sec <= 0.0:
            raise ValueError('Mode 4 gate timeouts must be positive')

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.operator_mode_publisher = self.create_publisher(
            Int32, '/operator_mode', latched
        )
        self.drive_mode_publisher = self.create_publisher(
            Int32, '/drive_mode', 10
        )
        self.request_publisher = self.create_publisher(
            Empty, '/mode3/demo_request', 10
        )
        self.cancel_publisher = self.create_publisher(
            Empty, '/autonomy_cancel', 10
        )
        self.create_subscription(
            String, '/mode3_demo/status', self._status_callback, latched
        )
        self.create_timer(0.1, self._tick)

        self.phase = 'WAIT_READY'
        self.last_status = ''
        self.started_at = time.monotonic()
        self.completion_deadline: Optional[float] = None
        self.exit_code: Optional[int] = None
        self.result_message = ''
        self.get_logger().info(
            '[MODE 4] 기존 Mode 3 node 준비와 '
            '실제 완료 신호를 기다립니다.'
        )

    def _finish(self, exit_code: int, message: str) -> None:
        if self.exit_code is not None:
            return
        self.exit_code = int(exit_code)
        self.result_message = str(message)
        if self.exit_code == 0:
            self.get_logger().info(message)
        else:
            self.cancel_publisher.publish(Empty())
            self.get_logger().error(message)

    def _status_callback(self, message: String) -> None:
        status = str(message.data).strip()
        self.last_status = status
        if status.startswith('ERROR:'):
            self._finish(1, f'[MODE 4] Mode 3 오류 상태 수신: {status}')
            return
        if status == 'CANCELLED' and self.phase != 'WAIT_READY':
            self._finish(1, '[MODE 4] Mode 3가 완료 전에 취소되었습니다.')
            return
        if self.phase == 'WAIT_RUNNING' and status.startswith(
            MODE3_RUNNING_PREFIX
        ):
            self.phase = 'WAIT_COMPLETE'
            self.get_logger().info('[MODE 4] Mode 3 동작 시작 확인')
            return
        if self.phase == 'WAIT_COMPLETE' and status == MODE3_COMPLETE:
            self._finish(
                0,
                '[MODE 4] Mode 3 완료 확인: COMPLETE:ONE_TURN',
            )

    def _tick(self) -> None:
        if self.exit_code is not None:
            return
        now = time.monotonic()
        if self.phase == 'WAIT_READY':
            if now - self.started_at > self.startup_timeout_sec:
                self._finish(1, '[MODE 4] Mode 3 node 준비 시간 초과')
                return
            if (
                self.last_status == MODE3_READY
                and self.request_publisher.get_subscription_count() > 0
                and self.operator_mode_publisher.get_subscription_count() > 0
                and self.drive_mode_publisher.get_subscription_count() > 0
            ):
                self.phase = 'WAIT_RUNNING'
                self.completion_deadline = now + self.completion_timeout_sec
                self.operator_mode_publisher.publish(Int32(data=3))
                self.drive_mode_publisher.publish(Int32(data=3))
                self.request_publisher.publish(Empty())
                self.get_logger().warning(
                    '[MODE 4] 기존 Mode 3 안내 음성 + 제자리 1회전 시작'
                )
            return
        if (
            self.completion_deadline is not None
            and now > self.completion_deadline
        ):
            self._finish(1, '[MODE 4] Mode 3 실제 완료 신호 대기 시간 초과')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = Mode4Mode3Gate()
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.exit_code is not None:
            exit_code = node.exit_code
    except KeyboardInterrupt:
        exit_code = 130
    except ValueError as error:
        print(f'mode4_mode3_gate: {error}', flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
