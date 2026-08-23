"""Configure and activate a small ordered set of ROS lifecycle nodes."""

from __future__ import annotations

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node


class LifecycleAutostart(Node):
    def __init__(self):
        super().__init__('lifecycle_autostart')
        self.declare_parameter('node_names', ['map_server'])
        self.declare_parameter('retry_period_sec', 0.25)
        self.node_names = [
            str(name).strip('/') for name in self.get_parameter(
                'node_names'
            ).value
        ]
        retry_period = float(self.get_parameter('retry_period_sec').value)
        if not self.node_names or any(not name for name in self.node_names):
            raise ValueError('node_names에는 하나 이상의 lifecycle node가 필요합니다.')
        if retry_period <= 0.0:
            raise ValueError('retry_period_sec는 0보다 커야 합니다.')
        self.index = 0
        self.transition_id = Transition.TRANSITION_CONFIGURE
        self.client = None
        self.future = None
        self.timer = self.create_timer(retry_period, self._advance)

    def _advance(self):
        if self.future is not None:
            if not self.future.done():
                return
            try:
                success = bool(self.future.result().success)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'lifecycle service call failed: {exc}')
                self.future = None
                return
            node_name = self.node_names[self.index]
            transition_name = (
                'configure'
                if self.transition_id == Transition.TRANSITION_CONFIGURE
                else 'activate'
            )
            if not success:
                self.get_logger().error(
                    f'{node_name} {transition_name} transition rejected'
                )
                self.timer.cancel()
                return
            self.get_logger().info(f'{node_name}: {transition_name} complete')
            self.future = None
            if self.transition_id == Transition.TRANSITION_CONFIGURE:
                self.transition_id = Transition.TRANSITION_ACTIVATE
            else:
                self.index += 1
                self.transition_id = Transition.TRANSITION_CONFIGURE
                self.client = None
                if self.index >= len(self.node_names):
                    self.get_logger().info('All lifecycle nodes are active')
                    self.timer.cancel()
                    return

        node_name = self.node_names[self.index]
        if self.client is None:
            self.client = self.create_client(
                ChangeState, f'/{node_name}/change_state'
            )
        if not self.client.service_is_ready():
            return
        request = ChangeState.Request()
        request.transition.id = self.transition_id
        self.future = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LifecycleAutostart()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
