"""ROS adapter for conservative C4001 motion-state inference."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from .mobility_classifier import (
    MobilityClassifier,
    MobilityConfig,
    human_state_from_mobility,
)


class MobilityNode(Node):
    def __init__(self) -> None:
        super().__init__('mmwave_mobility')
        defaults = {
            'moving_speed_threshold_mps': 0.20,
            'moving_confirm_samples': 3,
            'moving_confirm_sec': 0.20,
            'moving_hold_sec': 1.0,
            'assist_check_sec': 10.0,
            'robot_linear_threshold_mps': 0.01,
            'robot_angular_threshold_rps': 0.03,
            'robot_settle_sec': 2.0,
            'publish_rate_hz': 10.0,
            'sensor_stale_timeout_sec': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        config = MobilityConfig(
            moving_speed_threshold_mps=float(
                self.get_parameter('moving_speed_threshold_mps').value
            ),
            moving_confirm_samples=int(
                self.get_parameter('moving_confirm_samples').value
            ),
            moving_confirm_sec=float(
                self.get_parameter('moving_confirm_sec').value
            ),
            moving_hold_sec=float(self.get_parameter('moving_hold_sec').value),
            assist_check_sec=float(self.get_parameter('assist_check_sec').value),
            robot_linear_threshold_mps=float(
                self.get_parameter('robot_linear_threshold_mps').value
            ),
            robot_angular_threshold_rps=float(
                self.get_parameter('robot_angular_threshold_rps').value
            ),
            robot_settle_sec=float(
                self.get_parameter('robot_settle_sec').value
            ),
        )
        self.classifier = MobilityClassifier(config)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')
        self.sensor_stale_timeout = float(
            self.get_parameter('sensor_stale_timeout_sec').value
        )
        if self.sensor_stale_timeout <= 0.0:
            raise ValueError('sensor_stale_timeout_sec must be greater than zero')
        self._presence = False
        self._last_state = None
        self._last_sensor_update = float('-inf')
        self.state_publisher = self.create_publisher(
            String, '/mmwave/mobility_state', 10
        )
        self.human_state_publisher = self.create_publisher(
            String, '/mmwave/human_state', 10
        )
        self.still_publisher = self.create_publisher(
            Float32, '/mmwave/still_duration_sec', 10
        )
        self.create_subscription(
            Bool, '/mmwave/human_presence', self._presence_callback, 10
        )
        self.create_subscription(
            Float32, '/mmwave/filtered_speed_mps', self._speed_callback, 10
        )
        self.create_subscription(
            String, '/mmwave/sensor_state', self._sensor_callback, 10
        )
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self.create_timer(1.0 / rate, self._publish)

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _presence_callback(self, message: Bool) -> None:
        self._last_sensor_update = self._now()
        self._presence = bool(message.data)
        self.classifier.update_presence(self._presence, self._now())

    def _speed_callback(self, message: Float32) -> None:
        self._last_sensor_update = self._now()
        self.classifier.update_speed(float(message.data), self._now())

    def _sensor_callback(self, message: String) -> None:
        self._last_sensor_update = self._now()
        self.classifier.update_sensor_state(message.data)

    def _cmd_vel_callback(self, message: Twist) -> None:
        self.classifier.update_robot_command(
            message.linear.x, message.angular.z, self._now()
        )

    def _publish(self) -> None:
        now = self._now()
        if now - self._last_sensor_update > self.sensor_stale_timeout:
            self.classifier.update_sensor_state('OFFLINE')
        state = self.classifier.state(now)
        self.state_publisher.publish(String(data=state))
        self.human_state_publisher.publish(
            String(data=human_state_from_mobility(state))
        )
        self.still_publisher.publish(
            Float32(data=float(self.classifier.still_duration(now)))
        )
        if state != self._last_state:
            self.get_logger().info(f'mobility_state={state}')
            self._last_state = state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MobilityNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as error:
        if node is None:
            print(f'mmwave_mobility: {error}')
        else:
            node.get_logger().error(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
