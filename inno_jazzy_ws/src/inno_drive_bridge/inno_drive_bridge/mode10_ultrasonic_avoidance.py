"""HC-SR04 trigger with LiDAR-checked turn and optional cmd_vel safety filter."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from .mode10_avoidance_core import Mode10Avoidance, sector_clearance


def copy_twist(source):
    """Copy the planar fields used by this robot into a new Twist."""
    result = Twist()
    result.linear.x = float(source.linear.x)
    result.linear.y = float(source.linear.y)
    result.linear.z = float(source.linear.z)
    result.angular.x = float(source.angular.x)
    result.angular.y = float(source.angular.y)
    result.angular.z = float(source.angular.z)
    return result


class Mode10UltrasonicAvoidance(Node):
    def __init__(self):
        super().__init__('mode10_ultrasonic_avoidance')
        defaults = {
            'trigger_distance_m': 0.50,
            'clear_distance_m': 0.70,
            'hold_sec': 1.0,
            'cruise_speed_mps': 0.06,
            'turn_speed_radps': 0.35,
            'side_clearance_m': 0.70,
            'front_clearance_m': 0.70,
            'auto_start': True,
            # Empty keeps the original standalone Mode 10 behaviour.  The
            # integrated profile supplies /cmd_vel_before_ultrasonic here.
            'input_cmd_vel_topic': '',
            'output_cmd_vel_topic': '/cmd_vel',
            'input_timeout_sec': 0.35,
            'active_operator_modes': [2],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.control = Mode10Avoidance(
            trigger_m=float(get('trigger_distance_m')),
            clear_m=float(get('clear_distance_m')),
            hold_sec=float(get('hold_sec')),
            cruise_mps=float(get('cruise_speed_mps')),
            turn_radps=float(get('turn_speed_radps')),
            side_clearance_m=float(get('side_clearance_m')),
            front_clearance_m=float(get('front_clearance_m')),
        )
        self.control.set_armed(bool(get('auto_start')))
        self.input_cmd_vel_topic = str(get('input_cmd_vel_topic')).strip()
        self.output_cmd_vel_topic = str(get('output_cmd_vel_topic')).strip()
        self.input_timeout = float(get('input_timeout_sec'))
        self.active_operator_modes = {
            int(mode) for mode in get('active_operator_modes')
        }
        if not self.output_cmd_vel_topic or self.input_timeout <= 0.0:
            raise ValueError('output topic and input timeout must be valid')
        if (self.input_cmd_vel_topic
                and self.input_cmd_vel_topic == self.output_cmd_vel_topic):
            raise ValueError('input and output cmd_vel topics must differ')
        if not self.active_operator_modes:
            raise ValueError('active_operator_modes must not be empty')

        self.passthrough_enabled = bool(self.input_cmd_vel_topic)
        self.upstream_command = Twist()
        self.upstream_received_at = None
        self.operator_mode = 1
        self.velocity_publisher = self.create_publisher(
            Twist, self.output_cmd_vel_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, '/mode10/status', 10
        )
        self.create_subscription(
            Range, '/ultrasonic/front/range', self._on_range, 10
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        if self.passthrough_enabled:
            self.create_subscription(
                Twist, self.input_cmd_vel_topic, self._on_upstream_command, 10
            )
            self.create_subscription(
                Int32, '/operator_mode', self._on_operator_mode, 10
            )
        self.create_service(Trigger, '/mode10/start', self._start)
        self.create_service(Trigger, '/mode10/stop', self._stop)
        self.last_state = None
        self.last_distance_log = float('-inf')
        self.warning_active = False
        self.create_timer(0.05, self._tick)
        if self.passthrough_enabled:
            self.get_logger().info(
                '초음파 안전 필터 준비: Mode 2에서 정상 cmd_vel 통과, '
                '50 cm 미만 1초 지속 시 LiDAR 확인 회피'
            )
        else:
            self.get_logger().info(
                '모드 10 시작: 초음파와 LiDAR가 준비되면 저속 전진'
            )

    def _on_upstream_command(self, message):
        self.upstream_command = copy_twist(message)
        self.upstream_received_at = time.monotonic()

    def _on_operator_mode(self, message):
        mode = int(message.data)
        if mode != self.operator_mode:
            self.control.below_since = None
            self.control.turn_direction = 0
            self.control.turn_started = None
            self.control.clear_since = None
            if self.control.armed:
                self.control.state = 'WAIT_SENSOR'
        self.operator_mode = mode

    def _on_range(self, message):
        now = time.monotonic()
        distance = float(message.range)
        self.control.on_distance(distance, now)
        if not math.isfinite(distance):
            if now - self.last_distance_log >= 1.0:
                self.get_logger().warning('초음파 거리: 측정 실패')
                self.last_distance_log = now
            self.warning_active = False
            return
        if now - self.last_distance_log >= 0.5:
            self.get_logger().info(f'초음파 거리: {distance * 100.0:.1f} cm')
            self.last_distance_log = now
        if distance < self.control.trigger_m and not self.warning_active:
            self.get_logger().warning('전방장애물주의!')
            self.warning_active = True
        elif distance >= self.control.trigger_m:
            self.warning_active = False

    def _on_scan(self, message):
        self.control.on_scan(
            sector_clearance(message, -25.0, 25.0),
            sector_clearance(message, 35.0, 100.0),
            sector_clearance(message, -100.0, -35.0),
            time.monotonic(),
        )

    def _fresh_upstream(self, now):
        if (self.upstream_received_at is None
                or now - self.upstream_received_at > self.input_timeout):
            return None
        return copy_twist(self.upstream_command)

    def _integrated_command(self, now):
        upstream = self._fresh_upstream(now)
        if self.operator_mode not in self.active_operator_modes:
            return upstream or Twist(), (
                'BYPASS' if upstream is not None else 'WAIT_COMMAND'
            )
        if not self.control.armed:
            return upstream or Twist(), (
                'DISARMED_BYPASS' if upstream is not None else 'WAIT_COMMAND'
            )

        linear, angular, state = self.control.command(now)
        if state == 'CRUISE':
            if upstream is None:
                return Twist(), 'WAIT_COMMAND'
            return upstream, 'PASS'
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        return command, state

    def _tick(self):
        now = time.monotonic()
        if self.passthrough_enabled:
            command, state = self._integrated_command(now)
        else:
            linear, angular, state = self.control.command(now)
            command = Twist()
            command.linear.x = linear
            command.angular.z = angular
        self.velocity_publisher.publish(command)
        if state != self.last_state:
            self.status_publisher.publish(String(data=state))
            self.get_logger().info(f'초음파 안전 상태: {state}')
            if state == 'TURN':
                direction = '좌회전' if command.angular.z > 0 else '우회전'
                self.get_logger().warning(
                    f'50 cm 미만 1초 지속: {direction} 회피 모터 명령 발행'
                )
            elif state == 'BLOCKED':
                self.get_logger().error('회피할 공간이 없거나 너무 가까움: 모터 정지')
            self.last_state = state

    def _start(self, _request, response):
        self.control.set_armed(True)
        response.success = True
        response.message = 'ULTRASONIC_SAFETY_ARMED'
        return response

    def _stop(self, _request, response):
        self.control.set_armed(False)
        self.velocity_publisher.publish(Twist())
        response.success = True
        response.message = 'ULTRASONIC_SAFETY_DISARMED'
        return response

    def destroy_node(self):
        if rclpy.ok():
            self.velocity_publisher.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Mode10UltrasonicAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
