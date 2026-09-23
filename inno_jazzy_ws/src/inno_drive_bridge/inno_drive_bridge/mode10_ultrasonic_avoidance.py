"""Mode 10: HC-SR04-triggered local avoidance with LiDAR side clearance."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .mode10_avoidance_core import Mode10Avoidance, sector_clearance


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
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, '/mode10/status', 10)
        self.create_subscription(
            Range, '/ultrasonic/front/range', self._on_range, 10
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_service(Trigger, '/mode10/start', self._start)
        self.create_service(Trigger, '/mode10/stop', self._stop)
        self.last_state = None
        self.last_distance_log = float('-inf')
        self.warning_active = False
        self.create_timer(0.05, self._tick)
        self.get_logger().info('모드 10 시작: 초음파와 LiDAR가 준비되면 저속 전진')

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
        if now - self.last_distance_log >= 0.2:
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

    def _tick(self):
        linear, angular, state = self.control.command(time.monotonic())
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.velocity_publisher.publish(command)
        if state != self.last_state:
            self.status_publisher.publish(String(data=state))
            self.get_logger().info(f'모드 10 상태: {state}')
            if state == 'TURN':
                direction = '좌회전' if angular > 0 else '우회전'
                self.get_logger().warning(
                    f'50 cm 미만 1초 지속: {direction} 회피 모터 명령 발행'
                )
            elif state == 'BLOCKED':
                self.get_logger().error('회피할 공간이 없거나 너무 가까움: 모터 정지')
            self.last_state = state

    def _start(self, _request, response):
        self.control.set_armed(True)
        response.success = True
        response.message = 'MODE10_ARMED_WAITING_FOR_SENSORS'
        return response

    def _stop(self, _request, response):
        self.control.set_armed(False)
        self.velocity_publisher.publish(Twist())
        response.success = True
        response.message = 'MODE10_STOPPED'
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
