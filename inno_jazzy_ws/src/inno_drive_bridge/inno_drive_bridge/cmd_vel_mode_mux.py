"""Select exactly one keyboard or autonomous command source for the ESP32."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String

from .named_waypoint_input import command_source_for_drive_mode


def command_source_for_modes(drive_mode, operator_mode):
    """Select the isolated greeting source only for public operator Mode 3."""
    if int(operator_mode) == 3 and int(drive_mode) == 3:
        return 3
    return command_source_for_drive_mode(drive_mode)


class CmdVelModeMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mode_mux')
        self.declare_parameter('input_timeout_sec', 0.35)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('output_topic', '/cmd_vel')
        self.timeout = float(self.get_parameter('input_timeout_sec').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if self.timeout <= 0.0 or rate <= 0.0:
            raise ValueError('timeout and publish rate must be positive')
        self.mode = 1
        self.operator_mode = 1
        # Source 2 is navigation.  Source 3 is intentionally separate because
        # the inactive path follower continuously publishes zero on source 2.
        # Sharing it with the greeting spin made the turn alternate RUN/STOP.
        self.commands = {1: Twist(), 2: Twist(), 3: Twist()}
        self.received = {1: 0.0, 2: 0.0, 3: 0.0}
        output_topic = str(self.get_parameter('output_topic').value).strip()
        if not output_topic:
            raise ValueError('output_topic must not be empty')
        self.output = self.create_publisher(Twist, output_topic, 10)
        self.status = self.create_publisher(String, '/drive_mode_status', 10)
        self.create_subscription(
            Twist,
            '/cmd_vel_keyboard',
            lambda message: self._cmd(1, message),
            10,
        )
        self.create_subscription(
            Twist,
            '/cmd_vel_auto',
            lambda message: self._cmd(2, message),
            10,
        )
        self.create_subscription(
            Twist,
            '/cmd_vel_mode3_demo',
            lambda message: self._cmd(3, message),
            10,
        )
        self.create_subscription(Int32, '/drive_mode', self._mode, 10)
        operator_qos = QoSProfile(depth=1)
        operator_qos.reliability = ReliabilityPolicy.RELIABLE
        operator_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Int32, '/operator_mode', self._operator_mode, operator_qos
        )
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info('Drive mode 1 (keyboard) selected')

    def _cmd(self, source, message):
        self.commands[source] = message
        self.received[source] = time.monotonic()

    def _mode(self, message):
        try:
            command_source_for_drive_mode(message.data)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self.output.publish(Twist())
        self.mode = int(message.data)
        labels = {
            1: 'KEYBOARD',
            2: 'NAMED_WAYPOINT_STEP',
            3: 'MMWAVE_OBSTACLE_INSPECTION',
            4: 'CAMERA_LIDAR_SURVIVOR_INSPECTION',
            5: 'EVACUATION_DEMO',
        }
        label = labels[self.mode]
        self.status.publish(String(data=f'{self.mode}:{label}'))
        self.get_logger().info(f'Drive mode {self.mode} ({label}) selected')

    def _operator_mode(self, message):
        mode = int(message.data)
        if mode not in (1, 2, 3):
            self.get_logger().warning('operator mode must be 1, 2, or 3')
            return
        if mode != self.operator_mode:
            # Do not let a command cached from the previous public mode leak
            # through while drive-mode and operator-mode messages converge.
            self.output.publish(Twist())
        self.operator_mode = mode

    def _publish(self):
        source = command_source_for_modes(self.mode, self.operator_mode)
        command = self.commands[source]
        if time.monotonic() - self.received[source] > self.timeout:
            command = Twist()
        self.output.publish(command)

    def destroy_node(self):
        if rclpy.ok():
            self.output.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelModeMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
