"""Three-mode operator keyboard for manual, integrated, and greeting runs."""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Int32

from .named_waypoint_input import command_source_for_drive_mode


# Mode 2 reuses the field-tested evacuation state machine's internal drive
# source (5). /operator_mode remains the public 1/2/3 interface shown to users.
OPERATOR_TO_INTERNAL_DRIVE_MODE = {1: 1, 2: 5, 3: 3}


def mode_selection_for_key(key):
    """Return ``(operator_mode, internal_drive_mode)`` for keys 1..3."""
    try:
        operator_mode = int(str(key))
        return operator_mode, OPERATOR_TO_INTERNAL_DRIVE_MODE[operator_mode]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('operator mode key must be 1, 2, or 3') from error


class KeyboardCmdVelDemo(Node):
    def __init__(self):
        super().__init__('keyboard_cmdvel_demo')
        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_keyboard')
        self.declare_parameter('mode11_blackout_key', False)
        # 0 permits P in dedicated Mode 11 profiles; the integrated
        # profile sets 2 so accidental P presses in Modes 1/3 are ignored.
        self.declare_parameter('blackout_operator_mode', 0)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')

        self._owns_input_stream = False
        self._owns_terminal_output = False
        if sys.stdin.isatty():
            self._input_stream = sys.stdin
        else:
            try:
                self._input_stream = open('/dev/tty', encoding='utf-8')
                self._owns_input_stream = True
            except OSError as error:
                raise RuntimeError(
                    'keyboard input requires an interactive terminal (TTY)'
                ) from error
        try:
            self._terminal_output = open(
                '/dev/tty', 'w', encoding='utf-8', buffering=1
            )
            self._owns_terminal_output = True
        except OSError:
            self._terminal_output = sys.stdout

        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10
        )
        self.mode_publisher = self.create_publisher(Int32, '/drive_mode', 10)
        operator_qos = QoSProfile(depth=1)
        operator_qos.reliability = ReliabilityPolicy.RELIABLE
        operator_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.operator_mode_publisher = self.create_publisher(
            Int32, '/operator_mode', operator_qos
        )
        self.autonomy_cancel_publisher = self.create_publisher(
            Empty, '/autonomy_cancel', 10
        )
        self.mode3_demo_request_publisher = self.create_publisher(
            Empty, '/mode3/demo_request', 10
        )
        self.mode11_blackout_key = bool(
            self.get_parameter('mode11_blackout_key').value
        )
        self.blackout_operator_mode = int(
            self.get_parameter('blackout_operator_mode').value
        )
        if self.blackout_operator_mode not in (0, 1, 2, 3):
            raise ValueError('blackout_operator_mode must be 0, 1, 2, or 3')
        self.blackout_publisher = (
            self.create_publisher(Empty, '/mode11/blackout_request', 10)
            if self.mode11_blackout_key else None
        )

        self.operator_mode = 1
        self.drive_mode = 1
        self.create_subscription(
            Int32, '/drive_mode', self._external_drive_mode, 10
        )
        self.command = Twist()
        self._terminal_settings = termios.tcgetattr(self._input_stream)
        tty.setcbreak(self._input_stream.fileno())

        self.create_timer(1.0 / publish_rate, self._publish_command)
        self.create_timer(0.02, self._poll_keyboard)
        self.add_on_set_parameters_callback(self._set_speed_parameters)
        self.operator_mode_publisher.publish(Int32(data=1))
        self.get_logger().info(
            'Keyboard ready: 1=keyboard drive, 2=integrated fire evacuation, '
            '3=greeting spin, w/x/a/d/s (Mode 1), P=LiDAR blackout, '
            'c=cancel, q=quit'
        )
        if self.mode11_blackout_key:
            self.get_logger().info(
                '[MODE 2] P=RF2O localization blackout; switch to Encoder+IMU'
            )

    def _external_drive_mode(self, message):
        """Track internal mission transitions without changing operator mode."""
        try:
            command_source_for_drive_mode(message.data)
        except ValueError:
            return
        self.drive_mode = int(message.data)
        self.command = Twist()

    def _set_speed_parameters(self, parameters):
        linear_speed = self.linear_speed
        angular_speed = self.angular_speed
        changed = False
        for parameter in parameters:
            if parameter.name == 'linear_speed':
                linear_speed = float(parameter.value)
                changed = True
            elif parameter.name == 'angular_speed':
                angular_speed = float(parameter.value)
                changed = True
        if linear_speed <= 0.0 or angular_speed <= 0.0:
            return SetParametersResult(
                successful=False,
                reason='linear_speed and angular_speed must be positive',
            )
        if changed:
            self.linear_speed = linear_speed
            self.angular_speed = angular_speed
            self.get_logger().info(
                f'Manual speed updated: linear={linear_speed:.3f} m/s, '
                f'angular={angular_speed:.3f} rad/s'
            )
        return SetParametersResult(successful=True)

    def _select_operator_mode(self, operator_mode):
        internal_mode = OPERATOR_TO_INTERNAL_DRIVE_MODE[operator_mode]
        self.autonomy_cancel_publisher.publish(Empty())
        # Send an explicit zero before every source change.
        self.command = Twist()
        self.publisher.publish(self.command)
        self.operator_mode = operator_mode
        self.operator_mode_publisher.publish(Int32(data=operator_mode))
        self.drive_mode = internal_mode
        self.mode_publisher.publish(Int32(data=internal_mode))

        if operator_mode == 1:
            self.get_logger().info('[MODE 1] 키보드 수동주행')
        elif operator_mode == 2:
            self.get_logger().warning(
                '[MODE 2] 통합 화재대피 주행 시작 요청: '
                '센서 준비 확인 후 출구 탐색을 시작합니다.'
            )
        else:
            self.mode3_demo_request_publisher.publish(Empty())
            self.get_logger().warning(
                '[MODE 3] 안내 음성과 제자리 1회전 동시 시작'
            )

    def _poll_keyboard(self):
        readable, _, _ = select.select([self._input_stream], [], [], 0.0)
        if not readable:
            return
        key = self._input_stream.read(1).lower()

        if key == 'p' and self.mode11_blackout_key:
            required_mode = getattr(self, 'blackout_operator_mode', 0)
            current_mode = getattr(self, 'operator_mode', 1)
            if required_mode and current_mode != required_mode:
                self.get_logger().warning(
                    f'P blackout은 Mode {required_mode}에서만 사용할 수 있습니다.'
                )
                return
            self.blackout_publisher.publish(Empty())
            self.get_logger().info(
                '[MODE 2] P pressed: localization blackout requested'
            )
            return

        if key in ('1', '2', '3'):
            operator_mode, _ = mode_selection_for_key(key)
            self._select_operator_mode(operator_mode)
            return
        if key in ('4', '5'):
            self.get_logger().warning(
                '운영 모드는 1=키보드, 2=통합 화재대피, 3=안내 회전입니다.'
            )
            return
        if key == ' ':
            self.get_logger().info(
                '현재 운영 모드는 Space 입력을 사용하지 않습니다.'
            )
            return
        if key in ('c', 's') and self.operator_mode != 1:
            cancelled_mode = self.operator_mode
            self._stop_all_motion()
            self.get_logger().warning(
                f'[MODE {cancelled_mode}] 취소: 모터 정지 후 Mode 1 복귀'
            )
            return
        if getattr(self, 'operator_mode', 1) != 1 and key in ('w', 'x', 'a', 'd'):
            self.get_logger().warning('수동 주행은 먼저 1을 누르세요.')
            return

        command = Twist()
        label = None
        if key == 'w':
            command.linear.x = self.linear_speed
            label = 'FORWARD'
        elif key == 'x':
            command.linear.x = -self.linear_speed
            label = 'REVERSE'
        elif key == 'a':
            command.angular.z = self.angular_speed
            label = 'TURN LEFT'
        elif key == 'd':
            command.angular.z = -self.angular_speed
            label = 'TURN RIGHT'
        elif key == 's':
            label = 'STOP'
        elif key == 'q':
            self._stop_all_motion()
            self.get_logger().info('STOP, then quit')
            self.restore_terminal()
            raise KeyboardInterrupt
        else:
            return

        self.command = command
        self._publish_command()
        self.get_logger().info(
            f'{label}: linear.x={command.linear.x:.3f} m/s, '
            f'angular.z={command.angular.z:.3f} rad/s'
        )

    def _publish_command(self):
        if self.operator_mode == 1 and self.drive_mode == 1:
            self.publisher.publish(self.command)

    def _stop_all_motion(self):
        """Cancel autonomy and select the manual zero-velocity source."""
        self.autonomy_cancel_publisher.publish(Empty())
        self.command = Twist()
        self.publisher.publish(self.command)
        self.operator_mode = 1
        self.operator_mode_publisher.publish(Int32(data=1))
        self.drive_mode = 1
        self.mode_publisher.publish(Int32(data=1))

    def restore_terminal(self):
        if self._terminal_settings is not None:
            termios.tcsetattr(
                self._input_stream, termios.TCSADRAIN, self._terminal_settings
            )
            self._terminal_settings = None
        if self._owns_input_stream:
            self._input_stream.close()
            self._owns_input_stream = False
        if self._owns_terminal_output:
            self._terminal_output.close()
            self._owns_terminal_output = False

    def destroy_node(self):
        try:
            if rclpy.ok():
                self._stop_all_motion()
        finally:
            self.restore_terminal()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardCmdVelDemo()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'keyboard_cmdvel_demo: {error}', file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
