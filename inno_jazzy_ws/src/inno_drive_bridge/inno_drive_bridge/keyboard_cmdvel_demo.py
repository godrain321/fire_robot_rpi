import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32, String


class KeyboardCmdVelDemo(Node):
    def __init__(self):
        super().__init__('keyboard_cmdvel_demo')
        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_keyboard')

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')
        self._owns_input_stream = False
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

        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10
        )
        self.mode_publisher = self.create_publisher(Int32, '/drive_mode', 10)
        self.waypoint_command_publisher = self.create_publisher(
            String, '/waypoint_queue_command', 10
        )
        self.create_subscription(
            String, '/esp32/status', self._esp32_status_callback, 10
        )
        self.drive_mode = 1
        self.command = Twist()
        self._terminal_settings = termios.tcgetattr(self._input_stream)
        tty.setcbreak(self._input_stream.fileno())

        self.create_timer(1.0 / publish_rate, self._publish_command)
        self.create_timer(0.02, self._poll_keyboard)
        self.get_logger().info(
            'Keyboard ready: 1=manual, 2=waypoint select, g=run queue, '
            'c=clear queue, w/x/a/d/s, q=quit'
        )

    def _poll_keyboard(self):
        readable, _, _ = select.select([self._input_stream], [], [], 0.0)
        if not readable:
            return
        key = self._input_stream.read(1).lower()

        command = Twist()
        label = None
        if key in ('1', '2'):
            self.drive_mode = int(key)
            self.command = command
            self._publish_command()
            self.mode_publisher.publish(Int32(data=self.drive_mode))
            self.get_logger().info(
                'MODE 1: KEYBOARD' if self.drive_mode == 1
                else 'MODE 2: RViz / AUTONOMOUS'
            )
            return
        if key == 'g':
            if self.drive_mode != 2:
                self.get_logger().warning('Press 2 before starting waypoint driving.')
                return
            self.waypoint_command_publisher.publish(String(data='GO'))
            self.get_logger().info('Requested sequential waypoint driving')
            return
        if key == 'c':
            self.waypoint_command_publisher.publish(String(data='CLEAR'))
            self.get_logger().info('Requested waypoint queue clear')
            return
        if self.drive_mode != 1 and key in ('w', 'x', 'a', 'd'):
            self.get_logger().warning('Press 1 before using manual drive keys.')
            return
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
            self.command = command
            self._publish_command()
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

    def _esp32_status_callback(self, message):
        # Encoder values are shown only while mode 1 (manual keyboard) is active.
        if self.drive_mode != 1:
            return

        fields = message.data.split(',')
        if len(fields) < 5 or fields[0] != 'ENC_ABS':
            return

        try:
            encoder_angle_deg = float(fields[2])
            distance_mm = float(fields[4]) * 1000.0
        except ValueError:
            return

        # ros2 launch에서도 즉시 보이도록 매 측정값을 한 줄씩 출력한다.
        print(
            f'엔코더 각도: {encoder_angle_deg:8.2f}°, '
            f'이동거리: {distance_mm:8.2f} mm',
            flush=True,
        )

    def _publish_command(self):
        if self.drive_mode == 1:
            self.publisher.publish(self.command)

    def restore_terminal(self):
        if self._terminal_settings is not None:
            termios.tcsetattr(
                self._input_stream, termios.TCSADRAIN, self._terminal_settings
            )
            self._terminal_settings = None
        if self._owns_input_stream:
            self._input_stream.close()
            self._owns_input_stream = False

    def destroy_node(self):
        self.command = Twist()
        self._publish_command()
        self.restore_terminal()
        super().destroy_node()


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
