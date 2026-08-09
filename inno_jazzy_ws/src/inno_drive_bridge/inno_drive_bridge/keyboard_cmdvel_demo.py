import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Int32, String


def waypoint_command_on_mode_select(mode):
    """Start MODE 2 continuously; MODE 1/3 keep their existing controls."""
    if mode not in (1, 2, 3):
        raise ValueError("drive mode must be 1, 2, or 3")
    return "GO" if mode == 2 else None


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
        self.drive_mode = 1
        self.command = Twist()
        self._terminal_settings = termios.tcgetattr(self._input_stream)
        tty.setcbreak(self._input_stream.fileno())

        self.create_timer(1.0 / publish_rate, self._publish_command)
        self.create_timer(0.02, self._poll_keyboard)
        self.add_on_set_parameters_callback(self._set_speed_parameters)
        self.get_logger().info(
            'Keyboard ready: 1=manual, 2=waypoint auto-run, '
            '3=rescue waypoint, g=run all, SPACE=run next waypoint, c=clear, '
            'w/x/a/d/s, q=quit'
        )

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

    def _poll_keyboard(self):
        readable, _, _ = select.select([self._input_stream], [], [], 0.0)
        if not readable:
            return
        key = self._input_stream.read(1).lower()

        command = Twist()
        label = None
        if key in ('1', '2', '3'):
            self.drive_mode = int(key)
            self.command = command
            self._publish_command()
            self.mode_publisher.publish(Int32(data=self.drive_mode))
            labels = {
                1: 'MODE 1: KEYBOARD',
                2: 'MODE 2: WAYPOINT ONLY',
                3: 'MODE 3: RESCUE + DYNAMIC AVOIDANCE',
            }
            self.get_logger().info(labels[self.drive_mode])
            waypoint_command = waypoint_command_on_mode_select(self.drive_mode)
            if waypoint_command is not None:
                self.waypoint_command_publisher.publish(
                    String(data=waypoint_command)
                )
                self.get_logger().info(
                    "MODE 2: requested continuous waypoint driving "
                    "from first to last"
                )
            return
        if key == 'g':
            if self.drive_mode not in (2, 3):
                self.get_logger().warning(
                    'Press 2 or 3 before starting waypoint driving.'
                )
                return
            self.waypoint_command_publisher.publish(String(data='GO'))
            self.get_logger().info('Requested sequential waypoint driving')
            return
        if key == ' ':
            if self.drive_mode not in (2, 3):
                self.get_logger().warning(
                    'Press 2 or 3 before stepping waypoints.'
                )
                return
            self.waypoint_command_publisher.publish(String(data='STEP'))
            self.get_logger().info('Requested next waypoint only')
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
