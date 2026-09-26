from geometry_msgs.msg import Twist

from inno_drive_bridge.mode10_avoidance_core import Mode10Avoidance
from inno_drive_bridge.mode10_ultrasonic_avoidance import (
    Mode10UltrasonicAvoidance,
)


def node_at(operator_mode, now=10.0):
    node = object.__new__(Mode10UltrasonicAvoidance)
    node.control = Mode10Avoidance()
    node.control.set_armed(True)
    node.operator_mode = operator_mode
    node.active_operator_modes = {2}
    node.input_timeout = 0.35
    node.upstream_received_at = now
    node.upstream_command = Twist()
    node.upstream_command.linear.x = 0.12
    return node


def test_mode1_bypasses_ultrasonic_filter_without_sensor_data():
    node = node_at(1)

    command, state = node._integrated_command(10.0)

    assert state == 'BYPASS'
    assert command.linear.x == 0.12


def test_mode2_holds_zero_until_ultrasonic_and_lidar_are_ready():
    node = node_at(2)

    command, state = node._integrated_command(10.0)

    assert state == 'WAIT_SENSOR'
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0


def test_mode2_passes_planner_velocity_when_front_is_clear():
    node = node_at(2)
    node.control.on_distance(1.0, 10.0)
    node.control.on_scan(1.0, 1.0, 1.0, 10.0)

    command, state = node._integrated_command(10.0)

    assert state == 'PASS'
    assert command.linear.x == 0.12
