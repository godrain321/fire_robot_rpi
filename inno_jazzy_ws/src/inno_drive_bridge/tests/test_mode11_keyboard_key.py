"""P is opt-in and does not interrupt Mode 1 motor commands."""

from unittest.mock import Mock, patch

from geometry_msgs.msg import Twist

from inno_drive_bridge.keyboard_cmdvel_demo import KeyboardCmdVelDemo


def test_p_requests_blackout_without_stopping_current_keyboard_motion():
    node = object.__new__(KeyboardCmdVelDemo)
    node._input_stream = Mock()
    node._input_stream.read.return_value = 'P'
    node._waypoint_collecting = False
    node.mode11_blackout_key = True
    node.blackout_publisher = Mock()
    node.publisher = Mock()
    node.mode_publisher = Mock()
    node.autonomy_cancel_publisher = Mock()
    node.command = Twist()
    node.command.linear.x = 0.08
    node.get_logger = Mock(return_value=Mock())

    with patch('inno_drive_bridge.keyboard_cmdvel_demo.select.select',
               return_value=([node._input_stream], [], [])):
        node._poll_keyboard()

    node.blackout_publisher.publish.assert_called_once()
    node.publisher.publish.assert_not_called()
    node.mode_publisher.publish.assert_not_called()
    node.autonomy_cancel_publisher.publish.assert_not_called()
    assert node.command.linear.x == 0.08


def test_p_is_ignored_in_existing_mode_one_profile():
    node = object.__new__(KeyboardCmdVelDemo)
    node._input_stream = Mock()
    node._input_stream.read.return_value = 'p'
    node._waypoint_collecting = False
    node.mode11_blackout_key = False
    node.drive_mode = 1
    node.publisher = Mock()
    node.command = Twist()
    node.command.angular.z = 0.35

    with patch('inno_drive_bridge.keyboard_cmdvel_demo.select.select',
               return_value=([node._input_stream], [], [])):
        node._poll_keyboard()

    node.publisher.publish.assert_not_called()
    assert node.command.angular.z == 0.35


def _bare_keyboard_for_key(key):
    node = object.__new__(KeyboardCmdVelDemo)
    node._input_stream = Mock()
    node._input_stream.read.return_value = key
    node.mode11_blackout_key = True
    node.blackout_publisher = Mock()
    node.publisher = Mock()
    node.mode_publisher = Mock()
    node.operator_mode_publisher = Mock()
    node.autonomy_cancel_publisher = Mock()
    node.mode3_demo_request_publisher = Mock()
    node.command = Twist()
    node.operator_mode = 1
    node.drive_mode = 1
    node.get_logger = Mock(return_value=Mock())
    return node


def test_operator_mode_2_reuses_internal_integrated_drive_mode_5():
    node = _bare_keyboard_for_key('2')

    with patch('inno_drive_bridge.keyboard_cmdvel_demo.select.select',
               return_value=([node._input_stream], [], [])):
        node._poll_keyboard()

    assert node.operator_mode == 2
    assert node.drive_mode == 5
    assert node.operator_mode_publisher.publish.call_args.args[0].data == 2
    assert node.mode_publisher.publish.call_args.args[0].data == 5
    node.mode3_demo_request_publisher.publish.assert_not_called()


def test_operator_mode_3_selects_auto_source_and_requests_one_demo():
    node = _bare_keyboard_for_key('3')

    with patch('inno_drive_bridge.keyboard_cmdvel_demo.select.select',
               return_value=([node._input_stream], [], [])):
        node._poll_keyboard()

    assert node.operator_mode == 3
    assert node.drive_mode == 3
    assert node.operator_mode_publisher.publish.call_args.args[0].data == 3
    assert node.mode_publisher.publish.call_args.args[0].data == 3
    node.mode3_demo_request_publisher.publish.assert_called_once()


def test_integrated_blackout_key_is_ignored_outside_mode_two():
    node = _bare_keyboard_for_key('p')
    node.blackout_operator_mode = 2
    node.operator_mode = 1

    with patch('inno_drive_bridge.keyboard_cmdvel_demo.select.select',
               return_value=([node._input_stream], [], [])):
        node._poll_keyboard()

    node.blackout_publisher.publish.assert_not_called()
    node.get_logger.return_value.warning.assert_called_once()
