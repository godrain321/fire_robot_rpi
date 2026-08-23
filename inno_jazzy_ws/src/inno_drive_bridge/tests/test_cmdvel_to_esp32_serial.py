import sys
import unittest
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1] / 'inno_drive_bridge'))

from cmdvel_to_esp32_serial import CmdVelToEsp32Serial  # noqa: E402


class DummyLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class DummyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class CmdVelToEsp32SerialParserTest(unittest.TestCase):
    def test_enc_abs_message_is_accepted_without_warning(self):
        node = CmdVelToEsp32Serial.__new__(CmdVelToEsp32Serial)
        node.logger = DummyLogger()
        node.ticks_publisher = DummyPublisher()
        node.status_publisher = DummyPublisher()
        node._publish_status = lambda text: None
        node.get_logger = lambda: node.logger

        node._parse_line('ENC_ABS,12345,10.0,20.0,0.5,1.0,0.02,0.04')

        self.assertEqual(node.logger.warnings, [])
        self.assertEqual(node.ticks_publisher.messages, [])

    def test_motor_targets_are_published_separately(self):
        node = CmdVelToEsp32Serial.__new__(CmdVelToEsp32Serial)
        node.left_motor_publisher = DummyPublisher()
        node.right_motor_publisher = DummyPublisher()

        node._publish_motor_targets(-123, 456)

        self.assertEqual(node.left_motor_publisher.messages[-1].data, -123)
        self.assertEqual(node.right_motor_publisher.messages[-1].data, 456)


if __name__ == '__main__':
    unittest.main()
