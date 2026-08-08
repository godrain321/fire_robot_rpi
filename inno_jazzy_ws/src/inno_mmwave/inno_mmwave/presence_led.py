"""Latch ten Raspberry Pi GPIO LEDs when filtered mmWave presence is detected."""

from dataclasses import dataclass
import importlib
from typing import Optional, Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


PRESENCE_TOPIC = '/mmwave/filtered_presence'
LED_STATUS_TOPIC = '/mmwave/led_latched'
RESET_SERVICE = '/mmwave/reset_led'
DEFAULT_GPIO_LINES = (17, 27, 22, 23, 24, 25, 5, 6, 16, 26)
DEFAULT_PHYSICAL_PINS = (11, 13, 15, 16, 18, 22, 29, 31, 36, 37)


@dataclass
class PresenceLatch:
    """One-way software latch which only reset or shutdown can clear."""

    active: bool = False

    def observe(self, detected: bool) -> bool:
        previous = self.active
        if detected:
            self.active = True
        return self.active != previous

    def reset(self) -> bool:
        previous = self.active
        self.active = False
        return previous


class LgpioLedBank:
    """Own several gpiochip lines and always return all of them to OFF."""

    def __init__(
        self,
        chip_index: int,
        lines: Sequence[int],
        *,
        active_high: bool = True,
        lgpio_module=None,
    ) -> None:
        normalized = tuple(int(line) for line in lines)
        if (
            chip_index < 0
            or not normalized
            or any(line < 0 for line in normalized)
        ):
            raise ValueError('gpio chip and lines must be valid')
        if len(set(normalized)) != len(normalized):
            raise ValueError('gpio lines must not contain duplicates')
        self._lgpio = lgpio_module or importlib.import_module('lgpio')
        self.chip_index = int(chip_index)
        self.lines = normalized
        self.active_high = bool(active_high)
        self.handle: Optional[int] = None
        off_level = self._level(False)
        handle = self._lgpio.gpiochip_open(self.chip_index)
        claimed = []
        try:
            for line in self.lines:
                self._lgpio.gpio_claim_output(handle, line, off_level)
                claimed.append(line)
        except Exception:
            for line in reversed(claimed):
                try:
                    self._lgpio.gpio_free(handle, line)
                except Exception:
                    pass
            self._lgpio.gpiochip_close(handle)
            raise
        self.handle = handle

    def _level(self, enabled: bool) -> int:
        return int(bool(enabled) == self.active_high)

    def set_enabled(self, enabled: bool) -> None:
        if self.handle is None:
            raise RuntimeError('GPIO LED bank is already closed')
        level = self._level(bool(enabled))
        try:
            for line in self.lines:
                self._lgpio.gpio_write(self.handle, line, level)
        except Exception:
            off_level = self._level(False)
            for line in self.lines:
                try:
                    self._lgpio.gpio_write(self.handle, line, off_level)
                except Exception:
                    pass
            raise

    def close(self) -> None:
        if self.handle is None:
            return
        handle = self.handle
        self.handle = None
        off_level = self._level(False)
        for line in self.lines:
            try:
                self._lgpio.gpio_write(handle, line, off_level)
            except Exception:
                pass
        for line in reversed(self.lines):
            try:
                self._lgpio.gpio_free(handle, line)
            except Exception:
                pass
        self._lgpio.gpiochip_close(handle)


class PresenceLedNode(Node):
    """Drive ten latched LEDs from the conservative filtered-presence topic."""

    def __init__(self, gpio_factory=LgpioLedBank) -> None:
        super().__init__('mmwave_presence_led')
        self.declare_parameter('gpio_chip', 4)
        self.declare_parameter('gpio_lines', list(DEFAULT_GPIO_LINES))
        self.declare_parameter('active_high', True)
        chip = int(self.get_parameter('gpio_chip').value)
        lines = tuple(
            int(line) for line in self.get_parameter('gpio_lines').value
        )
        active_high = bool(self.get_parameter('active_high').value)

        self.latch = PresenceLatch()
        self.output = gpio_factory(chip, lines, active_high=active_high)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            Bool, LED_STATUS_TOPIC, qos
        )
        self.create_subscription(Bool, PRESENCE_TOPIC, self._presence, qos)
        self.create_service(Trigger, RESET_SERVICE, self._reset)
        self._publish_status()
        self.get_logger().info(
            f'10-LED bank ready: physical pins {DEFAULT_PHYSICAL_PINS}, '
            f'BCM GPIOs {lines}, gpiochip{chip}; waiting for presence'
        )

    def _publish_status(self) -> None:
        self.status_publisher.publish(Bool(data=self.latch.active))

    def _presence(self, message: Bool) -> None:
        if not self.latch.observe(bool(message.data)):
            return
        self.output.set_enabled(True)
        self._publish_status()
        self.get_logger().warning('PERSON DETECTED: ALL 10 LEDS LATCHED ON')

    def _reset(self, request, response):
        del request
        was_active = self.latch.reset()
        self.output.set_enabled(False)
        self._publish_status()
        response.success = True
        response.message = 'LED latch reset' if was_active else 'LEDs already off'
        self.get_logger().info(response.message)
        return response

    def destroy_node(self) -> None:
        self.latch.reset()
        self.output.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PresenceLedNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f'mmwave_presence_led error: {exc}', flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
