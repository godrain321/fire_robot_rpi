"""Compact, event-driven console output for the field driving demo."""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String


MODE_TITLES = {
    1: 'KEYBOARD',
    2: 'WAYPOINT AUTONOMOUS',
}
FILTERED_PRESENCE_TOPIC = '/mmwave/filtered_presence'
FILTERED_DISTANCE_TOPIC = '/mmwave/filtered_distance_m'
DYNAMIC_OBSTACLE_TOPIC = '/dynamic_obstacle_detected'


def waypoint_log_text(state: str) -> Optional[str]:
    """Format only operator-relevant waypoint progress events."""

    raw = state.strip().upper()
    if raw.startswith("RESTORED:"):
        try:
            return f"[웨이포인트] {int(raw.split(chr(58), 1)[1])}개 준비됨"
        except (TypeError, ValueError):
            return None
    if raw.startswith("RUNNING:"):
        try:
            current, total = (
                int(value) for value in raw.split(chr(58), 1)[1].split("/", 1)
            )
        except (TypeError, ValueError):
            return None
        if current <= 1:
            return f"[웨이포인트] 출발 → 1 주행 중 (1/{total})"
        return (
            f"[웨이포인트] {current - 1} → {current} 주행 중 "
            f"({current}/{total})"
        )
    if raw in ("MISSION_COMPLETE", "STEP_MISSION_COMPLETE"):
        return "[웨이포인트] 전체 주행 완료"
    return None


class StatusConsole(Node):
    """Show operator-relevant changes without repeating steady-state messages."""

    def __init__(self) -> None:
        super().__init__('mmwave_status_console')
        self.declare_parameter('distance_log_interval_sec', 1.0)
        self.declare_parameter('distance_log_delta_m', 0.10)
        self.declare_parameter('detection_distance_wait_sec', 0.15)
        self.distance_log_interval = float(
            self.get_parameter('distance_log_interval_sec').value
        )
        self.distance_log_delta = float(
            self.get_parameter('distance_log_delta_m').value
        )
        self.detection_distance_wait = float(
            self.get_parameter('detection_distance_wait_sec').value
        )
        if self.distance_log_interval <= 0.0:
            raise ValueError('distance_log_interval_sec must be positive')
        if self.distance_log_delta < 0.0:
            raise ValueError('distance_log_delta_m must not be negative')
        if self.detection_distance_wait <= 0.0:
            raise ValueError('detection_distance_wait_sec must be positive')

        self._mode: Optional[int] = 1
        self._unknown_mode_state: Optional[str] = None
        self._waypoint_state: Optional[str] = None
        self._follower_state: Optional[str] = None
        self._sensor_state: Optional[str] = None
        self._presence: Optional[bool] = None
        self._distance_m: Optional[float] = None
        self._last_distance_log_m: Optional[float] = None
        self._last_distance_log_time = 0.0
        self._pending_detection = False
        self._dynamic_detected: Optional[bool] = None

        self.create_subscription(
            String, '/drive_mode_status', self._on_drive_mode, 10
        )
        waypoint_qos = QoSProfile(depth=1)
        waypoint_qos.reliability = ReliabilityPolicy.RELIABLE
        waypoint_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, '/waypoint_queue_status', self._on_waypoint_state,
            waypoint_qos,
        )
        self.create_subscription(
            String, '/follower_state', self._on_follower_state, 10
        )
        self.create_subscription(
            Bool, FILTERED_PRESENCE_TOPIC, self._on_presence, 10
        )
        self.create_subscription(
            Float32, FILTERED_DISTANCE_TOPIC, self._on_distance, 10
        )
        self.create_subscription(
            String, '/mmwave/sensor_state', self._on_sensor_state, 10
        )
        self.create_subscription(
            Bool, DYNAMIC_OBSTACLE_TOPIC, self._on_dynamic_obstacle, 10
        )
        self._detection_timer = self.create_timer(
            self.detection_distance_wait, self._flush_pending_detection
        )
        self._detection_timer.cancel()

        self._print_mode(1)

    @staticmethod
    def _write(text: str) -> None:
        print(text, flush=True)

    def _print_mode(self, mode: int) -> None:
        title = MODE_TITLES[mode]
        self._write(f'\n{"═" * 12} MODE {mode} | {title} {"═" * 12}')

    def _on_drive_mode(self, message: String) -> None:
        raw = message.data.strip()
        try:
            mode = int(raw.split(':', 1)[0])
        except (TypeError, ValueError):
            if raw != self._unknown_mode_state:
                self._unknown_mode_state = raw
                self._write(f'[DRIVE MODE] {raw or "UNKNOWN"}')
            return
        if mode not in MODE_TITLES or mode == self._mode:
            return
        self._mode = mode
        self._print_mode(mode)
        if mode == 2:
            text = waypoint_log_text(self._waypoint_state or "")
            if text:
                self._write(text)
            if self._dynamic_detected:
                self._write("[동적장애물] 감지됨 - 회피 경로 계산")
            self._write("[조작] g=9개 웨이포인트 연속 주행")

    def _on_waypoint_state(self, message: String) -> None:
        state = message.data.strip()
        if not state or state == self._waypoint_state:
            return
        self._waypoint_state = state
        if self._mode != 2:
            return
        text = waypoint_log_text(state)
        if text:
            self._write(text)

    def _on_follower_state(self, message: String) -> None:
        state = message.data.strip()
        if not state or state == self._follower_state:
            return
        self._follower_state = state
        warnings = {
            "EMERGENCY_STOP": "전방 안전 정지",
            "NO_PATH": "회피 가능한 경로 없음",
        }
        if self._mode == 2 and state in warnings:
            self._write(f"[주행 경고] {warnings[state]}")

    def _on_dynamic_obstacle(self, message: Bool) -> None:
        detected = bool(message.data)
        previous = self._dynamic_detected
        self._dynamic_detected = detected
        if self._mode != 2 or detected == previous:
            return
        if detected:
            self._write("[동적장애물] 감지됨 - 회피 경로 계산")
        elif previous:
            self._write("[동적장애물] 감지 해제 - 웨이포인트 경로 복귀")

    def _on_sensor_state(self, message: String) -> None:
        state = message.data.strip()
        if not state or state == self._sensor_state:
            return
        self._sensor_state = state
        if state.upper() not in ("ONLINE", "CONNECTING"):
            self._write(f"[센서 경고] MMWAVE {state}")

    @staticmethod
    def _valid_distance(value: float) -> bool:
        return math.isfinite(value) and value > 0.0

    def _detection_text(self) -> str:
        if self._distance_m is None:
            return 'DETECT'
        return f'DETECT, {self._distance_m:.1f}m'

    def _record_distance_log(self, now: float) -> None:
        self._last_distance_log_time = now
        self._last_distance_log_m = self._distance_m

    def _emit_detection(self) -> None:
        self._write(self._detection_text())
        self._record_distance_log(time.monotonic())
        self._pending_detection = False
        self._detection_timer.cancel()

    def _flush_pending_detection(self) -> None:
        if self._pending_detection and self._presence:
            self._emit_detection()
        else:
            self._detection_timer.cancel()

    def _on_presence(self, message: Bool) -> None:
        present = bool(message.data)
        previous = self._presence
        self._presence = present
        if present == previous:
            return
        if present:
            self._pending_detection = True
            if self._distance_m is not None:
                self._emit_detection()
            else:
                self._detection_timer.reset()
            return
        if self._pending_detection:
            self._emit_detection()
        self._distance_m = None
        self._last_distance_log_m = None
        self._last_distance_log_time = 0.0
        if previous:
            self._write('CLEAR')

    def _on_distance(self, message: Float32) -> None:
        measured = float(message.data)
        if not self._valid_distance(measured):
            return
        self._distance_m = measured
        if not self._presence:
            return

        if self._pending_detection:
            self._emit_detection()
        # Presence transitions are logged once; raw distance jitter stays quiet.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = StatusConsole()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f'mmwave_status_console error: {exc}', flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
