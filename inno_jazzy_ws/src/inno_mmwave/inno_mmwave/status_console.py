"""Quiet, Korean-only operator console for the integrated field launch."""

import math
import time
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import Bool, Float32, Int64MultiArray, String
from tf2_ros import Buffer, TransformListener


MODE_TITLES = {
    1: '수동주행',
    2: '웨이포인트 주행',
    3: 'mmWave 사람 판별',
    4: '카메라 요구조자 판별',
    5: '자동 화재 대피',
}
FILTERED_PRESENCE_TOPIC = '/mmwave/human_presence'
FILTERED_DISTANCE_TOPIC = '/mmwave/calibrated_distance_m'
OPERATOR_PREFIX = '[ROBOT] '


def waypoint_log_text(state: str) -> Optional[str]:
    """Translate waypoint state codes into concise operator messages."""
    raw = state.strip()
    upper = raw.upper()
    if upper.startswith('RESTORED:'):
        try:
            return f'[웨이포인트] {int(raw.split(":", 1)[1])}개 준비됨'
        except (TypeError, ValueError):
            return None
    if upper.startswith('MODE2_ACCEPTED:'):
        route = raw.split(':', 1)[1].lower().replace('->', ' → ')
        return f'[경로] {route}'
    if upper.startswith('MODE2_RUNNING:'):
        try:
            progress, name = raw.split(':', 1)[1].rsplit(':', 1)
            current, total = progress.split('/', 1)
            return f'[주행] {name.lower()}로 이동 중 ({current}/{total})'
        except (TypeError, ValueError):
            return None
    if upper.startswith('MODE2_REACHED:') and ':SPACE_FOR:' in upper:
        try:
            reached, following = raw.split(':', 1)[1].split(
                ':SPACE_FOR:', 1
            )
            return (
                f'[도착] {reached.lower()} 도착 — '
                f'Space를 누르면 {following.lower()} 출발'
            )
        except (TypeError, ValueError):
            return None
    if upper.startswith('MODE2_MISSION_COMPLETE'):
        return '[완료] 선택한 웨이포인트 주행 완료'
    if upper == 'MODE2_CANCELLED':
        return '[취소] 모드 2 주행을 취소하고 모드 1로 복귀합니다.'
    if upper.startswith('MODE2_BUSY:'):
        return '[거절] 주행 중에는 다음 지점으로 넘길 수 없습니다.'
    if upper.startswith('MODE2_REJECTED:NO_SELECTION'):
        return '[거절] 먼저 2를 누르고 웨이포인트를 입력하세요.'
    if upper.startswith('MODE2_REJECTED:'):
        reason = raw.split(':', 2)[-1]
        return f'[오류] 웨이포인트 입력을 확인하세요: {reason}'
    return None


def mode3_log_text(state: str) -> Optional[str]:
    """Translate Mode 3 state codes."""
    raw = state.strip()
    upper = raw.upper()
    if upper == 'MODE3_READY:PRESS_SPACE':
        return '[준비] Space를 누르면 가장 가까운 빨간 장애물을 검사합니다.'
    if upper == 'MODE3_WAITING_FOR_DYNAMIC_OBSTACLE':
        return '[대기] 검사할 빨간 장애물을 기다립니다.'
    if upper.startswith('MODE3_APPROACHING:'):
        try:
            payload = raw.split(':', 1)[1]
            point, standoff = payload.split(':STANDOFF:', 1)
            distance = standoff.upper().removesuffix('M')
            return f'[접근] 장애물 ({point}) {distance}m 앞 검사 지점으로 이동 중'
        except ValueError:
            return '[접근] 장애물 검사 지점으로 이동 중'
    if upper == 'MODE3_AT_STANDOFF:ROBOT_SETTLING':
        return '[도착] 검사 지점 도착 — 로봇 정지 확인 중'
    if upper == 'MODE3_MMWAVE_OBSERVING':
        return '[판별] mmWave 사람 판별 시작'
    if upper.startswith('MODE3_PERSON_CONFIRMED'):
        return '[결과] 사람 감지! — 해당 점을 파란색으로 변경'
    if upper.startswith('MODE3_DYNAMIC_OBSTACLE_CONFIRMED'):
        return '[결과] 사람이 아닌 동적장애물 — 빨간색 유지'
    if upper.startswith('MODE3_SENSOR_UNAVAILABLE'):
        return (
            '[판정보류] mmWave 데이터 부족 또는 센서 연결 끊김 '
            '— 빨간색 유지'
        )
    if upper == 'MODE3_NO_PATH_TO_STANDOFF':
        return '[경고] 장애물 검사 지점까지 이동 가능한 경로가 없습니다.'
    if upper.startswith('MODE3_BUSY:'):
        return '[거절] 현재 장애물 검사가 진행 중입니다.'
    if upper == 'MODE3_CANCELLED':
        return '[취소] 모드 3 검사를 취소했습니다.'
    return None


def mode4_log_text(state: str) -> Optional[str]:
    """Translate Mode 4 state codes."""
    raw = state.strip()
    upper = raw.upper()
    if upper == 'MODE4_READY:PRESS_SPACE':
        return '[준비] Space를 누르면 가장 가까운 빨간 장애물을 검사합니다.'
    if upper == 'MODE4_WAITING_FOR_DYNAMIC_OBSTACLE':
        return '[대기] 검사할 빨간 장애물을 기다립니다.'
    if upper.startswith('MODE4_APPROACHING:'):
        try:
            payload = raw.split(':', 1)[1]
            point, standoff = payload.split(':STANDOFF:', 1)
            distance = standoff.upper().removesuffix('M')
            return f'[접근] 장애물 ({point}) {distance}m 앞 검사 지점으로 이동 중'
        except ValueError:
            return '[접근] 장애물 검사 지점으로 이동 중'
    if upper == 'MODE4_AT_STANDOFF:ROBOT_SETTLING':
        return '[도착] 검사 지점 도착 — 로봇 정지 확인 중'
    if upper == 'MODE4_CAMERA_YOLO_OBSERVING':
        return '[판별] 카메라와 LiDAR 요구조자 판별 시작'
    if upper.startswith('MODE4_SURVIVOR_CONFIRMED'):
        return '[결과] 요구조자 감지! — 해당 점을 파란색으로 변경'
    if upper.startswith('MODE4_NO_SURVIVOR'):
        return '[결과] 요구조자 미감지 — 빨간색 유지'
    if upper.startswith('MODE4_DETECTOR_UNAVAILABLE'):
        return (
            '[판정보류] 카메라 영상 또는 인식 데이터 없음 — 빨간색 유지'
        )
    if upper == 'MODE4_NO_PATH_TO_STANDOFF':
        return '[경고] 장애물 검사 지점까지 이동 가능한 경로가 없습니다.'
    if upper.startswith('MODE4_BUSY:'):
        return '[거절] 현재 장애물 검사가 진행 중입니다.'
    if upper == 'MODE4_CANCELLED':
        return '[취소] 모드 4 검사를 취소했습니다.'
    return None


class StatusConsole(Node):
    """Print only state changes that matter to the robot operator."""

    def __init__(self) -> None:
        super().__init__('operator_status_console')
        defaults = {
            'use_serial': True,
            'use_lidar': True,
            'use_mmwave': True,
            'use_camera': False,
            'use_thermal': False,
            'mode5_enabled': False,
            'esp32_port': '/dev/ttyUSB0',
            'lidar_port': '/dev/ttyUSB1',
            'mmwave_port': '/dev/ttyAMA0',
            'startup_timeout_sec': 10.0,
            'stream_stale_sec': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def value(name):
            return self.get_parameter(name).value

        self.enabled = {
            'esp32': bool(value('use_serial')),
            'lidar': bool(value('use_lidar')),
            'mmwave': bool(value('use_mmwave')),
            'camera': bool(value('use_camera')),
            'yolo': bool(value('use_camera')),
            'thermal': bool(value('use_thermal')),
            'map_tf': True,
        }
        self.mode5_enabled = bool(value('mode5_enabled'))
        self.ports = {
            'esp32': str(value('esp32_port')),
            'lidar': str(value('lidar_port')),
            'mmwave': str(value('mmwave_port')),
        }
        self.startup_timeout = float(value('startup_timeout_sec'))
        self.stale_timeout = float(value('stream_stale_sec'))
        if self.startup_timeout <= 0.0 or self.stale_timeout <= 0.0:
            raise ValueError('상태 확인 시간은 0보다 커야 합니다.')

        self.started_at = time.monotonic()
        self.health = {name: None for name in self.enabled}
        self.last_seen = {
            name: None for name in ('lidar', 'camera', 'thermal')
        }
        self._ready_printed = False
        self._mode: Optional[int] = None
        self._waypoint_state: Optional[str] = None
        self._follower_state: Optional[str] = None
        self._sensor_state: Optional[str] = None
        self._mode3_state: Optional[str] = None
        self._mode4_state: Optional[str] = None
        self._mode5_log: Optional[str] = None
        self._manual_command = None
        self._presence: Optional[bool] = None
        self._distance_m: Optional[float] = None
        self._dynamic_detected: Optional[bool] = None
        self._map_received = False

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, '/drive_mode_status', self._on_drive_mode, 10
        )
        self.create_subscription(
            Twist, '/cmd_vel_keyboard', self._on_manual_command, 10
        )
        self.create_subscription(
            String, '/waypoint_queue_status', self._on_waypoint_state,
            transient,
        )
        self.create_subscription(
            String, '/follower_state', self._on_follower_state, 10
        )
        self.create_subscription(
            Bool, '/dynamic_obstacle_detected', self._on_dynamic_obstacle, 10
        )
        self.create_subscription(
            String, '/mode3_status', self._on_mode3_state, transient
        )
        self.create_subscription(
            String, '/mode4_status', self._on_mode4_state, transient
        )
        self.create_subscription(
            String, '/evacuation_demo/log', self._on_mode5_log, transient
        )
        self.create_subscription(
            String, '/mmwave/sensor_state', self._on_sensor_state, transient
        )
        self.create_subscription(
            Bool, FILTERED_PRESENCE_TOPIC, self._on_presence, 10
        )
        self.create_subscription(
            Float32, FILTERED_DISTANCE_TOPIC, self._on_distance, 10
        )
        self.create_subscription(
            String, '/esp32/status', self._on_esp32_status, 10
        )
        self.create_subscription(
            Int64MultiArray, '/wheel_ticks', self._on_wheel_ticks, 10
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(
            Image, '/camera/image_raw', self._on_image, 10
        )
        self.create_subscription(
            String, '/camera/person_detector_status',
            self._on_detector_status, transient,
        )
        self.create_subscription(
            PointCloud2, '/thermal/arc_points', self._on_thermal, 10
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, transient)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.5, self._health_timer)

        self._write('========== 장치 연결 상태 ==========')
        for name, label in (
            ('esp32', 'ESP32'),
            ('lidar', 'LiDAR'),
            ('mmwave', 'mmWave'),
            ('camera', 'Camera'),
            ('yolo', 'YOLO'),
            ('thermal', '열화상 MLX90640'),
        ):
            if not self.enabled[name]:
                self.health[name] = True
                self._write(f'[사용 안 함] {label}')
        self._print_mode(1)

    @staticmethod
    def _write(text: str) -> None:
        print(OPERATOR_PREFIX + text, flush=True)

    def _set_health(self, name: str, healthy: bool, message: str) -> None:
        if not self.enabled[name] or self.health[name] == healthy:
            return
        self.health[name] = healthy
        self._write(message)
        if not healthy:
            self._ready_printed = False

    def _print_mode(self, mode: int) -> None:
        title = f'========== MODE {mode} | {MODE_TITLES[mode]} =========='
        self._write('')
        self._write(title)
        if mode == 1:
            self._write('[조작] W=전진 X=후진 A=좌회전 D=우회전 S=정지')
        elif mode == 2:
            self._write(
                '[입력] 이동할 웨이포인트를 쉼표로 구분해 입력하세요.'
            )
        elif mode == 3:
            self._write('[검사] 실제 정지거리 2.0m에서 mmWave로 판별합니다.')
        elif mode == 4:
            distance = '2.0m' if self.mode5_enabled else '1.5m'
            self._write(
                f'[검사] 실제 정지거리 {distance}에서 '
                '카메라와 LiDAR로 판별합니다.'
            )

    def _on_drive_mode(self, message: String) -> None:
        try:
            mode = int(message.data.strip().split(':', 1)[0])
        except (TypeError, ValueError):
            return
        if mode not in MODE_TITLES or mode == self._mode:
            return
        self._mode = mode
        self._manual_command = None
        self._print_mode(mode)

    def _on_manual_command(self, message: Twist) -> None:
        if self._mode not in (None, 1):
            return
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        signature = (round(linear, 4), round(angular, 4))
        if signature == self._manual_command:
            return
        self._manual_command = signature
        if abs(linear) < 1e-6 and abs(angular) < 1e-6:
            text = '[수동] 정지'
        elif linear > 0.0:
            text = f'[수동] 전진: {linear:.3f} m/s'
        elif linear < 0.0:
            text = f'[수동] 후진: {linear:.3f} m/s'
        elif angular > 0.0:
            text = f'[수동] 좌회전: {angular:.3f} rad/s'
        else:
            text = f'[수동] 우회전: {angular:.3f} rad/s'
        self._write(text)

    def _on_waypoint_state(self, message: String) -> None:
        state = message.data.strip()
        if not state or state == self._waypoint_state:
            return
        self._waypoint_state = state
        if self._mode == 2:
            text = waypoint_log_text(state)
            if text:
                self._write(text)

    def _on_follower_state(self, message: String) -> None:
        state = message.data.strip().upper()
        if not state or state == self._follower_state:
            return
        self._follower_state = state
        warnings = {
            'EMERGENCY_STOP': '[안전 정지] 전방 장애물이 너무 가깝습니다.',
            'NO_PATH': '[경고] 이동 가능한 경로가 없습니다.',
        }
        if self._mode in (2, 3, 4, 5) and state in warnings:
            self._write(warnings[state])

    def _on_dynamic_obstacle(self, message: Bool) -> None:
        detected = bool(message.data)
        previous = self._dynamic_detected
        self._dynamic_detected = detected
        if self._mode != 2 or detected == previous:
            return
        if detected:
            self._write('[동적장애물] 감지 — 회피 경로 계산')
        elif previous:
            self._write('[동적장애물] 해제 — 기존 경로로 복귀')

    def _on_mode3_state(self, message: String) -> None:
        state = message.data.strip()
        if not state or state == self._mode3_state:
            return
        self._mode3_state = state
        if self._mode == 3:
            text = mode3_log_text(state)
            if text:
                self._write(text)

    def _on_mode4_state(self, message: String) -> None:
        state = message.data.strip()
        if not state or state == self._mode4_state:
            return
        self._mode4_state = state
        if self._mode == 4:
            text = mode4_log_text(state)
            if text:
                self._write(text)

    def _on_mode5_log(self, message: String) -> None:
        text = message.data.strip()
        if not text or text == self._mode5_log:
            return
        self._mode5_log = text
        self._write(f'[모드 5] {text}')

    def _on_sensor_state(self, message: String) -> None:
        state = message.data.strip().upper()
        if not state or state == self._sensor_state:
            return
        self._sensor_state = state
        if state == 'ONLINE':
            self._set_health(
                'mmwave', True,
                f'[정상] mmWave C4001: {self.ports["mmwave"]} 데이터 수신',
            )
        elif state not in ('CONNECTING',):
            self._set_health(
                'mmwave', False,
                '[오류] mmWave 연결 끊김 또는 데이터 없음',
            )

    def _on_presence(self, message: Bool) -> None:
        present = bool(message.data)
        previous = self._presence
        self._presence = present
        if self._mode not in (3, 5) or present == previous:
            return
        if present:
            suffix = ''
            if self._distance_m is not None and self._distance_m > 0.0:
                suffix = f': {self._distance_m:.1f}m'
            self._write(f'[mmWave] 사람 후보 감지{suffix}')
        elif previous:
            self._write('[mmWave] 사람 후보 해제')

    def _on_distance(self, message: Float32) -> None:
        measured = float(message.data)
        if math.isfinite(measured) and measured > 0.0:
            self._distance_m = measured

    def _on_esp32_status(self, _message: String) -> None:
        self._set_health(
            'esp32', True,
            f'[정상] ESP32: {self.ports["esp32"]} 실제 데이터 통신 정상',
        )

    def _on_wheel_ticks(self, _message: Int64MultiArray) -> None:
        self._on_esp32_status(String())

    def _on_scan(self, _message: LaserScan) -> None:
        self.last_seen['lidar'] = time.monotonic()
        self._set_health(
            'lidar', True,
            f'[정상] LiDAR: {self.ports["lidar"]} /scan 수신',
        )

    def _on_image(self, message: Image) -> None:
        self.last_seen['camera'] = time.monotonic()
        self._set_health(
            'camera', True,
            f'[정상] Camera: 영상 수신 {message.width}x{message.height}',
        )

    def _on_detector_status(self, message: String) -> None:
        state = message.data.strip().upper()
        ready = state in (
            'ONLINE', 'READY_WAITING_FOR_MODE4', 'READY_WAITING_FOR_IMAGE'
        )
        if ready:
            self._set_health(
                'yolo', True, '[정상] YOLO: 모델 로드 및 추론 준비 완료'
            )
        elif state:
            self._set_health(
                'yolo', False,
                '[오류] YOLO 모델 또는 추론 준비 실패',
            )

    def _on_thermal(self, _message: PointCloud2) -> None:
        self.last_seen['thermal'] = time.monotonic()
        self._set_health(
            'thermal', True, '[정상] 열화상 MLX90640: 온도 데이터 수신'
        )

    def _on_map(self, _message: OccupancyGrid) -> None:
        self._map_received = True

    def _health_timer(self) -> None:
        now = time.monotonic()
        for name, label in (
            ('lidar', 'LiDAR /scan'),
            ('camera', 'Camera 영상'),
            ('thermal', '열화상 데이터'),
        ):
            seen = self.last_seen[name]
            if self.enabled[name] and seen is not None:
                if now - seen > self.stale_timeout:
                    self._set_health(
                        name, False, f'[오류] {label} 수신이 끊겼습니다.'
                    )

        tf_ready = self.tf_buffer.can_transform(
            'map', 'base_link', Time(), timeout=Duration(seconds=0.0)
        )
        if self._map_received and tf_ready:
            self._set_health(
                'map_tf', True,
                '[정상] 지도/위치추정: map → base_link 준비 완료',
            )
        elif self.health['map_tf'] is True:
            self._set_health(
                'map_tf', False,
                '[오류] 지도 또는 위치추정 연결이 끊겼습니다.',
            )

        if now - self.started_at >= self.startup_timeout:
            missing_messages = {
                'esp32': '[오류] ESP32 실제 데이터 통신을 확인할 수 없습니다.',
                'lidar': '[오류] LiDAR 데이터 없음: /scan 미수신',
                'mmwave': '[오류] mmWave 데이터 없음 또는 연결 실패',
                'camera': '[오류] Camera 영상 없음',
                'yolo': '[오류] YOLO 모델 또는 추론 준비 실패',
                'thermal': '[오류] 열화상 MLX90640 데이터 없음',
                'map_tf': '[오류] 지도 또는 위치추정 준비 안 됨',
            }
            for name, text in missing_messages.items():
                if self.enabled[name] and self.health[name] is None:
                    self._set_health(name, False, text)

        required_ready = all(
            self.health[name] is True
            for name, enabled in self.enabled.items()
            if enabled
        )
        if required_ready and not self._ready_printed:
            self._ready_printed = True
            self._write('[준비 완료] 모든 필수 장치가 정상입니다.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = StatusConsole()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as error:
        print(
            OPERATOR_PREFIX + f'[오류] 사용자 콘솔 시작 실패: {error}',
            flush=True,
        )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
