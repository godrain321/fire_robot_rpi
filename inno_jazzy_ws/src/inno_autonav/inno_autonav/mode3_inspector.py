"""Approach a LiDAR obstacle and classify mmWave presence in drive mode 3."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Iterable, Optional, Sequence, Tuple

from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float32, Int32, String

from .grid_utils import quaternion_from_yaw
from .tf_utils import TfHelper


Point2D = Tuple[float, float]


def select_nearest_candidate(
    robot_x: float, robot_y: float, candidates: Iterable[Point2D]
) -> Optional[Point2D]:
    """Return the nearest finite obstacle centroid, or ``None``."""
    valid = [
        (float(x), float(y))
        for x, y in candidates
        if math.isfinite(float(x)) and math.isfinite(float(y))
    ]
    if not valid:
        return None
    return min(
        valid,
        key=lambda point: math.hypot(point[0] - robot_x, point[1] - robot_y),
    )


def compute_inspection_goal(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    target_x: float,
    target_y: float,
    standoff_distance_m: float,
) -> Tuple[float, float, float]:
    """Place the robot on the target-to-robot line, facing the target."""
    if not math.isfinite(standoff_distance_m) or standoff_distance_m <= 0.0:
        raise ValueError('standoff_distance_m must be positive')
    dx = float(robot_x) - float(target_x)
    dy = float(robot_y) - float(target_y)
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        # Put the target in front of the robot when both positions coincide.
        dx = -math.cos(float(robot_yaw))
        dy = -math.sin(float(robot_yaw))
        distance = 1.0
    goal_x = float(target_x) + standoff_distance_m * dx / distance
    goal_y = float(target_y) + standoff_distance_m * dy / distance
    goal_yaw = math.atan2(float(target_y) - goal_y, float(target_x) - goal_x)
    return goal_x, goal_y, goal_yaw


@dataclass
class PresenceEvidence:
    """Count only fresh, online mmWave samples near the inspected target."""

    expected_distance_m: float
    distance_tolerance_m: float
    total_samples: int = 0
    positive_samples: int = 0

    def add(self, sensor_online: bool, presence: bool, distance_m: float) -> None:
        if not sensor_online:
            return
        self.total_samples += 1
        distance = float(distance_m)
        if (
            presence
            and math.isfinite(distance)
            and distance > 0.0
            and abs(distance - self.expected_distance_m)
            <= self.distance_tolerance_m
        ):
            self.positive_samples += 1

    def classify(
        self, sensor_online: bool, minimum_samples: int, positive_samples: int
    ) -> Optional[str]:
        if not sensor_online or self.total_samples < minimum_samples:
            return None
        if self.positive_samples >= positive_samples:
            return 'PERSON'
        return 'DYNAMIC_OBSTACLE'


class Mode3Inspector(Node):
    """Run one nearest-obstacle inspection after Space is pressed in mode 3."""

    def __init__(self) -> None:
        super().__init__('mode3_inspector')
        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_link',
            'standoff_distance_m': 1.5,
            'robot_settle_sec': 2.0,
            'observation_sec': 5.0,
            'distance_tolerance_m': 0.60,
            'minimum_mmwave_samples': 3,
            'person_positive_samples': 3,
            'sensor_stale_timeout_sec': 2.0,
            'update_rate_hz': 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.standoff_distance = float(
            self.get_parameter('standoff_distance_m').value
        )
        self.robot_settle_sec = float(
            self.get_parameter('robot_settle_sec').value
        )
        self.observation_sec = float(
            self.get_parameter('observation_sec').value
        )
        self.distance_tolerance = float(
            self.get_parameter('distance_tolerance_m').value
        )
        self.minimum_samples = int(
            self.get_parameter('minimum_mmwave_samples').value
        )
        self.positive_samples = int(
            self.get_parameter('person_positive_samples').value
        )
        self.sensor_stale_timeout = float(
            self.get_parameter('sensor_stale_timeout_sec').value
        )
        update_rate = float(self.get_parameter('update_rate_hz').value)
        if (
            self.standoff_distance <= 0.0
            or self.robot_settle_sec < 0.0
            or self.observation_sec <= 0.0
            or self.distance_tolerance < 0.0
            or self.minimum_samples <= 0
            or self.positive_samples <= 0
            or self.positive_samples > self.minimum_samples
            or self.sensor_stale_timeout <= 0.0
            or update_rate <= 0.0
        ):
            raise ValueError('MODE 3 inspection parameters are invalid')

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.tf = TfHelper(self)
        self.drive_mode = 1
        self.phase = 'IDLE'
        self.candidates: Sequence[Point2D] = []
        self.target: Optional[Point2D] = None
        self.waiting_for_departure = False
        self.phase_deadline = 0.0
        self.sensor_online = False
        self.last_sensor_update = float('-inf')
        self.latest_distance_m = 0.0
        self.evidence = PresenceEvidence(
            self.standoff_distance, self.distance_tolerance
        )

        self.goal_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        self.cancel_publisher = self.create_publisher(
            Empty, '/autonomy_cancel', 10
        )
        self.person_publisher = self.create_publisher(
            PointStamped, '/dynamic_obstacle_person', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/mode3_status', latched_qos
        )
        self.classification_publisher = self.create_publisher(
            String, '/mode3_classification', latched_qos
        )
        self.create_subscription(Int32, '/drive_mode', self._mode_callback, 10)
        self.create_subscription(
            String,
            '/obstacle_inspection_command',
            self._inspection_command_callback,
            10,
        )
        self.create_subscription(
            PoseArray,
            '/dynamic_obstacle_candidates',
            self._candidates_callback,
            latched_qos,
        )
        self.create_subscription(
            String, '/follower_state', self._follower_callback, 10
        )
        self.create_subscription(
            String, '/planner_state', self._planner_callback, 10
        )
        self.create_subscription(
            String, '/mmwave/sensor_state', self._sensor_state_callback,
            latched_qos,
        )
        self.create_subscription(
            Float32,
            '/mmwave/filtered_distance_m',
            self._distance_callback,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/mmwave/filtered_presence',
            self._presence_callback,
            latched_qos,
        )
        self.create_timer(1.0 / update_rate, self._timer_callback)
        self._state('MODE3_IDLE')

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _state(self, state: str) -> None:
        self.status_publisher.publish(String(data=state))
        self.get_logger().info(state)

    def _mode_callback(self, message: Int32) -> None:
        mode = int(message.data)
        previous = self.drive_mode
        self.drive_mode = mode
        if mode == 3 and previous != 3:
            self.phase = 'ARMED'
            self.target = None
            self.waiting_for_departure = False
            self._state('MODE3_READY:PRESS_SPACE')
        elif mode != 3 and previous == 3:
            self.cancel_publisher.publish(Empty())
            self.phase = 'IDLE'
            self.target = None
            self.waiting_for_departure = False
            self._state('MODE3_CANCELLED')

    def _inspection_command_callback(self, message: String) -> None:
        if message.data.strip().upper() != 'MODE3_START':
            return
        if self.drive_mode != 3:
            return
        if self.phase in ('NAVIGATING', 'SETTLING', 'OBSERVING'):
            self._state(f'MODE3_BUSY:{self.phase}')
            return
        self.cancel_publisher.publish(Empty())
        self.phase = 'WAITING_FOR_OBSTACLE'
        self.target = None
        self.waiting_for_departure = False
        self._state('MODE3_WAITING_FOR_DYNAMIC_OBSTACLE')
        self._try_start_inspection()

    def _candidates_callback(self, message: PoseArray) -> None:
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().warning(
                f'Ignored obstacle frame {message.header.frame_id!r}'
            )
            return
        self.candidates = [
            (pose.position.x, pose.position.y) for pose in message.poses
        ]
        self._try_start_inspection()

    def _try_start_inspection(self) -> None:
        if self.drive_mode != 3 or self.phase != 'WAITING_FOR_OBSTACLE':
            return
        robot = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if robot is None:
            return
        target = select_nearest_candidate(robot[0], robot[1], self.candidates)
        if target is None:
            return
        goal_x, goal_y, goal_yaw = compute_inspection_goal(
            robot[0], robot[1], robot[2], target[0], target[1],
            self.standoff_distance,
        )
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        qx, qy, qz, qw = quaternion_from_yaw(goal_yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        self.target = target
        self.waiting_for_departure = True
        self.phase = 'NAVIGATING'
        self.goal_publisher.publish(goal)
        self._state(
            f'MODE3_APPROACHING:{target[0]:.3f},{target[1]:.3f}:'
            f'STANDOFF:{self.standoff_distance:.2f}M'
        )

    def _follower_callback(self, message: String) -> None:
        if self.drive_mode != 3 or self.phase != 'NAVIGATING':
            return
        if message.data in (
            'PATH_ACCEPTED', 'FOLLOWING_PATH', 'ROTATING_IN_PLACE',
            'ALIGNING_GOAL_YAW',
        ):
            self.waiting_for_departure = False
        if message.data != 'GOAL_REACHED' or self.waiting_for_departure:
            return
        self.cancel_publisher.publish(Empty())
        self.phase = 'SETTLING'
        self.phase_deadline = self._now() + self.robot_settle_sec
        self._state('MODE3_AT_STANDOFF:ROBOT_SETTLING')

    def _planner_callback(self, message: String) -> None:
        if (
            self.drive_mode == 3
            and self.phase == 'NAVIGATING'
            and message.data == 'NO_PATH'
        ):
            self.cancel_publisher.publish(Empty())
            self.phase = 'NO_PATH'
            self._state('MODE3_NO_PATH_TO_STANDOFF')
            self.get_logger().warning('MODE 3 검사 지점까지 경로가 없습니다.')

    def _sensor_state_callback(self, message: String) -> None:
        self.last_sensor_update = self._now()
        self.sensor_online = message.data.strip().upper() == 'ONLINE'

    def _distance_callback(self, message: Float32) -> None:
        self.last_sensor_update = self._now()
        self.latest_distance_m = float(message.data)

    def _presence_callback(self, message: Bool) -> None:
        self.last_sensor_update = self._now()
        if self.phase != 'OBSERVING':
            return
        self.evidence.add(
            self.sensor_online, bool(message.data), self.latest_distance_m
        )

    def _start_observation(self) -> None:
        self.phase = 'OBSERVING'
        self.phase_deadline = self._now() + self.observation_sec
        self.evidence = PresenceEvidence(
            self.standoff_distance, self.distance_tolerance
        )
        self._state('MODE3_MMWAVE_OBSERVING')
        self.get_logger().warning(
            '[MODE 3] 정지 완료 - mmWave 생체신호 판별 시작'
        )

    def _finish_observation(self) -> None:
        now = self._now()
        online = (
            self.sensor_online
            and now - self.last_sensor_update <= self.sensor_stale_timeout
        )
        result = self.evidence.classify(
            online, self.minimum_samples, self.positive_samples
        )
        if result is None:
            self.phase = 'SENSOR_UNAVAILABLE'
            self._state('MODE3_SENSOR_UNAVAILABLE:KEEP_RED')
            self.get_logger().error(
                '[MODE 3] mmWave 데이터 없음 - 판정 보류, 빨간 점 유지'
            )
            return
        assert self.target is not None
        if result == 'PERSON':
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = self.map_frame
            point.point.x = self.target[0]
            point.point.y = self.target[1]
            point.point.z = 0.10
            self.person_publisher.publish(point)
            self.classification_publisher.publish(
                String(data=f'PERSON:{self.target[0]:.3f},{self.target[1]:.3f}')
            )
            self._state('MODE3_PERSON_CONFIRMED:MARKER_BLUE')
            self.get_logger().warning('사람 감지!')
        else:
            self.classification_publisher.publish(
                String(
                    data=(
                        f'DYNAMIC_OBSTACLE:{self.target[0]:.3f},'
                        f'{self.target[1]:.3f}'
                    )
                )
            )
            self._state('MODE3_DYNAMIC_OBSTACLE_CONFIRMED:KEEP_RED')
            self.get_logger().warning('동적장애물!')
        self.phase = 'COMPLETE'

    def _timer_callback(self) -> None:
        if self.drive_mode != 3:
            return
        if self.phase == 'WAITING_FOR_OBSTACLE':
            self._try_start_inspection()
        elif self.phase == 'SETTLING' and self._now() >= self.phase_deadline:
            self._start_observation()
        elif self.phase == 'OBSERVING' and self._now() >= self.phase_deadline:
            self._finish_observation()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Mode3Inspector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as error:
        if node is None:
            print(f'mode3_inspector: {error}')
        else:
            node.get_logger().error(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
