"""Fuse Camera Module 3 person boxes with LiDAR obstacles in drive mode 4."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Empty, Int32, String, UInt64

from .grid_utils import quaternion_from_yaw
from .mode3_inspector import compute_inspection_goal, select_nearest_candidate
from .tf_utils import TfHelper


Point2D = Tuple[float, float]


def parse_mode4_inspection_command(
    value: str,
) -> tuple[bool, Optional[Point2D]]:
    """Parse the manual command or Mode 5's explicit LiDAR target."""
    command = str(value).strip()
    if command.upper() == 'MODE4_START':
        return True, None
    prefix = 'MODE4_START_AT:'
    if not command.upper().startswith(prefix):
        return False, None
    try:
        point = tuple(float(item) for item in command[len(prefix):].split(','))
    except ValueError:
        return False, None
    if len(point) != 2 or not all(math.isfinite(item) for item in point):
        return False, None
    return True, (point[0], point[1])


@dataclass(frozen=True)
class PersonDetection:
    """Person bounding box received from the camera detector."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float
    confidence: float

    @property
    def center_x(self) -> float:
        return 0.5 * (self.x_min + self.x_max)


def parse_detection_message(
    payload: str, minimum_confidence: float
) -> Tuple[int, int, List[PersonDetection]]:
    """Validate the JSON transport published by camera_person_detector."""
    try:
        document = json.loads(payload)
        width = int(document['image_width'])
        height = int(document['image_height'])
        raw_detections = document['detections']
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise ValueError('invalid camera detection payload') from error
    if width <= 0 or height <= 0 or not isinstance(raw_detections, list):
        raise ValueError('invalid camera detection geometry')
    detections = []
    for raw in raw_detections:
        try:
            detection = PersonDetection(
                x_min=float(raw['x_min']),
                y_min=float(raw['y_min']),
                x_max=float(raw['x_max']),
                y_max=float(raw['y_max']),
                confidence=float(raw['confidence']),
            )
        except (KeyError, TypeError, ValueError):
            continue
        values = (
            detection.x_min,
            detection.y_min,
            detection.x_max,
            detection.y_max,
            detection.confidence,
        )
        if not all(math.isfinite(value) for value in values):
            continue
        if (
            detection.confidence < minimum_confidence
            or detection.x_max <= detection.x_min
            or detection.y_max <= detection.y_min
            or detection.center_x < 0.0
            or detection.center_x >= width
        ):
            continue
        detections.append(detection)
    return width, height, detections


class Mode4Inspector(Node):
    """Approach on Space, then classify the faced LiDAR target with YOLO."""

    def __init__(self) -> None:
        super().__init__('mode4_inspector')
        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_link',
            'detection_topic': '/camera/person_detections',
            'standoff_distance_m': 1.5,
            'robot_settle_sec': 2.0,
            # OpenCV-DNN inference on the Raspberry Pi 5 takes roughly
            # 12--15 seconds with the current 640 px model.  These defaults
            # deliberately cover one complete inference instead of treating
            # the slow (but healthy) detector as unavailable.
            'observation_sec': 20.0,
            'minimum_detection_frames': 1,
            'survivor_positive_frames': 1,
            'detector_stale_timeout_sec': 30.0,
            'detector_startup_timeout_sec': 30.0,
            'minimum_confidence': 0.40,
            'update_rate_hz': 10.0,
            'publish_canonical_plan': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        detection_topic = str(self.get_parameter('detection_topic').value)
        self.standoff_distance = float(
            self.get_parameter('standoff_distance_m').value
        )
        self.robot_settle_sec = float(
            self.get_parameter('robot_settle_sec').value
        )
        self.observation_sec = float(
            self.get_parameter('observation_sec').value
        )
        self.minimum_frames = int(
            self.get_parameter('minimum_detection_frames').value
        )
        self.positive_frames = int(
            self.get_parameter('survivor_positive_frames').value
        )
        self.detector_stale_timeout = float(
            self.get_parameter('detector_stale_timeout_sec').value
        )
        self.detector_startup_timeout = float(
            self.get_parameter('detector_startup_timeout_sec').value
        )
        self.minimum_confidence = float(
            self.get_parameter('minimum_confidence').value
        )
        update_rate = float(self.get_parameter('update_rate_hz').value)
        self.publish_canonical_plan = bool(
            self.get_parameter('publish_canonical_plan').value
        )
        if (
            self.standoff_distance <= 0.0
            or self.robot_settle_sec < 0.0
            or self.observation_sec <= 0.0
            or self.minimum_frames <= 0
            or self.positive_frames <= 0
            or self.positive_frames > self.minimum_frames
            or self.detector_stale_timeout <= 0.0
            or self.detector_startup_timeout <= 0.0
            or not 0.0 < self.minimum_confidence <= 1.0
            or update_rate <= 0.0
        ):
            raise ValueError('MODE 4 inspection parameters are invalid')

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.tf = TfHelper(self)
        self.drive_mode = 1
        self.phase = 'IDLE'
        self.candidates: Sequence[Point2D] = []
        self.observation_candidates: Sequence[Point2D] = []
        self.target: Optional[Point2D] = None
        self.requested_target: Optional[Point2D] = None
        self.hazard_revision = 0
        self.waiting_for_departure = False
        self.phase_deadline = 0.0
        self.detector_start_deadline = 0.0
        self.detector_status = 'UNKNOWN'
        self.last_detector_frame = float('-inf')
        self.frame_count = 0
        self.person_frame_count = 0
        self.positive_frame_count = 0
        self.person_detection_count = 0
        self.maximum_person_confidence = 0.0
        self.candidate_votes: Dict[int, int] = {}

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
            String, '/mode4_status', latched_qos
        )
        self.classification_publisher = self.create_publisher(
            String, '/mode4_classification', latched_qos
        )
        self.plan_publisher = self.create_publisher(
            String, '/evacuation/plan', latched_qos
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
            String, detection_topic, self._detection_callback, 10
        )
        self.create_subscription(
            String,
            '/camera/person_detector_status',
            self._detector_status_callback,
            latched_qos,
        )
        self.create_subscription(
            UInt64, '/hazard/revision', self._hazard_revision_callback,
            latched_qos,
        )
        self.create_timer(1.0 / update_rate, self._timer_callback)
        self._state('MODE4_IDLE')

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
        if mode == 4 and previous != 4:
            self.phase = 'ARMED'
            self.target = None
            self.requested_target = None
            self.waiting_for_departure = False
            self._state('MODE4_READY:PRESS_SPACE')
        elif mode != 4 and previous == 4:
            self.cancel_publisher.publish(Empty())
            self.phase = 'IDLE'
            self.target = None
            self.requested_target = None
            self.waiting_for_departure = False
            self._state('MODE4_CANCELLED')

    def _inspection_command_callback(self, message: String) -> None:
        accepted, requested_target = parse_mode4_inspection_command(message.data)
        if not accepted:
            return
        if self.drive_mode != 4:
            return
        if self.phase in ('NAVIGATING', 'SETTLING', 'OBSERVING'):
            self._state(f'MODE4_BUSY:{self.phase}')
            return
        self.cancel_publisher.publish(Empty())
        self.phase = 'WAITING_FOR_OBSTACLE'
        self.target = None
        self.requested_target = requested_target
        self.waiting_for_departure = False
        self._state('MODE4_WAITING_FOR_DYNAMIC_OBSTACLE')
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
        if self.drive_mode != 4 or self.phase != 'WAITING_FOR_OBSTACLE':
            return
        robot = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if robot is None:
            return
        target = self.requested_target or select_nearest_candidate(
            robot[0], robot[1], self.candidates
        )
        if target is None:
            return
        goal_x, goal_y, goal_yaw = compute_inspection_goal(
            robot[0],
            robot[1],
            robot[2],
            target[0],
            target[1],
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
        if self.publish_canonical_plan:
            now = self.get_clock().now()
            payload = {
                'success': True,
                'start_position_world': [robot[0], robot[1]],
                'selected_exit_id': 'MODE4_INSPECTION',
                'selected_exit_position_world': [target[0], target[1]],
                'selected_approach_position_world': [goal_x, goal_y],
                'selected_approach_yaw_rad': goal_yaw,
                'path_world': [],
                'path_grid': [],
                'selected_evaluation': None,
                'all_evaluations': [],
                'failure_reason': None,
                'selection_reason': 'mode5 moving LiDAR candidate inspection',
                'created_at': now.nanoseconds / 1e9,
                'hazard_revision': self.hazard_revision,
                'activated': True,
                'manager_status': 'MODE4_INSPECTION_ACTIVATED',
            }
            self.plan_publisher.publish(String(data=json.dumps(
                payload, sort_keys=True, separators=(',', ':'), allow_nan=False
            )))
        # Mode 5 consumes the canonical plan.  Suppressing the duplicate direct
        # goal preserves the inspection yaw when a shared field-test profile
        # accepts named Mode 2 goals as well as Mode 3/4 canonical plans.
        if not self.publish_canonical_plan:
            self.goal_publisher.publish(goal)
        self._state(
            f'MODE4_APPROACHING:{target[0]:.3f},{target[1]:.3f}:'
            f'STANDOFF:{self.standoff_distance:.2f}M'
        )

    def _follower_callback(self, message: String) -> None:
        if self.drive_mode != 4 or self.phase != 'NAVIGATING':
            return
        if message.data in (
            'PATH_ACCEPTED',
            'FOLLOWING_PATH',
            'ROTATING_IN_PLACE',
            'ALIGNING_GOAL_YAW',
        ):
            self.waiting_for_departure = False
        if message.data != 'GOAL_REACHED' or self.waiting_for_departure:
            return
        self.cancel_publisher.publish(Empty())
        self.phase = 'SETTLING'
        self.phase_deadline = self._now() + self.robot_settle_sec
        self._state('MODE4_AT_STANDOFF:ROBOT_SETTLING')

    def _planner_callback(self, message: String) -> None:
        if (
            self.drive_mode == 4
            and self.phase == 'NAVIGATING'
            and message.data == 'NO_PATH'
        ):
            self.cancel_publisher.publish(Empty())
            self.phase = 'NO_PATH'
            self._state('MODE4_NO_PATH_TO_STANDOFF')
            self.get_logger().warning('MODE 4 검사 지점까지 경로가 없습니다.')

    def _detector_status_callback(self, message: String) -> None:
        self.detector_status = message.data.strip().upper()

    def _hazard_revision_callback(self, message: UInt64) -> None:
        self.hazard_revision = int(message.data)

    def _detection_callback(self, message: String) -> None:
        now = self._now()
        self.last_detector_frame = now
        if self.phase != 'OBSERVING':
            return
        try:
            _, _, detections = parse_detection_message(
                message.data, self.minimum_confidence
            )
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        if self.target is None:
            return
        if self.frame_count == 0:
            # The observation duration starts with the first usable inference
            # result, not when the enable/status message was published.
            self.phase_deadline = now + self.observation_sec
        self.frame_count += 1
        if detections:
            self.person_frame_count += 1
            self.person_detection_count += len(detections)
            self.maximum_person_confidence = max(
                self.maximum_person_confidence,
                max(detection.confidence for detection in detections),
            )
        # The robot has already stopped with its goal yaw facing ``target``.
        # Field operation guarantees that a second person/dynamic object will
        # not share this camera frame, so any stable person detection belongs
        # to the one red point currently being inspected.  Deliberately avoid
        # camera-FOV, pixel-bearing, and LiDAR-angle gates here.
        if detections:
            self.positive_frame_count += 1
            self.candidate_votes[0] = self.candidate_votes.get(0, 0) + 1
        # A positive person result is conclusive in the field scenario: the
        # robot has stopped facing the one red candidate and no second person
        # shares the frame.  Finish immediately so another 12--15 second Pi
        # inference is not required before turning the marker blue.
        if (
            self.frame_count >= self.minimum_frames
            and self.positive_frame_count >= self.positive_frames
        ):
            self._finish_observation()

    def _start_observation(self) -> None:
        assert self.target is not None
        self.phase = 'OBSERVING'
        now = self._now()
        self.phase_deadline = float('inf')
        self.detector_start_deadline = now + self.detector_startup_timeout
        # Freeze exactly the selected red point.  Other red candidates in the
        # scene must never receive this camera result.
        self.observation_candidates = [self.target]
        self.frame_count = 0
        self.person_frame_count = 0
        self.positive_frame_count = 0
        self.person_detection_count = 0
        self.maximum_person_confidence = 0.0
        self.candidate_votes = {}
        self._state('MODE4_CAMERA_YOLO_OBSERVING')
        self.get_logger().warning(
            '[MODE 4] 정지 완료 - 카메라 YOLO 요구조자 판별 시작'
        )

    def _finish_observation(self) -> None:
        now = self._now()
        # The detector publishes the result immediately before its ONLINE
        # status.  A valid, fresh payload is therefore stronger evidence than
        # the asynchronously delivered status value and avoids rejecting the
        # first successful inference due to topic callback ordering.
        detector_online = (
            self.frame_count > 0
            and now - self.last_detector_frame <= self.detector_stale_timeout
        )
        if not detector_online or self.frame_count < self.minimum_frames:
            self.phase = 'DETECTOR_UNAVAILABLE'
            self._state('MODE4_DETECTOR_UNAVAILABLE:KEEP_RED')
            self.get_logger().error(
                '[MODE 4] 카메라/YOLO 데이터 없음 - 판정 보류, 빨간 점 유지'
            )
            return
        self._state(
            'MODE4_DETECTION_SUMMARY:'
            f'FRAMES={self.frame_count}:'
            f'PERSON_FRAMES={self.person_frame_count}:'
            f'VOTE_FRAMES={self.positive_frame_count}:'
            f'DETECTIONS={self.person_detection_count}:'
            f'MAX_CONF={self.maximum_person_confidence:.2f}'
        )
        ranked = sorted(
            self.candidate_votes.items(), key=lambda item: (-item[1], item[0])
        )
        confirmed = [
            item for item in ranked if item[1] >= self.positive_frames
        ][:1]
        if not confirmed:
            self.classification_publisher.publish(
                String(data='NO_SURVIVOR')
            )
            self._state('MODE4_NO_SURVIVOR:KEEP_RED')
            if self.person_frame_count == 0:
                reason = 'YOLO 사람 박스가 confidence 기준을 넘지 못했습니다.'
            else:
                reason = (
                    'YOLO 사람 박스의 연속 검출 투표가 기준 횟수보다 '
                    '부족했습니다.'
                )
            self.get_logger().warning(f'요구조자 미감지: {reason}')
            self.phase = 'COMPLETE'
            return
        labels = []
        for candidate_index, votes in confirmed:
            candidate = self.observation_candidates[candidate_index]
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = self.map_frame
            point.point.x = candidate[0]
            point.point.y = candidate[1]
            point.point.z = 0.10
            self.person_publisher.publish(point)
            labels.append(
                f'{candidate[0]:.3f},{candidate[1]:.3f},{votes}'
            )
        self.classification_publisher.publish(
            String(data='SURVIVOR:' + ';'.join(labels))
        )
        self._state('MODE4_SURVIVOR_CONFIRMED:MARKER_BLUE')
        self.get_logger().warning('요구조자 감지!')
        self.phase = 'COMPLETE'

    def _timer_callback(self) -> None:
        if self.drive_mode != 4:
            return
        if self.phase == 'WAITING_FOR_OBSTACLE':
            self._try_start_inspection()
        elif self.phase == 'SETTLING' and self._now() >= self.phase_deadline:
            self._start_observation()
        elif self.phase == 'OBSERVING':
            now = self._now()
            if self.frame_count == 0:
                if now >= self.detector_start_deadline:
                    self._finish_observation()
            elif now >= self.phase_deadline:
                self._finish_observation()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Mode4Inspector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as error:
        if node is None:
            print(f'mode4_inspector: {error}')
        else:
            node.get_logger().error(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
