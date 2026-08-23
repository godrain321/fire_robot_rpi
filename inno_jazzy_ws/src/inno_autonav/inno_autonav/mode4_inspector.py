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
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Empty, Int32, String

from .grid_utils import normalize_angle, quaternion_from_yaw
from .mode3_inspector import compute_inspection_goal, select_nearest_candidate
from .tf_utils import TfHelper


Point2D = Tuple[float, float]
RobotPose2D = Tuple[float, float, float]


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


@dataclass(frozen=True)
class CameraIntrinsics:
    """Horizontal pinhole projection values scaled to one image."""

    width: int
    fx: float
    cx: float


@dataclass(frozen=True)
class CandidateAssociation:
    """A one-to-one camera detection and LiDAR candidate association."""

    detection_index: int
    candidate_index: int
    candidate: Point2D
    pixel_error: float
    bearing_error_rad: float


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


def scale_intrinsics(
    intrinsics: CameraIntrinsics, image_width: int
) -> CameraIntrinsics:
    """Scale calibration when detector and CameraInfo image widths differ."""
    if intrinsics.width <= 0 or image_width <= 0 or intrinsics.fx <= 0.0:
        raise ValueError('camera intrinsics are invalid')
    scale = float(image_width) / float(intrinsics.width)
    return CameraIntrinsics(image_width, intrinsics.fx * scale, intrinsics.cx * scale)


def fallback_intrinsics(image_width: int, horizontal_fov_rad: float):
    """Create intrinsics from the measured horizontal field of view."""
    if image_width <= 0 or not 0.0 < horizontal_fov_rad < math.pi:
        raise ValueError('fallback camera field of view is invalid')
    fx = 0.5 * image_width / math.tan(0.5 * horizontal_fov_rad)
    return CameraIntrinsics(image_width, fx, 0.5 * image_width)


def project_candidate_u(
    robot_pose: RobotPose2D,
    candidate: Point2D,
    intrinsics: CameraIntrinsics,
    camera_yaw_offset_rad: float = 0.0,
) -> Optional[Tuple[float, float, float]]:
    """Project a map candidate to horizontal image pixel and relative bearing."""
    dx = candidate[0] - robot_pose[0]
    dy = candidate[1] - robot_pose[1]
    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        return None
    bearing = normalize_angle(
        math.atan2(dy, dx) - robot_pose[2] - camera_yaw_offset_rad
    )
    if abs(bearing) >= 0.5 * math.pi:
        return None
    pixel_u = intrinsics.cx - intrinsics.fx * math.tan(bearing)
    if pixel_u < 0.0 or pixel_u >= intrinsics.width:
        return None
    return pixel_u, bearing, distance


def associate_detections_to_candidates(
    robot_pose: RobotPose2D,
    inspection_target: Point2D,
    candidates: Sequence[Point2D],
    detections: Sequence[PersonDetection],
    intrinsics: CameraIntrinsics,
    camera_yaw_offset_rad: float,
    target_search_radius_m: float,
    maximum_candidate_distance_m: float,
    maximum_bearing_error_rad: float,
) -> List[CandidateAssociation]:
    """Greedily make one-to-one matches using horizontal camera bearing."""
    edges = []
    for candidate_index, candidate in enumerate(candidates):
        if math.hypot(
            candidate[0] - inspection_target[0],
            candidate[1] - inspection_target[1],
        ) > target_search_radius_m:
            continue
        projection = project_candidate_u(
            robot_pose, candidate, intrinsics, camera_yaw_offset_rad
        )
        if projection is None or projection[2] > maximum_candidate_distance_m:
            continue
        candidate_u, candidate_bearing, _ = projection
        for detection_index, detection in enumerate(detections):
            detection_bearing = math.atan2(
                intrinsics.cx - detection.center_x, intrinsics.fx
            )
            bearing_error = abs(
                normalize_angle(candidate_bearing - detection_bearing)
            )
            if bearing_error > maximum_bearing_error_rad:
                continue
            edges.append(
                (
                    bearing_error,
                    -detection.confidence,
                    abs(candidate_u - detection.center_x),
                    detection_index,
                    candidate_index,
                )
            )
    matched_detections = set()
    matched_candidates = set()
    associations = []
    for bearing_error, _, pixel_error, detection_index, candidate_index in sorted(
        edges
    ):
        if (
            detection_index in matched_detections
            or candidate_index in matched_candidates
        ):
            continue
        matched_detections.add(detection_index)
        matched_candidates.add(candidate_index)
        associations.append(
            CandidateAssociation(
                detection_index=detection_index,
                candidate_index=candidate_index,
                candidate=candidates[candidate_index],
                pixel_error=pixel_error,
                bearing_error_rad=bearing_error,
            )
        )
    return associations


class Mode4Inspector(Node):
    """Approach on Space, then classify the correct LiDAR point with YOLO."""

    def __init__(self) -> None:
        super().__init__('mode4_inspector')
        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_link',
            'camera_info_topic': '/camera/camera_info',
            'detection_topic': '/camera/person_detections',
            'standoff_distance_m': 1.5,
            'robot_settle_sec': 2.0,
            'observation_sec': 5.0,
            'minimum_detection_frames': 3,
            'survivor_positive_frames': 2,
            'detector_stale_timeout_sec': 2.0,
            'minimum_confidence': 0.50,
            'fallback_horizontal_fov_deg': 76.0,
            'camera_yaw_offset_deg': 0.0,
            'target_search_radius_m': 1.0,
            'maximum_candidate_distance_m': 3.0,
            'maximum_bearing_error_deg': 10.0,
            'update_rate_hz': 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
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
        self.minimum_confidence = float(
            self.get_parameter('minimum_confidence').value
        )
        self.fallback_fov = math.radians(
            float(self.get_parameter('fallback_horizontal_fov_deg').value)
        )
        self.camera_yaw_offset = math.radians(
            float(self.get_parameter('camera_yaw_offset_deg').value)
        )
        self.target_search_radius = float(
            self.get_parameter('target_search_radius_m').value
        )
        self.maximum_candidate_distance = float(
            self.get_parameter('maximum_candidate_distance_m').value
        )
        self.maximum_bearing_error = math.radians(
            float(self.get_parameter('maximum_bearing_error_deg').value)
        )
        update_rate = float(self.get_parameter('update_rate_hz').value)
        if (
            self.standoff_distance <= 0.0
            or self.robot_settle_sec < 0.0
            or self.observation_sec <= 0.0
            or self.minimum_frames <= 0
            or self.positive_frames <= 0
            or self.positive_frames > self.minimum_frames
            or self.detector_stale_timeout <= 0.0
            or not 0.0 < self.minimum_confidence <= 1.0
            or not 0.0 < self.fallback_fov < math.pi
            or self.target_search_radius <= 0.0
            or self.maximum_candidate_distance <= 0.0
            or not 0.0 < self.maximum_bearing_error < 0.5 * math.pi
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
        self.waiting_for_departure = False
        self.phase_deadline = 0.0
        self.detector_status = 'UNKNOWN'
        self.last_detector_frame = float('-inf')
        self.camera_intrinsics: Optional[CameraIntrinsics] = None
        self.frame_count = 0
        self.max_people_in_frame = 0
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
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
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
            self.waiting_for_departure = False
            self._state('MODE4_READY:PRESS_SPACE')
        elif mode != 4 and previous == 4:
            self.cancel_publisher.publish(Empty())
            self.phase = 'IDLE'
            self.target = None
            self.waiting_for_departure = False
            self._state('MODE4_CANCELLED')

    def _inspection_command_callback(self, message: String) -> None:
        if message.data.strip().upper() != 'MODE4_START':
            return
        if self.drive_mode != 4:
            return
        if self.phase in ('NAVIGATING', 'SETTLING', 'OBSERVING'):
            self._state(f'MODE4_BUSY:{self.phase}')
            return
        self.cancel_publisher.publish(Empty())
        self.phase = 'WAITING_FOR_OBSTACLE'
        self.target = None
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
        target = select_nearest_candidate(robot[0], robot[1], self.candidates)
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

    def _camera_info_callback(self, message: CameraInfo) -> None:
        if message.width <= 0 or len(message.k) < 3:
            return
        fx = float(message.k[0])
        cx = float(message.k[2])
        if math.isfinite(fx) and math.isfinite(cx) and fx > 0.0:
            self.camera_intrinsics = CameraIntrinsics(
                int(message.width), fx, cx
            )

    def _detector_status_callback(self, message: String) -> None:
        self.detector_status = message.data.strip().upper()

    def _intrinsics_for_image(self, image_width: int) -> CameraIntrinsics:
        if self.camera_intrinsics is not None:
            return scale_intrinsics(self.camera_intrinsics, image_width)
        return fallback_intrinsics(image_width, self.fallback_fov)

    def _detection_callback(self, message: String) -> None:
        now = self._now()
        self.last_detector_frame = now
        if self.phase != 'OBSERVING':
            return
        try:
            width, _, detections = parse_detection_message(
                message.data, self.minimum_confidence
            )
            intrinsics = self._intrinsics_for_image(width)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        robot = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if robot is None or self.target is None:
            return
        self.frame_count += 1
        associations = associate_detections_to_candidates(
            robot,
            self.target,
            self.observation_candidates,
            detections,
            intrinsics,
            self.camera_yaw_offset,
            self.target_search_radius,
            self.maximum_candidate_distance,
            self.maximum_bearing_error,
        )
        self.max_people_in_frame = max(
            self.max_people_in_frame, len(associations)
        )
        for association in associations:
            index = association.candidate_index
            self.candidate_votes[index] = self.candidate_votes.get(index, 0) + 1

    def _start_observation(self) -> None:
        assert self.target is not None
        self.phase = 'OBSERVING'
        self.phase_deadline = self._now() + self.observation_sec
        self.observation_candidates = list(self.candidates)
        if not any(
            math.hypot(point[0] - self.target[0], point[1] - self.target[1])
            < 0.05
            for point in self.observation_candidates
        ):
            self.observation_candidates = list(self.observation_candidates) + [
                self.target
            ]
        self.frame_count = 0
        self.max_people_in_frame = 0
        self.candidate_votes = {}
        self._state('MODE4_CAMERA_YOLO_OBSERVING')
        self.get_logger().warning(
            '[MODE 4] 정지 완료 - 카메라 YOLO 요구조자 판별 시작'
        )

    def _finish_observation(self) -> None:
        now = self._now()
        detector_online = (
            self.detector_status == 'ONLINE'
            and now - self.last_detector_frame <= self.detector_stale_timeout
        )
        if not detector_online or self.frame_count < self.minimum_frames:
            self.phase = 'DETECTOR_UNAVAILABLE'
            self._state('MODE4_DETECTOR_UNAVAILABLE:KEEP_RED')
            self.get_logger().error(
                '[MODE 4] 카메라/YOLO 데이터 없음 - 판정 보류, 빨간 점 유지'
            )
            return
        ranked = sorted(
            self.candidate_votes.items(), key=lambda item: (-item[1], item[0])
        )
        confirmed = [
            item for item in ranked if item[1] >= self.positive_frames
        ][:self.max_people_in_frame]
        if not confirmed:
            self.classification_publisher.publish(
                String(data='NO_SURVIVOR')
            )
            self._state('MODE4_NO_SURVIVOR:KEEP_RED')
            self.get_logger().warning('요구조자 미감지!')
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
        elif self.phase == 'OBSERVING' and self._now() >= self.phase_deadline:
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
