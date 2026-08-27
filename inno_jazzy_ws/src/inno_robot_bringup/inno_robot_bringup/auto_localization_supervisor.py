"""Automatically initialize AMCL and validate LiDAR-to-map alignment."""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener


Pose2D = Tuple[float, float, float]


def quaternion_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def normalize_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def scan_map_overlap_ratio(
    map_message: OccupancyGrid,
    scan: LaserScan,
    map_base_pose: Pose2D,
    base_laser_pose: Pose2D,
    tolerance_m: float = 0.20,
    beam_stride: int = 6,
) -> tuple[float, int]:
    """Score how many LiDAR endpoints land near occupied map cells."""
    width = int(map_message.info.width)
    height = int(map_message.info.height)
    resolution = float(map_message.info.resolution)
    if (
        width <= 0 or height <= 0 or resolution <= 0.0
        or len(map_message.data) != width * height
        or beam_stride <= 0 or tolerance_m < 0.0
    ):
        return 0.0, 0
    origin = map_message.info.origin
    origin_yaw = quaternion_yaw(origin.orientation)
    origin_cos = math.cos(origin_yaw)
    origin_sin = math.sin(origin_yaw)
    base_x, base_y, base_yaw = map_base_pose
    laser_x, laser_y, laser_yaw = base_laser_pose
    laser_cos = math.cos(laser_yaw)
    laser_sin = math.sin(laser_yaw)
    base_cos = math.cos(base_yaw)
    base_sin = math.sin(base_yaw)
    laser_map_x = base_x + base_cos * laser_x - base_sin * laser_y
    laser_map_y = base_y + base_sin * laser_x + base_cos * laser_y
    radius_cells = int(math.ceil(tolerance_m / resolution))
    matches = 0
    considered = 0
    angle = float(scan.angle_min)
    for index, measured_range in enumerate(scan.ranges):
        if index % beam_stride:
            angle += float(scan.angle_increment)
            continue
        distance = float(measured_range)
        if (
            not math.isfinite(distance)
            or distance < max(0.0, float(scan.range_min))
            or distance > float(scan.range_max)
        ):
            angle += float(scan.angle_increment)
            continue
        # Valid returns outside the map are evidence against this pose.  The
        # previous implementation discarded them, allowing a false pose near
        # a map edge to obtain an artificially high overlap score.
        considered += 1
        point_laser_x = distance * math.cos(angle)
        point_laser_y = distance * math.sin(angle)
        point_base_x = (
            laser_x + laser_cos * point_laser_x - laser_sin * point_laser_y
        )
        point_base_y = (
            laser_y + laser_sin * point_laser_x + laser_cos * point_laser_y
        )
        map_x = base_x + base_cos * point_base_x - base_sin * point_base_y
        map_y = base_y + base_sin * point_base_x + base_cos * point_base_y
        dx = map_x - float(origin.position.x)
        dy = map_y - float(origin.position.y)
        local_x = origin_cos * dx + origin_sin * dy
        local_y = -origin_sin * dx + origin_cos * dy
        col = int(math.floor(local_x / resolution))
        row = int(math.floor(local_y / resolution))
        if not (0 <= col < width and 0 <= row < height):
            angle += float(scan.angle_increment)
            continue
        # Endpoint-only matching is weak in corridors and repeated rooms: a
        # ray can end on a wall even though it crosses a nearer mapped wall.
        # Reject such physically impossible rays before counting the endpoint.
        ray_steps = max(1, int(math.ceil(distance / resolution)))
        endpoint_clearance_steps = radius_cells + 1
        ray_blocked = False
        for step in range(1, max(1, ray_steps - endpoint_clearance_steps)):
            ratio = step / ray_steps
            ray_x = laser_map_x + ratio * (map_x - laser_map_x)
            ray_y = laser_map_y + ratio * (map_y - laser_map_y)
            ray_dx = ray_x - float(origin.position.x)
            ray_dy = ray_y - float(origin.position.y)
            ray_local_x = origin_cos * ray_dx + origin_sin * ray_dy
            ray_local_y = -origin_sin * ray_dx + origin_cos * ray_dy
            ray_col = int(math.floor(ray_local_x / resolution))
            ray_row = int(math.floor(ray_local_y / resolution))
            if not (0 <= ray_col < width and 0 <= ray_row < height):
                ray_blocked = True
                break
            cell = int(map_message.data[ray_row * width + ray_col])
            if cell < 0 or cell >= 65:
                ray_blocked = True
                break
        if ray_blocked:
            angle += float(scan.angle_increment)
            continue
        matched = False
        for offset_y in range(-radius_cells, radius_cells + 1):
            check_row = row + offset_y
            if not 0 <= check_row < height:
                continue
            for offset_x in range(-radius_cells, radius_cells + 1):
                if math.hypot(offset_x, offset_y) > radius_cells + 1e-9:
                    continue
                check_col = col + offset_x
                if not 0 <= check_col < width:
                    continue
                if int(map_message.data[check_row * width + check_col]) >= 65:
                    matched = True
                    break
            if matched:
                break
        if matched:
            matches += 1
        angle += float(scan.angle_increment)
    return (matches / considered if considered else 0.0), considered


class AutoLocalizationSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('auto_localization_supervisor')
        defaults = {
            'map_topic': '/map',
            'scan_topic': '/scan',
            'pose_topic': '/amcl_pose',
            'base_frame': 'base_link',
            'global_localization_service': '/reinitialize_global_localization',
            'nomotion_update_service': '/request_nomotion_update',
            'nomotion_update_period_sec': 0.75,
            'startup_timeout_sec': 45.0,
            'minimum_localization_duration_sec': 5.0,
            'maximum_position_variance': 0.25,
            'maximum_yaw_variance': 0.25,
            'maximum_confirmation_position_jump_m': 0.15,
            'maximum_confirmation_yaw_jump_rad': 0.15,
            'scan_match_tolerance_m': 0.15,
            'minimum_scan_overlap_ratio': 0.65,
            'minimum_scan_beams': 50,
            'confirmation_count': 10,
            'beam_stride': 4,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.base_frame = str(value('base_frame'))
        self.nomotion_period = float(value('nomotion_update_period_sec'))
        self.timeout = float(value('startup_timeout_sec'))
        self.minimum_duration = float(
            value('minimum_localization_duration_sec')
        )
        self.maximum_position_variance = float(value('maximum_position_variance'))
        self.maximum_yaw_variance = float(value('maximum_yaw_variance'))
        self.maximum_position_jump = float(
            value('maximum_confirmation_position_jump_m')
        )
        self.maximum_yaw_jump = float(
            value('maximum_confirmation_yaw_jump_rad')
        )
        self.scan_match_tolerance = float(value('scan_match_tolerance_m'))
        self.minimum_overlap = float(value('minimum_scan_overlap_ratio'))
        self.minimum_beams = int(value('minimum_scan_beams'))
        self.confirmation_required = int(value('confirmation_count'))
        self.beam_stride = int(value('beam_stride'))
        if (
            self.nomotion_period <= 0.0 or self.timeout <= 0.0
            or self.minimum_duration <= 0.0
            or self.maximum_position_variance <= 0.0
            or self.maximum_yaw_variance <= 0.0
            or self.maximum_position_jump <= 0.0
            or self.maximum_yaw_jump <= 0.0
            or self.scan_match_tolerance < 0.0
            or not 0.0 <= self.minimum_overlap <= 1.0
            or self.minimum_beams <= 0 or self.confirmation_required <= 0
            or self.beam_stride <= 0
        ):
            raise ValueError('automatic localization parameters are invalid')

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.ready_publisher = self.create_publisher(
            Bool, '/localization_ready', transient
        )
        self.status_publisher = self.create_publisher(
            String, '/localization_status', transient
        )
        self.create_subscription(
            OccupancyGrid, str(value('map_topic')), self._map_callback, transient
        )
        self.create_subscription(
            LaserScan,
            str(value('scan_topic')),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(value('pose_topic')),
            self._pose_callback,
            10,
        )
        self.global_client = self.create_client(
            Empty, str(value('global_localization_service'))
        )
        self.nomotion_client = self.create_client(
            Empty, str(value('nomotion_update_service'))
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_message: Optional[OccupancyGrid] = None
        self.scan: Optional[LaserScan] = None
        self.pose: Optional[PoseWithCovarianceStamped] = None
        self.global_future = None
        self.nomotion_future = None
        self.global_requested = False
        self.started_at = time.monotonic()
        self.last_nomotion_at = float('-inf')
        self.confirmations = 0
        self.scan_sequence = 0
        self.last_evaluated_scan_sequence = -1
        self.last_confirmation_pose: Optional[Pose2D] = None
        self.ready = False
        self.failure_reported = False
        self.last_status = ''
        self.create_timer(0.20, self._timer_callback)
        self._publish_ready(False)
        self._state('WAITING_FOR_MAP_AND_SCAN')
        self.get_logger().info('[ROBOT] [위치] 자동 초기 위치 추정을 준비합니다.')

    def _publish_ready(self, ready: bool) -> None:
        self.ready_publisher.publish(Bool(data=bool(ready)))

    def _state(self, status: str) -> None:
        if status == self.last_status:
            return
        self.last_status = status
        self.status_publisher.publish(String(data=status))

    def _map_callback(self, message: OccupancyGrid) -> None:
        self.map_message = message

    def _scan_callback(self, message: LaserScan) -> None:
        self.scan = message
        self.scan_sequence += 1

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        self.pose = message

    def _laser_pose_in_base(self) -> Optional[Pose2D]:
        if self.scan is None:
            return None
        frame = self.scan.header.frame_id
        if not frame or frame == self.base_frame:
            return 0.0, 0.0, 0.0
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, frame, Time(), timeout=Duration(seconds=0.10)
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        return (
            float(translation.x),
            float(translation.y),
            quaternion_yaw(transform.transform.rotation),
        )

    def _pose_is_confident(self) -> tuple[bool, float, int]:
        if self.map_message is None or self.scan is None or self.pose is None:
            return False, 0.0, 0
        covariance = self.pose.pose.covariance
        covariance_ok = (
            len(covariance) >= 36
            and math.isfinite(float(covariance[0]))
            and math.isfinite(float(covariance[7]))
            and math.isfinite(float(covariance[35]))
            and min(
                float(covariance[0]), float(covariance[7]),
                float(covariance[35]),
            ) >= 0.0
            and max(float(covariance[0]), float(covariance[7]))
            <= self.maximum_position_variance
            and float(covariance[35]) <= self.maximum_yaw_variance
        )
        laser_pose = self._laser_pose_in_base()
        if laser_pose is None:
            return False, 0.0, 0
        pose = self.pose.pose.pose
        ratio, beams = scan_map_overlap_ratio(
            self.map_message,
            self.scan,
            (
                float(pose.position.x),
                float(pose.position.y),
                quaternion_yaw(pose.orientation),
            ),
            laser_pose,
            tolerance_m=self.scan_match_tolerance,
            beam_stride=self.beam_stride,
        )
        return (
            covariance_ok
            and beams >= self.minimum_beams
            and ratio >= self.minimum_overlap,
            ratio,
            beams,
        )

    def _timer_callback(self) -> None:
        if self.ready:
            self._publish_ready(True)
            return
        now = time.monotonic()
        if self.map_message is None or self.scan is None:
            self._state('WAITING_FOR_MAP_AND_SCAN')
            return
        if not self.global_requested:
            if not self.global_client.service_is_ready():
                self._state('WAITING_FOR_AMCL_GLOBAL_LOCALIZATION_SERVICE')
                return
            self.global_future = self.global_client.call_async(Empty.Request())
            self.global_requested = True
            self._state('GLOBAL_LOCALIZATION_REQUESTED')
            self.get_logger().info(
                '[ROBOT] [위치] 지도 전체에서 LiDAR 초기 위치를 찾는 중입니다.'
            )
            return
        if self.global_future is not None and not self.global_future.done():
            return
        if (
            self.nomotion_client.service_is_ready()
            and now - self.last_nomotion_at >= self.nomotion_period
            and (self.nomotion_future is None or self.nomotion_future.done())
        ):
            self.nomotion_future = self.nomotion_client.call_async(Empty.Request())
            self.last_nomotion_at = now
        if self.scan_sequence == self.last_evaluated_scan_sequence:
            return
        self.last_evaluated_scan_sequence = self.scan_sequence
        confident, overlap, beams = self._pose_is_confident()
        pose_consistent = False
        candidate_pose = None
        if confident and self.pose is not None:
            pose = self.pose.pose.pose
            candidate_pose = (
                float(pose.position.x),
                float(pose.position.y),
                quaternion_yaw(pose.orientation),
            )
            previous = self.last_confirmation_pose
            pose_consistent = previous is None or (
                math.hypot(
                    candidate_pose[0] - previous[0],
                    candidate_pose[1] - previous[1],
                ) <= self.maximum_position_jump
                and abs(normalize_angle(candidate_pose[2] - previous[2]))
                <= self.maximum_yaw_jump
            )
        if confident and pose_consistent and candidate_pose is not None:
            self.confirmations += 1
            self.last_confirmation_pose = candidate_pose
        else:
            self.confirmations = 0
            self.last_confirmation_pose = candidate_pose if confident else None
        self._state(
            f'LOCALIZING:OVERLAP={overlap:.3f}:BEAMS={beams}:'
            f'CONFIRM={self.confirmations}/{self.confirmation_required}'
        )
        if (
            self.confirmations >= self.confirmation_required
            and now - self.started_at >= self.minimum_duration
        ):
            self.ready = True
            self._publish_ready(True)
            self._state('LOCALIZATION_READY')
            self.get_logger().info(
                f'[ROBOT] [위치] 자동 초기 위치 추정 완료 '
                f'(스캔 일치율 {overlap:.0%}).'
            )
            return
        if now - self.started_at >= self.timeout and not self.failure_reported:
            self.failure_reported = True
            self._state('AUTO_LOCALIZATION_UNCERTAIN:USE_2D_POSE_ESTIMATE')
            self.get_logger().error(
                '[ROBOT] [위치] 자동 추정 신뢰도가 부족합니다. '
                '주행은 차단했습니다. RViz 2D Pose Estimate를 사용해 주세요.'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AutoLocalizationSupervisor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f'auto_localization_supervisor 오류: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
