"""Confirm large LiDAR clusters in static-free space for avoidance."""

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .grid_utils import (
    MapGrid,
    grid_to_world,
    inflate_occupied_cells,
    is_inside_grid,
    quaternion_from_yaw,
    yaw_from_quaternion,
    world_to_grid,
)
from .tf_utils import TfHelper


XY = Tuple[float, float]


@dataclass
class ObstacleClusterTrack:
    x: float
    y: float
    hits: int
    last_seen: float


def cluster_scan_points(
    points: Sequence[XY], connection_radius_m: float, min_points: int
) -> List[List[XY]]:
    """Return connected scan clusters containing at least min_points."""

    if connection_radius_m <= 0.0 or min_points <= 0:
        raise ValueError('cluster radius and min_points must be positive')
    remaining = set(range(len(points)))
    clusters = []
    while remaining:
        pending = [remaining.pop()]
        members = []
        while pending:
            current = pending.pop()
            members.append(points[current])
            neighbors = [
                index for index in tuple(remaining)
                if math.hypot(
                    points[index][0] - points[current][0],
                    points[index][1] - points[current][1],
                ) <= connection_radius_m
            ]
            for index in neighbors:
                remaining.remove(index)
                pending.append(index)
        if len(members) >= min_points:
            clusters.append(members)
    return clusters


class LargeObstacleTracker:
    """Require consecutive spatially-consistent large scan clusters."""

    def __init__(
        self, confirm_scans: int, match_radius_m: float, max_gap_sec: float
    ) -> None:
        if confirm_scans < 1 or match_radius_m <= 0.0 or max_gap_sec <= 0.0:
            raise ValueError('large obstacle tracker parameters are invalid')
        self.confirm_scans = int(confirm_scans)
        self.match_radius = float(match_radius_m)
        self.max_gap = float(max_gap_sec)
        self.tracks: List[ObstacleClusterTrack] = []

    def update(
        self, clusters: Sequence[Sequence[XY]], now: float
    ) -> List[List[XY]]:
        previous = [
            track for track in self.tracks
            if now - track.last_seen <= self.max_gap
        ]
        next_tracks = []
        confirmed = []
        used = set()
        for cluster in clusters:
            x = sum(point[0] for point in cluster) / len(cluster)
            y = sum(point[1] for point in cluster) / len(cluster)
            candidates = sorted(
                (
                    (math.hypot(track.x - x, track.y - y), index, track)
                    for index, track in enumerate(previous)
                    if index not in used
                ),
                key=lambda item: item[0],
            )
            if candidates and candidates[0][0] <= self.match_radius:
                _, index, track = candidates[0]
                used.add(index)
                hits = track.hits + 1
            else:
                hits = 1
            current = ObstacleClusterTrack(x, y, hits, float(now))
            next_tracks.append(current)
            if hits >= self.confirm_scans:
                confirmed.append(list(cluster))
        self.tracks = next_tracks
        return confirmed

    def clear(self) -> None:
        self.tracks.clear()


def grid_from_message(message: OccupancyGrid) -> MapGrid:
    orientation = message.info.origin.orientation
    return MapGrid(
        width=int(message.info.width),
        height=int(message.info.height),
        resolution=float(message.info.resolution),
        origin_x=float(message.info.origin.position.x),
        origin_y=float(message.info.origin.position.y),
        origin_yaw=yaw_from_quaternion(orientation),
        frame_id=message.header.frame_id,
        data=np.asarray(message.data, dtype=np.int8).reshape(
            int(message.info.height), int(message.info.width)
        ),
    )


def build_wall_exclusion_mask(
    data: np.ndarray, resolution: float, radius_m: float
) -> np.ndarray:
    """Mark static walls/unknown space and nearby cells as non-dynamic.

    A real wall return can land in a nearby free cell because of scan noise
    and small localization errors. Unknown cells are treated like walls
    because they cannot prove that a return is a new obstacle.
    """

    source = np.asarray(data, dtype=np.int8)
    if source.ndim != 2:
        raise ValueError('static occupancy data must be two-dimensional')
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError('map resolution must be positive')
    if not math.isfinite(radius_m) or radius_m < 0.0:
        raise ValueError('wall exclusion radius must not be negative')
    non_free = np.where(source == 0, 0, 100).astype(np.int8)
    radius_cells = int(math.ceil(radius_m / resolution))
    return inflate_occupied_cells(non_free, radius_cells) >= 100


def is_clear_dynamic_candidate(
    grid: MapGrid, exclusion_mask: np.ndarray, grid_x: int, grid_y: int
) -> bool:
    """Return true only for a known-free cell outside the wall buffer."""

    return (
        is_inside_grid(grid_x, grid_y, grid)
        and int(grid.data[grid_y, grid_x]) == 0
        and not bool(exclusion_mask[grid_y, grid_x])
    )


def scan_range_membership(
    distance: float,
    *,
    sensor_min: float,
    sensor_max: float,
    configured_min: float,
    avoidance_max: float,
    observation_max: float,
) -> Tuple[bool, bool]:
    """Return (victim-observation, avoidance) membership for one scan ray."""

    valid = (
        math.isfinite(distance)
        and max(configured_min, sensor_min) <= distance
        and distance <= min(observation_max, sensor_max)
    )
    return valid, valid and distance <= min(avoidance_max, sensor_max)


class DynamicObstacleLayer(Node):
    def __init__(self) -> None:
        super().__init__('dynamic_obstacle_layer')
        defaults = {
            'scan_topic': '/scan',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'min_range': 0.15,
            'max_range': 4.0,
            'observation_max_range': 4.0,
            'obstacle_confirm_count': 3,
            'min_cluster_points': 5,
            'cluster_radius_m': 0.18,
            'cluster_track_radius_m': 0.35,
            'cluster_max_scan_gap_sec': 0.35,
            'persistent_obstacles': False,
            'obstacle_timeout_sec': 1.2,
            'inflation_radius': 0.0,
            'marker_diameter_m': 0.60,
            'wall_exclusion_radius': 0.25,
            'publish_rate_hz': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.observation_max_range = float(
            self.get_parameter('observation_max_range').value
        )
        self.confirm_count = int(self.get_parameter('obstacle_confirm_count').value)
        self.min_cluster_points = int(
            self.get_parameter('min_cluster_points').value
        )
        self.cluster_radius = float(self.get_parameter('cluster_radius_m').value)
        self.cluster_track_radius = float(
            self.get_parameter('cluster_track_radius_m').value
        )
        self.cluster_max_scan_gap = float(
            self.get_parameter('cluster_max_scan_gap_sec').value
        )
        self.persistent = bool(self.get_parameter('persistent_obstacles').value)
        self.timeout = float(self.get_parameter('obstacle_timeout_sec').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.marker_diameter = float(
            self.get_parameter('marker_diameter_m').value
        )
        self.wall_exclusion_radius = float(
            self.get_parameter('wall_exclusion_radius').value
        )
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if not (
            0.0 <= self.min_range < self.max_range
            <= self.observation_max_range
        ):
            raise ValueError(
                'min_range/max_range/observation_max_range 값이 '
                '올바르지 않습니다.'
            )
        if (
            self.confirm_count <= 0
            or self.min_cluster_points < 5
            or publish_rate <= 0.0
            or self.cluster_radius <= 0.0
            or self.cluster_track_radius <= 0.0
            or self.cluster_max_scan_gap <= 0.0
            or self.inflation_radius < 0.0
            or self.marker_diameter <= 0.0
            or self.wall_exclusion_radius < 0.0
        ):
            raise ValueError(
                '동적장애물 군집/확인/크기/주기 파라미터가 올바르지 않습니다.'
            )

        self.mode3_enabled = False
        self.tf = TfHelper(self)
        self.static_grid = None
        self.wall_exclusion_mask = None
        self.confirmed: Dict[int, float] = {}
        self.cluster_tracker = LargeObstacleTracker(
            self.confirm_count,
            self.cluster_track_radius,
            self.cluster_max_scan_gap,
        )
        grid_qos = QoSProfile(depth=1)
        grid_qos.reliability = ReliabilityPolicy.RELIABLE
        grid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, '/planning_grid_static', self._static_callback, grid_qos
        )
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)
        self.create_subscription(Int32, '/drive_mode', self._mode_callback, 10)
        self.grid_publisher = self.create_publisher(
            OccupancyGrid, '/dynamic_obstacle_grid', grid_qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/dynamic_obstacle_markers', grid_qos
        )
        self.detected_publisher = self.create_publisher(
            Bool, '/dynamic_obstacle_detected', grid_qos
        )
        self._last_detected_state = None
        observation_qos = QoSProfile(depth=1)
        observation_qos.reliability = ReliabilityPolicy.RELIABLE
        self.observation_publisher = self.create_publisher(
            MarkerArray, '/dynamic_obstacle_observations', observation_qos
        )
        self.create_service(
            Trigger, '/clear_dynamic_obstacles', self._clear_callback
        )
        self.create_timer(1.0 / publish_rate, self._publish)
        self._publish_detection_state(force=True)
        self.get_logger().info(
            f'dynamic obstacle layer: scan={self.scan_topic}, '
            f'min_points={self.min_cluster_points}, '
            f'confirm_scans={self.confirm_count}, persistent={self.persistent}'
        )

    def _mode_callback(self, message: Int32) -> None:
        enabled = int(message.data) == 3
        if enabled == self.mode3_enabled:
            return
        self.mode3_enabled = enabled
        if not enabled:
            self.cluster_tracker.clear()
            self.confirmed.clear()
            self._publish_observations(())
            self._publish()
        self._publish_detection_state()
        self.get_logger().info(
            'MODE 3 dynamic avoidance enabled' if enabled
            else 'Dynamic avoidance disabled outside MODE 3'
        )

    def _static_callback(self, message: OccupancyGrid) -> None:
        try:
            incoming = grid_from_message(message)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'static grid 변환 실패: {exc}')
            return
        unchanged = (
            self.static_grid is not None
            and incoming.width == self.static_grid.width
            and incoming.height == self.static_grid.height
            and abs(incoming.resolution - self.static_grid.resolution) <= 1e-9
            and abs(incoming.origin_x - self.static_grid.origin_x) <= 1e-9
            and abs(incoming.origin_y - self.static_grid.origin_y) <= 1e-9
            and abs(incoming.origin_yaw - self.static_grid.origin_yaw) <= 1e-9
            and np.array_equal(incoming.data, self.static_grid.data)
        )
        if unchanged:
            return
        if self.static_grid is not None:
            self.cluster_tracker.clear()
            self.confirmed.clear()
            self.get_logger().warning(
                'static grid 변경: dynamic obstacle 초기화'
            )
        self.static_grid = incoming
        self.wall_exclusion_mask = build_wall_exclusion_mask(
            incoming.data,
            incoming.resolution,
            self.wall_exclusion_radius,
        )

    def _scan_callback(self, scan: LaserScan) -> None:
        if not self.mode3_enabled:
            return
        if self.static_grid is None or self.wall_exclusion_mask is None:
            return
        transform = self.tf.lookup_transform(self.map_frame, scan.header.frame_id)
        if transform is None:
            return
        angle = float(scan.angle_min)
        observations: List[XY] = []
        avoidance_observations: List[XY] = []
        for measured_range in scan.ranges:
            distance = float(measured_range)
            observe, avoid = scan_range_membership(
                distance,
                sensor_min=float(scan.range_min),
                sensor_max=float(scan.range_max),
                configured_min=self.min_range,
                avoidance_max=self.max_range,
                observation_max=self.observation_max_range,
            )
            if observe:
                scan_x = distance * math.cos(angle)
                scan_y = distance * math.sin(angle)
                map_x, map_y = self.tf.transform_point_2d(
                    transform, scan_x, scan_y
                )
                grid_x, grid_y = world_to_grid(map_x, map_y, self.static_grid)
                if is_clear_dynamic_candidate(
                    self.static_grid,
                    self.wall_exclusion_mask,
                    grid_x,
                    grid_y,
                ):
                    observations.append((map_x, map_y))
                    if avoid:
                        avoidance_observations.append((map_x, map_y))
            angle += float(scan.angle_increment)

        clusters = cluster_scan_points(
            avoidance_observations,
            self.cluster_radius,
            self.min_cluster_points,
        )
        now = time.monotonic()
        confirmed_clusters = self.cluster_tracker.update(clusters, now)
        for cluster in confirmed_clusters:
            for map_x, map_y in cluster:
                grid_x, grid_y = world_to_grid(map_x, map_y, self.static_grid)
                index = grid_y * self.static_grid.width + grid_x
                self.confirmed[index] = now
        self._publish_observations(observations)
        self._publish_detection_state()

    def _publish_detection_state(self, force: bool = False) -> None:
        detected = self.mode3_enabled and bool(self.confirmed)
        if force or detected != self._last_detected_state:
            self.detected_publisher.publish(Bool(data=detected))
            self._last_detected_state = detected

    def _publish_observations(self, points: Sequence[XY]) -> None:
        stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        marker = Marker()
        marker.header = clear.header
        marker.ns = 'current_dynamic_observations'
        marker.id = 2
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.a = 0.0
        marker.points = [Point(x=x, y=y, z=0.10) for x, y in points]
        self.observation_publisher.publish(MarkerArray(markers=[clear, marker]))

    def _expire(self) -> None:
        if self.persistent or self.timeout <= 0.0:
            return
        cutoff = time.monotonic() - self.timeout
        expired = [index for index, seen_at in self.confirmed.items() if seen_at < cutoff]
        for index in expired:
            self.confirmed.pop(index, None)

    def _dynamic_array(self) -> np.ndarray:
        data = np.zeros(
            (self.static_grid.height, self.static_grid.width), dtype=np.int8
        )
        for index in self.confirmed:
            y, x = divmod(index, self.static_grid.width)
            data[y, x] = 100
        radius_cells = int(math.ceil(self.inflation_radius / self.static_grid.resolution))
        return inflate_occupied_cells(data, radius_cells)

    def _publish(self) -> None:
        self._expire()
        self._publish_detection_state()
        if self.static_grid is None:
            return
        data = self._dynamic_array()
        stamp = self.get_clock().now().to_msg()
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self.static_grid.frame_id
        message.info.map_load_time = stamp
        message.info.resolution = self.static_grid.resolution
        message.info.width = self.static_grid.width
        message.info.height = self.static_grid.height
        message.info.origin.position.x = self.static_grid.origin_x
        message.info.origin.position.y = self.static_grid.origin_y
        qx, qy, qz, qw = quaternion_from_yaw(self.static_grid.origin_yaw)
        message.info.origin.orientation.x = qx
        message.info.origin.orientation.y = qy
        message.info.origin.orientation.z = qz
        message.info.origin.orientation.w = qw
        message.data = data.reshape(-1).astype(int).tolist()
        self.grid_publisher.publish(message)
        self._publish_markers(stamp)

    def _publish_markers(self, stamp) -> None:
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        marker = Marker()
        marker.header = clear.header
        marker.ns = 'persistent_dynamic_obstacles'
        marker.id = 1
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        diameter = self.marker_diameter
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = 0.15
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.0
        marker.color.a = 0.75
        marker.pose.orientation.w = 1.0
        for index in sorted(self.confirmed):
            grid_y, grid_x = divmod(index, self.static_grid.width)
            world_x, world_y = grid_to_world(grid_x, grid_y, self.static_grid)
            point = Point(x=world_x, y=world_y, z=0.10)
            marker.points.append(point)
        self.marker_publisher.publish(MarkerArray(markers=[clear, marker]))

    def _clear_callback(self, request, response):
        del request
        count = len(self.confirmed)
        self.cluster_tracker.clear()
        self.confirmed.clear()
        self._publish_detection_state()
        response.success = True
        response.message = f'{count}개 dynamic obstacle을 삭제했습니다.'
        self.get_logger().warning(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DynamicObstacleLayer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as exc:
        print(f'dynamic_obstacle_layer 오류: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
