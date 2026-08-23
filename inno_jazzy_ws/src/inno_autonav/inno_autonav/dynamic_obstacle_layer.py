"""Confirm new LiDAR endpoints in static-free space as persistent obstacles."""

import math
import time
from typing import Dict, Iterable, List, Set, Tuple

from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
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
    """Exclude saved walls, unknown cells, and their clearance buffer."""

    source = np.asarray(data, dtype=np.int8)
    if source.ndim != 2:
        raise ValueError("static occupancy data must be two-dimensional")
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("map resolution must be positive")
    if not math.isfinite(radius_m) or radius_m < 0.0:
        raise ValueError("wall exclusion radius must not be negative")
    non_free = np.where(source == 0, 0, 100).astype(np.int8)
    radius_cells = int(math.ceil(radius_m / resolution))
    return inflate_occupied_cells(non_free, radius_cells) >= 100


def is_clear_dynamic_candidate(
    grid: MapGrid, exclusion_mask: np.ndarray, grid_x: int, grid_y: int
) -> bool:
    """Accept only known-free cells farther than the saved-wall buffer."""

    return (
        is_inside_grid(grid_x, grid_y, grid)
        and int(grid.data[grid_y, grid_x]) == 0
        and not bool(exclusion_mask[grid_y, grid_x])
    )


def cluster_obstacle_indices(
    indices: Iterable[int], width: int, resolution: float, radius_m: float
) -> List[Set[int]]:
    """Group nearby LiDAR endpoint cells into physical obstacle candidates."""
    if width <= 0 or resolution <= 0.0 or radius_m < 0.0:
        raise ValueError('cluster geometry is invalid')
    remaining = {int(index) for index in indices}
    if not remaining:
        return []
    radius_cells = int(math.ceil(radius_m / resolution))
    offsets = [
        (dx, dy)
        for dy in range(-radius_cells, radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1)
        if math.hypot(dx, dy) <= radius_cells
    ]
    clusters = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        cluster = {seed}
        pending = [seed]
        while pending:
            current = pending.pop()
            current_y, current_x = divmod(current, width)
            for dx, dy in offsets:
                candidate_x = current_x + dx
                candidate_y = current_y + dy
                if candidate_x < 0 or candidate_y < 0 or candidate_x >= width:
                    continue
                candidate = candidate_y * width + candidate_x
                if candidate not in remaining:
                    continue
                remaining.remove(candidate)
                cluster.add(candidate)
                pending.append(candidate)
        clusters.append(cluster)
    return clusters


def match_people_to_clusters(
    clusters: Iterable[Tuple[Set[int], float, float]],
    classified_people: Iterable[Tuple[float, float, float]],
    match_radius_m: float,
) -> Set[int]:
    """Match each classified person to at most one nearest LiDAR cluster."""
    cluster_list = list(clusters)
    people = list(classified_people)
    edges = []
    for person_index, (person_x, person_y, _) in enumerate(people):
        for cluster_index, (_, center_x, center_y) in enumerate(cluster_list):
            distance = math.hypot(center_x - person_x, center_y - person_y)
            if distance <= match_radius_m:
                edges.append((distance, person_index, cluster_index))
    matched_people = set()
    matched_clusters = set()
    for _, person_index, cluster_index in sorted(edges):
        if person_index in matched_people or cluster_index in matched_clusters:
            continue
        matched_people.add(person_index)
        matched_clusters.add(cluster_index)
    return matched_clusters


class DynamicObstacleLayer(Node):
    def __init__(self) -> None:
        super().__init__('dynamic_obstacle_layer')
        defaults = {
            'scan_topic': '/scan',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'min_range': 0.15,
            'max_range': 4.0,
            'obstacle_confirm_count': 3,
            'persistent_obstacles': True,
            'obstacle_timeout_sec': 10.0,
            'inflation_radius': 0.30,
            'wall_exclusion_radius': 0.25,
            'cluster_radius_m': 0.50,
            'person_match_radius_m': 0.75,
            'person_dedup_radius_m': 0.20,
            'person_classification_timeout_sec': 0.0,
            'publish_rate_hz': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.confirm_count = int(self.get_parameter('obstacle_confirm_count').value)
        self.persistent = bool(self.get_parameter('persistent_obstacles').value)
        self.timeout = float(self.get_parameter('obstacle_timeout_sec').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.wall_exclusion_radius = float(
            self.get_parameter('wall_exclusion_radius').value
        )
        self.cluster_radius = float(
            self.get_parameter('cluster_radius_m').value
        )
        self.person_match_radius = float(
            self.get_parameter('person_match_radius_m').value
        )
        self.person_dedup_radius = float(
            self.get_parameter('person_dedup_radius_m').value
        )
        self.person_timeout = float(
            self.get_parameter('person_classification_timeout_sec').value
        )
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if not (0.0 <= self.min_range < self.max_range):
            raise ValueError('min_range/max_range 값이 올바르지 않습니다.')
        if (
            self.confirm_count <= 0
            or publish_rate <= 0.0
            or self.inflation_radius < 0.0
            or self.wall_exclusion_radius < 0.0
            or self.cluster_radius < 0.0
            or self.person_match_radius <= 0.0
            or self.person_dedup_radius < 0.0
            or self.person_timeout < 0.0
        ):
            raise ValueError('confirm_count/rate는 양수이고 반경은 0 이상이어야 합니다.')

        self.tf = TfHelper(self)
        self.static_grid = None
        self.wall_exclusion_mask = None
        self.counts: Dict[int, int] = {}
        self.confirmed: Dict[int, float] = {}
        self.classified_people: List[Tuple[float, float, float]] = []
        grid_qos = QoSProfile(depth=1)
        grid_qos.reliability = ReliabilityPolicy.RELIABLE
        grid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, '/planning_grid_static', self._static_callback, grid_qos
        )
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)
        self.grid_publisher = self.create_publisher(
            OccupancyGrid, '/dynamic_obstacle_grid', grid_qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/dynamic_obstacle_markers', grid_qos
        )
        self.detected_publisher = self.create_publisher(
            Bool, '/dynamic_obstacle_detected', grid_qos
        )
        self.candidate_publisher = self.create_publisher(
            PoseArray, '/dynamic_obstacle_candidates', grid_qos
        )
        self.create_subscription(
            PointStamped,
            '/dynamic_obstacle_person',
            self._person_callback,
            10,
        )
        self.create_service(
            Trigger, '/clear_dynamic_obstacles', self._clear_callback
        )
        self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            f'dynamic obstacle layer: scan={self.scan_topic}, '
            f'persistent={self.persistent}, confirm={self.confirm_count}'
        )

    def _static_callback(self, message: OccupancyGrid) -> None:
        try:
            incoming = grid_from_message(message)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'static grid 변환 실패: {exc}')
            return
        if self.static_grid is not None and (
            incoming.width != self.static_grid.width
            or incoming.height != self.static_grid.height
            or abs(incoming.resolution - self.static_grid.resolution) > 1e-9
        ):
            self.counts.clear()
            self.confirmed.clear()
            self.classified_people.clear()
            self.get_logger().warning('static grid geometry 변경: dynamic obstacle 초기화')
        self.static_grid = incoming
        self.wall_exclusion_mask = build_wall_exclusion_mask(
            incoming.data, incoming.resolution, self.wall_exclusion_radius
        )

    def _scan_callback(self, scan: LaserScan) -> None:
        if self.static_grid is None or self.wall_exclusion_mask is None:
            return
        transform = self.tf.lookup_transform(self.map_frame, scan.header.frame_id)
        if transform is None:
            return
        angle = float(scan.angle_min)
        seen: Set[int] = set()
        for measured_range in scan.ranges:
            distance = float(measured_range)
            if (
                math.isfinite(distance)
                and max(self.min_range, float(scan.range_min)) <= distance
                and distance <= min(self.max_range, float(scan.range_max))
            ):
                scan_x = distance * math.cos(angle)
                scan_y = distance * math.sin(angle)
                map_x, map_y = self.tf.transform_point_2d(
                    transform, scan_x, scan_y
                )
                grid_x, grid_y = world_to_grid(map_x, map_y, self.static_grid)
                if is_clear_dynamic_candidate(
                    self.static_grid, self.wall_exclusion_mask, grid_x, grid_y
                ):
                    index = grid_y * self.static_grid.width + grid_x
                    seen.add(index)
            angle += float(scan.angle_increment)

        now = time.monotonic()
        for index in seen:
            self.counts[index] = self.counts.get(index, 0) + 1
            if self.counts[index] >= self.confirm_count:
                self.confirmed[index] = now

    def _expire(self) -> None:
        now = time.monotonic()
        if not self.persistent and self.timeout > 0.0:
            cutoff = now - self.timeout
            expired = [
                index
                for index, seen_at in self.confirmed.items()
                if seen_at < cutoff
            ]
            for index in expired:
                self.confirmed.pop(index, None)
                self.counts.pop(index, None)
        if self.person_timeout > 0.0:
            person_cutoff = now - self.person_timeout
            self.classified_people = [
                item
                for item in self.classified_people
                if item[2] >= person_cutoff
            ]

    def _person_callback(self, message: PointStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().warning(
                f'Ignored person classification frame {message.header.frame_id!r}'
            )
            return
        x = float(message.point.x)
        y = float(message.point.y)
        if not math.isfinite(x) or not math.isfinite(y):
            return
        now = time.monotonic()
        nearest = None
        nearest_distance = math.inf
        for index, (old_x, old_y, _) in enumerate(self.classified_people):
            distance = math.hypot(x - old_x, y - old_y)
            if distance < nearest_distance:
                nearest = index
                nearest_distance = distance
        item = (x, y, now)
        if nearest is not None and nearest_distance <= self.person_dedup_radius:
            self.classified_people[nearest] = item
        else:
            self.classified_people.append(item)

    def _obstacle_clusters(self):
        clusters = cluster_obstacle_indices(
            self.confirmed,
            self.static_grid.width,
            self.static_grid.resolution,
            self.cluster_radius,
        )
        result = []
        for indices in clusters:
            points = []
            for index in indices:
                grid_y, grid_x = divmod(index, self.static_grid.width)
                points.append(
                    grid_to_world(grid_x, grid_y, self.static_grid)
                )
            center_x = sum(point[0] for point in points) / len(points)
            center_y = sum(point[1] for point in points) / len(points)
            result.append((indices, center_x, center_y))
        return result

    def _is_person(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - person_x, y - person_y) <= self.person_match_radius
            for person_x, person_y, _ in self.classified_people
        )

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
        if self.static_grid is None:
            return
        self._expire()
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
        self.detected_publisher.publish(Bool(data=bool(self.confirmed)))
        clusters = self._obstacle_clusters()
        matched_people = match_people_to_clusters(
            clusters, self.classified_people, self.person_match_radius
        )
        self._publish_candidates(stamp, clusters, matched_people)
        self._publish_markers(stamp, clusters, matched_people)

    def _publish_candidates(self, stamp, clusters, matched_people=None) -> None:
        matched_people = matched_people or set()
        message = PoseArray()
        message.header.frame_id = self.map_frame
        message.header.stamp = stamp
        for cluster_index, (_, center_x, center_y) in enumerate(clusters):
            if cluster_index in matched_people:
                continue
            pose = Pose()
            pose.position.x = center_x
            pose.position.y = center_y
            pose.orientation.w = 1.0
            message.poses.append(pose)
        self.candidate_publisher.publish(message)

    def _publish_markers(self, stamp, clusters, matched_people=None) -> None:
        if matched_people is None:
            matched_people = match_people_to_clusters(
                clusters, self.classified_people, self.person_match_radius
            )
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        red = Marker()
        red.header = clear.header
        red.ns = 'dynamic_obstacles'
        red.id = 1
        red.type = Marker.SPHERE_LIST
        red.action = Marker.ADD
        diameter = max(0.05, 2.0 * self.inflation_radius)
        red.scale.x = diameter
        red.scale.y = diameter
        red.scale.z = 0.20
        red.color.r = 1.0
        red.color.g = 0.1
        red.color.b = 0.0
        red.color.a = 0.85
        red.pose.orientation.w = 1.0
        blue = Marker()
        blue.header = clear.header
        blue.ns = 'classified_people'
        blue.id = 2
        blue.type = Marker.SPHERE_LIST
        blue.action = Marker.ADD
        blue.scale.x = diameter
        blue.scale.y = diameter
        blue.scale.z = 0.20
        blue.color.r = 0.0
        blue.color.g = 0.35
        blue.color.b = 1.0
        blue.color.a = 0.90
        blue.pose.orientation.w = 1.0
        for cluster_index, (_, center_x, center_y) in enumerate(clusters):
            point = Point(x=center_x, y=center_y, z=0.10)
            if cluster_index not in matched_people:
                red.points.append(point)
        for person_x, person_y, _ in self.classified_people:
            blue.points.append(Point(x=person_x, y=person_y, z=0.10))
        self.marker_publisher.publish(MarkerArray(markers=[clear, red, blue]))

    def _clear_callback(self, request, response):
        del request
        count = len(self.confirmed)
        self.counts.clear()
        self.confirmed.clear()
        self.classified_people.clear()
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
    except KeyboardInterrupt:
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
