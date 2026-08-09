"""Confirm new LiDAR endpoints in static-free space as persistent obstacles."""

import math
import time
from typing import Dict, Set

from geometry_msgs.msg import Point
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
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if not (0.0 <= self.min_range < self.max_range):
            raise ValueError('min_range/max_range 값이 올바르지 않습니다.')
        if (
            self.confirm_count <= 0
            or publish_rate <= 0.0
            or self.inflation_radius < 0.0
            or self.wall_exclusion_radius < 0.0
        ):
            raise ValueError('confirm_count/rate는 양수이고 반경은 0 이상이어야 합니다.')

        self.tf = TfHelper(self)
        self.static_grid = None
        self.wall_exclusion_mask = None
        self.counts: Dict[int, int] = {}
        self.confirmed: Dict[int, float] = {}
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
        if self.persistent or self.timeout <= 0.0:
            return
        cutoff = time.monotonic() - self.timeout
        expired = [index for index, seen_at in self.confirmed.items() if seen_at < cutoff]
        for index in expired:
            self.confirmed.pop(index, None)
            self.counts.pop(index, None)

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
        diameter = max(0.05, 2.0 * self.inflation_radius)
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
        self.counts.clear()
        self.confirmed.clear()
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
