"""Merge static/dynamic grids and continuously replan an 8-connected A* path."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .grid_utils import (
    MapGrid,
    grid_to_world,
    inflate_occupied_cells,
    is_inside_grid,
    normalize_angle,
    path_cells_collision,
    quaternion_from_yaw,
    world_to_grid,
    yaw_from_quaternion,
)
from .tf_utils import TfHelper
from .safe_path_simplifier import simplify_path_safely
from .weighted_planner import (
    combine_cost_grids,
    thermal_readiness_state,
    weighted_astar_search,
)


Cell = Tuple[int, int]


def astar_search(
    data: np.ndarray,
    start: Cell,
    goal: Cell,
    unknown_is_occupied: bool = True,
    allow_diagonal: bool = True,
) -> List[Cell]:
    result = weighted_astar_search(
        data, start, goal,
        unknown_is_occupied=unknown_is_occupied,
        allow_diagonal=allow_diagonal,
        thermal_cost_weight=0.0,
        thermal_cost_power=1.0,
    )
    return list(result.path)


def simplify_path(
    path: List[Cell], data: np.ndarray, unknown_is_occupied: bool
) -> List[Cell]:
    result = simplify_path_safely(
        path, data,
        unknown_is_occupied=unknown_is_occupied,
        thermal_cost_weight=0.0,
        thermal_cost_power=1.0,
    )
    return list(result.path)


def message_to_grid(message: OccupancyGrid) -> MapGrid:
    width, height = int(message.info.width), int(message.info.height)
    if width <= 0 or height <= 0 or len(message.data) != width * height:
        raise ValueError('OccupancyGrid geometry/data 길이가 올바르지 않습니다.')
    return MapGrid(
        width=width,
        height=height,
        resolution=float(message.info.resolution),
        origin_x=float(message.info.origin.position.x),
        origin_y=float(message.info.origin.position.y),
        origin_yaw=yaw_from_quaternion(message.info.origin.orientation),
        frame_id=message.header.frame_id,
        data=np.asarray(message.data, dtype=np.int8).reshape(height, width),
    )


class AstarReplanner(Node):
    def __init__(self) -> None:
        super().__init__('astar_replanner')
        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_link',
            'unknown_is_occupied': True,
            'replan_rate_hz': 1.0,
            'path_block_check_radius': 0.20,
            'start_clearance_radius': 0.18,
            'allow_diagonal': True,
            'thermal_grid_topic': '/thermal_cost_grid',
            'thermal_status_topic': '/thermal_cost_status',
            'require_thermal_grid': True,
            'require_thermal_active': True,
            'thermal_grid_timeout_sec': 1.0,
            'thermal_cost_weight': 8.0,
            'thermal_cost_power': 2.0,
            'simplification_maximum_risk_ratio': 1.0,
            'simplification_risk_absolute_tolerance': 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.unknown_is_occupied = bool(
            self.get_parameter('unknown_is_occupied').value
        )
        self.replan_rate = float(self.get_parameter('replan_rate_hz').value)
        self.clearance_radius = float(
            self.get_parameter('path_block_check_radius').value
        )
        self.start_clearance_radius = float(
            self.get_parameter('start_clearance_radius').value
        )
        self.allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        self.thermal_grid_topic = str(
            self.get_parameter('thermal_grid_topic').value
        )
        self.thermal_status_topic = str(
            self.get_parameter('thermal_status_topic').value
        )
        self.require_thermal_grid = bool(
            self.get_parameter('require_thermal_grid').value
        )
        self.require_thermal_active = bool(
            self.get_parameter('require_thermal_active').value
        )
        self.thermal_timeout = float(
            self.get_parameter('thermal_grid_timeout_sec').value
        )
        self.thermal_cost_weight = float(
            self.get_parameter('thermal_cost_weight').value
        )
        self.thermal_cost_power = float(
            self.get_parameter('thermal_cost_power').value
        )
        self.simplification_maximum_risk_ratio = float(
            self.get_parameter('simplification_maximum_risk_ratio').value
        )
        self.simplification_risk_absolute_tolerance = float(
            self.get_parameter('simplification_risk_absolute_tolerance').value
        )
        if (self.replan_rate <= 0.0 or self.clearance_radius < 0.0
                or self.start_clearance_radius < 0.0
                or self.thermal_timeout < 0.0
                or self.thermal_cost_weight < 0.0
                or self.thermal_cost_power <= 0.0
                or self.simplification_maximum_risk_ratio < 1.0
                or self.simplification_risk_absolute_tolerance < 0.0):
            raise ValueError(
                'rate/power는 양수, radius/timeout/weight는 0 이상, '
                'simplification risk ratio는 1 이상이어야 합니다.'
            )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.tf = TfHelper(self)
        self.static_grid: Optional[MapGrid] = None
        self.dynamic_grid: Optional[MapGrid] = None
        self.thermal_grid: Optional[MapGrid] = None
        self.thermal_status = ''
        self.thermal_received_ns: Optional[int] = None
        self.thermal_geometry_mismatch = False
        self.combined_grid: Optional[MapGrid] = None
        self.goal: Optional[PoseStamped] = None
        self.current_path_cells: List[Cell] = []
        self.current_simplified_cells: List[Cell] = []
        self.current_path_total_cost = math.inf
        self._dirty = False
        self._last_plan = 0.0
        self.create_subscription(
            OccupancyGrid, '/planning_grid_static', self._static_callback, qos
        )
        self.create_subscription(
            OccupancyGrid, '/dynamic_obstacle_grid', self._dynamic_callback, qos
        )
        self.create_subscription(
            OccupancyGrid, self.thermal_grid_topic, self._thermal_callback, qos
        )
        self.create_subscription(
            String, self.thermal_status_topic, self._thermal_status_callback, qos
        )
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_callback, 10)
        self.grid_publisher = self.create_publisher(
            OccupancyGrid, '/planning_grid', qos
        )
        self.path_publisher = self.create_publisher(Path, '/planned_path', qos)
        self.state_publisher = self.create_publisher(String, '/planner_state', 10)
        self.create_timer(1.0 / self.replan_rate, self._timer_callback)
        self._state('WAITING_FOR_GRID')

    @staticmethod
    def _same_geometry(first: MapGrid, second: MapGrid) -> bool:
        return (
            first.width == second.width
            and first.height == second.height
            and abs(first.resolution - second.resolution) < 1e-9
            and abs(first.origin_x - second.origin_x) < 1e-6
            and abs(first.origin_y - second.origin_y) < 1e-6
            and abs(first.origin_yaw - second.origin_yaw) < 1e-9
            and first.frame_id == second.frame_id
        )

    def _static_callback(self, message: OccupancyGrid) -> None:
        try:
            grid = message_to_grid(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        changed = (
            self.static_grid is None
            or not self._same_geometry(self.static_grid, grid)
            or not np.array_equal(self.static_grid.data, grid.data)
        )
        self.static_grid = grid
        if self.dynamic_grid is not None and not self._same_geometry(
            grid, self.dynamic_grid
        ):
            self.dynamic_grid = None
            self.get_logger().warning(
                'static geometry 변경으로 기존 dynamic grid를 초기화합니다.'
            )
        if self.thermal_grid is not None and not self._same_geometry(
            grid, self.thermal_grid
        ):
            self.thermal_geometry_mismatch = True
        if changed:
            self._dirty = True
            self._combine_and_publish()

    def _dynamic_callback(self, message: OccupancyGrid) -> None:
        try:
            grid = message_to_grid(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        if self.static_grid is not None and not self._same_geometry(self.static_grid, grid):
            self.get_logger().error('dynamic grid geometry가 static grid와 다릅니다.')
            return
        changed = self.dynamic_grid is None or not np.array_equal(
            self.dynamic_grid.data, grid.data
        )
        self.dynamic_grid = grid
        if changed:
            self._dirty = True
            self._combine_and_publish()

    def _thermal_callback(self, message: OccupancyGrid) -> None:
        try:
            grid = message_to_grid(message)
        except ValueError as exc:
            self.get_logger().error(f'thermal grid 오류: {exc}')
            return
        if np.any((grid.data < 0) | (grid.data > 100)):
            self.get_logger().error('thermal grid 값은 0~100이어야 합니다.')
            return
        if self.static_grid is not None and not self._same_geometry(
            self.static_grid, grid
        ):
            self.thermal_geometry_mismatch = True
            self._dirty = True
            self._state('THERMAL_GRID_MISMATCH')
            self._publish_empty_path()
            self.get_logger().error('thermal grid geometry가 static grid와 다릅니다.')
            return
        changed = self.thermal_grid is None or not np.array_equal(
            self.thermal_grid.data, grid.data
        )
        self.thermal_grid = grid
        self.thermal_received_ns = self.get_clock().now().nanoseconds
        self.thermal_geometry_mismatch = False
        if changed:
            self._dirty = True
            self._combine_and_publish()

    def _thermal_status_callback(self, message: String) -> None:
        changed = message.data != self.thermal_status
        self.thermal_status = message.data
        if changed:
            self._dirty = True
        if self.require_thermal_active and message.data != 'ACTIVE':
            self._state('WAITING_FOR_THERMAL_ACTIVE')
            self._publish_empty_path()

    def _combine_and_publish(self) -> None:
        if self.static_grid is None:
            return
        try:
            combined = combine_cost_grids(
                self.static_grid.data,
                None if self.dynamic_grid is None else self.dynamic_grid.data,
                (
                    None
                    if self.thermal_grid is None or self.thermal_geometry_mismatch
                    else self.thermal_grid.data
                ),
                unknown_is_occupied=self.unknown_is_occupied,
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        planning_source = combined.copy()
        if self.unknown_is_occupied:
            planning_source[planning_source < 0] = 100
        radius_cells = int(math.ceil(
            self.clearance_radius / self.static_grid.resolution
        ))
        planning_source = inflate_occupied_cells(planning_source, radius_cells)
        self.combined_grid = MapGrid(
            width=self.static_grid.width,
            height=self.static_grid.height,
            resolution=self.static_grid.resolution,
            origin_x=self.static_grid.origin_x,
            origin_y=self.static_grid.origin_y,
            origin_yaw=self.static_grid.origin_yaw,
            frame_id=self.map_frame,
            data=planning_source,
        )
        self._publish_grid()

    def _publish_grid(self) -> None:
        if self.combined_grid is None:
            return
        stamp = self.get_clock().now().to_msg()
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame
        message.info.map_load_time = stamp
        message.info.resolution = self.combined_grid.resolution
        message.info.width = self.combined_grid.width
        message.info.height = self.combined_grid.height
        message.info.origin.position.x = self.combined_grid.origin_x
        message.info.origin.position.y = self.combined_grid.origin_y
        qx, qy, qz, qw = quaternion_from_yaw(self.combined_grid.origin_yaw)
        message.info.origin.orientation.x = qx
        message.info.origin.orientation.y = qy
        message.info.origin.orientation.z = qz
        message.info.origin.orientation.w = qw
        message.data = self.combined_grid.data.reshape(-1).astype(int).tolist()
        self.grid_publisher.publish(message)

    def _goal_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self._state('INVALID_GOAL_FRAME')
            self.get_logger().error(
                f'goal frame={message.header.frame_id!r}; {self.map_frame!r}만 지원합니다.'
            )
            return
        self.goal = message
        self._dirty = True
        self._plan('NEW_GOAL')

    def _timer_callback(self) -> None:
        if self.goal is None:
            return
        # Periodic replanning also corrects for robot motion, even without grid changes.
        reason = 'GRID_UPDATE' if self._dirty else 'PERIODIC'
        self._plan(reason)

    def _thermal_failure(self) -> Optional[str]:
        age_sec = None
        if self.thermal_received_ns is not None:
            age_sec = max(
                0.0,
                (self.get_clock().now().nanoseconds - self.thermal_received_ns)
                / 1_000_000_000.0,
            )
        return thermal_readiness_state(
            require_grid=self.require_thermal_grid,
            require_active=self.require_thermal_active,
            grid_available=self.thermal_grid is not None,
            geometry_matches=not self.thermal_geometry_mismatch,
            status=self.thermal_status,
            age_sec=age_sec,
            timeout_sec=self.thermal_timeout,
        )

    def _plan(self, reason: str) -> None:
        if self.goal is None or self.combined_grid is None:
            self._state('WAITING_FOR_GRID')
            return
        thermal_failure = self._thermal_failure()
        if thermal_failure is not None:
            self.current_path_cells = []
            self.current_simplified_cells = []
            self.current_path_total_cost = math.inf
            self._state(thermal_failure)
            self._publish_empty_path()
            return
        pose = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if pose is None:
            self._state('WAITING_FOR_TF')
            self._publish_empty_path()
            return
        start = world_to_grid(pose[0], pose[1], self.combined_grid)
        goal = world_to_grid(
            self.goal.pose.position.x,
            self.goal.pose.position.y,
            self.combined_grid,
        )
        if not is_inside_grid(*start, self.combined_grid) or not is_inside_grid(
            *goal, self.combined_grid
        ):
            self._state('NO_PATH')
            self.get_logger().error(f'start={start} 또는 goal={goal}이 지도 밖입니다.')
            self._publish_empty_path()
            return
        if reason == 'GRID_UPDATE' and self.current_path_cells:
            if path_cells_collision(
                self.current_path_cells,
                self.combined_grid.data,
                self.unknown_is_occupied,
            ):
                self._state('REPLANNING')
        else:
            self._state('PLANNING')
        # Inflation can mark the cell occupied by the physical robot itself.
        # Clear only its already-occupied local footprint so A* can safely leave
        # the start, while all cells beyond this small disk remain protected.
        planning_data = self.combined_grid.data.copy()
        start_radius_cells = int(math.ceil(
            self.start_clearance_radius / self.combined_grid.resolution
        ))
        for dy in range(-start_radius_cells, start_radius_cells + 1):
            for dx in range(-start_radius_cells, start_radius_cells + 1):
                if math.hypot(dx, dy) > start_radius_cells:
                    continue
                sx, sy = start[0] + dx, start[1] + dy
                if 0 <= sy < planning_data.shape[0] and 0 <= sx < planning_data.shape[1]:
                    planning_data[sy, sx] = 0
        result = weighted_astar_search(
            planning_data,
            start,
            goal,
            unknown_is_occupied=self.unknown_is_occupied,
            allow_diagonal=self.allow_diagonal,
            thermal_cost_weight=self.thermal_cost_weight,
            thermal_cost_power=self.thermal_cost_power,
        )
        path = list(result.path)
        if not path:
            self.current_path_cells = []
            self.current_simplified_cells = []
            self.current_path_total_cost = math.inf
            self._state('NO_PATH')
            self._publish_empty_path()
            self.get_logger().error(f'A* 경로 없음: start={start}, goal={goal}')
            return
        simplification = simplify_path_safely(
            path,
            planning_data,
            unknown_is_occupied=self.unknown_is_occupied,
            thermal_cost_weight=self.thermal_cost_weight,
            thermal_cost_power=self.thermal_cost_power,
            maximum_risk_ratio=self.simplification_maximum_risk_ratio,
            risk_absolute_tolerance=self.simplification_risk_absolute_tolerance,
        )
        if not simplification.safe or not simplification.path:
            self.current_path_cells = []
            self.current_simplified_cells = []
            self.current_path_total_cost = math.inf
            self._state('NO_SAFE_PATH')
            self._publish_empty_path()
            self.get_logger().error('thermal-aware 경로 단순화 안전 검증 실패')
            return
        simplified = list(simplification.path)
        self.current_path_cells = path
        self.current_simplified_cells = simplified
        self.current_path_total_cost = result.total_cost
        self._dirty = False
        self._last_plan = self.get_clock().now().nanoseconds / 1_000_000_000.0
        self._publish_path(simplified)
        self._state('PATH_READY')
        self.get_logger().debug(
            f'weighted A* {reason}: raw={len(path)}, simplified={len(simplified)}, '
            f'cost={result.total_cost:.3f}, rejected_shortcuts='
            f'{simplification.rejected_shortcuts}'
        )

    def _publish_path(self, cells: List[Cell]) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        for index, cell in enumerate(cells):
            x, y = grid_to_world(cell[0], cell[1], self.combined_grid)
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if index + 1 < len(cells):
                next_x, next_y = grid_to_world(
                    cells[index + 1][0], cells[index + 1][1], self.combined_grid
                )
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                yaw = yaw_from_quaternion(self.goal.pose.orientation)
                pose.pose.position.x = self.goal.pose.position.x
                pose.pose.position.y = self.goal.pose.position.y
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            message.poses.append(pose)
        self.path_publisher.publish(message)

    def _publish_empty_path(self) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        self.path_publisher.publish(message)

    def _state(self, text: str) -> None:
        self.state_publisher.publish(String(data=text))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AstarReplanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f'astar_replanner 오류: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
