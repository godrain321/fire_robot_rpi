"""Merge static/dynamic grids and continuously replan an 8-connected A* path."""

from __future__ import annotations

import math
import json
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Int32, String
from std_msgs.msg import Float32MultiArray

from .grid_utils import (
    MapGrid,
    grid_to_world,
    inflate_occupied_cells,
    is_inside_grid,
    path_cells_collision,
    quaternion_from_yaw,
    world_to_grid,
    yaw_from_quaternion,
)
from .tf_utils import TfHelper
from .safe_path_simplifier import simplify_path_safely
from .reference_waypoint_graph import (
    PlanningGridGeometry,
    ReferenceWaypoint,
    ReferenceWaypointGraphConfig,
    ReferenceWaypointGraphPlanner,
)
from .waypoint_selection import (
    load_waypoint_document,
    named_waypoints_from_document,
)
from .weighted_planner import (
    combine_cost_grids,
    thermal_readiness_state,
    weighted_a_star_with_escape,
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
            'periodic_replanning_enabled': False,
            'replan_on_thermal_update': True,
            'replan_on_dynamic_update': True,
            'path_replan_thermal_threshold': 60,
            'goal_duplicate_tolerance_m': 0.01,
            # Stage 8-8: defaults to the original topic so Stage 1-7 behavior is
            # byte-identical when the waypoint planning pipeline is disabled.
            # The launch file overrides this to /astar_path only when
            # waypoint_planning_enabled is true, at which point PathSelector
            # (not astar_replanner) owns /planned_path.
            'path_output_topic': '/planned_path',
            'accept_goal_pose': True,
            'ignore_dynamic_modes': [2],
            'direct_planning_modes': [3, 4],
            'replan_request_topic': '/replanning/astar_request',
            'replan_result_topic': '/replanning/astar_result',
            'path_block_check_radius': 0.20,
            'start_clearance_radius': 0.18,
            'allow_diagonal': True,
            'thermal_grid_topic': '/thermal_cost_grid',
            'thermal_status_topic': '/thermal_cost_status',
            'require_thermal_grid': True,
            'require_thermal_active': True,
            'thermal_grid_timeout_sec': 1.0,
            'thermal_cost_weight': 24.0,
            'thermal_cost_power': 1.5,
            'fixed_co_ppm': 0.0,
            'co_safe_ppm': 0.0,
            'co_blocked_ppm': 1600.0,
            'co_cost_weight': 8.0,
            'co_cost_power': 2.0,
            'simplification_maximum_risk_ratio': 1.0,
            'simplification_risk_absolute_tolerance': 0.0,
            'reference_waypoint_graph_enabled': True,
            'reference_waypoint_file': '',
            'reference_neighbor_radius_m': 1.5,
            'reference_connector_search_radius_m': 3.0,
            'reference_connector_candidate_count': 8,
            'reference_fallback_to_cell_astar': True,
            'reference_waypoint_cost_radius_m': 0.10,
            'reference_waypoint_risk_weight': 1.0,
            'hazard_belief_enabled': False,
            'hazard_final_cost_topic': '/hazard/final_cost',
            'hazard_status_topic': '/hazard/status',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.unknown_is_occupied = bool(
            self.get_parameter('unknown_is_occupied').value
        )
        self.replan_rate = float(self.get_parameter('replan_rate_hz').value)
        self.periodic_replanning_enabled = bool(
            self.get_parameter('periodic_replanning_enabled').value
        )
        self.replan_on_thermal_update = bool(
            self.get_parameter('replan_on_thermal_update').value
        )
        self.replan_on_dynamic_update = bool(
            self.get_parameter('replan_on_dynamic_update').value
        )
        self.path_replan_thermal_threshold = int(
            self.get_parameter('path_replan_thermal_threshold').value
        )
        self.goal_duplicate_tolerance = float(
            self.get_parameter('goal_duplicate_tolerance_m').value
        )
        self.clearance_radius = float(
            self.get_parameter('path_block_check_radius').value
        )
        self.start_clearance_radius = float(
            self.get_parameter('start_clearance_radius').value
        )
        self.allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        self.ignore_dynamic_modes = {
            int(value) for value in self.get_parameter('ignore_dynamic_modes').value
        }
        self.direct_planning_modes = {
            int(value) for value in self.get_parameter('direct_planning_modes').value
        }
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
        self.fixed_co_ppm = float(self.get_parameter('fixed_co_ppm').value)
        self.co_safe_ppm = float(self.get_parameter('co_safe_ppm').value)
        self.co_blocked_ppm = float(
            self.get_parameter('co_blocked_ppm').value
        )
        self.co_cost_weight = float(
            self.get_parameter('co_cost_weight').value
        )
        self.co_cost_power = float(
            self.get_parameter('co_cost_power').value
        )
        self.simplification_maximum_risk_ratio = float(
            self.get_parameter('simplification_maximum_risk_ratio').value
        )
        self.simplification_risk_absolute_tolerance = float(
            self.get_parameter('simplification_risk_absolute_tolerance').value
        )
        self.reference_waypoint_file = str(
            self.get_parameter('reference_waypoint_file').value
        ).strip()
        self.hazard_belief_enabled = bool(
            self.get_parameter('hazard_belief_enabled').value
        )
        self.hazard_final_cost_topic = str(
            self.get_parameter('hazard_final_cost_topic').value
        )
        self.hazard_status_topic = str(
            self.get_parameter('hazard_status_topic').value
        )
        reference_config = ReferenceWaypointGraphConfig(
            enabled=bool(self.get_parameter(
                'reference_waypoint_graph_enabled'
            ).value),
            neighbor_radius_m=float(self.get_parameter(
                'reference_neighbor_radius_m'
            ).value),
            connector_search_radius_m=float(self.get_parameter(
                'reference_connector_search_radius_m'
            ).value),
            connector_candidate_count=int(self.get_parameter(
                'reference_connector_candidate_count'
            ).value),
            fallback_to_cell_astar=bool(self.get_parameter(
                'reference_fallback_to_cell_astar'
            ).value),
            waypoint_cost_radius_m=float(self.get_parameter(
                'reference_waypoint_cost_radius_m'
            ).value),
            waypoint_risk_weight=float(self.get_parameter(
                'reference_waypoint_risk_weight'
            ).value),
        )
        numeric_parameters = (
            self.replan_rate, self.clearance_radius, self.start_clearance_radius,
            self.thermal_timeout, self.thermal_cost_weight,
            self.thermal_cost_power, self.fixed_co_ppm, self.co_safe_ppm,
            self.co_blocked_ppm, self.co_cost_weight, self.co_cost_power,
            self.simplification_maximum_risk_ratio,
            self.simplification_risk_absolute_tolerance,
            self.goal_duplicate_tolerance,
        )
        if (not all(math.isfinite(value) for value in numeric_parameters)
                or self.replan_rate <= 0.0 or self.clearance_radius < 0.0
                or self.start_clearance_radius < 0.0
                or self.thermal_timeout < 0.0
                or self.thermal_cost_weight < 0.0
                or self.thermal_cost_power <= 0.0
                or self.fixed_co_ppm < 0.0
                or self.co_blocked_ppm <= self.co_safe_ppm
                or self.co_cost_weight < 0.0
                or self.co_cost_power <= 0.0
                or self.simplification_maximum_risk_ratio < 1.0
                or self.simplification_risk_absolute_tolerance < 0.0
                or self.goal_duplicate_tolerance < 0.0
                or not 0 <= self.path_replan_thermal_threshold <= 100):
            raise ValueError(
                'rate/power는 양수, radius/timeout/weight/CO는 0 이상, '
                'co_blocked_ppm은 co_safe_ppm보다 커야 하고, '
                'simplification risk ratio는 1 이상이어야 합니다.'
            )

        reference_waypoints = ()
        if reference_config.enabled:
            if not self.reference_waypoint_file:
                raise ValueError(
                    'reference waypoint graph가 활성화됐지만 '
                    'reference_waypoint_file이 비어 있습니다.'
                )
            document = load_waypoint_document(self.reference_waypoint_file)
            records = named_waypoints_from_document(document, self.map_frame)
            reference_waypoints = tuple(
                ReferenceWaypoint(
                    item.name, item.x, item.y, item.yaw
                ) for item in records
            )
        self.reference_graph_planner = ReferenceWaypointGraphPlanner(
            reference_waypoints, reference_config
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
        self.hazard_grid: Optional[MapGrid] = None
        self.hazard_status = ''
        self.goal: Optional[PoseStamped] = None
        self.current_path_cells: List[Cell] = []
        self.current_simplified_cells: List[Cell] = []
        self.current_path_total_cost = math.inf
        self._dirty = False
        self._replan_requested = False
        self._replan_reason = ''
        self._planning = False
        self._last_plan = 0.0
        self._last_published_path_stamp_ns: Optional[int] = None
        self._last_emitted_path_stamp_ns = -1
        self.drive_mode = 1
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
        self.create_subscription(
            Float32MultiArray, self.hazard_final_cost_topic,
            self._hazard_cost_callback, qos,
        )
        self.create_subscription(
            String, self.hazard_status_topic, self._hazard_status_callback, qos
        )
        if bool(self.get_parameter('accept_goal_pose').value):
            self.create_subscription(PoseStamped, '/goal_pose', self._goal_callback, 10)
        self.create_subscription(
            String, str(self.get_parameter('replan_request_topic').value),
            self._replan_request_callback, 10,
        )
        self.create_subscription(
            Empty, '/autonomy_cancel', self._cancel_callback, 10
        )
        self.create_subscription(Int32, '/drive_mode', self._mode_callback, 10)
        self.grid_publisher = self.create_publisher(
            OccupancyGrid, '/planning_grid', qos
        )
        self.path_publisher = self.create_publisher(
            Path, str(self.get_parameter('path_output_topic').value), qos
        )
        self.state_publisher = self.create_publisher(String, '/planner_state', 10)
        self.replan_result_publisher = self.create_publisher(
            String, str(self.get_parameter('replan_result_topic').value), 10,
        )
        self.create_timer(1.0 / self.replan_rate, self._timer_callback)
        if self.periodic_replanning_enabled:
            self.get_logger().info(
                f'Periodic replanning: ENABLED ({self.replan_rate:.2f} Hz)'
            )
        else:
            self.get_logger().info(
                'Periodic replanning: DISABLED. External event-driven '
                'replanning (replan_supervisor_node) owns replan decisions; '
                '/goal_pose still triggers immediate planning.'
            )
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
        if changed:
            self.hazard_grid = None
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
        previous = self.dynamic_grid
        changed = previous is None or not np.array_equal(previous.data, grid.data)
        self.dynamic_grid = grid
        if changed and self.drive_mode not in self.ignore_dynamic_modes:
            self._dirty = True
            self._combine_and_publish()
            if self.replan_on_dynamic_update and self._new_dynamic_block_on_path(
                previous, grid
            ):
                self._request_replan('DYNAMIC_PATH_BLOCKED')

    def _mode_callback(self, message: Int32) -> None:
        mode = int(message.data)
        if mode == self.drive_mode:
            return
        self.drive_mode = mode
        if self.static_grid is not None:
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
        previous = self.thermal_grid
        changed = previous is None or not np.array_equal(previous.data, grid.data)
        self.thermal_grid = grid
        self.thermal_received_ns = self.get_clock().now().nanoseconds
        self.thermal_geometry_mismatch = False
        if changed:
            self._dirty = True
            self._combine_and_publish()
            if self.replan_on_thermal_update and self._thermal_path_risk_increased(
                previous, grid
            ):
                self._request_replan('THERMAL_PATH_RISK_INCREASED')

    def _thermal_status_callback(self, message: String) -> None:
        if self.hazard_belief_enabled:
            return
        changed = message.data != self.thermal_status
        self.thermal_status = message.data
        if changed:
            self._dirty = True
        if self.require_thermal_active and message.data != 'ACTIVE':
            if message.data == 'THERMAL_DATA_STALE':
                self._state('THERMAL_GRID_STALE')
            else:
                self._state('WAITING_FOR_THERMAL_ACTIVE')
            self._publish_empty_path()

    def _hazard_status_callback(self, message: String) -> None:
        if message.data != self.hazard_status:
            self.hazard_status = message.data
            self._dirty = True

    def _hazard_cost_callback(self, message: Float32MultiArray) -> None:
        if self.static_grid is None:
            self._state('WAITING_FOR_HAZARD_STATIC_GRID')
            return
        dimensions = {item.label: int(item.size) for item in message.layout.dim}
        if dimensions.get('height') != self.static_grid.height or dimensions.get(
            'width'
        ) != self.static_grid.width:
            self._state('HAZARD_GRID_MISMATCH')
            self.get_logger().error('hazard float grid geometry가 static grid와 다릅니다.')
            return
        values = np.asarray(message.data, dtype=float)
        if values.size != self.static_grid.width * self.static_grid.height:
            self._state('HAZARD_GRID_MISMATCH')
            return
        data = values.reshape(self.static_grid.height, self.static_grid.width)
        if np.any(np.isnan(data)) or np.any(data[np.isfinite(data)] <= 0.0):
            self._state('INVALID_HAZARD_GRID')
            return
        self.hazard_grid = MapGrid(
            self.static_grid.width, self.static_grid.height,
            self.static_grid.resolution, self.static_grid.origin_x,
            self.static_grid.origin_y, self.static_grid.origin_yaw,
            self.static_grid.frame_id, data,
        )
        self._dirty = True

    def _combine_and_publish(self) -> None:
        if self.static_grid is None:
            return
        try:
            combined = combine_cost_grids(
                self.static_grid.data,
                (
                    None
                    if self.dynamic_grid is None
                    or self.drive_mode in self.ignore_dynamic_modes
                    else self.dynamic_grid.data
                ),
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
        if self._same_goal(self.goal, message):
            if not (self._replan_requested or self._dirty):
                self.get_logger().debug('동일한 /goal_pose 반복 수신을 무시합니다.')
                return
            # A supervisor may republish the active goal to request a replan.
            # Queue it instead of bypassing the same rate limiter used by grid
            # events; the supervisor has already asserted follower hold.
            self._request_replan('SAME_GOAL_EVENT')
            return
        self.goal = message
        self._dirty = True
        self._plan('NEW_GOAL')

    def _same_goal(
        self, first: Optional[PoseStamped], second: PoseStamped
    ) -> bool:
        if first is None:
            return False
        return math.hypot(
            first.pose.position.x - second.pose.position.x,
            first.pose.position.y - second.pose.position.y,
        ) <= self.goal_duplicate_tolerance

    def _remaining_path_cells(self, grid: MapGrid) -> List[Cell]:
        cells = list(self.current_path_cells)
        tf = getattr(self, 'tf', None)
        if not cells or tf is None:
            return cells
        pose = tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if pose is None:
            return cells
        robot = world_to_grid(pose[0], pose[1], grid)
        nearest = min(
            range(len(cells)),
            key=lambda index: (
                (cells[index][0] - robot[0]) ** 2
                + (cells[index][1] - robot[1]) ** 2
            ),
        )
        return cells[nearest:]

    def _path_values(
        self, grid: Optional[MapGrid], path_cells: Optional[List[Cell]] = None
    ) -> np.ndarray:
        if grid is None or not self.current_path_cells:
            return np.asarray([], dtype=float)
        values = []
        for col, row in (
            self.current_path_cells if path_cells is None else path_cells
        ):
            if 0 <= col < grid.width and 0 <= row < grid.height:
                values.append(float(grid.data[row, col]))
        return np.asarray(values, dtype=float)

    def _new_dynamic_block_on_path(
        self, previous: Optional[MapGrid], current: MapGrid
    ) -> bool:
        cells = self._remaining_path_cells(current)
        current_values = self._path_values(current, cells)
        if current_values.size == 0:
            return False
        previous_values = self._path_values(previous, cells)
        if previous_values.size != current_values.size:
            return bool(np.any(current_values >= 100.0))
        return bool(np.any((current_values >= 100.0) & (previous_values < 100.0)))

    def _thermal_path_risk_increased(
        self, previous: Optional[MapGrid], current: MapGrid
    ) -> bool:
        cells = self._remaining_path_cells(current)
        current_values = self._path_values(current, cells)
        if current_values.size == 0:
            return False
        previous_values = self._path_values(previous, cells)
        threshold = float(self.path_replan_thermal_threshold)
        if previous_values.size != current_values.size:
            return bool(np.any(current_values >= threshold))
        newly_blocked = (current_values >= 100.0) & (previous_values < 100.0)
        crossed_threshold = (
            (current_values >= threshold)
            & (previous_values < threshold)
            & (current_values > previous_values)
        )
        return bool(np.any(newly_blocked | crossed_threshold))

    def _request_replan(self, reason: str) -> None:
        if self.goal is None:
            return
        self._replan_requested = True
        self._replan_reason = reason

    def _replan_request_callback(self, message: String) -> None:
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict):
                return
            goal_world = request['goal_world']
            goal_x, goal_y = float(goal_world[0]), float(goal_world[1])
        except (TypeError, ValueError, KeyError, IndexError):
            return
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.w = 1.0
        self.goal = goal
        self._last_published_path_stamp_ns = None
        success = bool(self._plan('SAME_EXIT_FALLBACK'))
        result = dict(request)
        result.update(
            success=success,
            status="PATH_FOUND" if success else "NO_PATH",
            path_stamp_ns=self._last_published_path_stamp_ns,
        )
        self.replan_result_publisher.publish(String(data=json.dumps(result, sort_keys=True)))

    def _cancel_callback(self, _message: Empty) -> None:
        self.goal = None
        self.current_path_cells = []
        self._dirty = False
        self._replan_requested = False
        self._replan_reason = ''
        self._publish_empty_path()
        self._state('CANCELLED')

    def _timer_callback(self) -> None:
        if self.goal is None:
            return
        if self._planning:
            return
        if self._replan_requested:
            now = self.get_clock().now().nanoseconds / 1_000_000_000.0
            if now - self._last_plan < 1.0 / self.replan_rate:
                return
            reason = self._replan_reason or 'GRID_UPDATE'
            # Consume the coalesced request before planning. A failed plan
            # publishes an empty path (STOP) and waits for a genuinely new
            # grid/goal event; supervisor-owned operation retains its existing
            # bounded retry/backoff state machine.
            self._replan_requested = False
            self._replan_reason = ''
            self._plan(reason)
            return
        if self.periodic_replanning_enabled:
            self._plan('PERIODIC')

    def _thermal_failure(self) -> Optional[str]:
        if self.hazard_belief_enabled:
            if self.hazard_grid is None:
                return 'WAITING_FOR_HAZARD'
            if self.hazard_status not in (
                'ACTIVE', 'ACTIVE_THERMAL_ONLY',
                'ACTIVE_STATIC_DYNAMIC_ONLY',
            ):
                return 'HAZARD_NOT_READY:' + (self.hazard_status or 'NO_STATUS')
            return None
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

    def _plan(self, reason: str) -> bool:
        if self._planning:
            return False
        self._planning = True
        try:
            return self._plan_once(reason)
        finally:
            self._planning = False

    def _plan_once(self, reason: str) -> bool:
        planning_grid = (
            self.hazard_grid if self.hazard_belief_enabled
            else self.combined_grid
        )
        if self.goal is None or planning_grid is None:
            self._state('WAITING_FOR_GRID')
            return False
        thermal_failure = self._thermal_failure()
        if thermal_failure is not None:
            self.current_path_cells = []
            self.current_simplified_cells = []
            self.current_path_total_cost = math.inf
            self._state(thermal_failure)
            self._publish_empty_path()
            return False
        pose = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if pose is None:
            self._state('WAITING_FOR_TF')
            self._publish_empty_path()
            return False
        start = world_to_grid(pose[0], pose[1], planning_grid)
        goal = world_to_grid(
            self.goal.pose.position.x,
            self.goal.pose.position.y,
            planning_grid,
        )
        if not is_inside_grid(*start, planning_grid) or not is_inside_grid(
            *goal, planning_grid
        ):
            self._state('NO_PATH')
            self.get_logger().error(f'start={start} 또는 goal={goal}이 지도 밖입니다.')
            self._publish_empty_path()
            return False
        if reason not in ('NEW_GOAL', 'PERIODIC') and self.current_path_cells:
            if path_cells_collision(
                self.current_path_cells,
                planning_grid.data,
                self.unknown_is_occupied,
            ):
                self._state('REPLANNING')
        else:
            self._state('PLANNING')
        planning_data = planning_grid.data.copy()
        # Escape may cross thermal/dynamic/inflation cost around the robot, but
        # never a physical static-map obstacle. This is factory_v5's rule and
        # replaces the previous mutation that cleared start-neighbour costs.
        static_obstacles = self.static_grid.data >= 100
        geometry = PlanningGridGeometry(
            resolution=planning_grid.resolution,
            origin_x=planning_grid.origin_x,
            origin_y=planning_grid.origin_y,
            origin_yaw=planning_grid.origin_yaw,
            frame_id=planning_grid.frame_id,
        )
        planner_options = dict(
            unknown_is_occupied=self.unknown_is_occupied,
            allow_diagonal=self.allow_diagonal,
            thermal_cost_weight=self.thermal_cost_weight,
            thermal_cost_power=self.thermal_cost_power,
            fixed_co_ppm=self.fixed_co_ppm,
            co_safe_ppm=self.co_safe_ppm,
            co_blocked_ppm=self.co_blocked_ppm,
            co_cost_weight=self.co_cost_weight,
            co_cost_power=self.co_cost_power,
            costs_are_traversal=self.hazard_belief_enabled,
        )
        if self.drive_mode in self.direct_planning_modes:
            # Inspection goals are normally only a short distance in front of
            # the robot.  Routing them through a nearby reference waypoint can
            # put the first target behind the robot and cause a needless turn.
            result = weighted_a_star_with_escape(
                planning_data,
                start,
                goal,
                static_obstacles,
                **planner_options,
            )
            route_source = 'DIRECT_CELL_ASTAR'
        else:
            result = self.reference_graph_planner.plan(
                planning_data,
                start,
                goal,
                geometry,
                static_obstacles,
                waypoint_frame_id=self.map_frame,
                **planner_options,
            )
            route_source = (
                'REFERENCE_WAYPOINT_GRAPH'
                if result.used_reference_graph else 'CELL_ASTAR_FALLBACK'
            )
        path = list(result.path)
        if not path:
            self.current_path_cells = []
            self.current_simplified_cells = []
            self.current_path_total_cost = math.inf
            self._state('NO_PATH')
            self._publish_empty_path()
            self.get_logger().error(f'A* 경로 없음: start={start}, goal={goal}')
            return False
        # The escape prefix intentionally contains cells blocked by the current
        # risk map, so preserve it exactly and simplify only the finite-cost
        # weighted-A* suffix.
        escape_prefix = list(result.escape_path[:-1])
        simplification_source = path[len(escape_prefix):]
        simplification = simplify_path_safely(
            simplification_source,
            planning_data,
            unknown_is_occupied=self.unknown_is_occupied,
            thermal_cost_weight=self.thermal_cost_weight,
            thermal_cost_power=self.thermal_cost_power,
            fixed_co_ppm=self.fixed_co_ppm,
            co_safe_ppm=self.co_safe_ppm,
            co_blocked_ppm=self.co_blocked_ppm,
            co_cost_weight=self.co_cost_weight,
            co_cost_power=self.co_cost_power,
            maximum_risk_ratio=self.simplification_maximum_risk_ratio,
            risk_absolute_tolerance=self.simplification_risk_absolute_tolerance,
            costs_are_traversal=self.hazard_belief_enabled,
        )
        if not simplification.safe or not simplification.path:
            self.current_path_cells = []
            self.current_simplified_cells = []
            self.current_path_total_cost = math.inf
            self._state('NO_SAFE_PATH')
            self._publish_empty_path()
            self.get_logger().error('thermal-aware 경로 단순화 안전 검증 실패')
            return False
        simplified = escape_prefix + list(simplification.path)
        self.current_path_cells = path
        self.current_simplified_cells = simplified
        self.current_path_total_cost = result.total_cost
        self._dirty = False
        self._replan_requested = False
        self._replan_reason = ''
        self._last_plan = self.get_clock().now().nanoseconds / 1_000_000_000.0
        self._publish_path(simplified)
        self._state('PATH_READY')
        if reason == 'NEW_GOAL':
            self.get_logger().info(
                f'PATH_SELECTED: mode={self.drive_mode}, source={route_source}, '
                f'start={start}, goal={goal}, poses={len(simplified)}'
            )
        self.get_logger().debug(
            f'planner {reason}: raw={len(path)}, simplified={len(simplified)}, '
            f'cost={result.total_cost:.3f}, rejected_shortcuts='
            f'{simplification.rejected_shortcuts}, reference_graph='
            f'{result.used_reference_graph}, anchors='
            f'{result.reference_waypoint_ids}'
        )
        return True

    def _publish_path(self, cells: List[Cell]) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        clock_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        stamp_ns = max(clock_stamp_ns, getattr(self, '_last_emitted_path_stamp_ns', -1) + 1)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(
            stamp_ns, 1_000_000_000
        )
        self._last_emitted_path_stamp_ns = stamp_ns
        self._last_published_path_stamp_ns = stamp_ns
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
