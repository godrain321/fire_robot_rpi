"""Approach a LiDAR obstacle and classify mmWave presence in drive mode 3."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
import time
from typing import Iterable, Optional, Sequence, Tuple

from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float32, Int32, String, UInt64

from .astar_replanner import message_to_grid
from .grid_utils import (
    MapGrid,
    grid_to_world,
    is_inside_grid,
    quaternion_from_yaw,
    world_to_grid,
)
from .tf_utils import TfHelper
from .evacuation_demo import stationary_observation_displacement


Point2D = Tuple[float, float]


def parse_inspection_command(value: str) -> tuple[bool, Optional[Point2D]]:
    """Parse manual nearest-target or Mode 5 explicit-target commands."""
    command = str(value).strip()
    if command.upper() == 'MODE3_START':
        return True, None
    prefix = 'MODE3_START_AT:'
    if not command.upper().startswith(prefix):
        return False, None
    try:
        point = tuple(float(item) for item in command[len(prefix):].split(','))
    except ValueError:
        return False, None
    if len(point) != 2 or not all(math.isfinite(item) for item in point):
        return False, None
    return True, (point[0], point[1])


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


def select_tracked_candidate(
    previous: Point2D,
    candidates: Iterable[Point2D],
    maximum_match_distance_m: float,
) -> Optional[Point2D]:
    """Associate the latest LiDAR centroid with the currently inspected one."""
    maximum = float(maximum_match_distance_m)
    if not math.isfinite(maximum) or maximum <= 0.0:
        raise ValueError('target tracking distance must be positive')
    nearest = select_nearest_candidate(previous[0], previous[1], candidates)
    if nearest is None or math.dist(previous, nearest) > maximum:
        return None
    return nearest


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


def _same_grid_geometry(first: MapGrid, second: MapGrid) -> bool:
    return (
        first.width == second.width
        and first.height == second.height
        and abs(first.resolution - second.resolution) < 1e-9
        and abs(first.origin_x - second.origin_x) < 1e-6
        and abs(first.origin_y - second.origin_y) < 1e-6
        and abs(first.origin_yaw - second.origin_yaw) < 1e-9
        and first.frame_id.lstrip('/') == second.frame_id.lstrip('/')
    )


def _reachable_safe_cells(
    planning_grid: MapGrid,
    static_grid: MapGrid,
    start: tuple[int, int],
    *,
    unknown_is_occupied: bool,
    allow_diagonal: bool,
) -> np.ndarray:
    """Return safe cells reachable under the replanner's start-escape rules."""
    if not _same_grid_geometry(planning_grid, static_grid):
        raise ValueError('inspection planning/static grid geometry differs')
    costs = np.asarray(planning_grid.data, dtype=float)
    static = np.asarray(static_grid.data)
    blocked = ~np.isfinite(costs) | (costs >= 100.0)
    if unknown_is_occupied:
        blocked |= costs < 0.0
    static_obstacles = static >= 100
    reachable = np.zeros(costs.shape, dtype=bool)
    if not is_inside_grid(*start, planning_grid):
        return reachable
    start_x, start_y = start
    if static_obstacles[start_y, start_x]:
        return reachable

    # AstarReplanner permits a robot already caught in inflation/dynamic cost
    # to escape through any non-physical-static cell. Otherwise normal A*
    # connectivity is restricted to finite cells.
    allowed = ~static_obstacles if blocked[start_y, start_x] else ~blocked
    moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    if allow_diagonal:
        moves += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    queue = deque([(start_x, start_y)])
    reachable[start_y, start_x] = True
    while queue:
        x, y = queue.popleft()
        for dx, dy in moves:
            next_x, next_y = x + dx, y + dy
            if (
                not (0 <= next_x < planning_grid.width)
                or not (0 <= next_y < planning_grid.height)
                or reachable[next_y, next_x]
                or not allowed[next_y, next_x]
            ):
                continue
            if dx and dy and (
                not allowed[y, next_x] or not allowed[next_y, x]
            ):
                continue
            reachable[next_y, next_x] = True
            queue.append((next_x, next_y))
    return reachable & ~blocked


def select_reachable_inspection_goal(
    robot_world: Point2D,
    target_world: Point2D,
    preferred_goal_world: Point2D,
    standoff_distance_m: float,
    tolerance_m: float,
    planning_grid: MapGrid,
    static_grid: MapGrid,
    *,
    minimum_goal_distance_m: float = 0.0,
    unknown_is_occupied: bool = True,
    allow_diagonal: bool = True,
) -> Optional[Tuple[float, float, float]]:
    """Choose the nearest reachable safe goal on the standoff annulus."""
    desired = float(standoff_distance_m)
    tolerance = float(tolerance_m)
    if not math.isfinite(desired) or desired <= 0.0:
        raise ValueError('inspection standoff distance must be positive')
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError('inspection standoff tolerance must be non-negative')
    minimum_goal_distance = float(minimum_goal_distance_m)
    if (
        not math.isfinite(minimum_goal_distance)
        or minimum_goal_distance < 0.0
    ):
        raise ValueError('minimum inspection goal distance must be non-negative')
    start = world_to_grid(robot_world[0], robot_world[1], planning_grid)
    safe_reachable = _reachable_safe_cells(
        planning_grid,
        static_grid,
        start,
        unknown_is_occupied=unknown_is_occupied,
        allow_diagonal=allow_diagonal,
    )
    inner = max(0.0, desired - tolerance)
    outer = desired + tolerance
    epsilon = 1e-9

    preferred_cell = world_to_grid(
        preferred_goal_world[0], preferred_goal_world[1], planning_grid
    )
    if (
        is_inside_grid(*preferred_cell, planning_grid)
        and safe_reachable[preferred_cell[1], preferred_cell[0]]
        and inner - epsilon
        <= math.dist(preferred_goal_world, target_world)
        <= outer + epsilon
        and math.dist(preferred_goal_world, robot_world)
        >= minimum_goal_distance - epsilon
    ):
        goal_x, goal_y = map(float, preferred_goal_world)
        return (
            goal_x,
            goal_y,
            math.atan2(target_world[1] - goal_y, target_world[0] - goal_x),
        )

    target_cell = world_to_grid(target_world[0], target_world[1], planning_grid)
    radius_cells = int(math.ceil(outer / planning_grid.resolution)) + 2
    candidates = []
    for row in range(
        max(0, target_cell[1] - radius_cells),
        min(planning_grid.height, target_cell[1] + radius_cells + 1),
    ):
        for col in range(
            max(0, target_cell[0] - radius_cells),
            min(planning_grid.width, target_cell[0] + radius_cells + 1),
        ):
            if not safe_reachable[row, col]:
                continue
            world = grid_to_world(col, row, planning_grid)
            standoff = math.dist(world, target_world)
            if not (inner - epsilon <= standoff <= outer + epsilon):
                continue
            robot_distance = math.dist(world, robot_world)
            if robot_distance < minimum_goal_distance - epsilon:
                continue
            candidates.append((
                math.dist(world, preferred_goal_world),
                abs(standoff - desired),
                robot_distance,
                row,
                col,
                world,
            ))
    if not candidates:
        return None
    goal_x, goal_y = min(candidates)[-1]
    return (
        goal_x,
        goal_y,
        math.atan2(target_world[1] - goal_y, target_world[0] - goal_x),
    )


@dataclass
class PresenceEvidence:
    """Count fresh, online mmWave presence samples during observation."""

    total_samples: int = 0
    positive_samples: int = 0

    def add(self, sensor_online: bool, presence: bool) -> None:
        if not sensor_online:
            return
        self.total_samples += 1
        if presence:
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
            'standoff_distance_m': 2.0,
            'standoff_arrival_tolerance_m': 0.20,
            # Use the latest associated LiDAR position. mmWave inspection may
            # start anywhere at or below this live robot-to-target distance.
            'inspection_max_distance_m': 2.50,
            'target_tracking_radius_m': 1.00,
            'target_stale_timeout_sec': 2.00,
            'tracking_candidates_topic': '/dynamic_obstacle_all_candidates',
            'planning_grid_topic': '/planning_grid_active',
            'static_grid_topic': '/planning_grid_static',
            'unknown_is_occupied': True,
            'allow_diagonal': True,
            # If the live target is farther than the inspection range, issue a
            # visible approach path toward the nominal safe standoff.
            'minimum_approach_goal_distance_m': 0.45,
            'robot_settle_sec': 2.0,
            'observation_sec': 3.0,
            'minimum_mmwave_samples': 3,
            'person_positive_samples': 3,
            'sensor_stale_timeout_sec': 2.0,
            'update_rate_hz': 10.0,
            'publish_canonical_plan': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.planning_grid_topic = str(
            self.get_parameter('planning_grid_topic').value
        )
        self.static_grid_topic = str(
            self.get_parameter('static_grid_topic').value
        )
        self.unknown_is_occupied = bool(
            self.get_parameter('unknown_is_occupied').value
        )
        self.allow_diagonal = bool(
            self.get_parameter('allow_diagonal').value
        )
        self.standoff_distance = float(
            self.get_parameter('standoff_distance_m').value
        )
        self.standoff_arrival_tolerance = float(
            self.get_parameter('standoff_arrival_tolerance_m').value
        )
        self.inspection_max_distance = float(
            self.get_parameter('inspection_max_distance_m').value
        )
        self.target_tracking_radius = float(
            self.get_parameter('target_tracking_radius_m').value
        )
        self.target_stale_timeout = float(
            self.get_parameter('target_stale_timeout_sec').value
        )
        self.tracking_candidates_topic = str(
            self.get_parameter('tracking_candidates_topic').value
        )
        self.minimum_approach_goal_distance = float(
            self.get_parameter('minimum_approach_goal_distance_m').value
        )
        self.robot_settle_sec = float(
            self.get_parameter('robot_settle_sec').value
        )
        self.observation_sec = float(
            self.get_parameter('observation_sec').value
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
        self.publish_canonical_plan = bool(
            self.get_parameter('publish_canonical_plan').value
        )
        if (
            self.standoff_distance <= 0.0
            or self.standoff_arrival_tolerance < 0.0
            or self.inspection_max_distance <= 0.0
            or self.target_tracking_radius <= 0.0
            or self.target_stale_timeout <= 0.0
            or self.minimum_approach_goal_distance <= 0.0
            or self.robot_settle_sec < 0.0
            or self.observation_sec <= 0.0
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
        self.tracking_candidates: Sequence[Point2D] = []
        self.target: Optional[Point2D] = None
        self.requested_target: Optional[Point2D] = None
        self.last_candidates_update = float('-inf')
        self.target_last_seen = float('-inf')
        self.target_tracking_started_at = float('-inf')
        self.active_standoff_distance = self.standoff_distance
        self.hazard_revision = 0
        self.waiting_for_departure = False
        self.approach_started = False
        self.phase_deadline = 0.0
        self.sensor_online = False
        self.last_sensor_update = float('-inf')
        self.evidence = PresenceEvidence()
        self.observation_target_start = None
        self.observation_target_samples = []
        self.planning_grid: Optional[MapGrid] = None
        self.static_grid: Optional[MapGrid] = None
        self._grid_wait_reported = False

        self.goal_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        self.cancel_publisher = self.create_publisher(
            Empty, '/autonomy_cancel', 10
        )
        self.person_publisher = self.create_publisher(
            PointStamped, '/dynamic_obstacle_person', 10
        )
        self.assistance_publisher = self.create_publisher(
            PointStamped, '/dynamic_obstacle_assistance', latched_qos
        )
        self.status_publisher = self.create_publisher(
            String, '/mode3_status', latched_qos
        )
        self.classification_publisher = self.create_publisher(
            String, '/mode3_classification', latched_qos
        )
        self.plan_publisher = self.create_publisher(
            String, '/evacuation/plan', latched_qos
        )
        self.create_subscription(Int32, '/drive_mode', self._mode_callback, 10)
        self.create_subscription(
            OccupancyGrid,
            self.planning_grid_topic,
            self._planning_grid_callback,
            latched_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            self.static_grid_topic,
            self._static_grid_callback,
            latched_qos,
        )
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
            PoseArray,
            self.tracking_candidates_topic,
            self._tracking_candidates_callback,
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
            '/mmwave/calibrated_distance_m',
            self._distance_callback,
            latched_qos,
        )
        self.create_subscription(
            Bool,
            '/mmwave/human_presence',
            self._presence_callback,
            latched_qos,
        )
        self.create_subscription(
            UInt64, '/hazard/revision', self._hazard_revision_callback,
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

    def _planning_grid_callback(self, message: OccupancyGrid) -> None:
        try:
            grid = message_to_grid(message)
        except ValueError as error:
            self.get_logger().error(f'MODE 3 planning grid 오류: {error}')
            return
        if grid.frame_id.lstrip('/') != self.map_frame.lstrip('/'):
            return
        self.planning_grid = grid
        self._try_start_inspection()

    def _static_grid_callback(self, message: OccupancyGrid) -> None:
        try:
            grid = message_to_grid(message)
        except ValueError as error:
            self.get_logger().error(f'MODE 3 static grid 오류: {error}')
            return
        if grid.frame_id.lstrip('/') != self.map_frame.lstrip('/'):
            return
        self.static_grid = grid
        self._try_start_inspection()

    def _mode_callback(self, message: Int32) -> None:
        mode = int(message.data)
        previous = self.drive_mode
        self.drive_mode = mode
        if mode == 3 and previous != 3:
            self.phase = 'ARMED'
            self.target = None
            self.requested_target = None
            self.target_last_seen = float('-inf')
            self.target_tracking_started_at = float('-inf')
            self.waiting_for_departure = False
            self.approach_started = False
            self._grid_wait_reported = False
            self._state('MODE3_READY:PRESS_SPACE')
        elif mode != 3 and previous == 3:
            self.cancel_publisher.publish(Empty())
            self.phase = 'IDLE'
            self.target = None
            self.requested_target = None
            self.target_last_seen = float('-inf')
            self.target_tracking_started_at = float('-inf')
            self.waiting_for_departure = False
            self.approach_started = False
            self._state('MODE3_CANCELLED')

    def _inspection_command_callback(self, message: String) -> None:
        accepted, requested_target = parse_inspection_command(message.data)
        if not accepted:
            return
        if self.drive_mode != 3:
            return
        if self.phase in (
            'NAVIGATING', 'WAITING_FOR_LIVE_TARGET', 'SETTLING', 'OBSERVING'
        ):
            self._state(f'MODE3_BUSY:{self.phase}')
            return
        self.cancel_publisher.publish(Empty())
        self.phase = 'WAITING_FOR_OBSTACLE'
        self.target = None
        self.requested_target = requested_target
        # MODE3_START_AT carries a recently confirmed red-candidate position,
        # not proof that the same target exists in the current LiDAR scan.  A
        # match from tracking_candidates_topic is the only event that makes a
        # target position live/fresh.
        self.target_last_seen = float('-inf')
        self.target_tracking_started_at = self._now()
        self.waiting_for_departure = False
        self.approach_started = False
        self._grid_wait_reported = False
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

    def _tracking_candidates_callback(self, message: PoseArray) -> None:
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().warning(
                f'Ignored tracking obstacle frame {message.header.frame_id!r}'
            )
            return
        self.tracking_candidates = [
            (pose.position.x, pose.position.y) for pose in message.poses
        ]
        now = self._now()
        self.last_candidates_update = now
        if self.target is not None:
            try:
                tracked = select_tracked_candidate(
                    self.target,
                    self.tracking_candidates,
                    self.target_tracking_radius,
                )
            except ValueError as error:
                self.get_logger().error(f'MODE 3 대상 추적 오류: {error}')
                tracked = None
            if tracked is not None:
                self.target = tracked
                self.target_last_seen = now
                if self.phase == 'OBSERVING':
                    self.observation_target_samples.append(tracked)
        self._try_start_inspection()

    def _target_is_fresh(self) -> bool:
        return (
            self.target is not None
            and self._now() - self.target_last_seen
            <= self.target_stale_timeout
        )

    def _target_tracking_expired(self) -> bool:
        """Return true after a selected target has been absent for the grace period."""
        reference = self.target_last_seen
        if not math.isfinite(reference):
            reference = getattr(
                self, 'target_tracking_started_at', float('-inf')
            )
        return (
            math.isfinite(reference)
            and self._now() - reference > self.target_stale_timeout
        )

    def _latest_target_distance(
        self, robot: Optional[Tuple[float, float, float]] = None
    ) -> Optional[float]:
        if not self._target_is_fresh():
            return None
        if robot is None:
            robot = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if robot is None:
            return None
        assert self.target is not None
        return math.hypot(
            self.target[0] - robot[0], self.target[1] - robot[1]
        )

    def _enter_inspection_range(self, actual_distance: float) -> None:
        self.cancel_publisher.publish(Empty())
        self.phase = 'SETTLING'
        self.phase_deadline = self._now() + self.robot_settle_sec
        self._state('MODE3_AT_STANDOFF:ROBOT_SETTLING')
        self.get_logger().info(
            'MODE 3 latest LiDAR target entered inspection range: '
            f'actual={actual_distance:.2f}m, '
            f'maximum={self.inspection_max_distance:.2f}m'
        )

    def _restart_for_latest_target(self, actual_distance: float) -> None:
        self.cancel_publisher.publish(Empty())
        self.requested_target = self.target
        self.phase = 'WAITING_FOR_OBSTACLE'
        self.waiting_for_departure = False
        self.approach_started = False
        self._grid_wait_reported = False
        self._state(
            f'MODE3_TARGET_MOVED:REPLANNING:DISTANCE:{actual_distance:.2f}M'
        )
        self._try_start_inspection()

    def _fail_target_tracking(self) -> None:
        self.cancel_publisher.publish(Empty())
        self.phase = 'TARGET_LOST'
        self._state('MODE3_TARGET_TRACK_LOST')
        self.get_logger().warning(
            'MODE 3 최신 LiDAR 장애물 좌표를 확인할 수 없습니다.'
        )

    def _try_start_inspection(self) -> None:
        if self.drive_mode != 3 or self.phase != 'WAITING_FOR_OBSTACLE':
            return
        robot = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if robot is None:
            return
        target = self.target or self.requested_target or select_nearest_candidate(
            robot[0], robot[1], self.candidates
        )
        if target is None:
            return
        live_candidates = (
            self.tracking_candidates
            if math.isfinite(self.last_candidates_update)
            else self.candidates
        )
        try:
            tracked = select_tracked_candidate(
                target, live_candidates, self.target_tracking_radius
            )
        except ValueError as error:
            self.get_logger().error(f'MODE 3 대상 추적 오류: {error}')
            tracked = None
        if tracked is not None:
            target = tracked
            self.target_last_seen = (
                self.last_candidates_update
                if math.isfinite(self.last_candidates_update)
                else self._now()
            )
        self.target = target
        target_distance = self._latest_target_distance(robot)
        if (
            target_distance is not None
            and target_distance <= self.inspection_max_distance
        ):
            self._enter_inspection_range(target_distance)
            return
        # A* keeps a nominal 2 m target for safe approach. Inspection starts
        # independently when the latest LiDAR target is within 2.5 m.
        self.active_standoff_distance = self.standoff_distance
        goal_x, goal_y, goal_yaw = compute_inspection_goal(
            robot[0], robot[1], robot[2], target[0], target[1],
            self.active_standoff_distance,
        )
        planning_grid = getattr(self, 'planning_grid', None)
        static_grid = getattr(self, 'static_grid', None)
        if planning_grid is None or static_grid is None:
            if not getattr(self, '_grid_wait_reported', False):
                self._grid_wait_reported = True
                self._state('MODE3_WAITING_FOR_PLANNING_GRID')
            return
        try:
            selected_goal = select_reachable_inspection_goal(
                (robot[0], robot[1]),
                target,
                (goal_x, goal_y),
                self.active_standoff_distance,
                self.standoff_arrival_tolerance,
                planning_grid,
                static_grid,
                minimum_goal_distance_m=self.minimum_approach_goal_distance,
                unknown_is_occupied=getattr(
                    self, 'unknown_is_occupied', True
                ),
                allow_diagonal=getattr(self, 'allow_diagonal', True),
            )
        except ValueError as error:
            self.get_logger().error(f'MODE 3 검사 목표 보정 실패: {error}')
            selected_goal = None
        if selected_goal is None:
            self.cancel_publisher.publish(Empty())
            self.phase = 'NO_PATH'
            self._state('MODE3_NO_PATH_TO_STANDOFF:SAFE_RING_EMPTY')
            self.get_logger().warning(
                'MODE 3: 2m ±0.2m 범위에 도달 가능한 안전 검사 위치가 없습니다.'
            )
            return
        corrected_distance = math.dist(
            (goal_x, goal_y), selected_goal[:2]
        )
        goal_x, goal_y, goal_yaw = selected_goal
        if corrected_distance > planning_grid.resolution * 0.5:
            self.get_logger().info(
                'MODE 3 검사 목표를 안전 지점으로 보정: '
                f'이동={corrected_distance:.2f}m, '
                f'검사거리={math.dist((goal_x, goal_y), target):.2f}m'
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
        self.waiting_for_departure = True
        self.approach_started = False
        self.phase = 'NAVIGATING'
        if self.publish_canonical_plan:
            now = self.get_clock().now()
            payload = {
                'success': True,
                'start_position_world': [robot[0], robot[1]],
                'selected_exit_id': 'MODE3_INSPECTION',
                'selected_exit_position_world': [target[0], target[1]],
                'selected_approach_position_world': [goal_x, goal_y],
                'selected_approach_yaw_rad': goal_yaw,
                'path_world': [],
                'path_grid': [],
                'selected_evaluation': None,
                'all_evaluations': [],
                'failure_reason': None,
                'selection_reason': 'mode5 targeted mmWave inspection',
                'created_at': now.nanoseconds / 1e9,
                'hazard_revision': self.hazard_revision,
                'activated': True,
                'manager_status': 'MODE3_INSPECTION_ACTIVATED',
            }
            self.plan_publisher.publish(String(data=json.dumps(
                payload, sort_keys=True, separators=(',', ':'), allow_nan=False
            )))
        # A canonical plan is consumed by the Mode 5 waypoint planner.  Do not
        # also publish the direct goal in that profile: it could overwrite the
        # canonical goal (including its final inspection yaw) in a test profile
        # that deliberately accepts both input forms.
        if not self.publish_canonical_plan:
            self.goal_publisher.publish(goal)
        self._state(
            f'MODE3_APPROACHING:{target[0]:.3f},{target[1]:.3f}:'
            f'STANDOFF:{self.active_standoff_distance:.2f}M'
        )

    def _follower_callback(self, message: String) -> None:
        if self.drive_mode != 3 or self.phase != 'NAVIGATING':
            return
        # PATH_ACCEPTED alone does not prove that the robot moved.  Requiring
        # an actual path-following command prevents an in-tolerance goal from
        # jumping directly to mmWave observation.
        if message.data == 'FOLLOWING_PATH':
            self.approach_started = True
            self.waiting_for_departure = False
        if (
            message.data != 'GOAL_REACHED'
            or self.waiting_for_departure
            or not self.approach_started
        ):
            return
        robot = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if robot is None or self.target is None:
            self.cancel_publisher.publish(Empty())
            self.phase = 'ARRIVAL_UNCONFIRMED'
            self._state('MODE3_ARRIVAL_NOT_CONFIRMED')
            return
        actual_distance = self._latest_target_distance(robot)
        if actual_distance is None:
            # The selected red candidate is retained slightly longer than a
            # current-scan candidate.  At the old approach goal, keep the robot
            # stopped and wait for a real current-scan association instead of
            # either pretending the retained coordinate is live or failing the
            # entire Mode 5 inspection after a brief LiDAR dropout.
            self.cancel_publisher.publish(Empty())
            self.phase = 'WAITING_FOR_LIVE_TARGET'
            self._state('MODE3_WAITING_FOR_LIVE_TARGET')
            return
        if actual_distance > self.inspection_max_distance:
            self._restart_for_latest_target(actual_distance)
            return
        self._enter_inspection_range(actual_distance)

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

    def _hazard_revision_callback(self, message: UInt64) -> None:
        self.hazard_revision = int(message.data)

    def _distance_callback(self, message: Float32) -> None:
        del message
        self.last_sensor_update = self._now()

    def _presence_callback(self, message: Bool) -> None:
        self.last_sensor_update = self._now()
        if self.phase != 'OBSERVING':
            return
        self.evidence.add(self.sensor_online, bool(message.data))

    def _start_observation(self) -> None:
        self.phase = 'OBSERVING'
        self.phase_deadline = self._now() + self.observation_sec
        self.evidence = PresenceEvidence()
        self.observation_target_start = self.target
        self.observation_target_samples = (
            [] if self.target is None else [self.target]
        )
        self._state('MODE3_MMWAVE_OBSERVING')
        self.get_logger().info(
            'MODE 3 mmWave presence-only observation: distance gate disabled'
        )
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
            displacement = stationary_observation_displacement(
                getattr(self, 'observation_target_start', None),
                getattr(self, 'observation_target_samples', ()),
            )
            if displacement <= 0.20 + 1e-9:
                assistance = PointStamped()
                assistance.header = point.header
                assistance.point = point.point
                self.assistance_publisher.publish(assistance)
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
        if (
            self.phase in (
                'WAITING_FOR_OBSTACLE', 'NAVIGATING',
                'WAITING_FOR_LIVE_TARGET', 'SETTLING', 'OBSERVING',
            )
            and self._target_tracking_expired()
        ):
            self._fail_target_tracking()
            return
        if self.phase == 'WAITING_FOR_OBSTACLE':
            self._try_start_inspection()
        elif self.phase in ('NAVIGATING', 'WAITING_FOR_LIVE_TARGET'):
            actual_distance = self._latest_target_distance()
            if (
                actual_distance is not None
                and actual_distance <= self.inspection_max_distance
            ):
                self._enter_inspection_range(actual_distance)
            elif (
                actual_distance is not None
                and self.phase == 'WAITING_FOR_LIVE_TARGET'
            ):
                self._restart_for_latest_target(actual_distance)
        elif self.phase in ('SETTLING', 'OBSERVING'):
            # Preserve the latest real observation only for the configured
            # short LiDAR dropout grace period. The early stale check above
            # cancels inspection instead of classifying a disappeared target.
            actual_distance = self._latest_target_distance()
            if (
                actual_distance is not None
                and actual_distance > self.inspection_max_distance
            ):
                self._restart_for_latest_target(actual_distance)
                return
            if self._now() < self.phase_deadline:
                return
            if self.phase == 'SETTLING':
                self._start_observation()
            else:
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
