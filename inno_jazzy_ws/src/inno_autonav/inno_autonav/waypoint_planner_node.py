"""ROS adapter: /planning_grid + canonical selected exit + current pose ->
WaypointCostProjector -> WaypointGraphPlanner -> simplify_waypoint_route ->
/waypoint_path.

Thin by design: no graph search or cost-projection logic lives here, only I/O
wiring and the small amount of glue (start/goal waypoint selection, nav_msgs
construction) that has to live at the ROS boundary. Never publishes /goal_pose
or /planned_path. EvacuationManager owns the former in evacuation mode; the
explicitly-enabled static field profile consumes MissionCommander's direct goal.
PathSelector owns the latter.
"""

from __future__ import annotations

import json
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .astar_replanner import message_to_grid
from .grid_utils import quaternion_from_yaw
from .replan_supervisor import parse_active_goal_payload
from .replan_supervisor import ActiveGoal
from .tf_utils import TfHelper
from .waypoint_cost_projector import WaypointCostProjector, WaypointCostProjectorConfig
from .waypoint_graph_planner import (
    WaypointGraphPlanner,
    WaypointGraphPlannerConfig,
    nearest_safe_waypoint,
)
from .waypoint_route_simplifier import (
    WaypointRouteSimplifierConfig,
    simplify_waypoint_route,
)
from .waypoint_selection import load_waypoint_document, named_waypoints_from_document


class WaypointPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_planner_node")
        defaults = {
            "enabled": False,
            "map_frame": "map",
            "base_frame": "base_link",
            "waypoint_file": "",
            "planning_grid_topic": "/planning_grid",
            "evacuation_plan_topic": "/evacuation/plan",
            "direct_goal_topic": "/goal_pose",
            "accept_direct_goal": False,
            "waypoint_path_topic": "/waypoint_path",
            "replan_request_topic": "/replanning/waypoint_request",
            "replan_result_topic": "/replanning/waypoint_result",
            "neighbor_radius_m": 1.5,
            "waypoint_cost_radius_m": 0.8,
            "unknown_is_occupied": True,
            "thermal_cost_weight": 24.0,
            "thermal_cost_power": 1.5,
            "fixed_co_ppm": 0.0,
            "co_safe_ppm": 0.0,
            "co_blocked_ppm": 1600.0,
            "co_cost_weight": 8.0,
            "co_cost_power": 2.0,
            "simplification_maximum_risk_ratio": 1.0,
            "simplification_risk_absolute_tolerance": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.enabled = bool(value("enabled"))
        self.accept_direct_goal = bool(value("accept_direct_goal"))
        self.map_frame = str(value("map_frame"))
        self.base_frame = str(value("base_frame"))
        waypoint_file = str(value("waypoint_file")).strip()
        if not waypoint_file:
            raise ValueError("waypoint_file is required")
        records = named_waypoints_from_document(
            load_waypoint_document(waypoint_file), self.map_frame,
        )
        self.waypoints_world = {item.name: (item.x, item.y) for item in records}

        self.projector = WaypointCostProjector(
            self.waypoints_world,
            WaypointCostProjectorConfig(
                waypoint_cost_radius_m=float(value("waypoint_cost_radius_m")),
                unknown_is_occupied=bool(value("unknown_is_occupied")),
            ),
        )
        self.graph_planner = WaypointGraphPlanner(
            self.waypoints_world,
            WaypointGraphPlannerConfig(neighbor_radius_m=float(value("neighbor_radius_m"))),
        )
        self.simplifier_config = WaypointRouteSimplifierConfig(
            unknown_is_occupied=bool(value("unknown_is_occupied")),
            thermal_cost_weight=float(value("thermal_cost_weight")),
            thermal_cost_power=float(value("thermal_cost_power")),
            fixed_co_ppm=float(value("fixed_co_ppm")),
            co_safe_ppm=float(value("co_safe_ppm")),
            co_blocked_ppm=float(value("co_blocked_ppm")),
            co_cost_weight=float(value("co_cost_weight")),
            co_cost_power=float(value("co_cost_power")),
            maximum_risk_ratio=float(value("simplification_maximum_risk_ratio")),
            risk_absolute_tolerance=float(value("simplification_risk_absolute_tolerance")),
        )

        self.grid = None
        self.active_goal = None
        self._last_costs = None
        self._last_goal = None
        self._last_published_path_stamp_ns = None
        self._last_emitted_path_stamp_ns = -1
        self._goal_received_ns = None
        self.tf = TfHelper(self)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, str(value("planning_grid_topic")), self._on_grid, qos,
        )
        self.create_subscription(
            String, str(value("evacuation_plan_topic")), self._on_plan, qos,
        )
        if self.accept_direct_goal:
            self.create_subscription(
                PoseStamped, str(value("direct_goal_topic")), self._on_direct_goal, 10,
            )
        self.waypoint_path_publisher = self.create_publisher(
            Path, str(value("waypoint_path_topic")), qos,
        )
        self.create_subscription(
            String, str(value("replan_request_topic")), self._on_replan_request, 10,
        )
        self.replan_result_publisher = self.create_publisher(
            String, str(value("replan_result_topic")), 10,
        )

    def _on_grid(self, message: OccupancyGrid) -> None:
        try:
            self.grid = message_to_grid(message)
        except ValueError as exc:
            self.get_logger().error(f"invalid planning grid: {exc}")
            return
        self._replan()

    def _on_plan(self, message: String) -> None:
        goal = parse_active_goal_payload(message.data)
        if goal != self.active_goal:
            self.active_goal = goal
            self._goal_received_ns = getattr(self.get_clock().now(), "nanoseconds", 0)
            self._replan()

    def _on_direct_goal(self, message: PoseStamped) -> None:
        if not self.accept_direct_goal:
            return
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().error(
                f"direct goal frame={message.header.frame_id!r}; "
                f"{self.map_frame!r} only"
            )
            return
        goal = ActiveGoal(
            "DIRECT_GOAL",
            (float(message.pose.position.x), float(message.pose.position.y)),
            0,
        )
        if goal != self.active_goal:
            self.active_goal = goal
            self._goal_received_ns = getattr(self.get_clock().now(), "nanoseconds", 0)
            self._replan()

    def _on_replan_request(self, message: String) -> None:
        try:
            request = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(request, dict):
            return
        goal = self.active_goal
        if goal is None or request.get("exit_id") != goal.exit_id or tuple(
            request.get("goal_world", ())
        ) != goal.approach_world:
            return
        self._last_published_path_stamp_ns = None
        success, status = self._replan(force=True)
        result = dict(request)
        result.update(
            success=success, status=status,
            path_stamp_ns=self._last_published_path_stamp_ns,
        )
        self.replan_result_publisher.publish(String(data=json.dumps(result, sort_keys=True)))

    def _replan(self, force: bool = False) -> tuple[bool, str]:
        if not self.enabled or self.grid is None or self.active_goal is None:
            return False, "NOT_READY"
        pose = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if pose is None:
            return False, "NO_TF"
        costs = self.projector.project_costs(self.grid)
        if not force and costs == self._last_costs and self.active_goal == self._last_goal:
            return False, "UNCHANGED"
        self._last_costs = costs
        self._last_goal = self.active_goal

        start_id = nearest_safe_waypoint((pose[0], pose[1]), self.waypoints_world, costs)
        goal_id = nearest_safe_waypoint(
            self.active_goal.approach_world, self.waypoints_world, costs,
        )
        if start_id is None or goal_id is None:
            self.get_logger().error("no safe start/goal waypoint available")
            self._publish_empty_path()
            return False, "NO_SAFE_ENDPOINT"

        result = self.graph_planner.plan(costs, start_id, goal_id)
        if not result.success:
            self.get_logger().error(f"waypoint graph planning failed: {result.status}")
            self._publish_empty_path()
            return False, result.status

        simplification = simplify_waypoint_route(
            result.waypoint_ids, self.waypoints_world, self.grid, self.simplifier_config,
        )
        if not simplification.success:
            self.get_logger().error(
                f"waypoint route simplification failed: {simplification.detail}"
            )
            self._publish_empty_path()
            return False, simplification.detail

        points = [self.waypoints_world[wid] for wid in simplification.simplified_ids]
        points.append(self.active_goal.approach_world)
        self._publish_path(points)
        return True, "PATH_FOUND"

    def _publish_path(self, points: list[tuple[float, float]]) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        clock_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        stamp_ns = max(clock_stamp_ns, getattr(self, "_last_emitted_path_stamp_ns", -1) + 1)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(
            stamp_ns, 1_000_000_000
        )
        self._last_emitted_path_stamp_ns = stamp_ns
        self._last_published_path_stamp_ns = stamp_ns
        message.header.frame_id = self.map_frame
        for index, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if index + 1 < len(points):
                next_x, next_y = points[index + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                # Matches EvacuationManagerNode's own /goal_pose convention
                # (orientation.w=1.0); /evacuation/plan carries no orientation
                # to align to, so nothing is invented here.
                yaw = 0.0
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            message.poses.append(pose)
        self.waypoint_path_publisher.publish(message)
        goal_received_ns = getattr(self, "_goal_received_ns", None)
        if goal_received_ns is not None:
            elapsed_ms = (
                getattr(self.get_clock().now(), "nanoseconds", 0) - goal_received_ns
            ) / 1_000_000.0
            log_info = getattr(self.get_logger(), "info", None)
            if log_info is not None:
                log_info(
                    f"waypoint path ready: poses={len(message.poses)}, "
                    f"goal_to_path_ms={elapsed_ms:.3f}"
                )
            self._goal_received_ns = None

    def _publish_empty_path(self) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        self.waypoint_path_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f"waypoint_planner_node 오류: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
