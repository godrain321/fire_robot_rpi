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

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray

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
            "route_status_topic": "/waypoint_planner/route_status",
            "route_markers_topic": "/waypoint_route_markers",
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

        def value(name):
            return self.get_parameter(name).value
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
        self.route_status_publisher = self.create_publisher(
            String, str(value("route_status_topic")), qos,
        )
        self.route_markers_publisher = self.create_publisher(
            MarkerArray, str(value("route_markers_topic")), qos,
        )
        self.create_subscription(
            String, str(value("replan_request_topic")), self._on_replan_request, 10,
        )
        self.create_subscription(
            Empty, "/autonomy_cancel", self._on_cancel, 10,
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

    def _on_cancel(self, _message: Empty) -> None:
        """Invalidate the waypoint mission so a later grid update cannot revive it."""
        self.active_goal = None
        self._last_costs = None
        self._last_goal = None
        self._last_published_path_stamp_ns = None
        self._goal_received_ns = None
        self._publish_empty_path()

    def _replan(self, force: bool = False) -> tuple[bool, str]:
        if not self.enabled or self.grid is None or self.active_goal is None:
            return False, "NOT_READY"
        pose = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if pose is None:
            return False, "NO_TF"
        costs = self.projector.project_costs(self.grid)
        if not force and costs == self._last_costs and self.active_goal == self._last_goal:
            return False, "UNCHANGED"
        is_replan = self._last_costs is not None and self._last_goal == self.active_goal
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
        self._publish_path(
            points,
            waypoint_ids=list(simplification.simplified_ids),
            reference_waypoint_ids=list(result.waypoint_ids),
            is_replan=is_replan,
        )
        return True, "PATH_FOUND"

    def _publish_path(
        self,
        points: list[tuple[float, float]],
        waypoint_ids: list[str] | None = None,
        reference_waypoint_ids: list[str] | None = None,
        is_replan: bool = False,
    ) -> None:
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
                # Normal evacuation plans omit yaw and retain the historical
                # identity orientation.  Mode 3's targeted inspection plan
                # supplies an optional yaw so the sensor faces the obstacle.
                yaw = (
                    self.active_goal.approach_yaw_rad
                    if self.active_goal.approach_yaw_rad is not None
                    else 0.0
                )
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            message.poses.append(pose)
        self.waypoint_path_publisher.publish(message)
        if waypoint_ids is not None:
            self._publish_route_status(
                waypoint_ids, is_replan, reference_waypoint_ids
            )
            self._publish_route_markers(
                message.header, waypoint_ids, points,
                reference_waypoint_ids,
            )
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

    def _publish_route_status(
        self, waypoint_ids: list[str], is_replan: bool,
        reference_waypoint_ids: list[str] | None = None,
    ) -> None:
        """Publish a machine-readable route and the Korean operator log."""
        event = "REPLANNED" if is_replan else "PATH_CREATED"
        payload = {
            "event": event,
            "goal_id": self.active_goal.exit_id,
            "hazard_revision": self.active_goal.hazard_revision,
            "waypoints": waypoint_ids,
            "reference_waypoints": (
                waypoint_ids
                if reference_waypoint_ids is None
                else reference_waypoint_ids
            ),
            "final_goal_world": list(self.active_goal.approach_world),
        }
        publisher = getattr(self, "route_status_publisher", None)
        if publisher is not None:
            publisher.publish(String(data=json.dumps(
                payload, ensure_ascii=False, sort_keys=True,
                separators=(",", ":"), allow_nan=False,
            )))
        route = " -> ".join(waypoint_ids) if waypoint_ids else "직접 목표점"
        reference_route = " -> ".join(
            waypoint_ids
            if reference_waypoint_ids is None
            else reference_waypoint_ids
        )
        prefix = "상황 변화로 경로 재생성" if is_replan else "경로 생성"
        log_info = getattr(self.get_logger(), "info", None)
        if log_info is not None:
            log_info(
                f"{prefix}: 기반={reference_route}; "
                f"실제주행={route} -> {self.active_goal.exit_id}"
            )

    def _publish_route_markers(
        self, header, waypoint_ids, points, reference_waypoint_ids=None
    ) -> None:
        """Overlay the currently selected waypoint route in RViz."""
        publisher = getattr(self, "route_markers_publisher", None)
        if publisher is None:
            return
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL

        line = Marker()
        line.header = header
        line.ns = "selected_waypoint_route"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.13
        line.color.r = 0.1
        line.color.g = 0.9
        line.color.b = 1.0
        line.color.a = 0.95
        line.points = [Point(x=float(x), y=float(y), z=0.08) for x, y in points]

        markers = [clear, line]
        reference_ids = list(
            waypoint_ids
            if reference_waypoint_ids is None
            else reference_waypoint_ids
        )
        active_ids = set(waypoint_ids)

        support_points = Marker()
        support_points.header = header
        support_points.ns = "selected_waypoint_route_support_points"
        support_points.id = 0
        support_points.type = Marker.SPHERE_LIST
        support_points.action = Marker.ADD
        support_points.pose.orientation.w = 1.0
        support_points.scale.x = 0.24
        support_points.scale.y = 0.24
        support_points.scale.z = 0.24
        support_points.color.r = 1.0
        support_points.color.g = 0.42
        support_points.color.b = 0.05
        support_points.color.a = 1.0
        support_points.points = [
            Point(
                x=float(self.waypoints_world[waypoint_id][0]),
                y=float(self.waypoints_world[waypoint_id][1]),
                z=0.12,
            )
            for waypoint_id in reference_ids
            if waypoint_id in self.waypoints_world
            and waypoint_id not in active_ids
        ]
        if support_points.points:
            markers.append(support_points)

        active_points = Marker()
        active_points.header = header
        active_points.ns = "selected_waypoint_route_active_points"
        active_points.id = 0
        active_points.type = Marker.SPHERE_LIST
        active_points.action = Marker.ADD
        active_points.pose.orientation.w = 1.0
        active_points.scale.x = 0.40
        active_points.scale.y = 0.40
        active_points.scale.z = 0.40
        active_points.color.r = 1.0
        active_points.color.g = 0.05
        active_points.color.b = 0.75
        active_points.color.a = 1.0
        active_points.points = [
            Point(
                x=float(self.waypoints_world[waypoint_id][0]),
                y=float(self.waypoints_world[waypoint_id][1]),
                z=0.15,
            )
            for waypoint_id in waypoint_ids
            if waypoint_id in self.waypoints_world
        ]
        if active_points.points:
            markers.append(active_points)

        for index, waypoint_id in enumerate(reference_ids):
            if waypoint_id not in self.waypoints_world:
                continue
            x, y = self.waypoints_world[waypoint_id]
            label = Marker()
            label.header = header
            label.ns = "selected_waypoint_route_labels"
            label.id = index + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(x)
            label.pose.position.y = float(y)
            label.pose.position.z = 0.42
            label.pose.orientation.w = 1.0
            label.scale.z = 0.30
            if waypoint_id in active_ids:
                label.color.r = 1.0
                label.color.g = 0.2
                label.color.b = 0.85
                label.text = f"DRIVE {waypoint_id}"
            else:
                label.color.r = 1.0
                label.color.g = 0.55
                label.color.b = 0.1
                label.text = f"via {waypoint_id}"
            label.color.a = 1.0
            markers.append(label)

        if points:
            goal_x, goal_y = points[-1]
            goal = Marker()
            goal.header = header
            goal.ns = "selected_waypoint_route_goal"
            goal.id = 0
            goal.type = Marker.SPHERE
            goal.action = Marker.ADD
            goal.pose.position.x = float(goal_x)
            goal.pose.position.y = float(goal_y)
            goal.pose.position.z = 0.16
            goal.pose.orientation.w = 1.0
            goal.scale.x = goal.scale.y = goal.scale.z = 0.46
            goal.color.r = 0.1
            goal.color.g = 1.0
            goal.color.b = 0.2
            goal.color.a = 1.0
            markers.append(goal)

            goal_label = Marker()
            goal_label.header = header
            goal_label.ns = "selected_waypoint_route_goal_label"
            goal_label.id = 0
            goal_label.type = Marker.TEXT_VIEW_FACING
            goal_label.action = Marker.ADD
            goal_label.pose.position.x = float(goal_x)
            goal_label.pose.position.y = float(goal_y)
            goal_label.pose.position.z = 0.62
            goal_label.pose.orientation.w = 1.0
            goal_label.scale.z = 0.34
            goal_label.color.r = 0.2
            goal_label.color.g = 1.0
            goal_label.color.b = 0.2
            goal_label.color.a = 1.0
            goal_label.text = f"SELECTED {self.active_goal.exit_id}"
            markers.append(goal_label)
        publisher.publish(MarkerArray(markers=markers))

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
