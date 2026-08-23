"""ROS adapter: /planning_grid + canonical selected exit + current pose ->
WaypointCostProjector -> WaypointGraphPlanner -> simplify_waypoint_route ->
/waypoint_path.

Thin by design: no graph search or cost-projection logic lives here, only I/O
wiring and the small amount of glue (start/goal waypoint selection, nav_msgs
construction) that has to live at the ROS boundary. Never publishes /goal_pose
or /planned_path -- EvacuationManagerNode keeps sole ownership of the former,
PathSelector of the latter (Stage 8-8).
"""

from __future__ import annotations

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
            "waypoint_path_topic": "/waypoint_path",
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
        self.waypoint_path_publisher = self.create_publisher(
            Path, str(value("waypoint_path_topic")), qos,
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
            self._replan()

    def _replan(self) -> None:
        if not self.enabled or self.grid is None or self.active_goal is None:
            return
        pose = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if pose is None:
            return
        costs = self.projector.project_costs(self.grid)
        if costs == self._last_costs and self.active_goal == self._last_goal:
            return  # nothing that could change the route actually changed
        self._last_costs = costs
        self._last_goal = self.active_goal

        start_id = nearest_safe_waypoint((pose[0], pose[1]), self.waypoints_world, costs)
        goal_id = nearest_safe_waypoint(
            self.active_goal.approach_world, self.waypoints_world, costs,
        )
        if start_id is None or goal_id is None:
            self.get_logger().error("no safe start/goal waypoint available")
            self._publish_empty_path()
            return

        result = self.graph_planner.plan(costs, start_id, goal_id)
        if not result.success:
            self.get_logger().error(f"waypoint graph planning failed: {result.status}")
            self._publish_empty_path()
            return

        simplification = simplify_waypoint_route(
            result.waypoint_ids, self.waypoints_world, self.grid, self.simplifier_config,
        )
        if not simplification.success:
            self.get_logger().error(
                f"waypoint route simplification failed: {simplification.detail}"
            )
            self._publish_empty_path()
            return

        points = [self.waypoints_world[wid] for wid in simplification.simplified_ids]
        points.append(self.active_goal.approach_world)
        self._publish_path(points)

    def _publish_path(self, points: list[tuple[float, float]]) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
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
