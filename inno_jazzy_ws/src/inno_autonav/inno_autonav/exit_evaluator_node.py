"""On-demand ROS adapter for the pure ExitEvaluator core."""

from __future__ import annotations

import json
import math

from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

from inno_hazard.hazard_belief import HazardGridGeometry
from inno_hazard.hazard_snapshot import decode_hazard_snapshot_message

from .exit_evaluator import (
    ExitEvaluationConfig, ExitEvaluator, ExitHazardSnapshot,
    apply_static_clearance_to_snapshot, exit_evaluator_readiness,
    load_exit_registry,
)
from .grid_utils import build_static_clearance_mask, yaw_from_quaternion
from .reference_waypoint_graph import (
    PlanningGridGeometry, ReferenceWaypoint, ReferenceWaypointGraphConfig,
    ReferenceWaypointGraphPlanner,
)
from .tf_utils import TfHelper
from .waypoint_selection import load_waypoint_document, named_waypoints_from_document


class ExitEvaluatorNode(Node):
    def __init__(self):
        super().__init__("exit_evaluator_node")
        defaults = {
            "map_frame": "map",
            "base_frame": "base_link",
            "exit_registry_file": "",
            "reference_waypoint_file": "",
            "hazard_snapshot_topic": "/hazard/snapshot",
            "hazard_status_topic": "/hazard/status",
            "static_grid_topic": "/planning_grid_static",
            "result_topic": "/exit_evaluations",
            "status_topic": "/exit_evaluator/status",
            "service_name": "/evaluate_exits",
            "exit_neighborhood_radius_m": 1.0,
            "approach_search_radius_m": 1.0,
            "path_block_check_radius": 0.20,
            "reject_blocked_exit": True,
            "reject_dangerous_exit": True,
            "reject_path_over_threshold": True,
            "reject_invalid_cost": True,
            "usable_confirmation_distance_m": 3.0,
            "dangerous_accumulated_risk_cost": -1.0,
            "dangerous_average_risk_cost": -1.0,
            "dangerous_max_cell_risk_cost": -1.0,
            "reference_waypoint_graph_enabled": True,
            "reference_neighbor_radius_m": 1.5,
            "reference_connector_search_radius_m": 3.0,
            "reference_connector_candidate_count": 8,
            "reference_fallback_to_cell_astar": True,
            "reference_waypoint_cost_radius_m": 0.10,
            "reference_waypoint_risk_weight": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.map_frame = str(value("map_frame"))
        self.base_frame = str(value("base_frame"))
        self.clearance_radius = float(value("path_block_check_radius"))
        if (
            not math.isfinite(self.clearance_radius)
            or self.clearance_radius < 0.0
        ):
            raise ValueError(
                "path_block_check_radius must be finite and non-negative"
            )
        exit_file = str(value("exit_registry_file")).strip()
        waypoint_file = str(value("reference_waypoint_file")).strip()
        if not exit_file or not waypoint_file:
            raise ValueError("exit_registry_file and reference_waypoint_file are required")
        self.exits = load_exit_registry(exit_file, self.map_frame)
        records = named_waypoints_from_document(
            load_waypoint_document(waypoint_file), self.map_frame
        )
        reference_config = ReferenceWaypointGraphConfig(
            enabled=bool(value("reference_waypoint_graph_enabled")),
            neighbor_radius_m=float(value("reference_neighbor_radius_m")),
            connector_search_radius_m=float(value("reference_connector_search_radius_m")),
            connector_candidate_count=int(value("reference_connector_candidate_count")),
            fallback_to_cell_astar=bool(value("reference_fallback_to_cell_astar")),
            waypoint_cost_radius_m=float(value("reference_waypoint_cost_radius_m")),
            waypoint_risk_weight=float(value("reference_waypoint_risk_weight")),
        )
        self.reference_planner = ReferenceWaypointGraphPlanner(tuple(
            ReferenceWaypoint(item.name, item.x, item.y, item.yaw)
            for item in records
        ), reference_config)

        def threshold(name):
            number = float(value(name))
            return None if number < 0.0 else number

        config = ExitEvaluationConfig(
            exit_neighborhood_radius_m=float(value("exit_neighborhood_radius_m")),
            approach_search_radius_m=float(value("approach_search_radius_m")),
            reject_blocked_exit=bool(value("reject_blocked_exit")),
            reject_dangerous_exit=bool(value("reject_dangerous_exit")),
            reject_path_over_threshold=bool(value("reject_path_over_threshold")),
            reject_invalid_cost=bool(value("reject_invalid_cost")),
            usable_confirmation_distance_m=float(value("usable_confirmation_distance_m")),
            dangerous_accumulated_risk_cost=threshold("dangerous_accumulated_risk_cost"),
            dangerous_average_risk_cost=threshold("dangerous_average_risk_cost"),
            dangerous_max_cell_risk_cost=threshold("dangerous_max_cell_risk_cost"),
        )
        self.evaluator = ExitEvaluator(config, path_planner=self._plan)
        self.tf = TfHelper(self)
        self.raw_snapshot = None
        self.snapshot = None
        self.snapshot_status = ""
        self.snapshot_is_initial_route_view = False
        self.static_geometry = None
        self.static_clearance_mask = None
        self.status = "EXIT_EVALUATOR_NOT_READY"

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.result_publisher = self.create_publisher(
            String, str(value("result_topic")), qos
        )
        self.status_publisher = self.create_publisher(
            String, str(value("status_topic")), qos
        )
        self.create_subscription(
            Float32MultiArray, str(value("hazard_snapshot_topic")),
            self._snapshot_callback, qos,
        )
        self.create_subscription(
            String, str(value("hazard_status_topic")),
            self._hazard_status_callback, qos,
        )
        self.create_subscription(
            OccupancyGrid, str(value("static_grid_topic")),
            self._static_callback, qos,
        )
        self.create_service(Trigger, str(value("service_name")), self._evaluate)
        self._set_status(self.status)

    def _set_status(self, status):
        changed = status != self.status
        if changed:
            self.status = status
        self.status_publisher.publish(String(data=self.status))
        if changed:
            self.get_logger().info(f"exit evaluator status: {self.status}")

    def _static_callback(self, message):
        info, origin = message.info, message.info.origin
        try:
            geometry = HazardGridGeometry(
                int(info.width), int(info.height), float(info.resolution),
                float(origin.position.x), float(origin.position.y),
                yaw_from_quaternion(origin.orientation),
                str(message.header.frame_id),
            )
            static_data = np.asarray(message.data, dtype=np.int16)
            if static_data.size != geometry.width * geometry.height:
                raise ValueError("static occupancy data length is invalid")
            static_data = static_data.reshape(
                geometry.height,
                geometry.width,
            )
            clearance = build_static_clearance_mask(
                static_data,
                self.clearance_radius,
                geometry.resolution,
                unknown_is_occupied=True,
            )
        except ValueError as exc:
            self.get_logger().error(f"invalid static planning grid: {exc}")
            self.static_geometry = None
            self.static_clearance_mask = None
            self.snapshot = None
            self._refresh_readiness()
            return
        self.static_geometry = geometry
        self.static_clearance_mask = clearance
        self._apply_static_clearance()
        self._refresh_readiness()

    def _snapshot_callback(self, message):
        try:
            metadata, layers = decode_hazard_snapshot_message(message)
            geometry = HazardGridGeometry(
                layers["final_cost"].shape[1], layers["final_cost"].shape[0],
                float(metadata["resolution"]), float(metadata["origin_x"]),
                float(metadata["origin_y"]), float(metadata["origin_yaw"]),
                str(metadata["frame_id"]),
            )
            self.raw_snapshot = ExitHazardSnapshot(
                geometry, layers["final_cost"], layers["temperature_c"],
                layers["co_ppm"], layers["observed"].astype(bool),
                layers["temperature_observed"].astype(bool),
                layers["co_observed"].astype(bool), layers["fire_probability"],
                layers["static_obstacle"].astype(bool),
                layers["dynamic_obstacle"].astype(bool),
                layers["blocked"].astype(bool), int(metadata["revision"]),
                float(metadata["temperature_blocked_c"]),
                float(metadata["co_blocked_ppm"]), float(metadata["base_cost"]),
            )
            snapshot_channels = tuple(metadata.get("channels", ()))
            self.snapshot_is_initial_route_view = (
                bool(snapshot_channels)
                and "temperature_c" not in snapshot_channels
                and str(metadata["status"])
                == "ACTIVE_INITIAL_STATIC_DYNAMIC_ONLY"
            )
            # Metadata status remains a compatibility fallback. Live readiness
            # changes arrive independently on /hazard/status and do not force
            # this large snapshot to be serialized again.
            if not self.snapshot_status:
                self.snapshot_status = str(metadata["status"])
            self._apply_static_clearance()
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid hazard snapshot: {exc}")
            self.raw_snapshot = None
            self.snapshot = None
            self.snapshot_is_initial_route_view = False
            self.snapshot_status = "INVALID_HAZARD_SNAPSHOT"
        self._refresh_readiness()

    def _hazard_status_callback(self, message):
        self.snapshot_status = str(message.data)
        self._refresh_readiness()

    def _apply_static_clearance(self):
        if self.raw_snapshot is None:
            self.snapshot = None
            return
        if self.static_clearance_mask is None:
            self.snapshot = self.raw_snapshot
            return
        try:
            self.snapshot = apply_static_clearance_to_snapshot(
                self.raw_snapshot,
                self.static_clearance_mask,
            )
        except ValueError as exc:
            self.get_logger().error(
                f"invalid static clearance overlay: {exc}"
            )
            self.snapshot = None

    def _refresh_readiness(self):
        if (
            self.snapshot_status == "ACTIVE_INITIAL_STATIC_DYNAMIC_ONLY"
            and not self.snapshot_is_initial_route_view
        ):
            self._set_status(
                "HAZARD_NOT_READY:INITIAL_ROUTE_VIEW_PENDING"
            )
            return
        self._set_status(exit_evaluator_readiness(
            self.snapshot, self.static_geometry,
            getattr(self, "snapshot_status", ""), self.map_frame,
        ))

    def _plan(self, snapshot, start, goal):
        geometry = PlanningGridGeometry(
            snapshot.geometry.resolution, snapshot.geometry.origin_x,
            snapshot.geometry.origin_y, snapshot.geometry.origin_yaw,
            snapshot.geometry.frame_id,
        )
        return self.reference_planner.plan(
            snapshot.final_cost, start, goal, geometry,
            snapshot.static_obstacle_map,
            costs_are_traversal=True,
            unknown_is_occupied=True,
            allow_diagonal=True,
            thermal_cost_weight=0.0,
            thermal_cost_power=1.0,
            waypoint_frame_id=self.map_frame,
        )

    def _evaluate(self, request, response):
        del request
        self._refresh_readiness()
        if self.status != "READY":
            response.success = False
            response.message = self.status
            return response
        pose = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if pose is None:
            response.success = False
            response.message = "ROBOT_POSE_NOT_READY"
            return response
        # Copy the immutable reference once. Later callbacks replace self.snapshot
        # but cannot change the revision used by this complete batch.
        snapshot = self.snapshot
        now = self.get_clock().now().nanoseconds / 1e9
        batch = self.evaluator.evaluate_all(
            self.exits, (pose[0], pose[1]), snapshot=snapshot,
            evaluated_at=now,
        )
        payload = json.dumps(batch.to_dict(), separators=(",", ":"), allow_nan=False)
        self.result_publisher.publish(String(data=payload))
        response.success = True
        response.message = payload
        self._set_status("READY")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ExitEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
