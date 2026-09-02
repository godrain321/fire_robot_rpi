"""On-demand ROS orchestration for exit evaluation, selection, and goal activation."""

from __future__ import annotations

import json

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger

from .exit_evaluator import load_exit_registry
from .evacuation_planner import (
    EvacuationPlanner, ExitSelectionConfig, build_evacuation_decision,
)


class EvacuationManagerNode(Node):
    def __init__(self):
        super().__init__("evacuation_manager_node")
        defaults = {
            "enabled": False,
            "exit_registry_file": "",
            "activate_selected_route": False,
            "risk_first": False,
            "map_frame": "map",
            "exit_evaluation_service": "/evaluate_exits",
            "plan_service": "/plan_evacuation",
            "planner_goal_topic": "/goal_pose",
            "hazard_revision_topic": "/hazard/revision",
            "plan_topic": "/evacuation/plan",
            "selected_exit_topic": "/evacuation/selected_exit",
            "status_topic": "/evacuation/status",
            "evaluation_service_timeout_s": 30.0,
            "prefer_confirmed_usable_exit": True,
            "fallback_to_shortest_reachable_exit": True,
            "primary_key": "path_length_m",
            "secondary_key": "accumulated_risk_cost",
            "final_tie_breaker": "exit_id",
            "float_tolerance": 1e-6,
            "switch_request_topic": "/evacuation/switch_request",
            "switch_result_topic": "/evacuation/switch_result",
            "blocked_exits_topic": "/evacuation/blocked_exits",
            "danger_expected_exits_topic": (
                "/evacuation/danger_expected_exits"
            ),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.enabled = bool(value("enabled"))
        self.activate = bool(value("activate_selected_route"))
        self.risk_first = bool(value("risk_first"))
        self.map_frame = str(value("map_frame"))
        exit_file = str(value("exit_registry_file")).strip()
        self.registered_exits = (
            {item.exit_id: item for item in load_exit_registry(exit_file, self.map_frame)}
            if exit_file else {}
        )
        self.timeout = float(value("evaluation_service_timeout_s"))
        if self.timeout <= 0.0:
            raise ValueError("evaluation_service_timeout_s must be positive")
        self.planner = EvacuationPlanner(ExitSelectionConfig(
            prefer_confirmed_usable_exit=bool(value("prefer_confirmed_usable_exit")),
            fallback_to_shortest_reachable_exit=bool(value("fallback_to_shortest_reachable_exit")),
            primary_key=str(value("primary_key")),
            secondary_key=str(value("secondary_key")),
            final_tie_breaker=str(value("final_tie_breaker")),
            float_tolerance=float(value("float_tolerance")),
        ))
        self.current_hazard_revision = None
        self.externally_blocked_exit_ids = set()
        self.danger_expected_exit_ids = set()
        service_group = MutuallyExclusiveCallbackGroup()
        client_group = MutuallyExclusiveCallbackGroup()
        self.evaluation_client = self.create_client(
            Trigger, str(value("exit_evaluation_service")),
            callback_group=client_group,
        )
        self.create_service(
            Trigger, str(value("plan_service")), self._plan,
            callback_group=service_group,
        )
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        command_qos = QoSProfile(depth=10)
        command_qos.reliability = ReliabilityPolicy.RELIABLE
        self.plan_publisher = self.create_publisher(
            String, str(value("plan_topic")), qos
        )
        self.selected_publisher = self.create_publisher(
            String, str(value("selected_exit_topic")), qos
        )
        self.status_publisher = self.create_publisher(
            String, str(value("status_topic")), qos
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, str(value("planner_goal_topic")), 10
        )
        self.create_subscription(
            UInt64, str(value("hazard_revision_topic")),
            self._revision, qos,
        )
        self.switch_result_publisher = self.create_publisher(
            String, str(value("switch_result_topic")), qos
        )
        self.create_subscription(
            String, str(value("switch_request_topic")),
            self._on_switch_request, command_qos,
        )
        self.create_subscription(
            String, str(value("blocked_exits_topic")), self._on_blocked_exits, qos,
        )
        self.create_subscription(
            String, str(value("danger_expected_exits_topic")),
            self._on_danger_expected_exits, qos,
        )
        self._status("READY" if self.enabled else "DISABLED")

    def _revision(self, message):
        self.current_hazard_revision = int(message.data)

    def _status(self, value):
        self.status_publisher.publish(String(data=str(value)))

    def _failure(self, response, status):
        self._status(status)
        response.success = False
        response.message = str(status)
        return response

    def _select_and_activate(
        self, *, excluded_exit_ids=(), candidate_exit_ids=None,
        risk_first=None, force_activate=False, publish_on_failure=True,
    ):
        """Shared Stage 4 evaluate -> Stage 5 select -> canonical-state pipeline.

        Used by both the `/plan_evacuation` Trigger (all-default args, exactly
        today's behavior) and Stage 7's `/evacuation/switch_request` handler
        (excluded/candidate ids, forced activation, and -- critically --
        ``publish_on_failure=False`` so a failed switch never overwrites the
        still-canonical previous exit/goal). Returns
        ``(plan_or_None, status, activated, serialized_or_None)``.
        """
        if not self.evaluation_client.wait_for_service(timeout_sec=0.0):
            return None, "EVALUATION_SERVICE_UNAVAILABLE", False, None
        try:
            evaluation_response = self.evaluation_client.call(
                Trigger.Request(), timeout_sec=self.timeout
            )
        except Exception as exc:  # rclpy transport/service errors
            self.get_logger().error(f"exit evaluation service failed: {exc}")
            return None, "EVALUATION_SERVICE_FAILED", False, None
        if evaluation_response is None:
            return None, "EVALUATION_SERVICE_TIMEOUT", False, None
        if not evaluation_response.success:
            return (
                None, "EXIT_EVALUATOR_NOT_READY:" + evaluation_response.message,
                False, None,
            )
        try:
            plan, status, activated = build_evacuation_decision(
                evaluation_response.message, self.planner,
                expected_frame=self.map_frame,
                risk_first=self.risk_first if risk_first is None else risk_first,
                activate=self.activate or force_activate,
                current_revision=self.current_hazard_revision,
                excluded_exit_ids=excluded_exit_ids,
                candidate_exit_ids=candidate_exit_ids,
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid exit evaluation result: {exc}")
            return None, "INVALID_EVALUATION_RESPONSE", False, None

        payload = plan.to_dict()
        payload["activated"] = activated
        payload["manager_status"] = status
        serialized = json.dumps(
            payload, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        if plan.success or publish_on_failure:
            self.plan_publisher.publish(String(data=serialized))
            if plan.success:
                self.selected_publisher.publish(String(data=plan.selected_exit_id))
            if activated:
                goal = PoseStamped()
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.header.frame_id = self.map_frame
                goal.pose.position.x = plan.selected_approach_position_world[0]
                goal.pose.position.y = plan.selected_approach_position_world[1]
                goal.pose.orientation.w = 1.0
                self.goal_publisher.publish(goal)
        self._status(status)
        return plan, status, activated, serialized

    def _activate_registered_exit(self, exit_id):
        """Activate one registered exit without waiting for a fresh evaluation."""
        item = getattr(self, "registered_exits", {}).get(str(exit_id))
        if item is None:
            return False, "DIRECT_TARGET_NOT_REGISTERED"
        approach = item.approach_position_world or item.position_world
        revision_value = getattr(self, "current_hazard_revision", None)
        revision = 0 if revision_value is None else int(revision_value)
        now = self.get_clock().now()
        payload = {
            "success": True,
            "start_position_world": list(approach),
            "selected_exit_id": item.exit_id,
            "selected_exit_position_world": list(item.position_world),
            "selected_approach_position_world": list(approach),
            "path_world": [],
            "path_grid": [],
            "selected_evaluation": None,
            "all_evaluations": [],
            "failure_reason": None,
            "selection_reason": "direct_registered_exit_activation",
            "created_at": now.nanoseconds / 1e9,
            "hazard_revision": revision,
            "activated": True,
            "manager_status": "ROUTE_ACTIVATED",
        }
        serialized = json.dumps(
            payload, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        self.plan_publisher.publish(String(data=serialized))
        self.selected_publisher.publish(String(data=item.exit_id))
        goal = PoseStamped()
        goal.header.stamp = now.to_msg()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x = approach[0]
        goal.pose.position.y = approach[1]
        goal.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal)
        self._status("ROUTE_ACTIVATED")
        return True, "ROUTE_ACTIVATED"

    def _plan(self, request, response):
        del request
        if not self.enabled:
            return self._failure(response, "DISABLED")
        plan, status, _activated, serialized = self._select_and_activate(
            excluded_exit_ids=tuple(sorted(
                set(getattr(self, "externally_blocked_exit_ids", ()))
                | set(getattr(self, "danger_expected_exit_ids", ()))
            ))
        )
        if plan is None:
            return self._failure(response, status)
        response.success = plan.success and status not in {
            "EVALUATION_STALE", "HAZARD_REVISION_NOT_READY",
            "SELECTED_APPROACH_MISSING",
        }
        response.message = serialized
        return response

    def _on_blocked_exits(self, message):
        try:
            values = json.loads(message.data)
            if not isinstance(values, list):
                raise ValueError("blocked exits must be a JSON list")
            blocked = {str(item).strip() for item in values if str(item).strip()}
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid blocked-exit registry: {exc}")
            return
        self.externally_blocked_exit_ids = blocked
        self.get_logger().info(
            "Externally blocked exits: "
            + (", ".join(sorted(blocked)) if blocked else "none")
        )

    def _on_danger_expected_exits(self, message):
        try:
            values = json.loads(message.data)
            if not isinstance(values, list):
                raise ValueError("danger-expected exits must be a JSON list")
            dangerous = {
                str(item).strip() for item in values if str(item).strip()
            }
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f"invalid danger-expected exit registry: {exc}"
            )
            return
        self.danger_expected_exit_ids = dangerous
        self.get_logger().warning(
            "DANGER_EXPECTED exits: "
            + (", ".join(sorted(dangerous)) if dangerous else "none")
        )

    def _on_switch_request(self, message):
        if not self.enabled:
            return
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict):
                raise ValueError("switch request must be a JSON object")
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid switch request payload: {exc}")
            return
        current_exit_id = request.get("current_exit_id")
        excluded = request.get("excluded_exit_ids")
        excluded = (
            tuple(str(item) for item in excluded) if excluded
            else ((str(current_exit_id),) if current_exit_id else ())
        )
        excluded = tuple(sorted(
            set(excluded) | set(getattr(self, "externally_blocked_exit_ids", ()))
            | set(getattr(self, "danger_expected_exit_ids", ()))
        ))
        candidate = request.get("candidate_exit_ids")
        candidate = None if candidate is None else tuple(str(item) for item in candidate)
        if bool(request.get("direct_target_activation", False)):
            target = (
                candidate[0]
                if candidate is not None and len(candidate) == 1
                else None
            )
            if target is None:
                success, status = False, "DIRECT_TARGET_INVALID"
            else:
                success, status = self._activate_registered_exit(target)
            self.switch_result_publisher.publish(String(data=json.dumps({
                "request_id": request.get("request_id"),
                "success": success,
                "activated": success,
                "status": status,
                "selected_exit_id": target if success else None,
            }, sort_keys=True, separators=(",", ":"), allow_nan=False)))
            return
        plan, status, activated, _serialized = self._select_and_activate(
            excluded_exit_ids=excluded, candidate_exit_ids=candidate,
            risk_first=bool(request.get("risk_first", False)),
            force_activate=True, publish_on_failure=False,
        )
        self.switch_result_publisher.publish(String(data=json.dumps({
            "request_id": request.get("request_id"),
            "success": bool(plan is not None and plan.success),
            "activated": bool(activated),
            "status": status,
            "selected_exit_id": (
                None if plan is None or not plan.success else plan.selected_exit_id
            ),
        }, sort_keys=True, separators=(",", ":"), allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = EvacuationManagerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
