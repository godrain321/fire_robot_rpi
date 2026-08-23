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

from .evacuation_planner import (
    EvacuationPlanner, ExitSelectionConfig, build_evacuation_decision,
)


class EvacuationManagerNode(Node):
    def __init__(self):
        super().__init__("evacuation_manager_node")
        defaults = {
            "enabled": False,
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
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.enabled = bool(value("enabled"))
        self.activate = bool(value("activate_selected_route"))
        self.risk_first = bool(value("risk_first"))
        self.map_frame = str(value("map_frame"))
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

    def _plan(self, request, response):
        del request
        if not self.enabled:
            return self._failure(response, "DISABLED")
        if not self.evaluation_client.wait_for_service(timeout_sec=0.0):
            return self._failure(response, "EVALUATION_SERVICE_UNAVAILABLE")
        try:
            evaluation_response = self.evaluation_client.call(
                Trigger.Request(), timeout_sec=self.timeout
            )
        except Exception as exc:  # rclpy transport/service errors
            self.get_logger().error(f"exit evaluation service failed: {exc}")
            return self._failure(response, "EVALUATION_SERVICE_FAILED")
        if evaluation_response is None:
            return self._failure(response, "EVALUATION_SERVICE_TIMEOUT")
        if not evaluation_response.success:
            return self._failure(
                response, "EXIT_EVALUATOR_NOT_READY:" + evaluation_response.message
            )
        try:
            plan, status, activated = build_evacuation_decision(
                evaluation_response.message, self.planner,
                expected_frame=self.map_frame, risk_first=self.risk_first,
                activate=self.activate,
                current_revision=self.current_hazard_revision,
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid exit evaluation result: {exc}")
            return self._failure(response, "INVALID_EVALUATION_RESPONSE")

        payload = plan.to_dict()
        payload["activated"] = activated
        payload["manager_status"] = status
        serialized = json.dumps(
            payload, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
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
        response.success = plan.success and status not in {
            "EVALUATION_STALE", "HAZARD_REVISION_NOT_READY",
            "SELECTED_APPROACH_MISSING",
        }
        response.message = serialized
        return response


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
