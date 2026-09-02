"""ROS adapter wiring topics to the ROS-independent ExitSwitchingCore.

Stage 7. Subscribes to Stage 5/6's existing interfaces only (/replanning/status,
/evacuation/plan, /hazard/snapshot, /planned_path) plus the new
/evacuation/switch_result ack, and calls the existing read-only /evaluate_exits
service itself to cheaply *rank* soft-switch candidates (a "peek") without ever
activating anything -- activation only ever happens inside
EvacuationManagerNode, via the new /evacuation/switch_request it owns. This node
never runs A*, never publishes /planned_path or /cmd_vel*, and never calls
/plan_evacuation.
"""

from __future__ import annotations

import json

import numpy as np

from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Int32, String
from std_srvs.srv import Trigger

from inno_hazard.hazard_belief import HazardGridGeometry
from inno_hazard.hazard_snapshot import decode_hazard_snapshot_message

from .evacuation_planner import (
    EvacuationPlanner, ExitSelectionConfig, parse_evaluation_batch_json,
)
from .exit_evaluator import ExitHazardSnapshot, load_exit_registry
from .exit_switching import ExitSwitchingConfig, evaluate_path_cost
from .exit_switching_orchestrator import (
    ExitSwitchingCore, ForcedProximitySwitch, PeekResult,
    parse_switch_result_payload,
)
from .replan_supervisor import parse_active_goal_payload
from .tf_utils import TfHelper


def decode_live_temperature_observations(message):
    """Decode volatile ``col,row,celsius`` observations from one frame."""
    values = np.asarray(message.data, dtype=float)
    if values.size % 3 != 0:
        raise ValueError("live temperature payload length must be divisible by 3")
    if len(message.layout.dim) == 2:
        rows = int(message.layout.dim[0].size)
        fields = int(message.layout.dim[1].size)
        if fields != 3 or rows * fields != values.size:
            raise ValueError("live temperature dimensions are invalid")
    return tuple(
        (int(col), int(row), float(temperature))
        for col, row, temperature in values.reshape(-1, 3)
        if all(np.isfinite((col, row, temperature)))
    )


def decode_xy_pairs(values):
    """Decode a flat ROS numeric array as finite map-frame x,y pairs."""
    numbers = tuple(float(value) for value in values)
    if not numbers or len(numbers) % 2 != 0 or not all(np.isfinite(numbers)):
        raise ValueError("trigger waypoint positions must be finite x,y pairs")
    return tuple(zip(numbers[0::2], numbers[1::2]))


class ExitSwitchingNode(Node):
    def __init__(self) -> None:
        super().__init__("exit_switching_node")
        defaults = {
            "enabled": False,
            "map_frame": "map",
            "base_frame": "base_link",
            "exit_registry_file": "",
            "supervisor_status_topic": "/replanning/status",
            "evacuation_plan_topic": "/evacuation/plan",
            "switch_request_topic": "/evacuation/switch_request",
            "switch_result_topic": "/evacuation/switch_result",
            "hazard_snapshot_topic": "/hazard/snapshot",
            "live_temperature_observations_topic": (
                "/hazard/live_temperature_observations"
            ),
            "planned_path_topic": "/planned_path",
            "status_topic": "/exit_switching/status",
            "danger_expected_exits_topic": (
                "/evacuation/danger_expected_exits"
            ),
            "exit_evaluation_service": "/evaluate_exits",
            "evaluation_service_timeout_s": 10.0,
            "status_rate_hz": 2.0,
            "evaluation_window": 6,
            "danger_expected_min_temperature_c": 36.0,
            "danger_expected_confirmation_sec": 3.0,
            "danger_expected_max_observation_gap_sec": 1.0,
            "danger_expected_path_radius_m": 0.30,
            "minimum_direction_difference_deg": 90.0,
            "switch_cooldown_sec": 10.0,
            "additional_travel_before_switch_m": 1.0,
            "prefer_confirmed_usable_exit": True,
            "fallback_to_shortest_reachable_exit": True,
            "primary_key": "path_length_m",
            "secondary_key": "accumulated_risk_cost",
            "final_tie_breaker": "exit_id",
            "float_tolerance": 1e-6,
            "drive_mode_topic": "/drive_mode",
            "pause_drive_modes": [3, 4],
            # final2 demonstration: while navigating EXIT2, entering the 1 m
            # neighbourhood of w79, w75, or w78 marks EXIT2 DANGER_EXPECTED and
            # requests EXIT3 as the only replacement candidate.
            "demo_force_proximity_switch_enabled": False,
            # A non-empty float default makes rclpy declare DOUBLE_ARRAY.
            # The field profile overrides these inert coordinates whenever
            # the trigger is enabled.
            "demo_force_trigger_waypoint_positions": [0.0, 0.0],
            "demo_force_trigger_radius_m": 1.0,
            "demo_force_danger_exit_id": "EXIT2",
            "demo_force_target_exit_id": "EXIT3",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.map_frame = str(value("map_frame"))
        self.base_frame = str(value("base_frame"))
        exit_file = str(value("exit_registry_file")).strip()
        if not exit_file:
            raise ValueError("exit_registry_file is required")
        exits = load_exit_registry(exit_file, self.map_frame)
        exit_positions_world = {item.exit_id: item.position_world for item in exits}
        self._exit_ids = frozenset(exit_positions_world)
        self.evaluation_timeout = float(value("evaluation_service_timeout_s"))
        status_rate = float(value("status_rate_hz"))
        if self.evaluation_timeout <= 0.0 or status_rate <= 0.0:
            raise ValueError("evaluation_service_timeout_s/status_rate_hz must be positive")

        config = ExitSwitchingConfig(
            evaluation_window=int(value("evaluation_window")),
            danger_expected_min_temperature_c=float(value("danger_expected_min_temperature_c")),
            danger_expected_confirmation_sec=float(
                value("danger_expected_confirmation_sec")
            ),
            danger_expected_max_observation_gap_sec=float(
                value("danger_expected_max_observation_gap_sec")
            ),
            danger_expected_path_radius_m=float(
                value("danger_expected_path_radius_m")
            ),
            minimum_direction_difference_deg=float(value("minimum_direction_difference_deg")),
            switch_cooldown_sec=float(value("switch_cooldown_sec")),
            additional_travel_before_switch_m=float(value("additional_travel_before_switch_m")),
        )
        forced_proximity_switch = None
        if bool(value("demo_force_proximity_switch_enabled")):
            forced_proximity_switch = ForcedProximitySwitch(
                source_exit_id=str(value("demo_force_danger_exit_id")),
                target_exit_id=str(value("demo_force_target_exit_id")),
                trigger_positions_world=decode_xy_pairs(
                    value("demo_force_trigger_waypoint_positions")
                ),
                radius_m=float(value("demo_force_trigger_radius_m")),
            )
        self.core = ExitSwitchingCore(
            config, exit_positions_world, forced_proximity_switch,
        )
        self._configured_enabled = bool(value("enabled"))
        self._pause_drive_modes = {
            int(mode) for mode in value("pause_drive_modes")
        }
        self.core.set_enabled(self._configured_enabled)
        self.peek_planner = EvacuationPlanner(ExitSelectionConfig(
            prefer_confirmed_usable_exit=bool(value("prefer_confirmed_usable_exit")),
            fallback_to_shortest_reachable_exit=bool(value("fallback_to_shortest_reachable_exit")),
            primary_key=str(value("primary_key")),
            secondary_key=str(value("secondary_key")),
            final_tie_breaker=str(value("final_tie_breaker")),
            float_tolerance=float(value("float_tolerance")),
        ))
        self.snapshot: ExitHazardSnapshot | None = None
        self.tf = TfHelper(self)

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.evaluation_client = self.create_client(
            Trigger, str(value("exit_evaluation_service")),
        )
        self.create_subscription(
            String, str(value("supervisor_status_topic")), self._on_supervisor_status, transient,
        )
        self.create_subscription(
            String, str(value("evacuation_plan_topic")), self._on_plan, transient,
        )
        self.create_subscription(
            String, str(value("switch_result_topic")), self._on_switch_result, transient,
        )
        self.create_subscription(
            Float32MultiArray, str(value("hazard_snapshot_topic")), self._on_snapshot, transient,
        )
        self.create_subscription(
            Float32MultiArray,
            str(value("live_temperature_observations_topic")),
            self._on_live_temperature_observations, 10,
        )
        self.create_subscription(
            Path, str(value("planned_path_topic")), self._on_path, transient,
        )
        self.create_subscription(
            Int32, str(value("drive_mode_topic")), self._on_drive_mode, 10,
        )
        self.switch_request_publisher = self.create_publisher(
            String, str(value("switch_request_topic")), 10,
        )
        self.status_publisher = self.create_publisher(
            String, str(value("status_topic")), transient,
        )
        self.danger_expected_exits_publisher = self.create_publisher(
            String, str(value("danger_expected_exits_topic")), transient,
        )
        self._last_danger_expected_exit_ids = ()
        self.create_timer(1.0 / status_rate, self._on_timer)
        self._apply(self.core.current_output())

    def _perform_peek(self, request) -> PeekResult:
        if self.snapshot is None or not self.evaluation_client.wait_for_service(timeout_sec=0.0):
            return PeekResult(False, None, None, False)
        try:
            response = self.evaluation_client.call(
                Trigger.Request(), timeout_sec=self.evaluation_timeout,
            )
        except Exception as exc:  # rclpy transport/service errors
            self.get_logger().error(f"peek exit evaluation failed: {exc}")
            return PeekResult(False, None, None, False)
        if response is None or not response.success:
            return PeekResult(False, None, None, False)
        try:
            batch = parse_evaluation_batch_json(response.message, self.map_frame)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid peek evaluation batch: {exc}")
            return PeekResult(False, None, None, False)
        # A single coherent batch (one hazard_revision) is reused for both the
        # opposite-direction attempt and the any-safe-exit fallback -- never two
        # separate /evaluate_exits calls with potentially different revisions
        # (spec section 36).
        plan = self.peek_planner.plan(
            batch, risk_first=request.risk_first,
            excluded_exit_ids=request.excluded_exit_ids,
            candidate_exit_ids=request.candidate_exit_ids,
        )
        used_fallback = False
        if not plan.success and request.candidate_exit_ids is not None:
            plan = self.peek_planner.plan(
                batch, risk_first=request.risk_first,
                excluded_exit_ids=request.excluded_exit_ids, candidate_exit_ids=None,
            )
            used_fallback = True
        if not plan.success:
            return PeekResult(False, None, None, used_fallback)
        cost = evaluate_path_cost(plan.path_grid, self.snapshot.final_cost)
        return PeekResult(True, plan.selected_exit_id, None if cost is None else cost[1], used_fallback)

    def _apply(self, output) -> None:
        danger_ids = tuple(output.status.get("danger_expected_exit_ids", ()))
        if danger_ids != self._last_danger_expected_exit_ids:
            added = sorted(
                set(danger_ids) - set(self._last_danger_expected_exit_ids)
            )
            self._last_danger_expected_exit_ids = danger_ids
            self.danger_expected_exits_publisher.publish(String(data=json.dumps(
                list(danger_ids), separators=(",", ":"),
            )))
            for exit_id in added:
                self.get_logger().warning(
                    f"{exit_id} -> DANGER_EXPECTED"
                )
        if output.peek_request is not None:
            self._apply(self.core.on_peek_result(self._perform_peek(output.peek_request)))
            return
        if output.switch_request is not None:
            if output.switch_request.reason.startswith(
                "waypoint_proximity:"
            ):
                target = output.switch_request.candidate_exit_ids[0]
                self.get_logger().warning(
                    "[출구 전환] w79/w75/w78 1m 이내 진입: "
                    f"{output.switch_request.current_exit_id}를 "
                    f"DANGER_EXPECTED로 변경하고 {target} 경로를 요청합니다."
                )
            self.switch_request_publisher.publish(String(data=json.dumps(
                output.switch_request.to_payload(), sort_keys=True, separators=(",", ":"),
            )))
        self.status_publisher.publish(String(data=json.dumps(
            output.status, sort_keys=True, separators=(",", ":"),
        )))

    def _on_supervisor_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        state = payload.get("state") if isinstance(payload, dict) else None
        if isinstance(state, str):
            self._apply(self.core.on_supervisor_status(state))

    def _on_plan(self, message: String) -> None:
        goal = parse_active_goal_payload(message.data)
        # /evacuation/plan is also reused by Mode 3 inspection.  Such a plan
        # has an id such as MODE3_INSPECTION and must not replace the real
        # evacuation exit retained across the short inspection pause.
        if goal is None or goal.exit_id not in self._exit_ids:
            return
        self._apply(self.core.on_active_goal(goal))

    def _on_drive_mode(self, message: Int32) -> None:
        """Prevent exit-switch requests while the inspector exclusively drives."""
        paused = int(message.data) in self._pause_drive_modes
        if paused:
            self.core.set_enabled(False)
            # Keep the active evacuation exit. Mode 5 resumes the same route
            # after Mode 3/4, and it may not republish the canonical plan.
        self._apply(self.core.set_enabled(
            self._configured_enabled and not paused
        ))

    def _on_switch_result(self, message: String) -> None:
        ack = parse_switch_result_payload(message.data)
        if ack is not None:
            self._apply(self.core.on_switch_result(ack))

    def _on_snapshot(self, message: Float32MultiArray) -> None:
        try:
            metadata, layers = decode_hazard_snapshot_message(message)
            geometry = HazardGridGeometry(
                layers["final_cost"].shape[1], layers["final_cost"].shape[0],
                float(metadata["resolution"]), float(metadata["origin_x"]),
                float(metadata["origin_y"]), float(metadata["origin_yaw"]),
                str(metadata["frame_id"]),
            )
            snapshot = ExitHazardSnapshot(
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
        except (TypeError, ValueError, KeyError) as exc:
            self.get_logger().error(f"invalid hazard snapshot: {exc}")
            return
        if geometry.frame_id.lstrip("/") != self.map_frame.lstrip("/"):
            self.get_logger().error("hazard snapshot frame differs from map_frame")
            return
        self.snapshot = snapshot
        self._apply(self.core.on_hazard_snapshot(snapshot))

    def _on_live_temperature_observations(
        self, message: Float32MultiArray,
    ) -> None:
        try:
            observations = decode_live_temperature_observations(message)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f"invalid live temperature observations: {exc}"
            )
            return
        now = self.get_clock().now().nanoseconds / 1e9
        self._apply(self.core.on_live_temperature_observations(
            observations, now,
        ))

    def _on_path(self, message: Path) -> None:
        coords = tuple(
            (pose.pose.position.x, pose.pose.position.y) for pose in message.poses
        )
        self._apply(self.core.on_planned_path(coords))

    def _on_timer(self) -> None:
        pose = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if pose is None:
            return
        elapsed = self.get_clock().now().nanoseconds / 1e9
        self._apply(self.core.tick((pose[0], pose[1]), elapsed, pose[2]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ExitSwitchingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f"exit_switching_node 오류: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
