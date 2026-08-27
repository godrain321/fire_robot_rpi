"""ROS adapter wiring topics to the ROS-independent ReplanSupervisorCore.

Standalone compatibility republishes the same canonical ``/goal_pose`` exactly as
Stage 6 did.  With waypoint planning enabled, the same core attempt is routed through
identity-stamped waypoint then A* requests and PathSelector; this node still computes
no path and publishes neither ``/planned_path`` nor ``/cmd_vel``.
"""

from __future__ import annotations

import json

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, Int32, String

from inno_hazard.hazard_belief import HazardGridGeometry
from inno_hazard.hazard_snapshot import decode_hazard_snapshot_message

from .event_replanning import EventReplanningConfig
from .exit_evaluator import ExitHazardSnapshot
from .replan_supervisor import ReplanSupervisorCore, RetryConfig, parse_active_goal_payload
from .same_exit_replanning import SameExitReplanCoordinator
from .tf_utils import TfHelper


class ReplanSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("replan_supervisor_node")
        defaults = {
            "enabled": False,
            "map_frame": "map",
            "base_frame": "base_link",
            "hazard_snapshot_topic": "/hazard/snapshot",
            "planned_path_topic": "/planned_path",
            "waypoint_path_topic": "/waypoint_path",
            "astar_path_topic": "/astar_path",
            "planner_state_topic": "/planner_state",
            "evacuation_plan_topic": "/evacuation/plan",
            "goal_topic": "/goal_pose",
            "hold_topic": "/replanning/hold",
            "status_topic": "/replanning/status",
            "status_rate_hz": 2.0,
            "periodic_enabled": True,
            "periodic_interval_s": 5.0,
            "periodic_travel_distance_m": 2.0,
            "temperature_release_c": 55.0,
            "co_release_ppm": 1400.0,
            "release_confirmation_observations": 3,
            "minimum_replan_interval_s": 0.2,
            "ignore_same_reason_same_revision": True,
            "max_replan_attempts": 5,
            "cooldown_seconds": 0.5,
            "replan_timeout_s": 3.0,
            "waypoint_planning_enabled": False,
            "waypoint_request_topic": "/replanning/waypoint_request",
            "waypoint_result_topic": "/replanning/waypoint_result",
            "astar_request_topic": "/replanning/astar_request",
            "astar_result_topic": "/replanning/astar_result",
            "selector_mode_topic": "/path_selector/mode",
            "drive_mode_topic": "/drive_mode",
            "pause_drive_modes": [3, 4],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.map_frame = str(value("map_frame"))
        self.base_frame = str(value("base_frame"))
        status_rate = float(value("status_rate_hz"))
        if status_rate <= 0.0:
            raise ValueError("status_rate_hz must be positive")
        self._config_overrides = dict(
            enabled=True,
            periodic_enabled=bool(value("periodic_enabled")),
            periodic_interval_s=float(value("periodic_interval_s")),
            periodic_travel_distance_m=float(value("periodic_travel_distance_m")),
            temperature_release_c=float(value("temperature_release_c")),
            co_release_ppm=float(value("co_release_ppm")),
            release_confirmation_observations=int(
                value("release_confirmation_observations")
            ),
            minimum_replan_interval_s=float(value("minimum_replan_interval_s")),
            ignore_same_reason_same_revision=bool(
                value("ignore_same_reason_same_revision")
            ),
        )
        retry = RetryConfig(
            max_replan_attempts=int(value("max_replan_attempts")),
            cooldown_seconds=float(value("cooldown_seconds")),
            replan_timeout_s=float(value("replan_timeout_s")),
        )
        self.core = ReplanSupervisorCore(EventReplanningConfig(**self._config_overrides), retry)
        self._configured_enabled = bool(value("enabled"))
        self._pause_drive_modes = {
            int(mode) for mode in value("pause_drive_modes")
        }
        self.core.set_enabled(self._configured_enabled)
        self.waypoint_planning_enabled = bool(value("waypoint_planning_enabled"))
        self.coordinator = SameExitReplanCoordinator()
        self._awaiting_replacement_path = False

        self.tf = TfHelper(self)

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            Float32MultiArray, str(value("hazard_snapshot_topic")),
            self._on_snapshot, transient,
        )
        self.create_subscription(
            Path, str(value("planned_path_topic")), self._on_path, transient,
        )
        self.create_subscription(
            Path, str(value("waypoint_path_topic")), self._on_waypoint_candidate, transient,
        )
        self.create_subscription(
            Path, str(value("astar_path_topic")), self._on_astar_candidate, transient,
        )
        self.create_subscription(
            String, str(value("planner_state_topic")), self._on_planner_state, 10,
        )
        self.create_subscription(
            String, str(value("evacuation_plan_topic")), self._on_plan, transient,
        )
        self.create_subscription(
            Int32, str(value("drive_mode_topic")), self._on_drive_mode, 10,
        )
        self.create_subscription(
            String, str(value("waypoint_result_topic")), self._on_waypoint_result, 10,
        )
        self.create_subscription(
            String, str(value("astar_result_topic")), self._on_astar_result, 10,
        )
        self.goal_publisher = None
        if not self.waypoint_planning_enabled:
            self.goal_publisher = self.create_publisher(
                PoseStamped, str(value("goal_topic")), 10,
            )
        self.hold_publisher = self.create_publisher(
            Bool, str(value("hold_topic")), transient,
        )
        self.status_publisher = self.create_publisher(
            String, str(value("status_topic")), transient,
        )
        self.waypoint_request_publisher = self.create_publisher(
            String, str(value("waypoint_request_topic")), 10,
        )
        self.astar_request_publisher = self.create_publisher(
            String, str(value("astar_request_topic")), 10,
        )
        self.selector_mode_publisher = self.create_publisher(
            String, str(value("selector_mode_topic")), 10,
        )
        self.create_timer(1.0 / status_rate, self._on_timer)
        self._publish(self.core.current_output())

    def _rebuild_core_if_thresholds_changed(self, snapshot: ExitHazardSnapshot) -> None:
        # Stage 3 thresholds are static ROS parameters for the node's lifetime, so
        # this only ever fires once (on the first snapshot); rebuilding drops
        # in-flight supervisor state, which is safe before any goal is active.
        current = self.core.config
        if (
            current.temperature_block_c == snapshot.temperature_blocked_c
            and current.co_block_ppm == snapshot.co_blocked_ppm
        ):
            return
        was_enabled = self.core.enabled
        new_config = EventReplanningConfig(
            **{
                **self._config_overrides,
                "temperature_block_c": snapshot.temperature_blocked_c,
                "co_block_ppm": snapshot.co_blocked_ppm,
            }
        )
        self.core = ReplanSupervisorCore(new_config, self.core.retry)
        self.core.set_enabled(was_enabled)

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
        self._rebuild_core_if_thresholds_changed(snapshot)
        self._publish(self.core.on_hazard_snapshot(snapshot))

    def _on_path(self, message: Path) -> None:
        coords = tuple(
            (pose.pose.position.x, pose.pose.position.y) for pose in message.poses
        )
        self._publish(self.core.on_planned_path(coords))
        if self._awaiting_replacement_path and coords:
            self._awaiting_replacement_path = False
            self._publish(self.core.on_planner_state("PATH_READY"))

    def _on_planner_state(self, message: String) -> None:
        if self.waypoint_planning_enabled:
            # Global legacy state has no request identity. Stage 8 accepts only the
            # matching JSON result callbacks below; standalone mode keeps Stage 6.
            return
        self._publish(self.core.on_planner_state(message.data))

    @staticmethod
    def _path_stamp_ns(message: Path) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def _on_waypoint_candidate(self, message: Path) -> None:
        self._on_candidate("WAYPOINT", message)

    def _on_astar_candidate(self, message: Path) -> None:
        self._on_candidate("A_STAR", message)

    def _on_candidate(self, source: str, message: Path) -> None:
        if not message.poses:
            return
        final = message.poses[-1].pose.position
        activation = self.coordinator.on_candidate_path(
            source, stamp_ns=self._path_stamp_ns(message),
            goal_world=(final.x, final.y), nonempty=True,
        )
        self._activate_candidate(activation)

    def _activate_candidate(self, activation) -> None:
        if not isinstance(activation, dict):
            return
        self._awaiting_replacement_path = True
        self.selector_mode_publisher.publish(String(
            data=json.dumps(activation, sort_keys=True)
        ))

    def _on_plan(self, message: String) -> None:
        goal = parse_active_goal_payload(message.data)
        if self.waypoint_planning_enabled and self.coordinator.on_goal(goal):
            self._awaiting_replacement_path = False
            self.selector_mode_publisher.publish(String(data="WAYPOINT"))
        self._publish(self.core.on_active_goal(goal))

    def _on_drive_mode(self, message: Int32) -> None:
        """Release replan holds while Mode 3/4 owns an inspection approach."""
        paused = int(message.data) in self._pause_drive_modes
        if paused:
            self.coordinator.on_goal(None)
            self._awaiting_replacement_path = False
            self.core.on_active_goal(None)
        self._publish(self.core.set_enabled(
            self._configured_enabled and not paused
        ))

    def _decode_result(self, message: String):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return None
        return value if isinstance(value, dict) else None

    def _on_waypoint_result(self, message: String) -> None:
        result = self._decode_result(message)
        event = None if result is None else self.coordinator.on_waypoint_result(result)
        if isinstance(event, dict):
            self._activate_candidate(event)
        elif event is not None:
            self._publish(self.core.on_replan_progress())
            self.astar_request_publisher.publish(String(data=json.dumps(event.payload)))

    def _on_astar_result(self, message: String) -> None:
        result = self._decode_result(message)
        event = None if result is None else self.coordinator.on_astar_result(result)
        if isinstance(event, dict):
            self._activate_candidate(event)
        elif event == "A_STAR_FAILED":
            self._publish(self.core.on_planner_state(str(result.get("status", "NO_PATH"))))

    def _on_timer(self) -> None:
        pose = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        pose_2d = None if pose is None else (pose[0], pose[1])
        elapsed = self.get_clock().now().nanoseconds / 1e9
        self._publish(self.core.tick(pose_2d, elapsed))

    def _publish(self, output) -> None:
        self.hold_publisher.publish(Bool(data=bool(output.hold)))
        if output.publish_goal is not None:
            if getattr(self, "waypoint_planning_enabled", False):
                command = self.coordinator.start(
                    output.status.get("hazard_revision"),
                    int(output.status.get("attempt_count", 0)),
                )
                if command is not None:
                    self.waypoint_request_publisher.publish(String(
                        data=json.dumps(command.payload, sort_keys=True)
                    ))
                self.status_publisher.publish(String(
                    data=json.dumps(output.status, sort_keys=True, separators=(",", ":"))
                ))
                return
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = self.map_frame
            goal.pose.position.x, goal.pose.position.y = output.publish_goal
            goal.pose.orientation.w = 1.0
            if self.goal_publisher is not None:
                self.goal_publisher.publish(goal)
            self.get_logger().info(
                f"replan requested: reason={output.status.get('last_replan_reason')} "
                f"attempt={output.status.get('attempt_count')}"
            )
        self.status_publisher.publish(String(
            data=json.dumps(output.status, sort_keys=True, separators=(",", ":"))
        ))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ReplanSupervisorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f"replan_supervisor_node 오류: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
