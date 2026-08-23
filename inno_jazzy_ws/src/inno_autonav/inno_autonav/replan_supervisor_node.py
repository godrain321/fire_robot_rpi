"""ROS adapter wiring topics to the ROS-independent ReplanSupervisorCore.

Stage 6. Subscribes to the existing Stage 3/5 interfaces only (/hazard/snapshot,
/planned_path, /planner_state, /evacuation/plan) and reuses the existing /goal_pose
interface to request a replan -- it never runs A* itself and never publishes
/planned_path (that stays astar_replanner's job) or /cmd_vel* (that stays
skid_path_follower's job). See replan_supervisor.py for the actual decision logic.
"""

from __future__ import annotations

import json

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, String

from inno_hazard.hazard_belief import HazardGridGeometry
from inno_hazard.hazard_snapshot import decode_hazard_snapshot_message

from .event_replanning import EventReplanningConfig
from .exit_evaluator import ExitHazardSnapshot
from .replan_supervisor import ReplanSupervisorCore, RetryConfig, parse_active_goal_payload
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
        self.core.set_enabled(bool(value("enabled")))

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
            String, str(value("planner_state_topic")), self._on_planner_state, 10,
        )
        self.create_subscription(
            String, str(value("evacuation_plan_topic")), self._on_plan, transient,
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, str(value("goal_topic")), 10,
        )
        self.hold_publisher = self.create_publisher(
            Bool, str(value("hold_topic")), transient,
        )
        self.status_publisher = self.create_publisher(
            String, str(value("status_topic")), transient,
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

    def _on_planner_state(self, message: String) -> None:
        self._publish(self.core.on_planner_state(message.data))

    def _on_plan(self, message: String) -> None:
        self._publish(self.core.on_active_goal(parse_active_goal_payload(message.data)))

    def _on_timer(self) -> None:
        pose = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        pose_2d = None if pose is None else (pose[0], pose[1])
        elapsed = self.get_clock().now().nanoseconds / 1e9
        self._publish(self.core.tick(pose_2d, elapsed))

    def _publish(self, output) -> None:
        self.hold_publisher.publish(Bool(data=bool(output.hold)))
        if output.publish_goal is not None:
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = self.map_frame
            goal.pose.position.x, goal.pose.position.y = output.publish_goal
            goal.pose.orientation.w = 1.0
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
