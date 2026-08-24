"""Thin ROS adapter: relays whichever upstream Path source the active mode
selects onto the single canonical /planned_path. Never recomputes path fields --
the exact incoming Path message is republished as-is when selected.
"""

from __future__ import annotations

import json

from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

from .path_selector import PathSelectorCore
from .replan_supervisor import parse_active_goal_payload


class PathSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("path_selector_node")
        defaults = {
            "mode": "WAYPOINT",
            "waypoint_path_topic": "/waypoint_path",
            "astar_path_topic": "/astar_path",
            "planned_path_topic": "/planned_path",
            "mode_topic": "/path_selector/mode",
            "evacuation_plan_topic": "/evacuation/plan",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.core = PathSelectorCore(str(value("mode")))
        self._active_goal = None
        self._pending_activation = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Path, str(value("waypoint_path_topic")), self._on_waypoint_path, qos,
        )
        self.create_subscription(
            Path, str(value("astar_path_topic")), self._on_astar_path, qos,
        )
        self.create_subscription(String, str(value("mode_topic")), self._on_mode, 10)
        self.create_subscription(
            String, str(value("evacuation_plan_topic")), self._on_new_goal, qos,
        )
        self.create_subscription(
            Empty, "/autonomy_cancel", self._on_cancel, 10,
        )
        self.path_publisher = self.create_publisher(
            Path, str(value("planned_path_topic")), qos,
        )

    def _on_waypoint_path(self, message: Path) -> None:
        if not self._matches_active_goal(message):
            return
        if self._complete_pending("WAYPOINT", message):
            return
        self._apply(self.core.on_waypoint_path(message))

    def _on_astar_path(self, message: Path) -> None:
        if not self._matches_active_goal(message):
            return
        if self._complete_pending("A_STAR", message):
            return
        self._apply(self.core.on_astar_path(message))

    @staticmethod
    def _path_stamp_ns(message: Path) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def _complete_pending(self, source: str, message: Path) -> bool:
        pending = self._pending_activation
        if not isinstance(pending, dict) or pending.get("mode") != source:
            return False
        if self._path_stamp_ns(message) != int(pending.get("path_stamp_ns", -1)):
            return False
        self._pending_activation = None
        self.core.set_mode(source)
        output = (
            self.core.on_waypoint_path(message) if source == "WAYPOINT"
            else self.core.on_astar_path(message)
        )
        self._apply(output)
        return True

    def _matches_active_goal(self, message: Path) -> bool:
        if self._active_goal is None:
            return True
        if not message.poses:
            return False
        final = message.poses[-1].pose.position
        gx, gy = self._active_goal.approach_world
        return abs(final.x - gx) <= 1e-6 and abs(final.y - gy) <= 1e-6

    def _on_mode(self, message: String) -> None:
        try:
            activation = json.loads(message.data)
        except (TypeError, ValueError):
            activation = None
        if not isinstance(activation, dict):
            self._pending_activation = None
            self.core.set_mode(message.data)
            return
        source = str(activation.get("mode", ""))
        if source not in ("WAYPOINT", "A_STAR"):
            return
        self._pending_activation = activation
        cached = self.core.latest("waypoint" if source == "WAYPOINT" else "astar")
        if cached is not None:
            self._complete_pending(source, cached)

    def _on_new_goal(self, message: String) -> None:
        goal = parse_active_goal_payload(message.data)
        if goal != self._active_goal:
            self._active_goal = goal
            self._pending_activation = None
            output = self.core.set_mode("WAYPOINT")
            # If the new waypoint Path beat this plan message across DDS topics,
            # release it only when its endpoint identifies this canonical goal.
            if output.publish and goal is not None and output.payload.poses:
                final = output.payload.poses[-1].pose.position
                if (final.x, final.y) == goal.approach_world:
                    self._apply(output)

    def _on_cancel(self, _message: Empty) -> None:
        """Clear selected/cached paths and explicitly clear /planned_path."""
        self._active_goal = None
        self._pending_activation = None
        self.core.clear()
        self.path_publisher.publish(Path())

    def _apply(self, output) -> None:
        if output.publish and output.payload is not None:
            self.path_publisher.publish(output.payload)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PathSelectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
