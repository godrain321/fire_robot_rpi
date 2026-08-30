"""Thin ROS adapter: relays whichever upstream Path source the active mode
selects onto the single canonical /planned_path. Never recomputes path fields --
the exact incoming Path message is republished as-is when selected.
"""

from __future__ import annotations

import json
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Int32, String

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
            "direct_goal_topic": "/goal_pose",
            "drive_mode_topic": "/drive_mode",
            "direct_goal_modes": [3, 4],
            "map_frame": "map",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.core = PathSelectorCore(str(value("mode")))
        self.map_frame = str(value("map_frame"))
        self.direct_goal_modes = {
            int(mode) for mode in value("direct_goal_modes")
        }
        self._drive_mode = 1
        self._direct_goal_world = None
        self._active_goal = None
        self._pending_activation = None
        self._waypoint_failed_for_goal = False
        self._automatic_astar_fallback = False

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
            PoseStamped, str(value("direct_goal_topic")), self._on_direct_goal, 10,
        )
        self.create_subscription(
            Int32, str(value("drive_mode_topic")), self._on_drive_mode, 10,
        )
        self.create_subscription(
            Empty, "/autonomy_cancel", self._on_cancel, 10,
        )
        self.path_publisher = self.create_publisher(
            Path, str(value("planned_path_topic")), qos,
        )

    def _on_waypoint_path(self, message: Path) -> None:
        if not message.poses:
            if self._drive_mode == 5 and self._active_goal is not None:
                self._waypoint_failed_for_goal = True
                self._activate_astar_fallback_if_ready()
            return
        if not self._matches_active_goal(message):
            return
        self._waypoint_failed_for_goal = False
        if self._automatic_astar_fallback:
            self._automatic_astar_fallback = False
            self._pending_activation = None
            self.core.set_mode("WAYPOINT")
            self._apply(self.core.on_waypoint_path(message))
            self.get_logger().info(
                "Waypoint path recovered; leaving automatic A* fallback."
            )
            return
        if self._complete_pending("WAYPOINT", message):
            return
        self._apply(self.core.on_waypoint_path(message))

    def _on_astar_path(self, message: Path) -> None:
        if not self._matches_active_goal(message):
            return
        if self._complete_pending("A_STAR", message):
            return
        output = self.core.on_astar_path(message)
        if self._activate_astar_fallback_if_ready():
            return
        self._apply(output)

    def _activate_astar_fallback_if_ready(self) -> bool:
        """Use safe A* when Mode 5 has no valid path for the active goal."""
        if (
            self._drive_mode != 5
            or self._active_goal is None
            or self._direct_goal_world is not None
            or self._automatic_astar_fallback
        ):
            return False
        waypoint = self.core.latest("waypoint")
        if (
            not self._waypoint_failed_for_goal
            and waypoint is not None
            and waypoint.poses
            and self._matches_active_goal(waypoint)
        ):
            return False
        candidate = self.core.latest("astar")
        if (
            candidate is None
            or not candidate.poses
            or not self._matches_active_goal(candidate)
        ):
            return False
        self._pending_activation = None
        self._automatic_astar_fallback = True
        self._apply(self.core.set_mode("A_STAR"))
        self.get_logger().warning(
            "Mode 5 has no valid matching waypoint path; "
            "using the matching safe A* path."
        )
        return True

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
        direct_goal = getattr(self, "_direct_goal_world", None)
        if direct_goal is None and self._active_goal is None:
            return True
        if not message.poses:
            return False
        final = message.poses[-1].pose.position
        gx, gy = (
            direct_goal
            if direct_goal is not None
            else self._active_goal.approach_world
        )
        return abs(final.x - gx) <= 1e-6 and abs(final.y - gy) <= 1e-6

    def _on_drive_mode(self, message: Int32) -> None:
        previous = self._drive_mode
        self._drive_mode = int(message.data)
        if (
            previous in self.direct_goal_modes
            and self._drive_mode not in self.direct_goal_modes
        ):
            # Never release the cached inspection path after returning to Mode 5.
            # The orchestrator republishes the interrupted/new canonical route.
            self._direct_goal_world = None
            self._pending_activation = None
            self._waypoint_failed_for_goal = False
            self._automatic_astar_fallback = False
            self.core.clear()
            self.core.set_mode("WAYPOINT")
            self.path_publisher.publish(Path())

    def _on_direct_goal(self, message: PoseStamped) -> None:
        """Select direct A* only for the proven Mode 3/4 inspection path."""
        if self._drive_mode not in self.direct_goal_modes:
            return
        if (
            message.header.frame_id
            and message.header.frame_id.lstrip("/") != self.map_frame.lstrip("/")
        ):
            return
        goal = (float(message.pose.position.x), float(message.pose.position.y))
        if not all(math.isfinite(value) for value in goal):
            return
        self._direct_goal_world = goal
        self._pending_activation = None
        output = self.core.set_mode("A_STAR")
        # DDS may deliver /astar_path before this /goal_pose callback. Release
        # that cached path only when its endpoint proves it belongs to this goal.
        if output.publish and self._matches_active_goal(output.payload):
            self._apply(output)

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
            self._direct_goal_world = None
            self._active_goal = goal
            self._pending_activation = None
            self._waypoint_failed_for_goal = False
            self._automatic_astar_fallback = False
            output = self.core.set_mode("WAYPOINT")
            # If the new waypoint Path beat this plan message across DDS topics,
            # release it only when its endpoint identifies this canonical goal.
            if output.publish and goal is not None and output.payload.poses:
                final = output.payload.poses[-1].pose.position
                if (final.x, final.y) == goal.approach_world:
                    self._apply(output)
            # DDS can also deliver the matching A* Path before this plan. If
            # there is no valid waypoint path for the newly activated goal,
            # release that safe candidate immediately instead of waiting for
            # an explicit empty waypoint failure message.
            self._activate_astar_fallback_if_ready()

    def _on_cancel(self, _message: Empty) -> None:
        """Clear selected/cached paths and explicitly clear /planned_path."""
        self._active_goal = None
        self._direct_goal_world = None
        self._pending_activation = None
        self._waypoint_failed_for_goal = False
        self._automatic_astar_fallback = False
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
