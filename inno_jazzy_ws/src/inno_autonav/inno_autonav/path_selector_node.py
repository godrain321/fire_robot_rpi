"""Thin ROS adapter: relays whichever upstream Path source the active mode
selects onto the single canonical /planned_path. Never recomputes path fields --
the exact incoming Path message is republished as-is when selected.
"""

from __future__ import annotations

from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .path_selector import PathSelectorCore


class PathSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("path_selector_node")
        defaults = {
            "mode": "WAYPOINT",
            "waypoint_path_topic": "/waypoint_path",
            "astar_path_topic": "/astar_path",
            "planned_path_topic": "/planned_path",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.core = PathSelectorCore(str(value("mode")))

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Path, str(value("waypoint_path_topic")), self._on_waypoint_path, qos,
        )
        self.create_subscription(
            Path, str(value("astar_path_topic")), self._on_astar_path, qos,
        )
        self.path_publisher = self.create_publisher(
            Path, str(value("planned_path_topic")), qos,
        )

    def _on_waypoint_path(self, message: Path) -> None:
        self._apply(self.core.on_waypoint_path(message))

    def _on_astar_path(self, message: Path) -> None:
        self._apply(self.core.on_astar_path(message))

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
