"""Compose inno_hazard's gas cost overlay onto the existing /planning_grid.

Stage 5 gas -> planner wiring. This node adds NO cost algorithm and knows
nothing about gas thresholds: it max-merges ``/hazard/gas_cost_grid``
(0..99 ratio cost / 100 blocked, produced by ``hazard_belief_node``) onto
``/planning_grid`` (static + dynamic + thermal, produced by
``astar_replanner``) using the exact convention
``inno_autonav.weighted_planner.combine_cost_grids`` already uses for the
thermal layer, and republishes the result on ``/planning_grid_hazard`` for
``waypoint_planner_node``.

A* is untouched -- ``astar_replanner`` already consumes ``/hazard/final_cost``
directly when ``hazard_belief_enabled`` is set.
"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .gas_planning_grid import merge_planning_cells


class PlanningGridHazardMerge(Node):
    def __init__(self) -> None:
        super().__init__("planning_grid_hazard_merge")
        self.declare_parameter("base_grid_topic", "/planning_grid")
        self.declare_parameter("gas_grid_topic", "/hazard/gas_cost_grid")
        self.declare_parameter("output_topic", "/planning_grid_hazard")
        self.declare_parameter("unknown_is_occupied", True)
        base_topic = str(self.get_parameter("base_grid_topic").value)
        gas_topic = str(self.get_parameter("gas_grid_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self.unknown_is_occupied = bool(
            self.get_parameter("unknown_is_occupied").value
        )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._base: OccupancyGrid | None = None
        self._gas: OccupancyGrid | None = None
        self._publisher = self.create_publisher(OccupancyGrid, out_topic, qos)
        self.create_subscription(OccupancyGrid, base_topic, self._on_base, qos)
        self.create_subscription(OccupancyGrid, gas_topic, self._on_gas, qos)

    def _on_base(self, message: OccupancyGrid) -> None:
        self._base = message
        self._publish_merged()

    def _on_gas(self, message: OccupancyGrid) -> None:
        self._gas = message
        self._publish_merged()

    def _publish_merged(self) -> None:
        base = self._base
        if base is None:
            return
        gas = self._gas
        if gas is None or (
            gas.info.width != base.info.width
            or gas.info.height != base.info.height
        ):
            # No usable gas overlay yet: pass the existing grid through
            # unchanged so the waypoint planner degrades to Stage 1-4 behaviour.
            if gas is not None:
                self.get_logger().warn(
                    "gas grid geometry != /planning_grid; passing base through"
                )
            self._publisher.publish(base)
            return

        merged = merge_planning_cells(
            base.data, gas.data, unknown_is_occupied=self.unknown_is_occupied,
        )
        out = OccupancyGrid()
        out.header = base.header
        out.info = base.info
        out.data = merged.astype("int8").reshape(-1).astype(int).tolist()
        self._publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PlanningGridHazardMerge()
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
