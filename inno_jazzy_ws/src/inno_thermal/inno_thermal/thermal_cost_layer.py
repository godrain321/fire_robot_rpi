"""Project thermal arc observations into a separate static-map-sized grid."""

from __future__ import annotations

from copy import deepcopy
import math

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from inno_thermal.thermal_cost_geometry import (
    GridGeometry,
    ThermalCostState,
    aggregate_cell_costs,
    temperature_to_cost,
    transform_point,
    world_to_grid,
)


WAITING_FOR_STATIC_GRID = "WAITING_FOR_STATIC_GRID"
WAITING_FOR_TF = "WAITING_FOR_TF"
ACTIVE = "ACTIVE"
INVALID_POINTCLOUD = "INVALID_POINTCLOUD"
REQUIRED_POINT_FIELDS = ("x", "y", "z", "intensity")


def _clean_frame(frame_id: str) -> str:
    return str(frame_id).strip().lstrip("/")


class ThermalCostLayer(Node):
    def __init__(self) -> None:
        super().__init__("thermal_cost_layer")
        defaults = {
            "static_grid_topic": "/planning_grid_static",
            "thermal_arc_topic": "/thermal/arc_points",
            "thermal_cost_grid_topic": "/thermal_cost_grid",
            "status_topic": "/thermal_cost_status",
            "target_frame": "map",
            "safe_temperature_c": 20.0,
            "blocked_temperature_c": 60.0,
            # Store the linear normalized ratio. The factory_v5 exponent is
            # applied exactly once by inno_autonav's weighted planner.
            "temperature_power": 1.0,
            "observation_timeout_sec": 2.0,
            "inflation_radius_m": 0.0,
            "publish_rate_hz": 4.0,
            "tf_timeout_sec": 0.2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.static_grid_topic = str(self.get_parameter("static_grid_topic").value)
        self.thermal_arc_topic = str(self.get_parameter("thermal_arc_topic").value)
        self.cost_grid_topic = str(
            self.get_parameter("thermal_cost_grid_topic").value
        )
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.configured_target_frame = str(
            self.get_parameter("target_frame").value
        )
        self.safe_temperature_c = float(
            self.get_parameter("safe_temperature_c").value
        )
        self.blocked_temperature_c = float(
            self.get_parameter("blocked_temperature_c").value
        )
        self.temperature_power = float(self.get_parameter("temperature_power").value)
        observation_timeout_sec = float(
            self.get_parameter("observation_timeout_sec").value
        )
        inflation_radius_m = float(self.get_parameter("inflation_radius_m").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self._validate_parameters(observation_timeout_sec, inflation_radius_m)

        self.state = ThermalCostState(observation_timeout_sec, inflation_radius_m)
        self._static_info = None
        self._static_frame_id = ""
        self._status = None

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.grid_publisher = self.create_publisher(
            OccupancyGrid, self.cost_grid_topic, transient_qos
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, transient_qos
        )
        self.create_subscription(
            OccupancyGrid,
            self.static_grid_topic,
            self._static_grid_callback,
            transient_qos,
        )
        self.create_subscription(
            PointCloud2, self.thermal_arc_topic, self._thermal_arc_callback, 10
        )
        self.create_service(Trigger, "/clear_thermal_costs", self._clear_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._timer_callback
        )
        self._set_status(WAITING_FOR_STATIC_GRID)
        self.get_logger().info(
            f"thermal cost layer: static={self.static_grid_topic}, "
            f"arc={self.thermal_arc_topic}, output={self.cost_grid_topic}"
        )

    def _validate_parameters(
        self, observation_timeout_sec: float, inflation_radius_m: float
    ) -> None:
        # Reuse the pure conversion function for threshold/power validation.
        temperature_to_cost(
            self.safe_temperature_c,
            self.safe_temperature_c,
            self.blocked_temperature_c,
            self.temperature_power,
        )
        if not math.isfinite(observation_timeout_sec) or observation_timeout_sec < 0.0:
            raise ValueError("observation_timeout_sec must be finite and non-negative")
        if not math.isfinite(inflation_radius_m) or inflation_radius_m < 0.0:
            raise ValueError("inflation_radius_m must be finite and non-negative")
        if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and positive")
        if not math.isfinite(self.tf_timeout_sec) or self.tf_timeout_sec < 0.0:
            raise ValueError("tf_timeout_sec must be finite and non-negative")

    def _set_status(self, status: str) -> None:
        if status == self._status:
            return
        self._status = status
        self.status_publisher.publish(String(data=status))
        self.get_logger().info(f"thermal cost status: {status}")

    @staticmethod
    def _geometry_from_message(message: OccupancyGrid) -> GridGeometry:
        origin = message.info.origin
        return GridGeometry(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(origin.position.x),
            origin_y=float(origin.position.y),
            origin_z=float(origin.position.z),
            origin_qx=float(origin.orientation.x),
            origin_qy=float(origin.orientation.y),
            origin_qz=float(origin.orientation.z),
            origin_qw=float(origin.orientation.w),
            frame_id=str(message.header.frame_id),
        )

    def _static_grid_callback(self, message: OccupancyGrid) -> None:
        try:
            geometry = self._geometry_from_message(message)
            if len(message.data) != geometry.width * geometry.height:
                raise ValueError("static grid data length does not match width x height")
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid planning_grid_static ignored: {exc}")
            self._set_status(WAITING_FOR_STATIC_GRID)
            return
        if not _clean_frame(geometry.frame_id):
            self.get_logger().error("planning_grid_static has an empty frame_id")
            self._set_status(WAITING_FOR_STATIC_GRID)
            return

        changed = self.state.set_geometry(geometry)
        self._static_info = deepcopy(message.info)
        self._static_frame_id = geometry.frame_id
        configured = _clean_frame(self.configured_target_frame)
        actual = _clean_frame(self._static_frame_id)
        if configured and configured != actual:
            self.get_logger().warning(
                f"target_frame={self.configured_target_frame!r} differs from static "
                f"grid frame={self._static_frame_id!r}; using the static grid frame"
            )
        if changed:
            self.get_logger().warning(
                "planning_grid_static geometry initialized/changed; all thermal costs cleared: "
                f"{geometry.width}x{geometry.height}, {geometry.resolution:.6f} m/cell, "
                f"frame={geometry.frame_id}"
            )
        if changed or self._status == WAITING_FOR_STATIC_GRID:
            self._set_status(WAITING_FOR_TF)
        self._publish_grid()

    def _thermal_arc_callback(self, message: PointCloud2) -> None:
        if self.state.geometry is None:
            self._set_status(WAITING_FOR_STATIC_GRID)
            return
        source_frame = _clean_frame(message.header.frame_id)
        target_frame = _clean_frame(self._static_frame_id)
        if not source_frame or not target_frame:
            self.get_logger().warning("thermal arc source or static target frame is empty")
            self._set_status(WAITING_FOR_TF)
            return

        field_names = {field.name for field in message.fields}
        missing = set(REQUIRED_POINT_FIELDS) - field_names
        if missing:
            self.get_logger().warning(
                f"thermal PointCloud2 missing required fields: {sorted(missing)}"
            )
            self._set_status(INVALID_POINTCLOUD)
            return
        if int(message.width) * int(message.height) == 0:
            self.get_logger().warning("thermal PointCloud2 is empty")
            self._set_status(INVALID_POINTCLOUD)
            return

        transform = None
        if source_frame != target_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time.from_msg(message.header.stamp),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )
            except TransformException as exc:
                self.get_logger().warning(
                    f"thermal TF unavailable ({target_frame} <- {source_frame}): {exc}"
                )
                self._set_status(WAITING_FOR_TF)
                return

        valid_points = 0
        cell_cost_pairs = []
        try:
            points = point_cloud2.read_points(
                message, field_names=REQUIRED_POINT_FIELDS, skip_nans=False
            )
            for point in points:
                x, y, z, intensity = (float(value) for value in point)
                if not all(math.isfinite(value) for value in (x, y, z, intensity)):
                    self.get_logger().debug("ignoring non-finite thermal arc point")
                    continue
                valid_points += 1
                if transform is not None:
                    item = transform.transform
                    map_x, map_y, _ = transform_point(
                        (x, y, z),
                        (item.translation.x, item.translation.y, item.translation.z),
                        (item.rotation.x, item.rotation.y, item.rotation.z, item.rotation.w),
                    )
                else:
                    map_x, map_y = x, y
                cell = world_to_grid(map_x, map_y, self.state.geometry)
                if cell is None:
                    self.get_logger().debug("ignoring thermal arc point outside static grid")
                    continue
                cost = temperature_to_cost(
                    intensity,
                    self.safe_temperature_c,
                    self.blocked_temperature_c,
                    self.temperature_power,
                )
                cell_cost_pairs.append((cell, cost))
        except (TypeError, ValueError, IndexError) as exc:
            self.get_logger().warning(f"invalid thermal PointCloud2 ignored: {exc}")
            self._set_status(INVALID_POINTCLOUD)
            return

        if valid_points == 0:
            self.get_logger().warning("thermal PointCloud2 has no finite points")
            self._set_status(INVALID_POINTCLOUD)
            return

        frame_costs = aggregate_cell_costs(cell_cost_pairs)
        now_ns = self.get_clock().now().nanoseconds
        self.state.apply_frame(frame_costs, now_ns)
        self._set_status(ACTIVE)
        self._publish_grid()

    def _timer_callback(self) -> None:
        if self.state.geometry is None:
            self._set_status(WAITING_FOR_STATIC_GRID)
            return
        self.state.expire(self.get_clock().now().nanoseconds)
        self._publish_grid()

    def _publish_grid(self) -> None:
        if self.state.geometry is None or self._static_info is None:
            return
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._static_frame_id
        message.info = deepcopy(self._static_info)
        message.data = self.state.flattened()
        self.grid_publisher.publish(message)

    def _clear_callback(self, request, response):
        del request
        count = self.state.clear()
        if self.state.geometry is not None:
            self._publish_grid()
        response.success = True
        response.message = f"cleared {count} thermal cost cells"
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ThermalCostLayer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f"thermal_cost_layer parameter error: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
