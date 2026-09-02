"""Adapt ROS sensor observations to the ROS-independent HazardBelief."""

from __future__ import annotations

from copy import deepcopy

import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import (
    Bool, Float32, Float32MultiArray, MultiArrayDimension, String, UInt64,
)
from tf2_ros import Buffer, TransformException, TransformListener

from inno_thermal.thermal_cost_geometry import (
    GridGeometry,
    latest_transform_is_fresh,
    quaternion_to_yaw,
)

from .gas_planning_grid import gas_overlay_cells
from .hazard_belief import HazardBelief, HazardBeliefConfig, HazardGridGeometry
from .hazard_snapshot import hazard_snapshot_message
from .thermal_adapter import localized_temperature_cells


FIELDS = ("x", "y", "z", "intensity")


def float_grid_message(values):
    array = np.asarray(values, dtype=np.float32)
    message = Float32MultiArray()
    height = MultiArrayDimension(label="height", size=array.shape[0], stride=array.size)
    width = MultiArrayDimension(label="width", size=array.shape[1], stride=array.shape[1])
    message.layout.dim = [height, width]
    message.data = array.reshape(-1).tolist()
    return message


def live_temperature_observations_message(observations):
    """Encode only this thermal frame's localized ``col,row,celsius`` cells."""
    values = np.asarray(
        [(cell[0], cell[1], temperature) for cell, temperature in observations],
        dtype=np.float32,
    ).reshape(-1, 3)
    message = Float32MultiArray()
    message.layout.dim = [
        MultiArrayDimension(
            label="observations", size=values.shape[0], stride=values.size,
        ),
        MultiArrayDimension(label="col,row,celsius", size=3, stride=3),
    ]
    message.data = values.reshape(-1).tolist()
    return message


class HazardBeliefNode(Node):
    def __init__(self):
        super().__init__("hazard_belief_node")
        defaults = {
            "static_grid_topic": "/planning_grid_static",
            "dynamic_grid_topic": "/dynamic_obstacle_grid",
            "thermal_arc_topic": "/thermal/arc_points",
            "live_temperature_observations_topic": (
                "/hazard/live_temperature_observations"
            ),
            "thermal_enabled": True,
            "co_topic": "/hazard/co_ppm",
            "base_frame": "base_link",
            "co_enabled": False,
            "gas_input_mode": "legacy_ppm",
            "base_cost": 1.0,
            "temperature_safe_c": 40.0,
            "temperature_cost_scale_max_c": 60.0,
            "temperature_blocked_c": 60.0,
            "temperature_weight": 24.0,
            "temperature_power": 1.5,
            "co_safe_ppm": 100.0,
            "co_blocked_ppm": 1600.0,
            "gas_safe_adc": 0.0,
            "gas_blocked_adc": 4096.0,
            "co_weight": 8.0,
            "co_power": 2.0,
            "gas_update_radius_m": 0.0,
            "gas_gaussian_sigma_m": 0.5,
            "unknown_penalty": 0.0,
            "unobserved_temperature_penalty": 0.0,
            "unobserved_co_penalty": 0.0,
            "stale_enabled": True,
            "stale_grace_period_s": 5.0,
            "stale_cost_per_second": 0.05,
            "stale_maximum_cost": 2.0,
            "stale_apply_to_temperature": True,
            "stale_apply_to_co": True,
            "fire_localization_enabled": False,
            "thermal_stream_timeout_s": 3.0,
            "tf_timeout_s": 0.2,
            "latest_tf_fallback_tolerance_sec": 1.0,
            "publish_rate_hz": 4.0,
            "initial_route_bypass_topic": "/hazard/initial_route_bypass",
            "publish_diagnostic_grids": False,
            "publish_visualization_grid": True,
            "visualization_publish_rate_hz": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.static_topic = str(self.get_parameter("static_grid_topic").value)
        self.dynamic_topic = str(self.get_parameter("dynamic_grid_topic").value)
        self.thermal_topic = str(self.get_parameter("thermal_arc_topic").value)
        self.live_temperature_observations_topic = str(
            self.get_parameter("live_temperature_observations_topic").value
        )
        self.thermal_enabled = bool(
            self.get_parameter("thermal_enabled").value
        )
        self.co_topic = str(self.get_parameter("co_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.co_enabled = bool(self.get_parameter("co_enabled").value)
        self.fire_localization_enabled = bool(
            self.get_parameter("fire_localization_enabled").value
        )
        self.thermal_timeout = float(
            self.get_parameter("thermal_stream_timeout_s").value
        )
        self.tf_timeout = float(self.get_parameter("tf_timeout_s").value)
        self.latest_tf_fallback_tolerance_sec = float(
            self.get_parameter("latest_tf_fallback_tolerance_sec").value
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.initial_route_bypass_topic = str(
            self.get_parameter("initial_route_bypass_topic").value
        )
        self.publish_diagnostic_grids = bool(
            self.get_parameter("publish_diagnostic_grids").value
        )
        self.publish_visualization_grid = bool(
            self.get_parameter("publish_visualization_grid").value
        )
        visualization_rate = float(
            self.get_parameter("visualization_publish_rate_hz").value
        )
        self.config = HazardBeliefConfig(
            base_cost=float(self.get_parameter("base_cost").value),
            temperature_safe_c=float(self.get_parameter("temperature_safe_c").value),
            temperature_cost_scale_max_c=float(
                self.get_parameter("temperature_cost_scale_max_c").value
            ),
            temperature_blocked_c=float(self.get_parameter("temperature_blocked_c").value),
            temperature_weight=float(self.get_parameter("temperature_weight").value),
            temperature_power=float(self.get_parameter("temperature_power").value),
            co_enabled=self.co_enabled,
            gas_input_mode=str(self.get_parameter("gas_input_mode").value),
            co_safe_ppm=float(self.get_parameter("co_safe_ppm").value),
            co_blocked_ppm=float(self.get_parameter("co_blocked_ppm").value),
            gas_safe_adc=float(self.get_parameter("gas_safe_adc").value),
            gas_blocked_adc=float(self.get_parameter("gas_blocked_adc").value),
            co_weight=float(self.get_parameter("co_weight").value),
            co_power=float(self.get_parameter("co_power").value),
            gas_update_radius_m=float(self.get_parameter("gas_update_radius_m").value),
            gas_gaussian_sigma_m=float(self.get_parameter("gas_gaussian_sigma_m").value),
            unknown_penalty=float(self.get_parameter("unknown_penalty").value),
            unobserved_temperature_penalty=float(
                self.get_parameter("unobserved_temperature_penalty").value
            ),
            unobserved_co_penalty=float(
                self.get_parameter("unobserved_co_penalty").value
            ),
            stale_enabled=bool(self.get_parameter("stale_enabled").value),
            stale_grace_period_s=float(self.get_parameter("stale_grace_period_s").value),
            stale_cost_per_second=float(self.get_parameter("stale_cost_per_second").value),
            stale_maximum_cost=float(self.get_parameter("stale_maximum_cost").value),
            stale_apply_to_temperature=bool(
                self.get_parameter("stale_apply_to_temperature").value
            ),
            stale_apply_to_co=bool(
                self.get_parameter("stale_apply_to_co").value
            ),
        )
        if (
            publish_rate <= 0.0
            or visualization_rate <= 0.0
            or self.thermal_timeout < 0.0
            or self.tf_timeout < 0.0
        ):
            raise ValueError("hazard timing parameters are invalid")
        latest_transform_is_fresh(
            0, 0, self.latest_tf_fallback_tolerance_sec
        )
        if self.fire_localization_enabled:
            raise ValueError(
                "fire_localization_enabled requires depth/ray-cell metadata; "
                "/thermal/arc_points does not provide it"
            )
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.belief = None
        self.grid_geometry = None
        self.static_info = None
        self.frame_id = ""
        self.last_thermal_ns = None
        self.initial_route_bypass = False
        self.last_visual_publish_ns = None
        self.visualization_publish_period_ns = int(1e9 / visualization_rate)
        self.status = ""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.exact_publisher = self.create_publisher(
            Float32MultiArray, "/hazard/final_cost", qos
        )
        self.vis_publisher = self.create_publisher(
            OccupancyGrid, "/hazard/final_cost_grid_vis", qos
        )
        self.temperature_publisher = self.create_publisher(
            Float32MultiArray, "/hazard/temperature_grid", qos
        )
        self.co_publisher = self.create_publisher(
            Float32MultiArray, "/hazard/co_grid", qos
        )
        # Stage 5: gas belief re-expressed in the raw /planning_grid encoding
        # (0..99 ratio cost, 100 = at/above blocked threshold) so the waypoint
        # pipeline can consume gas the same way it already consumes thermal.
        self.gas_cost_grid_publisher = self.create_publisher(
            OccupancyGrid, "/hazard/gas_cost_grid", qos
        )
        self.fire_publisher = self.create_publisher(
            Float32MultiArray, "/hazard/fire_probability_grid", qos
        )
        self.revision_publisher = self.create_publisher(UInt64, "/hazard/revision", qos)
        self.status_publisher = self.create_publisher(String, "/hazard/status", qos)
        self.snapshot_publisher = self.create_publisher(
            Float32MultiArray, "/hazard/snapshot", qos
        )
        # Volatile by design: a newly started consumer must never receive an
        # old hot frame and mistake it for a current observation.
        self.live_temperature_observations_publisher = self.create_publisher(
            Float32MultiArray, self.live_temperature_observations_topic, 10
        )
        self.snapshot_subscription_count = (
            self.snapshot_publisher.get_subscription_count()
        )
        self.create_subscription(OccupancyGrid, self.static_topic, self._static, qos)
        self.create_subscription(OccupancyGrid, self.dynamic_topic, self._dynamic, qos)
        self.create_subscription(
            Bool, self.initial_route_bypass_topic,
            self._initial_route_bypass_callback, qos,
        )
        if self.thermal_enabled:
            self.create_subscription(
                PointCloud2, self.thermal_topic, self._thermal, 10
            )
        if self.co_enabled:
            self.create_subscription(Float32, self.co_topic, self._co, 10)
        self.create_timer(1.0 / publish_rate, self._timer)
        self._set_status("WAITING_FOR_STATIC_GRID")

    @staticmethod
    def _ros_geometry(message):
        origin = message.info.origin
        return GridGeometry(
            int(message.info.width), int(message.info.height),
            float(message.info.resolution),
            float(origin.position.x), float(origin.position.y),
            float(origin.position.z), float(origin.orientation.x),
            float(origin.orientation.y), float(origin.orientation.z),
            float(origin.orientation.w), str(message.header.frame_id),
        )

    def _set_status(self, value):
        changed = value != self.status
        if changed:
            self.status = value
            self.status_publisher.publish(String(data=value))
        return changed

    def _current_status(self):
        if self.belief is None:
            return "WAITING_FOR_STATIC_GRID"
        if self.initial_route_bypass:
            return "ACTIVE_INITIAL_STATIC_DYNAMIC_ONLY"
        if not self.thermal_enabled:
            return "ACTIVE_STATIC_DYNAMIC_ONLY"
        if self.last_thermal_ns is None:
            return "WAITING_FOR_THERMAL"
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_thermal_ns > self.thermal_timeout * 1e9:
            return "THERMAL_STREAM_STALE"
        return "ACTIVE_THERMAL_ONLY" if not self.co_enabled else "ACTIVE"

    def _initial_route_bypass_callback(self, message):
        requested = bool(message.data)
        if requested == self.initial_route_bypass:
            return
        self.initial_route_bypass = requested
        if self.belief is None:
            return
        self._set_status(self._current_status())
        # This is a real planning-view change: publish the static/dynamic-only
        # view on entry and the accumulated thermal view once on release.
        self._publish()

    def _static(self, message):
        try:
            geometry = self._ros_geometry(message)
            data = np.asarray(message.data, dtype=np.int16).reshape(
                geometry.height, geometry.width
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid static grid: {exc}")
            self._set_status("INVALID_STATIC_GRID")
            return
        static_obstacles = data >= 100
        same_map = (
            self.belief is not None
            and geometry == self.grid_geometry
            and np.array_equal(
                static_obstacles, self.belief.static_obstacle_map
            )
        )
        self.grid_geometry = geometry
        self.frame_id = geometry.frame_id
        self.static_info = deepcopy(message.info)
        if same_map:
            # The transient-local static publisher may repeat an unchanged map.
            # Preserve accumulated sensor belief in that common case.
            return
        belief_geometry = HazardGridGeometry(
            geometry.width, geometry.height, geometry.resolution,
            geometry.origin_x, geometry.origin_y,
            quaternion_to_yaw(
                geometry.origin_qx, geometry.origin_qy,
                geometry.origin_qz, geometry.origin_qw,
            ), geometry.frame_id,
        )
        # planning_grid_static is already planner-inflated. Treat it as the
        # authoritative static blocked mask and do not inflate again.
        self.belief = HazardBelief(belief_geometry, static_obstacles, self.config)
        self.last_thermal_ns = None
        self._set_status(self._current_status())
        self._publish()

    def _dynamic(self, message):
        if self.belief is None:
            return
        try:
            geometry = self._ros_geometry(message)
            if geometry != self.grid_geometry:
                raise ValueError("dynamic geometry differs from static")
            data = np.asarray(message.data).reshape(
                geometry.height, geometry.width
            )
            update = self.belief.update_dynamic_obstacles(
                data >= 100, already_inflated=True
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid dynamic grid: {exc}")
            self._set_status("INVALID_DYNAMIC_GRID")
            return
        if update.changed_cells:
            self._publish()

    def _thermal(self, message):
        if self.belief is None:
            self._set_status("WAITING_FOR_STATIC_GRID")
            return
        fields = {field.name for field in message.fields}
        if set(FIELDS) - fields or not message.header.frame_id:
            self._set_status("INVALID_THERMAL_POINTCLOUD")
            return
        transform_values = None
        if message.header.frame_id.lstrip("/") != self.frame_id.lstrip("/"):
            message_time = Time.from_msg(message.header.stamp)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id, message.header.frame_id,
                    message_time,
                    timeout=Duration(seconds=self.tf_timeout),
                ).transform
            except TransformException as stamped_error:
                try:
                    latest = self.tf_buffer.lookup_transform(
                        self.frame_id, message.header.frame_id,
                        Time(),
                        timeout=Duration(seconds=self.tf_timeout),
                    )
                except TransformException as latest_error:
                    self.get_logger().warning(
                        "thermal TF unavailable "
                        f"({self.frame_id} <- {message.header.frame_id}): "
                        f"stamped={stamped_error}; latest={latest_error}"
                    )
                    if self.last_thermal_ns is None:
                        self._set_status("WAITING_FOR_THERMAL_TF")
                    return
                latest_time = Time.from_msg(latest.header.stamp)
                if not latest_transform_is_fresh(
                    message_time.nanoseconds,
                    latest_time.nanoseconds,
                    self.latest_tf_fallback_tolerance_sec,
                ):
                    skew_sec = abs(
                        message_time.nanoseconds - latest_time.nanoseconds
                    ) / 1e9
                    self.get_logger().warning(
                        f"latest hazard thermal TF is {skew_sec:.3f}s from "
                        "the sensor frame; "
                        f"limit={self.latest_tf_fallback_tolerance_sec:.3f}s"
                    )
                    if self.last_thermal_ns is None:
                        self._set_status("WAITING_FOR_THERMAL_TF")
                    return
                transform = latest.transform
                self.get_logger().debug(
                    "using latest hazard thermal TF because the stamped "
                    f"lookup was temporarily unavailable: {stamped_error}"
                )
            transform_values = (
                (transform.translation.x, transform.translation.y, transform.translation.z),
                (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w),
            )
        points = point_cloud2.read_points(
            message, field_names=FIELDS, skip_nans=False
        )
        observations = localized_temperature_cells(
            points, self.grid_geometry, self.belief.static_obstacle_map,
            transform_values,
        )
        self.live_temperature_observations_publisher.publish(
            live_temperature_observations_message(observations)
        )
        observation_ns = self.get_clock().now().nanoseconds
        observation_time = observation_ns / 1e9
        update = self.belief.update_temperature_observations(
            observations, observation_time
        )
        self.last_thermal_ns = observation_ns
        self._set_status(self._current_status())
        # Publishing a 543x453 multi-layer snapshot for an unchanged 4 Hz
        # frame can starve the next thermal callback. Readiness is carried on
        # /hazard/status, so status-only recovery never republishes a snapshot.
        # While the initial route bypass is active, thermal changes remain
        # accumulated internally but are intentionally absent from planning.
        if update.changed_cells and not self.initial_route_bypass:
            self._publish()

    def _co(self, message):
        if self.belief is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, self.base_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
        except TransformException:
            self._set_status("WAITING_FOR_CO_TF")
            return
        now = self.get_clock().now().nanoseconds / 1e9
        self.belief.update_co_observation(
            transform.translation.x, transform.translation.y,
            message.data, now,
        )
        self._publish()

    def _timer(self):
        if self.belief is None:
            return
        snapshot_subscribers = self.snapshot_publisher.get_subscription_count()
        replay_snapshot = (
            snapshot_subscribers > self.snapshot_subscription_count
        )
        self.snapshot_subscription_count = snapshot_subscribers
        if self.initial_route_bypass:
            self._set_status("ACTIVE_INITIAL_STATIC_DYNAMIC_ONLY")
            if replay_snapshot:
                self._publish()
            return
        if not self.thermal_enabled:
            self._set_status("ACTIVE_STATIC_DYNAMIC_ONLY")
            if replay_snapshot:
                self._publish()
                self.get_logger().info(
                    "hazard snapshot replayed for a newly discovered subscriber"
                )
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.last_thermal_ns is None:
            self._set_status("WAITING_FOR_THERMAL")
        elif now_ns - self.last_thermal_ns > self.thermal_timeout * 1e9:
            self._set_status("THERMAL_STREAM_STALE")
        if (
            not self.belief.temperature_observed_mask.any()
            and not self.belief.co_observed_mask.any()
        ):
            if replay_snapshot:
                self._publish()
            return
        update = self.belief.advance_time(now_ns / 1e9)
        if update.changed_cells or replay_snapshot:
            self._publish()

    def _publish(self):
        if self.belief is None:
            return
        planning_cost = (
            self.belief.cost_without_temperature()
            if self.initial_route_bypass
            else self.belief.final_cost_map
        )
        self.exact_publisher.publish(float_grid_message(planning_cost))
        fire_probability = np.zeros(self.belief.shape, dtype=np.float32)
        if self.publish_diagnostic_grids:
            self.temperature_publisher.publish(float_grid_message(
                self.belief.temperature_belief_map
            ))
            if self.co_enabled:
                self.co_publisher.publish(float_grid_message(
                    self.belief.co_belief_map
                ))
            if self.fire_localization_enabled:
                self.fire_publisher.publish(float_grid_message(fire_probability))
        self.snapshot_publisher.publish(hazard_snapshot_message(
            self.belief, fire_probability, status=self.status,
            final_cost=planning_cost,
            include_temperature=(
                self.thermal_enabled and not self.initial_route_bypass
            ),
            include_co=self.co_enabled,
            include_fire=self.fire_localization_enabled,
        ))
        self.revision_publisher.publish(UInt64(data=self.belief.revision))
        now_ns = self.get_clock().now().nanoseconds
        publish_visual = self.publish_visualization_grid and (
            self.last_visual_publish_ns is None
            or now_ns - self.last_visual_publish_ns
            >= self.visualization_publish_period_ns
        )
        if self.co_enabled:
            gas_grid = OccupancyGrid()
            gas_grid.header.stamp = self.get_clock().now().to_msg()
            gas_grid.header.frame_id = self.frame_id
            gas_grid.info = deepcopy(self.static_info)
            gas_grid.data = self._gas_cost_cells()
            self.gas_cost_grid_publisher.publish(gas_grid)
        if not publish_visual:
            return
        self.last_visual_publish_ns = now_ns
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.info = deepcopy(self.static_info)
        finite = planning_cost[np.isfinite(planning_cost)]
        scale = max(float(finite.max() - self.config.base_cost), 1e-12) if finite.size else 1.0
        vis = np.clip(
            np.rint(99.0 * (planning_cost - self.config.base_cost) / scale),
            0, 99,
        )
        vis[~np.isfinite(planning_cost)] = 100
        message.data = vis.astype(np.int8).reshape(-1).astype(int).tolist()
        self.vis_publisher.publish(message)

    def _gas_cost_cells(self):
        """Gas belief as a raw /planning_grid overlay (see gas_planning_grid).

        Same ``(value - safe) / (blocked - safe)`` ratio encoding inno_thermal's
        cost layer uses to hand a hazard to the planners -- the weight/power
        stays in the planner, exactly as it does for thermal.
        """
        cells = gas_overlay_cells(
            self.belief.co_belief_map, self.belief.co_observed_mask,
            self.config.gas_safe_threshold, self.config.gas_blocked_threshold,
        )
        return cells.astype(np.int8).reshape(-1).astype(int).tolist()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = HazardBeliefNode()
        # Thermal callbacks perform a stamped TF lookup. Keep the
        # TransformListener's reentrant callbacks on another thread so the TF
        # buffer can advance while a thermal callback is waiting for its stamp.
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
