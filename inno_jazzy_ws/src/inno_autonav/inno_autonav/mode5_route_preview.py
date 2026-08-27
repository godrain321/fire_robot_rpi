"""Offline RViz preview of Mode 5 after EXIT1 is confirmed blocked."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Sequence

from geometry_msgs.msg import Point, PoseStamped
from inno_hazard.hazard_belief import HazardGridGeometry
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .evacuation_planner import EvacuationPlanner
from .exit_evaluator import (
    ExitEvaluationConfig,
    ExitEvaluator,
    ExitHazardSnapshot,
    load_exit_registry,
)
from .grid_utils import load_pgm_as_occupancy, quaternion_from_yaw
from .project_paths import project_path
from .reference_waypoint_graph import (
    PlanningGridGeometry,
    ReferenceWaypoint,
    ReferenceWaypointGraphConfig,
    ReferenceWaypointGraphPlanner,
)
from .waypoint_cost_projector import (
    WaypointCostProjector,
    WaypointCostProjectorConfig,
)
from .waypoint_graph_planner import (
    nearest_safe_waypoint,
    WaypointGraphPlanner,
    WaypointGraphPlannerConfig,
)
from .waypoint_route_simplifier import (
    simplify_waypoint_route,
    WaypointRouteSimplifierConfig,
)
from .waypoint_selection import (
    load_waypoint_document,
    named_waypoints_from_document,
)


Point2D = tuple[float, float]


@dataclass(frozen=True)
class Mode5Preview:
    """One deterministic static-only Mode 5 route preview."""

    start_world: Point2D
    selected_exit_id: str
    selected_exit_world: Point2D
    selected_approach_world: Point2D
    blocked_exit_ids: tuple[str, ...]
    exit_evaluation_waypoints: tuple[str, ...]
    reference_waypoints: tuple[str, ...]
    drive_waypoints: tuple[str, ...]
    drive_points: tuple[Point2D, ...]
    evaluated_path_length_m: float
    selection_reason: str
    exit_positions: dict[str, Point2D]
    waypoint_positions: dict[str, Point2D]


def _load_init_position(filename: str) -> Point2D:
    source = Path(filename).expanduser().resolve(strict=False)
    document = yaml.safe_load(source.read_text(encoding='utf-8'))
    try:
        value = document['semantic_points']['init']
        point = float(value['x']), float(value['y'])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('semantic init point is invalid') from error
    if not all(math.isfinite(item) for item in point):
        raise ValueError('semantic init point must be finite')
    return point


def compute_mode5_preview(
    map_yaml: str,
    waypoint_file: str,
    semantic_file: str,
    blocked_exit_ids: Sequence[str] = ('EXIT1',),
) -> tuple[Mode5Preview, object]:
    """Run the deployed static-only exit and waypoint algorithms offline."""
    grid = load_pgm_as_occupancy(map_yaml, 'map')
    geometry = HazardGridGeometry(
        grid.width,
        grid.height,
        grid.resolution,
        grid.origin_x,
        grid.origin_y,
        grid.origin_yaw,
        grid.frame_id,
    )
    static = np.asarray(grid.data) >= 100
    shape = static.shape
    final_cost = np.ones(shape, dtype=float)
    final_cost[static] = math.inf
    empty_float = np.full(shape, np.nan, dtype=float)
    empty_bool = np.zeros(shape, dtype=bool)
    snapshot = ExitHazardSnapshot(
        geometry=geometry,
        final_cost=final_cost,
        temperature_c=empty_float,
        co_ppm=empty_float,
        observed_mask=empty_bool,
        temperature_observed_mask=empty_bool,
        co_observed_mask=empty_bool,
        fire_probability=np.zeros(shape, dtype=float),
        static_obstacle_map=static,
        dynamic_obstacle_map=empty_bool,
        blocked_mask=static,
        revision=0,
        temperature_blocked_c=60.0,
        co_blocked_ppm=1600.0,
        base_cost=1.0,
    )

    records = named_waypoints_from_document(
        load_waypoint_document(waypoint_file), 'map'
    )
    waypoint_positions = {
        record.name: (record.x, record.y) for record in records
    }
    reference_planner = ReferenceWaypointGraphPlanner(
        tuple(
            ReferenceWaypoint(
                record.name, record.x, record.y, record.yaw
            )
            for record in records
        ),
        ReferenceWaypointGraphConfig(),
    )

    def reference_path(snapshot_value, start, goal):
        planning_geometry = PlanningGridGeometry(
            snapshot_value.geometry.resolution,
            snapshot_value.geometry.origin_x,
            snapshot_value.geometry.origin_y,
            snapshot_value.geometry.origin_yaw,
            snapshot_value.geometry.frame_id,
        )
        return reference_planner.plan(
            snapshot_value.final_cost,
            start,
            goal,
            planning_geometry,
            snapshot_value.static_obstacle_map,
            costs_are_traversal=True,
            unknown_is_occupied=True,
            allow_diagonal=True,
            thermal_cost_weight=0.0,
            thermal_cost_power=1.0,
            waypoint_frame_id='map',
        )

    exits = load_exit_registry(semantic_file, 'map')
    start_world = _load_init_position(semantic_file)
    evaluator = ExitEvaluator(
        ExitEvaluationConfig(), path_planner=reference_path
    )
    batch = evaluator.evaluate_all(
        exits, start_world, snapshot=snapshot, evaluated_at=0.0
    )
    blocked = tuple(sorted({str(item).upper() for item in blocked_exit_ids}))
    selection = EvacuationPlanner().plan(
        batch, excluded_exit_ids=blocked
    )
    if not selection.success or selection.selected_evaluation is None:
        raise ValueError('Mode 5 preview could not select a safe exit')

    projector = WaypointCostProjector(
        waypoint_positions,
        WaypointCostProjectorConfig(
            waypoint_cost_radius_m=0.8,
            unknown_is_occupied=True,
        ),
    )
    waypoint_costs = projector.project_costs(grid)
    start_id = nearest_safe_waypoint(
        start_world, waypoint_positions, waypoint_costs
    )
    goal_id = nearest_safe_waypoint(
        selection.selected_approach_position_world,
        waypoint_positions,
        waypoint_costs,
    )
    if start_id is None or goal_id is None:
        raise ValueError('Mode 5 preview has no safe waypoint endpoint')
    graph_result = WaypointGraphPlanner(
        waypoint_positions,
        WaypointGraphPlannerConfig(neighbor_radius_m=1.5),
    ).plan(waypoint_costs, start_id, goal_id)
    if not graph_result.success:
        raise ValueError(
            f'Mode 5 preview waypoint route failed: {graph_result.status}'
        )
    simplified = simplify_waypoint_route(
        graph_result.waypoint_ids,
        waypoint_positions,
        grid,
        WaypointRouteSimplifierConfig(),
    )
    if not simplified.success:
        raise ValueError(
            f'Mode 5 preview simplification failed: {simplified.detail}'
        )
    drive_points = tuple(
        waypoint_positions[item] for item in simplified.simplified_ids
    ) + (tuple(selection.selected_approach_position_world),)
    evaluation = selection.selected_evaluation
    return Mode5Preview(
        start_world=tuple(start_world),
        selected_exit_id=str(selection.selected_exit_id),
        selected_exit_world=tuple(selection.selected_exit_position_world),
        selected_approach_world=tuple(
            selection.selected_approach_position_world
        ),
        blocked_exit_ids=blocked,
        exit_evaluation_waypoints=tuple(
            evaluation.reference_waypoint_ids
        ),
        reference_waypoints=tuple(graph_result.waypoint_ids),
        drive_waypoints=tuple(simplified.simplified_ids),
        drive_points=drive_points,
        evaluated_path_length_m=float(evaluation.path_length_m),
        selection_reason=str(selection.selection_reason),
        exit_positions={
            item.exit_id: tuple(item.position_world) for item in exits
        },
        waypoint_positions=waypoint_positions,
    ), grid


class Mode5RoutePreviewNode(Node):
    """Publish a motor-disabled Mode 5 preview for RViz."""

    def __init__(self) -> None:
        super().__init__('mode5_route_preview')
        defaults = {
            'map_yaml': project_path('maps', 'inno_map_nav.yaml'),
            'waypoint_file': project_path(
                'docs', 'full_map_waypoints_1m_numbered.yaml'
            ),
            'semantic_file': project_path(
                'inno_jazzy_ws', 'src', 'inno_autonav', 'config',
                'semantic_points.yaml'
            ),
            'blocked_exit_ids': ['EXIT1'],
            'path_topic': '/mode5_route_preview/path',
            'marker_topic': '/mode5_route_preview/markers',
            'status_topic': '/mode5_route_preview/status',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def value(name):
            return self.get_parameter(name).value

        self.preview, self.grid = compute_mode5_preview(
            str(value('map_yaml')),
            str(value('waypoint_file')),
            str(value('semantic_file')),
            tuple(value('blocked_exit_ids')),
        )
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/map', qos
        )
        self.path_publisher = self.create_publisher(
            PathMessage, str(value('path_topic')), qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, str(value('marker_topic')), qos
        )
        self.status_publisher = self.create_publisher(
            String, str(value('status_topic')), qos
        )
        self.publish_preview()

    def _map_message(self, stamp) -> OccupancyGrid:
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self.grid.frame_id
        message.info.map_load_time = stamp
        message.info.resolution = self.grid.resolution
        message.info.width = self.grid.width
        message.info.height = self.grid.height
        message.info.origin.position.x = self.grid.origin_x
        message.info.origin.position.y = self.grid.origin_y
        qx, qy, qz, qw = quaternion_from_yaw(self.grid.origin_yaw)
        message.info.origin.orientation.x = qx
        message.info.origin.orientation.y = qy
        message.info.origin.orientation.z = qz
        message.info.origin.orientation.w = qw
        message.data = self.grid.data.reshape(-1).astype(int).tolist()
        return message

    def _path_message(self, stamp) -> PathMessage:
        message = PathMessage()
        message.header.stamp = stamp
        message.header.frame_id = self.grid.frame_id
        for index, (x, y) in enumerate(self.preview.drive_points):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            if index + 1 < len(self.preview.drive_points):
                next_x, next_y = self.preview.drive_points[index + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                yaw = 0.0
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            message.poses.append(pose)
        return message

    @staticmethod
    def _sphere_list(header, namespace, points, scale, color):
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        marker.points = [
            Point(x=float(x), y=float(y), z=0.14) for x, y in points
        ]
        return marker

    @staticmethod
    def _text(header, namespace, marker_id, point, text, color, z=0.55):
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.32
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        marker.text = text
        return marker

    def _marker_message(self, header) -> MarkerArray:
        preview = self.preview
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        active = set(preview.drive_waypoints)
        support_ids = [
            item for item in preview.reference_waypoints
            if item not in active
        ]
        markers = [clear]
        if support_ids:
            markers.append(self._sphere_list(
                header,
                'mode5_reference_waypoints',
                [preview.waypoint_positions[item] for item in support_ids],
                0.26,
                (1.0, 0.42, 0.05),
            ))
        markers.append(self._sphere_list(
            header,
            'mode5_drive_waypoints',
            [preview.waypoint_positions[item]
             for item in preview.drive_waypoints],
            0.42,
            (1.0, 0.05, 0.75),
        ))
        for index, waypoint_id in enumerate(preview.reference_waypoints):
            selected = waypoint_id in active
            markers.append(self._text(
                header,
                'mode5_waypoint_labels',
                index,
                preview.waypoint_positions[waypoint_id],
                ('DRIVE ' if selected else 'via ') + waypoint_id,
                (1.0, 0.2, 0.85) if selected else (1.0, 0.55, 0.1),
            ))
        for index, exit_id in enumerate(preview.blocked_exit_ids):
            point = preview.exit_positions.get(exit_id)
            if point is None:
                continue
            blocked = Marker()
            blocked.header = header
            blocked.ns = 'mode5_blocked_exits'
            blocked.id = index
            blocked.type = Marker.CYLINDER
            blocked.action = Marker.ADD
            blocked.pose.position.x = float(point[0])
            blocked.pose.position.y = float(point[1])
            blocked.pose.position.z = 0.20
            blocked.pose.orientation.w = 1.0
            blocked.scale.x = blocked.scale.y = 0.85
            blocked.scale.z = 0.40
            blocked.color.r = 1.0
            blocked.color.g = 0.02
            blocked.color.b = 0.02
            blocked.color.a = 0.9
            markers.extend((
                blocked,
                self._text(
                    header,
                    'mode5_blocked_exit_labels',
                    index,
                    point,
                    f'{exit_id} BLOCKED\n(dynamic obstacle)',
                    (1.0, 0.15, 0.15),
                    z=0.85,
                ),
            ))
        markers.append(self._text(
            header,
            'mode5_selected_exit_label',
            0,
            preview.selected_approach_world,
            f'SELECTED {preview.selected_exit_id}',
            (0.1, 1.0, 0.2),
            z=0.72,
        ))
        markers.append(self._sphere_list(
            header,
            'mode5_selected_exit',
            [preview.selected_approach_world],
            0.52,
            (0.1, 1.0, 0.2),
        ))
        markers.append(self._text(
            header,
            'mode5_start_label',
            0,
            preview.start_world,
            'ROBOT INIT',
            (1.0, 1.0, 1.0),
        ))
        return MarkerArray(markers=markers)

    def publish_preview(self) -> None:
        stamp = self.get_clock().now().to_msg()
        map_message = self._map_message(stamp)
        path_message = self._path_message(stamp)
        self.map_publisher.publish(map_message)
        self.path_publisher.publish(path_message)
        self.marker_publisher.publish(
            self._marker_message(path_message.header)
        )
        payload = {
            'blocked_exit_ids': list(self.preview.blocked_exit_ids),
            'selected_exit_id': self.preview.selected_exit_id,
            'selection_reason': self.preview.selection_reason,
            'evaluated_path_length_m': self.preview.evaluated_path_length_m,
            'exit_evaluation_waypoints': list(
                self.preview.exit_evaluation_waypoints
            ),
            'reference_waypoints': list(self.preview.reference_waypoints),
            'drive_waypoints': list(self.preview.drive_waypoints),
            'drive_points': [list(item) for item in self.preview.drive_points],
            'thermal_costmap_used': False,
            'motor_output_enabled': False,
        }
        self.status_publisher.publish(String(data=json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            separators=(',', ':'),
            allow_nan=False,
        )))
        self.get_logger().info(
            'MODE5 PREVIEW: '
            f"blocked={','.join(self.preview.blocked_exit_ids)}; "
            f'selected={self.preview.selected_exit_id}; '
            f"reference={' -> '.join(self.preview.reference_waypoints)}; "
            f"drive={' -> '.join(self.preview.drive_waypoints)}"
        )
        self.get_logger().warning(
            'PREVIEW ONLY: /cmd_vel 및 ESP32 모터 명령은 발행하지 않습니다.'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Mode5RoutePreviewNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
