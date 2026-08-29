"""Contract tests for WaypointPlannerNode using the fake-self + MethodType pattern
(same as test_evacuation_manager_switch_request.py) -- no live rclpy context.
"""

import json
from pathlib import Path as FsPath
from types import MethodType, SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
import pytest
from std_msgs.msg import String

from inno_autonav.astar_replanner import message_to_grid
from inno_autonav.grid_utils import yaw_from_quaternion
from inno_autonav.project_paths import project_path
from inno_autonav.waypoint_cost_projector import WaypointCostProjector, WaypointCostProjectorConfig
from inno_autonav.waypoint_graph_planner import WaypointGraphPlanner, WaypointGraphPlannerConfig
from inno_autonav.waypoint_planner_node import WaypointPlannerNode
from inno_autonav.waypoint_route_simplifier import nearest_reachable_waypoint
from inno_autonav.waypoint_route_simplifier import WaypointRouteSimplifierConfig
from inno_autonav.waypoint_selection import load_waypoint_document, named_waypoints_from_document


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeClock:
    class _Time:
        def to_msg(self):
            return Time()

    def now(self):
        return self._Time()


class FakeTf:
    def __init__(self, pose):
        self.pose = pose

    def lookup_pose_2d(self, map_frame, base_frame):
        return self.pose


def occupancy_grid_message(size=10, resolution=1.0, origin=(0.0, 0.0), value=0):
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = size
    message.info.height = size
    message.info.resolution = resolution
    message.info.origin.position.x = origin[0]
    message.info.origin.position.y = origin[1]
    message.info.origin.orientation = Quaternion(w=1.0)
    message.data = [value] * (size * size)
    return message


def node(waypoints_world, *, pose=(0.5, 0.5)):
    value = SimpleNamespace(
        enabled=True,
        accept_direct_goal=True,
        map_frame="map",
        base_frame="base_link",
        waypoints_world=waypoints_world,
        projector=WaypointCostProjector(waypoints_world, WaypointCostProjectorConfig(waypoint_cost_radius_m=0.6)),
        graph_planner=WaypointGraphPlanner(waypoints_world, WaypointGraphPlannerConfig(neighbor_radius_m=1.5)),
        simplifier_config=WaypointRouteSimplifierConfig(),
        grid=None,
        active_goal=None,
        _last_costs=None,
        _last_goal=None,
        _last_grid_data=None,
        tf=FakeTf(pose),
        waypoint_path_publisher=Publisher(),
        route_status_publisher=Publisher(),
        route_markers_publisher=Publisher(),
        get_clock=lambda: FakeClock(),
        get_logger=lambda: SimpleNamespace(
            error=lambda *a, **k: None,
            info=lambda *a, **k: None,
        ),
    )
    value._replan = MethodType(WaypointPlannerNode._replan, value)
    value._publish_path = MethodType(WaypointPlannerNode._publish_path, value)
    value._publish_empty_path = MethodType(WaypointPlannerNode._publish_empty_path, value)
    value._publish_route_status = MethodType(
        WaypointPlannerNode._publish_route_status, value
    )
    value._publish_route_markers = MethodType(
        WaypointPlannerNode._publish_route_markers, value
    )
    return value


def small_waypoints():
    return {f"W{i}": (float(i), 0.5) for i in range(6)}  # a straight 6-point line


def plan_payload(exit_id, approach, revision=1, yaw=None):
    payload = {
        "success": True, "activated": True, "selected_exit_id": exit_id,
        "selected_approach_position_world": list(approach), "hazard_revision": revision,
    }
    if yaw is not None:
        payload["selected_approach_yaw_rad"] = yaw
    return json.dumps(payload)


def test_grid_and_plan_input_produce_a_waypoint_path():
    waypoints = small_waypoints()
    value = node(waypoints)
    WaypointPlannerNode._on_plan(value, String(data=plan_payload("EXIT1", (5.0, 0.5))))
    WaypointPlannerNode._on_grid(value, occupancy_grid_message(size=10))
    assert value.waypoint_path_publisher.messages
    message = value.waypoint_path_publisher.messages[-1]
    assert message.header.frame_id == "map"
    assert message.poses  # a real route was produced
    # Final pose is snapped exactly to the exit approach position, not a waypoint.
    last = message.poses[-1].pose.position
    assert (last.x, last.y) == (5.0, 0.5)


def test_no_replan_without_a_selected_exit():
    value = node(small_waypoints())
    WaypointPlannerNode._on_grid(value, occupancy_grid_message())
    assert not value.waypoint_path_publisher.messages


def test_static_profile_direct_goal_produces_waypoint_path_without_evacuation_plan():
    value = node(small_waypoints())
    WaypointPlannerNode._on_grid(value, occupancy_grid_message())
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x, goal.pose.position.y = (5.0, 0.5)
    WaypointPlannerNode._on_direct_goal(value, goal)
    assert value.active_goal.exit_id == "DIRECT_GOAL"
    assert value.waypoint_path_publisher.messages[-1].poses


def test_current_pose_is_used_as_the_start_waypoint_source():
    waypoints = small_waypoints()
    value = node(waypoints, pose=(4.0, 0.5))  # start near the far end of the line
    WaypointPlannerNode._on_plan(value, String(data=plan_payload("EXIT1", (0.0, 0.5))))
    WaypointPlannerNode._on_grid(value, occupancy_grid_message())
    message = value.waypoint_path_publisher.messages[-1]
    first = message.poses[0].pose.position
    assert first.x >= 3.0  # started near W4, not W0


def test_waypoint_path_poses_follow_the_simplified_route_order():
    waypoints = small_waypoints()
    value = node(waypoints)
    WaypointPlannerNode._on_plan(value, String(data=plan_payload("EXIT1", (5.0, 0.5))))
    WaypointPlannerNode._on_grid(value, occupancy_grid_message())
    message = value.waypoint_path_publisher.messages[-1]
    xs = [pose.pose.position.x for pose in message.poses]
    assert xs == sorted(xs)  # monotonically progresses along the line toward the exit


def test_blocked_final_waypoint_to_goal_connector_publishes_no_path():
    waypoints = {"W0": (0.5, 0.5), "W1": (1.5, 0.5)}
    value = node(waypoints)
    message = occupancy_grid_message(size=6)
    message.data[2] = 100  # (2, 0), between W1 and the exact goal
    WaypointPlannerNode._on_plan(
        value, String(data=plan_payload("EXIT1", (3.5, 0.5)))
    )
    WaypointPlannerNode._on_grid(value, message)
    assert value.waypoint_path_publisher.messages
    assert not value.waypoint_path_publisher.messages[-1].poses


def test_start_inside_inflated_area_rejects_every_connector():
    waypoints = {"W0": (2.5, 0.5), "W1": (3.5, 0.5)}
    value = node(waypoints, pose=(0.5, 0.5))
    message = occupancy_grid_message(size=6)
    message.data[0] = 100  # robot start blocked; safe waypoint route is clear
    WaypointPlannerNode._on_plan(
        value, String(data=plan_payload("EXIT1", (3.5, 0.5)))
    )
    WaypointPlannerNode._on_grid(value, message)
    assert not value.waypoint_path_publisher.messages[-1].poses


def test_nearest_reachable_waypoint_skips_blocked_start_connector():
    waypoints = {"nearest": (2.5, 0.5), "reachable": (0.5, 2.5)}
    message = occupancy_grid_message(size=5)
    message.data[1] = 100
    grid = message_to_grid(message)
    costs = {waypoint_id: 0.0 for waypoint_id in waypoints}
    assert nearest_reachable_waypoint(
        (0.5, 0.5), waypoints, costs, grid, True,
    ) == "reachable"


def test_no_reachable_start_waypoint_returns_none():
    waypoints = {"east": (2.5, 0.5), "north": (0.5, 2.5)}
    message = occupancy_grid_message(size=5)
    message.data[1] = 100
    message.data[5] = 100
    grid = message_to_grid(message)
    costs = {waypoint_id: 0.0 for waypoint_id in waypoints}
    assert nearest_reachable_waypoint(
        (0.5, 0.5), waypoints, costs, grid, True,
    ) is None


def test_mode3_inspection_yaw_reaches_final_waypoint_pose():
    value = node(small_waypoints())
    WaypointPlannerNode._on_plan(value, String(data=plan_payload(
        "MODE3_INSPECTION", (5.0, 0.5), yaw=1.25,
    )))
    WaypointPlannerNode._on_grid(value, occupancy_grid_message(size=10))
    orientation = value.waypoint_path_publisher.messages[-1].poses[-1].pose.orientation
    assert yaw_from_quaternion(orientation) == pytest.approx(1.25)


def test_route_status_and_rviz_overlay_distinguish_initial_plan_from_replan():
    value = node(small_waypoints())
    WaypointPlannerNode._on_plan(value, String(data=plan_payload(
        "EXIT1", (5.0, 0.5)
    )))
    WaypointPlannerNode._on_grid(value, occupancy_grid_message(size=10))

    initial = json.loads(value.route_status_publisher.messages[-1].data)
    assert initial["event"] == "PATH_CREATED"
    assert initial["waypoints"]
    assert initial["reference_waypoints"]
    markers = value.route_markers_publisher.messages[-1].markers
    assert markers
    namespaces = {marker.ns for marker in markers}
    assert "selected_waypoint_route_active_points" in namespaces
    assert "selected_waypoint_route_goal" in namespaces

    value._replan(force=True)
    replanned = json.loads(value.route_status_publisher.messages[-1].data)
    assert replanned["event"] == "REPLANNED"
    assert replanned["waypoints"] == initial["waypoints"]
    assert replanned["reference_waypoints"] == initial["reference_waypoints"]


def test_real_159_waypoint_file_drives_a_full_replan():
    document = load_waypoint_document(project_path(
        "docs", "full_map_waypoints_1m_numbered.yaml"
    ))
    records = named_waypoints_from_document(document, "map")
    waypoints_world = {item.name: (item.x, item.y) for item in records}
    names = list(waypoints_world)
    start_pos = waypoints_world[names[0]]
    goal_pos = waypoints_world[names[-1]]
    value = node(waypoints_world, pose=start_pos)
    # A large, fully-clear synthetic grid covering every real waypoint.
    xs = [x for x, _ in waypoints_world.values()]
    ys = [y for _, y in waypoints_world.values()]
    resolution = 0.5
    origin = (min(xs) - 2.0, min(ys) - 2.0)
    size = int(max(max(xs) - origin[0], max(ys) - origin[1]) / resolution) + 8
    WaypointPlannerNode._on_plan(value, String(data=plan_payload("EXIT1", goal_pos)))
    WaypointPlannerNode._on_grid(
        value, occupancy_grid_message(size=size, resolution=resolution, origin=origin),
    )
    assert value.waypoint_path_publisher.messages
    assert value.waypoint_path_publisher.messages[-1].poses


# -- ownership: never touches /goal_pose or /planned_path --------------------

def test_module_never_constructs_a_goal_pose_or_planned_path_publisher():
    text = (FsPath(__file__).parents[1] / "inno_autonav" / "waypoint_planner_node.py").read_text(
        encoding="utf-8"
    )
    assert "create_publisher(PoseStamped" not in text
    assert "'/planned_path'" not in text and '"/planned_path"' not in text
    # A static/non-evacuation profile may subscribe to /goal_pose when the
    # explicit accept_direct_goal gate is enabled.  Ownership prohibits this
    # node from publishing that topic; the publisher assertion above enforces it.
