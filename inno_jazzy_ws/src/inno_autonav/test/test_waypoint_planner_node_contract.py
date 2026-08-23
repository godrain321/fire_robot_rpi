"""Contract tests for WaypointPlannerNode using the fake-self + MethodType pattern
(same as test_evacuation_manager_switch_request.py) -- no live rclpy context.
"""

import json
from pathlib import Path as FsPath
from types import MethodType, SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
import numpy as np
from std_msgs.msg import String

from inno_autonav.grid_utils import quaternion_from_yaw
from inno_autonav.project_paths import project_path
from inno_autonav.replan_supervisor import ActiveGoal
from inno_autonav.waypoint_cost_projector import WaypointCostProjector, WaypointCostProjectorConfig
from inno_autonav.waypoint_graph_planner import WaypointGraphPlanner, WaypointGraphPlannerConfig
from inno_autonav.waypoint_planner_node import WaypointPlannerNode
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
        tf=FakeTf(pose),
        waypoint_path_publisher=Publisher(),
        get_clock=lambda: FakeClock(),
        get_logger=lambda: SimpleNamespace(error=lambda *a, **k: None),
    )
    value._replan = MethodType(WaypointPlannerNode._replan, value)
    value._publish_path = MethodType(WaypointPlannerNode._publish_path, value)
    value._publish_empty_path = MethodType(WaypointPlannerNode._publish_empty_path, value)
    return value


def small_waypoints():
    return {f"W{i}": (float(i), 0.5) for i in range(6)}  # a straight 6-point line


def plan_payload(exit_id, approach, revision=1):
    return json.dumps({
        "success": True, "activated": True, "selected_exit_id": exit_id,
        "selected_approach_position_world": list(approach), "hazard_revision": revision,
    })


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
    goal = PoseStamped(); goal.header.frame_id = "map"
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


def test_real_159_waypoint_file_drives_a_full_replan():
    document = load_waypoint_document(project_path("maps", "waypoint_queue_latest.yaml"))
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
    assert "'/goal_pose'" not in text and '"/goal_pose"' not in text
