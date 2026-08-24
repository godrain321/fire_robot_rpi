from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

from inno_autonav.astar_replanner import AstarReplanner
from inno_autonav.path_selector import PathSelectorCore
from inno_autonav.skid_path_follower import SkidPathFollower
from inno_autonav.waypoint_planner_node import WaypointPlannerNode


def test_autonomy_cancel_clears_goal_and_publishes_empty_path():
    planner = object.__new__(AstarReplanner)
    planner.goal = PoseStamped()
    planner.current_path_cells = [(1, 2), (2, 3)]
    planner._dirty = True
    paths = []
    states = []
    planner._publish_empty_path = lambda: paths.append('empty')
    planner._state = states.append

    planner._cancel_callback(Empty())

    assert planner.goal is None
    assert planner.current_path_cells == []
    assert planner._dirty is False
    assert paths == ['empty']
    assert states == ['CANCELLED']


def test_autonomy_cancel_stops_follower_and_forgets_selected_path():
    follower = object.__new__(SkidPathFollower)
    follower.path = object()
    follower.rotating_in_place = True
    follower.rotation_direction = 1.0
    states = []
    follower._publish_stop = states.append

    follower._cancel_callback(Empty())

    assert follower.path is None
    assert follower.rotating_in_place is False
    assert follower.rotation_direction == 0.0
    assert states == ['CANCELLED']


def test_autonomy_cancel_prevents_waypoint_route_from_reviving_on_grid_update():
    planner = object.__new__(WaypointPlannerNode)
    planner.active_goal = object()
    planner._last_costs = {'w1': 1.0}
    planner._last_goal = object()
    planner._last_published_path_stamp_ns = 123
    planner._goal_received_ns = 456
    paths = []
    planner._publish_empty_path = lambda: paths.append('empty')

    planner._on_cancel(Empty())

    assert planner.active_goal is None
    assert planner._last_costs is None
    assert planner._last_goal is None
    assert planner._last_published_path_stamp_ns is None
    assert planner._goal_received_ns is None
    assert paths == ['empty']


def test_path_selector_clear_forgets_both_latched_sources():
    selector = PathSelectorCore('WAYPOINT')
    selector.on_waypoint_path(object())
    selector.on_astar_path(object())

    selector.clear()

    assert selector.status() == {
        'mode': 'WAYPOINT',
        'has_waypoint_path': False,
        'has_astar_path': False,
    }
