from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

from inno_autonav.astar_replanner import AstarReplanner


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
