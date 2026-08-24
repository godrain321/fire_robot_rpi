from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int32, String

from inno_autonav.skid_path_follower import SkidPathFollower
from inno_autonav.waypoint_queue import WaypointQueue


def test_new_path_is_acknowledged_before_control_cycle():
    follower = object.__new__(SkidPathFollower)
    states = []
    follower._state = states.append

    path = Path()
    path.poses.append(PoseStamped())
    follower._path_callback(path)

    assert follower.path is path
    assert states == ['PATH_ACCEPTED']


def test_modes3_and4_enable_final_yaw_alignment_for_forward_sensors():
    follower = object.__new__(SkidPathFollower)
    follower.default_align_goal_yaw = False
    follower.align_goal_yaw = False

    follower._mode_callback(Int32(data=3))
    assert follower.align_goal_yaw is True

    follower._mode_callback(Int32(data=4))
    assert follower.align_goal_yaw is True

    follower._mode_callback(Int32(data=2))
    assert follower.align_goal_yaw is False


def test_mode5_survivor_follow_hold_is_independent_from_replanning_hold():
    follower = object.__new__(SkidPathFollower)
    follower.hold = False
    follower.survivor_follow_hold = False

    follower._survivor_follow_hold_callback(Bool(data=True))

    assert follower.survivor_follow_hold is True
    assert follower.hold is False


def test_immediately_reached_waypoint_advances_after_path_acceptance():
    queue = object.__new__(WaypointQueue)
    queue.queue = [PoseStamped(), PoseStamped()]
    queue.current_index = 0
    queue.waiting_for_departure = True
    queue.execution_mode = 'continuous'
    queue.step_index = 0
    sent_indices = []
    queue._send_current_goal = lambda: sent_indices.append(queue.current_index)
    queue._state = lambda _state: None

    queue._follower(String(data='GOAL_REACHED'))
    assert queue.current_index == 0
    assert sent_indices == []

    queue._follower(String(data='PATH_ACCEPTED'))
    queue._follower(String(data='GOAL_REACHED'))

    assert queue.current_index == 1
    assert sent_indices == [1]


def test_unrelated_stop_state_does_not_unlock_stale_goal_reached():
    queue = object.__new__(WaypointQueue)
    queue.queue = [PoseStamped(), PoseStamped()]
    queue.current_index = 0
    queue.waiting_for_departure = True
    queue.execution_mode = 'continuous'
    queue.step_index = 0
    queue._send_current_goal = lambda: None
    queue._state = lambda _state: None


def test_step_mode_stops_after_one_waypoint():
    queue = object.__new__(WaypointQueue)
    queue.queue = [PoseStamped(), PoseStamped()]
    queue.current_index = 0
    queue.step_index = 0
    queue.execution_mode = 'step'
    queue.waiting_for_departure = False
    states = []
    queue._state = states.append
    queue._publish_queue = lambda: None

    queue._follower(String(data='GOAL_REACHED'))

    assert queue.current_index is None
    assert queue.step_index == 1
    assert states == ['STEP_COMPLETE:1/2:SPACE_FOR:2']


def test_mode2_reaches_first_selected_goal_then_waits_for_space():
    queue = object.__new__(WaypointQueue)
    queue.queue = [PoseStamped() for _ in range(6)]
    queue.waypoint_names = [f'w{i}' for i in range(1, 7)]
    queue.current_index = None
    queue.waiting_for_departure = False
    queue.step_index = 0
    queue.execution_mode = 'continuous'
    queue.selected_names = []
    queue.selected_indices = []
    queue.selected_next_position = 0
    queue.selected_current_position = None
    states = []
    sent = []
    queue._state = states.append
    queue._publish_queue = lambda: None
    queue._publish_autonomy_cancel = lambda: None
    queue._send_current_goal = lambda: sent.append(queue.current_index)

    queue._start_named_mission('w1,w5,w6')
    assert queue.current_index == 0
    assert sent == [0]

    queue.waiting_for_departure = False
    queue._follower(String(data='GOAL_REACHED'))
    assert queue.current_index is None
    assert queue.selected_next_position == 1
    assert states[-1] == 'MODE2_REACHED:w1:SPACE_FOR:w5'

    queue._named_next()
    assert queue.current_index == 4
    assert sent == [0, 4]


def test_mode2_space_while_driving_does_not_skip_goal():
    queue = object.__new__(WaypointQueue)
    queue.execution_mode = 'named'
    queue.selected_names = ['w1', 'w5']
    queue.selected_indices = [0, 4]
    queue.selected_next_position = 0
    queue.selected_current_position = 0
    queue.current_index = 0
    states = []
    queue._state = states.append

    queue._named_next()

    assert queue.current_index == 0
    assert queue.selected_next_position == 0
    assert states == ['MODE2_BUSY:w1']


def test_mode2_cancel_stops_autonomy_and_resets_selection():
    queue = object.__new__(WaypointQueue)
    queue.current_index = 4
    queue.waiting_for_departure = False
    queue.execution_mode = 'named'
    queue.selected_names = ['w1', 'w5']
    queue.selected_indices = [0, 4]
    queue.selected_next_position = 1
    queue.selected_current_position = 1
    states = []
    cancels = []
    queue._state = states.append
    queue._publish_queue = lambda: None
    queue._publish_autonomy_cancel = lambda: cancels.append(True)

    queue._cancel_named_mission()

    assert cancels == [True]
    assert queue.current_index is None
    assert queue.execution_mode == 'idle'
    assert queue.selected_names == []
    assert states == ['MODE2_CANCELLED']
