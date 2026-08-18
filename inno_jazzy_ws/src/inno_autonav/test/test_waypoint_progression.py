from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

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


def test_mode4_reaches_first_goal_then_waits_for_space():
    queue = object.__new__(WaypointQueue)
    queue.queue = [PoseStamped() for _ in range(6)]
    queue.waypoint_names = [f'w{i}' for i in range(1, 7)]
    queue.current_index = None
    queue.waiting_for_departure = False
    queue.step_index = 0
    queue.execution_mode = 'continuous'
    queue.mode4_names = []
    queue.mode4_indices = []
    queue.mode4_next_position = 0
    queue.mode4_current_position = None
    states = []
    sent = []
    queue._state = states.append
    queue._publish_queue = lambda: None
    queue._send_current_goal = lambda: sent.append(queue.current_index)

    queue._start_mode4('w1,w5,w6')
    assert queue.current_index == 0
    assert sent == [0]

    queue.waiting_for_departure = False
    queue._follower(String(data='GOAL_REACHED'))
    assert queue.current_index is None
    assert queue.mode4_next_position == 1
    assert states[-1] == 'MODE4_REACHED:w1:SPACE_FOR:w5'

    queue._mode4_next()
    assert queue.current_index == 4
    assert sent == [0, 4]


def test_mode4_space_while_driving_does_not_skip_goal():
    queue = object.__new__(WaypointQueue)
    queue.execution_mode = 'mode4'
    queue.mode4_names = ['w1', 'w5']
    queue.mode4_indices = [0, 4]
    queue.mode4_next_position = 0
    queue.mode4_current_position = 0
    queue.current_index = 0
    states = []
    queue._state = states.append

    queue._mode4_next()

    assert queue.current_index == 0
    assert queue.mode4_next_position == 0
    assert states == ['MODE4_BUSY:w1']
