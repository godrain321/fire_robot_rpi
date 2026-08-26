import json
import math

from builtin_interfaces.msg import Time
from std_msgs.msg import Int32, String

from inno_autonav.mode3_inspector import (
    Mode3Inspector,
    PresenceEvidence,
    compute_inspection_goal,
    is_at_standoff,
    parse_inspection_command,
    select_nearest_candidate,
)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Clock:
    class _Now:
        nanoseconds = 2_000_000_000

        @staticmethod
        def to_msg():
            return Time(sec=2)

    def now(self):
        return self._Now()


class _Tf:
    @staticmethod
    def lookup_pose_2d(_map_frame, _base_frame):
        return 0.0, 0.0, 0.0


def test_select_nearest_dynamic_obstacle():
    assert select_nearest_candidate(0.0, 0.0, [(4.0, 0.0), (2.0, 1.0)]) == (
        2.0,
        1.0,
    )


def test_inspection_goal_is_2m_from_target_and_faces_it():
    goal_x, goal_y, goal_yaw = compute_inspection_goal(
        0.0, 0.0, 0.0, 4.0, 0.0, 2.0
    )
    assert math.isclose(goal_x, 2.0)
    assert math.isclose(goal_y, 0.0)
    assert math.isclose(math.hypot(4.0 - goal_x, goal_y), 2.0)
    assert math.isclose(goal_yaw, 0.0)


def test_mode5_can_request_an_explicit_two_metre_inspection_target():
    accepted, target = parse_inspection_command('MODE3_START_AT:4.0,2.5')
    assert accepted
    assert target == (4.0, 2.5)
    goal_x, goal_y, _ = compute_inspection_goal(
        0.0, 0.0, 0.0, target[0], target[1], 2.0
    )
    assert math.isclose(math.dist((goal_x, goal_y), target), 2.0)


def test_standoff_tolerance_accepts_stationary_field_check():
    assert is_at_standoff(0.0, 0.0, 1.5, 0.0, 2.0, 0.6)
    assert not is_at_standoff(0.0, 0.0, 1.3, 0.0, 2.0, 0.6)


def test_presence_requires_online_samples_at_expected_distance():
    evidence = PresenceEvidence(1.5, 0.4)
    evidence.add(False, True, 1.5)
    evidence.add(True, True, 3.0)
    evidence.add(True, True, 1.4)
    evidence.add(True, True, 1.6)
    assert evidence.total_samples == 3
    assert evidence.positive_samples == 2
    assert evidence.classify(True, minimum_samples=3, positive_samples=2) == (
        'PERSON'
    )


def test_online_samples_without_presence_are_dynamic_obstacle():
    evidence = PresenceEvidence(1.5, 0.4)
    for _ in range(3):
        evidence.add(True, False, 0.0)
    assert evidence.classify(True, minimum_samples=3, positive_samples=2) == (
        'DYNAMIC_OBSTACLE'
    )


def test_offline_sensor_never_labels_obstacle():
    evidence = PresenceEvidence(1.5, 0.4)
    for _ in range(3):
        evidence.add(True, False, 0.0)
    assert evidence.classify(False, minimum_samples=3, positive_samples=2) is None


def test_mode3_waits_for_space_before_trying_nearest_obstacle():
    inspector = object.__new__(Mode3Inspector)
    inspector.drive_mode = 1
    inspector.phase = 'IDLE'
    inspector.target = None
    inspector.waiting_for_departure = False
    inspector.cancel_publisher = _Publisher()
    states = []
    starts = []
    inspector._state = states.append
    inspector._try_start_inspection = lambda: starts.append(True)

    inspector._mode_callback(Int32(data=3))

    assert inspector.phase == 'ARMED'
    assert starts == []
    assert states[-1] == 'MODE3_READY:PRESS_SPACE'

    inspector._inspection_command_callback(String(data='MODE3_START'))

    assert inspector.phase == 'WAITING_FOR_OBSTACLE'
    assert starts == [True]


def test_mode5_mode3_publishes_only_canonical_plan_with_inspection_yaw():
    inspector = object.__new__(Mode3Inspector)
    inspector.drive_mode = 3
    inspector.phase = 'WAITING_FOR_OBSTACLE'
    inspector.map_frame = 'map'
    inspector.base_frame = 'base_link'
    inspector.tf = _Tf()
    inspector.candidates = []
    inspector.requested_target = (4.0, 0.0)
    inspector.standoff_distance = 2.0
    inspector.standoff_arrival_tolerance = 0.6
    inspector.publish_canonical_plan = True
    inspector.hazard_revision = 7
    inspector.goal_publisher = _Publisher()
    inspector.plan_publisher = _Publisher()
    inspector.waiting_for_departure = False
    inspector._state = lambda _state: None
    inspector.get_clock = lambda: _Clock()

    inspector._try_start_inspection()

    assert inspector.goal_publisher.messages == []
    payload = json.loads(inspector.plan_publisher.messages[-1].data)
    assert payload['selected_exit_id'] == 'MODE3_INSPECTION'
    assert payload['selected_approach_position_world'] == [2.0, 0.0]
    assert payload['selected_approach_yaw_rad'] == 0.0


def test_mode3_already_at_standoff_skips_navigation_and_starts_settling():
    inspector = object.__new__(Mode3Inspector)
    inspector.drive_mode = 3
    inspector.phase = 'WAITING_FOR_OBSTACLE'
    inspector.map_frame = 'map'
    inspector.base_frame = 'base_link'
    inspector.tf = _Tf()
    inspector.candidates = [(1.5, 0.0)]
    inspector.requested_target = None
    inspector.standoff_distance = 2.0
    inspector.standoff_arrival_tolerance = 0.6
    inspector.robot_settle_sec = 2.0
    inspector.publish_canonical_plan = True
    inspector.hazard_revision = 0
    inspector.cancel_publisher = _Publisher()
    inspector.goal_publisher = _Publisher()
    inspector.plan_publisher = _Publisher()
    inspector.waiting_for_departure = False
    states = []
    inspector._state = states.append
    inspector._now = lambda: 10.0
    inspector.get_logger = lambda: type(
        '_Logger', (), {'info': lambda self, _message: None}
    )()

    inspector._try_start_inspection()

    assert inspector.phase == 'SETTLING'
    assert inspector.phase_deadline == 12.0
    assert states[-1] == 'MODE3_AT_STANDOFF:ROBOT_SETTLING'
    assert inspector.goal_publisher.messages == []
    assert inspector.plan_publisher.messages == []
