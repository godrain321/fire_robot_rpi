import json
import math
from pathlib import Path

from builtin_interfaces.msg import Time
import numpy as np
from std_msgs.msg import Int32, String

from inno_autonav.grid_utils import MapGrid, world_to_grid
from inno_autonav.mode3_inspector import (
    Mode3Inspector,
    PresenceEvidence,
    compute_inspection_goal,
    parse_inspection_command,
    select_reachable_inspection_goal,
    select_nearest_candidate,
    select_tracked_candidate,
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


class _TfAtInspectionGoal:
    @staticmethod
    def lookup_pose_2d(_map_frame, _base_frame):
        return 0.04, 0.0, 0.0


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


def _grid(data=None):
    values = (
        np.zeros((200, 200), dtype=np.int8)
        if data is None else np.asarray(data, dtype=np.int8)
    )
    height, width = values.shape
    return MapGrid(width, height, 0.1, -10.0, -10.0, 0.0, 'map', values)


def _set_free_planning_grids(inspector):
    if not hasattr(inspector, 'target'):
        inspector.target = None
    inspector.planning_grid = _grid()
    inspector.static_grid = _grid()
    inspector.unknown_is_occupied = True
    inspector.allow_diagonal = True
    inspector.inspection_max_distance = 2.5
    inspector.target_tracking_radius = 1.0
    inspector.target_stale_timeout = 2.0
    inspector.tracking_candidates = list(inspector.candidates)
    inspector.last_candidates_update = 10.0
    inspector.target_last_seen = float('-inf')
    inspector._now = lambda: 10.0
    inspector.get_logger = lambda: _Logger()


def test_select_nearest_dynamic_obstacle():
    assert select_nearest_candidate(0.0, 0.0, [(4.0, 0.0), (2.0, 1.0)]) == (
        2.0,
        1.0,
    )


def test_selected_obstacle_tracks_the_latest_nearby_lidar_position():
    assert select_tracked_candidate(
        (7.63, -6.19), [(7.91, -6.02), (3.0, 1.0)], 1.0
    ) == (7.91, -6.02)
    assert select_tracked_candidate(
        (7.91, -6.02), [(9.2, -6.0)], 1.0
    ) is None


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


def test_blocked_nominal_goal_is_corrected_to_reachable_two_metre_ring():
    planning = np.zeros((120, 120), dtype=np.int8)
    planning[47:54, 67:74] = 100
    planning_grid = MapGrid(
        120, 120, 0.1, 0.0, 0.0, 0.0, 'map', planning
    )
    static_grid = MapGrid(
        120, 120, 0.1, 0.0, 0.0, 0.0, 'map',
        np.zeros_like(planning),
    )

    selected = select_reachable_inspection_goal(
        (8.0, 5.0), (5.0, 5.0), (7.0, 5.0), 2.0, 0.2,
        planning_grid, static_grid,
    )

    assert selected is not None
    assert math.dist(selected[:2], (7.0, 5.0)) > 0.2
    assert 1.8 <= math.dist(selected[:2], (5.0, 5.0)) <= 2.2
    cell = world_to_grid(selected[0], selected[1], planning_grid)
    assert planning_grid.data[cell[1], cell[0]] == 0


def test_inspection_goal_returns_none_when_safe_ring_is_unreachable():
    planning = np.full((120, 120), 100, dtype=np.int8)
    planning[50, 80] = 0
    planning_grid = MapGrid(
        120, 120, 0.1, 0.0, 0.0, 0.0, 'map', planning
    )
    static_grid = MapGrid(
        120, 120, 0.1, 0.0, 0.0, 0.0, 'map',
        np.zeros_like(planning),
    )

    assert select_reachable_inspection_goal(
        (8.0, 5.0), (5.0, 5.0), (7.0, 5.0), 2.0, 0.2,
        planning_grid, static_grid,
    ) is None


def test_inspection_goal_rejects_a_nominal_goal_too_close_to_robot():
    planning_grid = _grid()
    static_grid = _grid()

    selected = select_reachable_inspection_goal(
        (7.1, 5.0), (5.0, 5.0), (7.0, 5.0), 2.0, 0.2,
        planning_grid, static_grid, minimum_goal_distance_m=0.45,
    )

    assert selected is not None
    assert math.dist(selected[:2], (7.1, 5.0)) >= 0.45 - 1e-9
    assert 1.8 <= math.dist(selected[:2], (5.0, 5.0)) <= 2.2


def test_presence_counts_online_samples_without_a_distance_gate():
    evidence = PresenceEvidence()
    evidence.add(False, True)
    evidence.add(True, True)
    evidence.add(True, True)
    evidence.add(True, True)
    assert evidence.total_samples == 3
    assert evidence.positive_samples == 3
    assert evidence.classify(True, minimum_samples=3, positive_samples=3) == (
        'PERSON'
    )


def test_online_samples_without_presence_are_dynamic_obstacle():
    evidence = PresenceEvidence()
    for _ in range(3):
        evidence.add(True, False)
    assert evidence.classify(True, minimum_samples=3, positive_samples=2) == (
        'DYNAMIC_OBSTACLE'
    )


def test_offline_sensor_never_labels_obstacle():
    evidence = PresenceEvidence()
    for _ in range(3):
        evidence.add(True, False)
    assert evidence.classify(False, minimum_samples=3, positive_samples=2) is None


def test_mode3_uses_three_second_presence_only_field_profile():
    package = Path(__file__).resolve().parents[1]
    source = (package / 'inno_autonav' / 'mode3_inspector.py').read_text(
        encoding='utf-8'
    )
    config = (package / 'config' / 'autonav_params.yaml').read_text(
        encoding='utf-8'
    )

    assert "'observation_sec': 3.0" in source
    assert 'observation_sec: 3.0' in config
    assert 'distance_tolerance_m' not in source
    assert 'distance_tolerance_m' not in config


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
    inspector.standoff_arrival_tolerance = 0.3
    inspector.minimum_approach_goal_distance = 0.45
    inspector.publish_canonical_plan = True
    inspector.hazard_revision = 7
    inspector.goal_publisher = _Publisher()
    inspector.plan_publisher = _Publisher()
    inspector.waiting_for_departure = False
    inspector.approach_started = False
    _set_free_planning_grids(inspector)
    inspector._state = lambda _state: None
    inspector.get_clock = lambda: _Clock()

    inspector._try_start_inspection()

    assert inspector.goal_publisher.messages == []
    payload = json.loads(inspector.plan_publisher.messages[-1].data)
    assert payload['selected_exit_id'] == 'MODE3_INSPECTION'
    assert payload['selected_approach_position_world'] == [2.0, 0.0]
    assert payload['selected_approach_yaw_rad'] == 0.0


def test_mode3_starts_inspection_immediately_inside_latest_2_5m_range():
    inspector = object.__new__(Mode3Inspector)
    inspector.drive_mode = 3
    inspector.phase = 'WAITING_FOR_OBSTACLE'
    inspector.map_frame = 'map'
    inspector.base_frame = 'base_link'
    inspector.tf = _Tf()
    inspector.candidates = [(1.5, 0.0)]
    inspector.requested_target = None
    inspector.standoff_distance = 2.0
    inspector.standoff_arrival_tolerance = 0.3
    inspector.minimum_approach_goal_distance = 0.45
    inspector.robot_settle_sec = 2.0
    inspector.publish_canonical_plan = True
    inspector.hazard_revision = 0
    inspector.cancel_publisher = _Publisher()
    inspector.goal_publisher = _Publisher()
    inspector.plan_publisher = _Publisher()
    inspector.waiting_for_departure = False
    inspector.approach_started = False
    _set_free_planning_grids(inspector)
    states = []
    inspector._state = states.append
    inspector._now = lambda: 10.0
    inspector.get_logger = lambda: type(
        '_Logger', (), {'info': lambda self, _message: None}
    )()
    inspector.get_clock = lambda: _Clock()

    inspector._try_start_inspection()

    assert inspector.phase == 'SETTLING'
    assert inspector.approach_started is False
    assert inspector.waiting_for_departure is False
    assert states[-1] == 'MODE3_AT_STANDOFF:ROBOT_SETTLING'
    assert inspector.goal_publisher.messages == []
    assert inspector.plan_publisher.messages == []
    assert len(inspector.cancel_publisher.messages) == 1


def test_mode3_does_not_observe_until_following_and_arrival_are_confirmed():
    inspector = object.__new__(Mode3Inspector)
    inspector.drive_mode = 3
    inspector.phase = 'NAVIGATING'
    inspector.waiting_for_departure = True
    inspector.approach_started = False
    inspector.map_frame = 'map'
    inspector.base_frame = 'base_link'
    inspector.target = (2.0, 0.0)
    inspector.active_standoff_distance = 2.0
    inspector.standoff_distance = 2.0
    inspector.standoff_arrival_tolerance = 0.20
    inspector.inspection_max_distance = 2.5
    inspector.target_stale_timeout = 2.0
    inspector.target_last_seen = 10.0
    inspector.robot_settle_sec = 2.0
    inspector.tf = _TfAtInspectionGoal()
    inspector.cancel_publisher = _Publisher()
    inspector._now = lambda: 10.0
    inspector.get_logger = lambda: _Logger()
    states = []
    inspector._state = states.append

    inspector._follower_callback(String(data='PATH_ACCEPTED'))
    inspector._follower_callback(String(data='GOAL_REACHED'))
    assert inspector.phase == 'NAVIGATING'
    assert states == []

    inspector._follower_callback(String(data='FOLLOWING_PATH'))
    inspector._follower_callback(String(data='GOAL_REACHED'))
    assert inspector.phase == 'SETTLING'
    assert inspector.phase_deadline == 12.0
    assert states[-1] == 'MODE3_AT_STANDOFF:ROBOT_SETTLING'


def test_navigation_uses_latest_target_distance_to_enter_2_5m_range():
    inspector = object.__new__(Mode3Inspector)
    inspector.drive_mode = 3
    inspector.phase = 'NAVIGATING'
    inspector.map_frame = 'map'
    inspector.base_frame = 'base_link'
    inspector.target = (2.49, 0.0)
    inspector.target_last_seen = 10.0
    inspector.target_stale_timeout = 2.0
    inspector.inspection_max_distance = 2.5
    inspector.robot_settle_sec = 2.0
    inspector.tf = _Tf()
    inspector.cancel_publisher = _Publisher()
    inspector._now = lambda: 10.0
    inspector.get_logger = lambda: _Logger()
    states = []
    inspector._state = states.append

    inspector._timer_callback()

    assert inspector.phase == 'SETTLING'
    assert inspector.phase_deadline == 12.0
    assert states[-1] == 'MODE3_AT_STANDOFF:ROBOT_SETTLING'


def test_retained_explicit_target_is_not_treated_as_a_live_lidar_sample():
    inspector = object.__new__(Mode3Inspector)
    inspector.drive_mode = 3
    inspector.phase = 'ARMED'
    inspector.target = None
    inspector.requested_target = None
    inspector.waiting_for_departure = False
    inspector.approach_started = False
    inspector.cancel_publisher = _Publisher()
    inspector._now = lambda: 10.0
    inspector._state = lambda _state: None
    inspector._try_start_inspection = lambda: None

    inspector._inspection_command_callback(String(data='MODE3_START_AT:1.5,0.0'))

    assert inspector.requested_target == (1.5, 0.0)
    assert inspector.target_last_seen == float('-inf')


def test_goal_reached_waits_for_current_scan_when_target_is_stale():
    inspector = object.__new__(Mode3Inspector)
    inspector.drive_mode = 3
    inspector.phase = 'NAVIGATING'
    inspector.waiting_for_departure = False
    inspector.approach_started = True
    inspector.map_frame = 'map'
    inspector.base_frame = 'base_link'
    inspector.target = (2.0, 0.0)
    inspector.target_last_seen = 1.0
    inspector.target_stale_timeout = 2.0
    inspector.inspection_max_distance = 2.5
    inspector.tf = _Tf()
    inspector.cancel_publisher = _Publisher()
    inspector._now = lambda: 10.0
    states = []
    inspector._state = states.append

    inspector._follower_callback(String(data='GOAL_REACHED'))

    assert inspector.phase == 'WAITING_FOR_LIVE_TARGET'
    assert states[-1] == 'MODE3_WAITING_FOR_LIVE_TARGET'
    assert len(inspector.cancel_publisher.messages) == 1


def test_settling_survives_current_scan_dropout_within_grace_period():
    inspector = object.__new__(Mode3Inspector)
    inspector.drive_mode = 3
    inspector.phase = 'SETTLING'
    inspector.map_frame = 'map'
    inspector.base_frame = 'base_link'
    inspector.target = (2.2, 0.0)
    inspector.target_last_seen = 10.0
    inspector.target_stale_timeout = 2.0
    inspector.inspection_max_distance = 2.5
    inspector.phase_deadline = 11.5
    inspector.tf = _Tf()
    inspector._now = lambda: 11.5
    restarts = []
    inspector._restart_for_latest_target = restarts.append
    inspector._start_observation = lambda: setattr(
        inspector, 'phase', 'OBSERVING'
    )

    inspector._timer_callback()

    assert inspector.phase == 'OBSERVING'
    assert restarts == []


def test_settling_cancels_when_lidar_target_disappears_past_timeout():
    inspector = object.__new__(Mode3Inspector)
    inspector.drive_mode = 3
    inspector.phase = 'SETTLING'
    inspector.target = (2.2, 0.0)
    inspector.target_last_seen = 10.0
    inspector.target_stale_timeout = 2.0
    inspector.phase_deadline = 13.0
    inspector.cancel_publisher = _Publisher()
    inspector._now = lambda: 13.0
    states = []
    inspector._state = states.append
    inspector.get_logger = lambda: _Logger()
    started = []
    inspector._start_observation = lambda: started.append(True)

    inspector._timer_callback()

    assert inspector.phase == 'TARGET_LOST'
    assert states[-1] == 'MODE3_TARGET_TRACK_LOST'
    assert len(inspector.cancel_publisher.messages) == 1
    assert started == []


def test_confirmed_stationary_person_publishes_assistance_location():
    inspector = object.__new__(Mode3Inspector)
    inspector.map_frame = 'map'
    inspector.target = (3.42, -7.18)
    inspector.observation_target_start = (3.40, -7.20)
    inspector.observation_target_samples = [(3.42, -7.18), (3.45, -7.16)]
    inspector.sensor_online = True
    inspector.last_sensor_update = 10.0
    inspector.sensor_stale_timeout = 2.0
    inspector.minimum_samples = 3
    inspector.positive_samples = 3
    inspector.evidence = type(
        'Evidence', (), {'classify': lambda self, *_args: 'PERSON'}
    )()
    inspector._now = lambda: 10.0
    inspector.get_clock = lambda: _Clock()
    inspector.person_publisher = _Publisher()
    inspector.assistance_publisher = _Publisher()
    inspector.classification_publisher = _Publisher()
    inspector._state = lambda _state: None
    inspector.get_logger = lambda: _Logger()

    inspector._finish_observation()

    assert len(inspector.person_publisher.messages) == 1
    assert len(inspector.assistance_publisher.messages) == 1
    point = inspector.assistance_publisher.messages[0]
    assert point.header.frame_id == 'map'
    assert (point.point.x, point.point.y) == (3.42, -7.18)


def test_confirmed_person_moving_over_threshold_is_not_assistance_case():
    inspector = object.__new__(Mode3Inspector)
    inspector.map_frame = 'map'
    inspector.target = (3.30, -7.00)
    inspector.observation_target_start = (3.00, -7.00)
    inspector.observation_target_samples = [(3.30, -7.00)]
    inspector.sensor_online = True
    inspector.last_sensor_update = 10.0
    inspector.sensor_stale_timeout = 2.0
    inspector.minimum_samples = 3
    inspector.positive_samples = 3
    inspector.evidence = type(
        'Evidence', (), {'classify': lambda self, *_args: 'PERSON'}
    )()
    inspector._now = lambda: 10.0
    inspector.get_clock = lambda: _Clock()
    inspector.person_publisher = _Publisher()
    inspector.assistance_publisher = _Publisher()
    inspector.classification_publisher = _Publisher()
    inspector._state = lambda _state: None
    inspector.get_logger = lambda: _Logger()

    inspector._finish_observation()

    assert len(inspector.person_publisher.messages) == 1
    assert inspector.assistance_publisher.messages == []
