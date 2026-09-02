from collections import Counter
import json
import math
import time
from types import SimpleNamespace

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32, String

from inno_autonav.evacuation_demo import MovingCandidateTracker
from inno_autonav.evacuation_demo_orchestrator import (
    EvacuationDemoOrchestrator,
    exit_navigation_log,
    mode3_inspection_progress_log,
)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeTf:
    def __init__(self, pose):
        self.pose = pose

    def lookup_pose_2d(self, _map_frame, _base_frame):
        return self.pose


def pose_array(x, y):
    message = PoseArray()
    message.header.frame_id = 'map'
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.w = 1.0
    message.poses.append(pose)
    return message


def nearest_inspection_node(candidates):
    node = object.__new__(EvacuationDemoOrchestrator)
    node.map_frame = 'map'
    node.base_frame = 'base_link'
    node.tf = FakeTf((0.0, 0.0, 0.0))
    node._phase = 'NAVIGATING_EXIT'
    node._route_activated = False
    node.active_survivor_position = None
    node.candidates = candidates
    node.moving_priority_enabled = False
    node.inspected_dynamic_positions = []
    node.candidate_suppression_radius = 1.0
    node.survivor_exit_id = None
    node.inspection_after_motion_delay = 2.0
    node.waiting_for_departure = False
    node._inspection_allowed_after = 0.0
    node.classification_radius = 0.75
    node.current_exit_id = 'exit1'
    node.current_exit_position = (2.0, 0.0)
    node.current_approach_position = (1.5, 0.0)
    node.current_plan_payload = '{"selected_exit_id":"exit1"}'
    node.exit_obstacle_radius = 0.6
    node.inspection_target = None
    node.inspection_start_position = None
    node.inspection_blocks_current_exit = False
    node.mode3_status = ''
    node._inspection_command_sent = True
    node.cancel_publisher = Publisher()
    node.blocked_exits_publisher = Publisher()
    node.plan_publisher = Publisher()
    node.selected_exit_publisher = Publisher()
    node.published_goals = []
    node._publish_goal = lambda position: node.published_goals.append(
        tuple(position)
    )
    node._set_status = lambda _message: None
    node._log = lambda _message: None
    node._select_drive_mode = lambda mode: setattr(node, 'selected_mode', mode)
    return node


def moving_priority_node(candidates):
    node = nearest_inspection_node(candidates)
    node.moving_priority_enabled = True
    node.moving_priority_wait = 2.0
    node.moving_priority_target_timeout = 2.0
    node.moving_association_radius = 0.75
    node.moving_priority_targets = []
    node._candidate_wait_started_at = None
    return node


def test_moving_red_candidate_beats_nearer_stationary_candidate():
    node = moving_priority_node([(1.0, 0.0), (2.0, 0.0)])
    node.moving_priority_targets = [((2.1, 0.0), time.monotonic())]
    node._maybe_start_nearest_inspection()
    assert node.inspection_target == (2.0, 0.0)
    assert node.selected_mode == 3


def test_stationary_candidate_wait_deadline_is_not_restarted():
    node = moving_priority_node([(1.0, 0.0)])
    node._maybe_start_nearest_inspection()
    started = node._candidate_wait_started_at
    node._maybe_start_nearest_inspection()
    assert node._candidate_wait_started_at == started
    node._candidate_wait_started_at = time.monotonic() - 2.1
    node._maybe_start_nearest_inspection()
    assert node.inspection_target == (1.0, 0.0)


def test_mode5_locks_only_closest_red_candidate_and_selects_mode3():
    node = nearest_inspection_node([(3.0, 0.0), (1.5, 0.0), (2.0, 1.0)])

    node._maybe_start_nearest_inspection()

    assert node.inspection_target == (1.5, 0.0)
    assert node.inspection_blocks_current_exit is True
    assert node._phase == 'SELECTING_MODE3'
    assert node.selected_mode == 3
    assert len(node.cancel_publisher.messages) == 1


def test_mode9_selects_only_lidar_confirmed_stationary_red_candidate():
    node = nearest_inspection_node([(1.0, 0.0), (2.0, 0.0)])
    node.stationary_combined_inspection_enabled = True
    node.stationary_target_timeout = 1.0
    node.stationary_targets = [((2.05, 0.0), time.monotonic())]
    node.moving_association_radius = 0.75

    node._maybe_start_nearest_inspection()

    assert node.inspection_target == (2.0, 0.0)
    assert node.combined_inspection_active is True
    assert node._phase == 'SELECTING_MODE3'
    assert node.selected_mode == 3


def test_mode9_stationary_candidate_waits_for_mmwave_and_yolo_results():
    node = nearest_inspection_node([(1.5, 0.0)])
    node._phase = 'INSPECTING_CANDIDATE'
    node.inspection_target = (1.5, 0.0)
    node.inspection_blocks_current_exit = True
    node.combined_inspection_active = True
    node.combined_mmwave_person = None
    node.mode4_status = ''
    node.checked_exit_ids = set()
    node.blocked_exit_ids = set()

    node._on_mode3_classification(String(data='DYNAMIC_OBSTACLE:1.5,0.0'))

    assert node._phase == 'SELECTING_MODE4'
    assert node.combined_mmwave_person is False
    assert node.selected_mode == 4

    node._phase = 'INSPECTING_MOVING_CANDIDATE'
    node._on_mode4_classification(String(data='NO_SURVIVOR'))

    assert node._phase == 'RETURNING_MODE5'
    assert node.blocked_exit_ids == {'exit1'}
    assert node.combined_inspection_active is False
    assert node.selected_mode == 5


def test_mode5_waits_for_real_route_motion_before_candidate_preemption():
    node = nearest_inspection_node([(1.5, 0.0)])
    node.waiting_for_departure = True
    node._inspection_allowed_after = float('inf')

    node._maybe_start_nearest_inspection()
    assert node._phase == 'NAVIGATING_EXIT'
    assert not node.cancel_publisher.messages

    node._on_follower_state(String(data='PATH_ACCEPTED'))
    assert node._phase == 'NAVIGATING_EXIT'
    assert not node.cancel_publisher.messages

    node._on_follower_state(String(data='FOLLOWING_PATH'))
    assert math.isfinite(node._inspection_allowed_after)
    assert node._phase == 'NAVIGATING_EXIT'
    assert not node.cancel_publisher.messages

    node._inspection_allowed_after = 0.0
    node._maybe_start_nearest_inspection()
    assert node._phase == 'SELECTING_MODE3'
    assert len(node.cancel_publisher.messages) == 1


def test_first_motor_motion_releases_initial_thermal_bypass():
    node = nearest_inspection_node([])
    node.initial_route_ignore_thermal = True
    node._initial_hazard_bypass_active = True
    node.initial_hazard_bypass_publisher = Publisher()

    node._on_follower_state(String(data='PATH_ACCEPTED'))
    assert node._initial_hazard_bypass_active is True
    assert node.initial_hazard_bypass_publisher.messages == []

    node._on_follower_state(String(data='FOLLOWING_PATH'))
    assert node._initial_hazard_bypass_active is False
    assert node.initial_hazard_bypass_publisher.messages[-1].data is False


def test_mode5_logs_danger_expected_and_syncs_replacement_exit_plan():
    node = nearest_inspection_node([])
    node._requested = True
    node.checked_exit_ids = set()
    node.danger_expected_exit_ids = set()
    statuses = []
    logs = []
    node._set_status = statuses.append
    node._log = logs.append

    node._on_danger_expected_exits(String(data='["exit1"]'))
    assert node.checked_exit_ids == {"exit1"}
    assert len(node.cancel_publisher.messages) == 1
    assert statuses[-1] == "SEARCH_EXITS:DANGER_EXPECTED:exit1"
    assert "위험 예상 상태로 판정했습니다" in logs[-1]
    assert "36°C" not in logs[-1]
    assert "3초 연속" not in logs[-1]
    assert "근접 조건" not in logs[-1]

    node._on_canonical_evacuation_plan(String(data=json.dumps({
        "success": True,
        "activated": True,
        "manager_status": "ROUTE_ACTIVATED",
        "selected_exit_id": "exit2",
        "selected_exit_position_world": [8.0, 1.0],
        "selected_approach_position_world": [7.5, 1.0],
    })))
    assert node.current_exit_id == "exit2"
    assert node.current_exit_position == (8.0, 1.0)
    assert node.current_approach_position == (7.5, 1.0)
    assert node.waiting_for_departure is True
    assert statuses[-1] == "SEARCH_EXITS:NAVIGATING:exit2"
    assert "이용 가능한 출구 exit2" in logs[-1]


def test_danger_expected_switches_survivor_escort_from_exit2_to_exit3():
    node = nearest_inspection_node([])
    node._requested = True
    node._phase = "ESCORTING_SURVIVOR"
    node.current_exit_id = None
    node.survivor_exit_id = "EXIT2"
    node._route_activated = True
    node.checked_exit_ids = set()
    node.danger_expected_exit_ids = set()
    statuses = []
    logs = []
    node._set_status = statuses.append
    node._log = logs.append

    node._on_danger_expected_exits(String(data='["EXIT2"]'))
    assert len(node.cancel_publisher.messages) == 1
    assert node.checked_exit_ids == {"EXIT2"}
    assert statuses[-1] == "SEARCH_EXITS:DANGER_EXPECTED:EXIT2"

    payload = json.dumps({
        "success": True,
        "activated": True,
        "manager_status": "ROUTE_ACTIVATED",
        "selected_exit_id": "EXIT3",
        "selected_exit_position_world": [-0.295, 0.048],
        "selected_approach_position_world": [-0.003, -0.447],
        "hazard_revision": 9,
    })
    node._on_canonical_evacuation_plan(String(data=payload))

    assert node.survivor_exit_id == "EXIT3"
    assert node.current_exit_id == "EXIT3"
    assert node._phase == "ESCORTING_SURVIVOR"
    assert node._route_activated is True
    assert node.waiting_for_departure is True
    assert node.plan_publisher.messages[-1].data == payload
    assert node.selected_exit_publisher.messages[-1].data == "EXIT3"
    assert node.published_goals[-1] == (-0.003, -0.447)
    assert statuses[-1] == "ESCORTING_SURVIVOR:EXIT3"
    assert "이용 가능한 출구 EXIT3" in logs[-1]


def test_mode5_operator_logs_name_inspection_and_exit_change_phases():
    assert mode3_inspection_progress_log(
        'MODE3_AT_STANDOFF:ROBOT_SETTLING'
    ).startswith('[도착]')
    assert mode3_inspection_progress_log(
        'MODE3_MMWAVE_OBSERVING'
    ).startswith('[생체 판별]')
    assert mode3_inspection_progress_log(
        'MODE3_TARGET_MOVED:REPLANNING:DISTANCE:2.70M'
    ).startswith('[재접근]')
    assert mode3_inspection_progress_log('MODE3_READY:PRESS_SPACE') is None
    assert exit_navigation_log('exit2', set()) == (
        '[출구 선택] 가장 가까운 출구 exit2로 이동합니다.'
    )
    changed = exit_navigation_log('exit3', {'exit2', 'exit1'})
    assert changed.startswith('[출구 변경]')
    assert '목록(exit1, exit2)' in changed
    assert '다음 출구 exit3' in changed


def test_mode5_updates_active_inspection_target_from_latest_lidar_candidate():
    node = nearest_inspection_node([(1.5, 0.0)])
    node._phase = 'INSPECTING_CANDIDATE'
    node.inspection_target = (1.5, 0.0)

    node._on_all_candidates(pose_array(1.9, 0.1))

    assert node.inspection_target == (1.9, 0.1)


def test_disappeared_lidar_target_cancels_inspection_and_resumes_route():
    node = nearest_inspection_node([(0.5, 2.0), (4.0, 0.0)])
    node._maybe_start_nearest_inspection()
    node._phase = 'INSPECTING_CANDIDATE'
    node.all_candidates = [(0.55, 2.05), (4.0, 0.0)]
    node.moving_priority_targets = [((0.55, 2.05), time.monotonic())]
    node.stationary_targets = [((0.55, 2.05), time.monotonic())]
    node.combined_inspection_active = False
    node.combined_mmwave_person = None
    node.inspected_dynamic_positions = []
    statuses = []
    logs = []
    node._set_status = statuses.append
    node._log = logs.append

    node._on_mode3_status(String(data='MODE3_TARGET_TRACK_LOST'))

    assert node._phase == 'RETURNING_MODE5'
    assert node.selected_mode == 5
    assert node.current_exit_id == 'exit1'
    assert node._resume_phase_after_inspection == 'RESUME_EXIT_ROUTE'
    assert node.inspection_target is None
    assert node.inspection_start_position is None
    assert node.inspection_blocks_current_exit is False
    assert node.inspected_dynamic_positions == []
    assert node.candidates == [(4.0, 0.0)]
    assert node.all_candidates == [(4.0, 0.0)]
    assert node.moving_priority_targets == []
    assert node.stationary_targets == []
    assert statuses[-1] == 'SEARCH_EXITS:INSPECTION_TARGET_DISAPPEARED'
    assert any('기존 대피 경로로 복귀' in value for value in logs)


def test_external_mode5_command_starts_idle_orchestrator():
    node = object.__new__(EvacuationDemoOrchestrator)
    node.enabled = True
    node._future = None
    node._requested = False
    node.drive_mode_status = '1:KEYBOARD'
    node._internal_mode_commands = Counter()
    reset_calls = []
    statuses = []
    logs = []
    selected_modes = []
    node._reset_exploration = lambda: reset_calls.append(True)
    node._set_status = statuses.append
    node._log = logs.append
    node._select_drive_mode = selected_modes.append

    node._on_mode_command(Int32(data=5))

    assert node._requested is True
    assert node.drive_mode_status == ''
    assert reset_calls == [True]
    assert statuses == ['SEARCH_EXITS:STARTING']
    assert selected_modes == [5]
    assert any('숫자 5 입력' in message for message in logs)


def test_mmwave_nonperson_at_exit_blocks_it_and_returns_to_mode5():
    node = nearest_inspection_node([(1.5, 0.0)])
    node._maybe_start_nearest_inspection()
    node._phase = 'INSPECTING_CANDIDATE'
    node.checked_exit_ids = set()
    node.blocked_exit_ids = set()

    node._on_mode3_classification(String(data='DYNAMIC_OBSTACLE:1.5,0.0'))

    assert node.checked_exit_ids == {'exit1'}
    assert node.blocked_exit_ids == {'exit1'}
    assert node.current_exit_id is None
    assert node._resume_phase_after_inspection == 'STARTING'
    assert node._phase == 'RETURNING_MODE5'
    assert node.selected_mode == 5


def test_mmwave_nonperson_away_from_exit_resumes_same_route():
    node = nearest_inspection_node([(0.5, 2.0)])
    node._maybe_start_nearest_inspection()
    node._phase = 'INSPECTING_CANDIDATE'
    node.checked_exit_ids = set()
    node.blocked_exit_ids = set()

    node._on_mode3_classification(String(data='DYNAMIC_OBSTACLE:0.5,2.0'))

    assert node.blocked_exit_ids == set()
    assert node.current_exit_id == 'exit1'
    assert node._resume_phase_after_inspection == 'RESUME_EXIT_ROUTE'
    assert node._phase == 'RETURNING_MODE5'


def test_completed_inspection_suppresses_initial_and_latest_map_positions():
    node = nearest_inspection_node([(0.5, 2.0)])
    node._maybe_start_nearest_inspection()
    assert node.inspection_start_position == (0.5, 2.0)
    node._phase = 'INSPECTING_CANDIDATE'
    node.checked_exit_ids = set()
    node.blocked_exit_ids = set()

    # The same LiDAR cluster moves while Mode 3 approaches it.
    node._on_all_candidates(pose_array(1.1, 2.0))
    node._on_mode3_classification(String(data='DYNAMIC_OBSTACLE:1.1,2.0'))

    assert (0.5, 2.0) in node.inspected_dynamic_positions
    assert (1.1, 2.0) in node.inspected_dynamic_positions

    # Both the stale original marker and a jittered latest marker are ignored;
    # only a genuinely separate map location may start another inspection.
    node._phase = 'NAVIGATING_EXIT'
    node.waiting_for_departure = False
    node._inspection_allowed_after = 0.0
    node.candidates = [(0.4, 2.1), (1.8, 2.0), (3.5, 0.0)]
    node._maybe_start_nearest_inspection()

    assert node.inspection_target == (3.5, 0.0)


def test_camera_non_survivor_records_map_position_before_returning_mode5():
    node = nearest_inspection_node([(0.5, 2.0)])
    node._maybe_start_nearest_inspection()
    node._phase = 'INSPECTING_MOVING_CANDIDATE'
    node.checked_exit_ids = set()
    node.blocked_exit_ids = set()

    node._on_mode4_classification(String(data='NO_SURVIVOR'))

    assert (0.5, 2.0) in node.inspected_dynamic_positions
    assert node.inspection_start_position is None
    assert node._phase == 'RETURNING_MODE5'


def test_moving_lidar_track_preempts_exit_route_and_selects_mode4():
    node = object.__new__(EvacuationDemoOrchestrator)
    node.map_frame = 'map'
    node.active_survivor_position = None
    node.moving_survivor_enabled = True
    node._phase = 'NAVIGATING_EXIT'
    node._route_activated = False
    node.moving_tracker = MovingCandidateTracker(
        association_radius_m=0.75,
        minimum_displacement_m=0.20,
        minimum_observations=3,
    )
    node.cancel_publisher = Publisher()
    node.all_candidates = []
    node.inspection_target = None
    node.mode4_status = ''
    node._inspection_command_sent = True
    states = []
    logs = []
    selected_modes = []
    node._set_status = states.append
    node._log = logs.append
    node._select_drive_mode = selected_modes.append

    node._on_all_candidates(pose_array(1.00, 2.0))
    node._on_all_candidates(pose_array(1.15, 2.0))
    node._on_all_candidates(pose_array(1.35, 2.0))

    assert node._phase == 'SELECTING_MODE4'
    assert node.inspection_target == (1.35, 2.0)
    assert selected_modes == [4]
    assert any('움직이는 LiDAR' in message for message in logs)


def escort_node(robot_pose, survivor_position):
    node = object.__new__(EvacuationDemoOrchestrator)
    node.map_frame = 'map'
    node.base_frame = 'base_link'
    node.tf = FakeTf(robot_pose)
    node.active_survivor_position = survivor_position
    node.active_survivor_seen_at = time.monotonic()
    node.survivor_track_stale = 2.0
    node.follow_stop_distance = 2.5
    node.follow_resume_distance = 2.0
    node.exit_arrival_distance = 2.5
    node.survivor_exit_id = 'exit2'
    node._phase = 'ESCORTING_SURVIVOR'
    node._survivor_hold = False
    node.follow_hold_publisher = Publisher()
    node.status_publisher = Publisher()
    node.log_publisher = Publisher()
    node._status_value = ''
    node.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    return node


def test_survivor_escort_stops_when_far_and_resumes_when_close():
    node = escort_node((0.0, 0.0, 0.0), (3.0, 0.0))

    node._tick_survivor_escort()
    assert node._survivor_hold is True
    assert node.follow_hold_publisher.messages[-1].data is True

    node.active_survivor_position = (1.5, 0.0)
    node.active_survivor_seen_at = time.monotonic()
    node._tick_survivor_escort()

    assert node._survivor_hold is False
    assert node.follow_hold_publisher.messages[-1].data is False


def test_survivor_must_reach_the_exit_before_mode5_completes():
    node = escort_node((8.0, 0.0, 0.0), (8.5, 0.0))
    node._phase = 'WAITING_SURVIVOR_AT_EXIT'
    node._survivor_hold = True

    node._tick_survivor_escort()

    assert node._phase == 'EVACUATION_COMPLETE'
    assert node._status_value == 'EVACUATION_COMPLETE:SURVIVOR:exit2'
    assert '동행 대피 완료' in node.log_publisher.messages[-1].data
