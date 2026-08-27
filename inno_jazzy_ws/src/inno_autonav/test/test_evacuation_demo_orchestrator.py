from collections import Counter
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
    node.inspected_dynamic_positions = []
    node.candidate_suppression_radius = 1.0
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
    node.inspection_blocks_current_exit = False
    node.mode3_status = ''
    node._inspection_command_sent = True
    node.cancel_publisher = Publisher()
    node.blocked_exits_publisher = Publisher()
    node._set_status = lambda _message: None
    node._log = lambda _message: None
    node._select_drive_mode = lambda mode: setattr(node, 'selected_mode', mode)
    return node


def test_mode5_locks_only_closest_red_candidate_and_selects_mode3():
    node = nearest_inspection_node([(3.0, 0.0), (1.5, 0.0), (2.0, 1.0)])

    node._maybe_start_nearest_inspection()

    assert node.inspection_target == (1.5, 0.0)
    assert node.inspection_blocks_current_exit is True
    assert node._phase == 'SELECTING_MODE3'
    assert node.selected_mode == 3
    assert len(node.cancel_publisher.messages) == 1


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


def test_mode5_operator_logs_name_inspection_and_exit_change_phases():
    assert mode3_inspection_progress_log(
        'MODE3_AT_STANDOFF:ROBOT_SETTLING'
    ).startswith('[도착]')
    assert mode3_inspection_progress_log(
        'MODE3_MMWAVE_OBSERVING'
    ).startswith('[생체 판별]')
    assert mode3_inspection_progress_log('MODE3_READY:PRESS_SPACE') is None
    assert exit_navigation_log('exit2', set()) == (
        '[출구 선택] 가장 가까운 출구 exit2로 이동합니다.'
    )
    changed = exit_navigation_log('exit3', {'exit2', 'exit1'})
    assert changed.startswith('[출구 변경]')
    assert '목록(exit1, exit2)' in changed
    assert '다음 출구 exit3' in changed


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
