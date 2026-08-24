import time
from types import SimpleNamespace

from geometry_msgs.msg import Pose, PoseArray

from inno_autonav.evacuation_demo import MovingCandidateTracker
from inno_autonav.evacuation_demo_orchestrator import EvacuationDemoOrchestrator


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
