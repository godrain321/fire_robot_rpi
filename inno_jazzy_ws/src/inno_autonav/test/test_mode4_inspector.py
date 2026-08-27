import json
import math

from builtin_interfaces.msg import Time

from std_msgs.msg import Int32, String

from inno_autonav.mode4_inspector import (
    Mode4Inspector,
    parse_detection_message,
    parse_mode4_inspection_command,
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


def test_parse_detection_payload_filters_low_confidence_box():
    payload = (
        '{"image_width":1000,"image_height":700,"detections":['
        '{"x_min":600,"y_min":10,"x_max":800,"y_max":690,'
        '"confidence":0.91},'
        '{"x_min":100,"y_min":10,"x_max":200,"y_max":690,'
        '"confidence":0.2}]}'
    )

    width, height, detections = parse_detection_message(payload, 0.5)

    assert (width, height) == (1000, 700)
    assert len(detections) == 1
    assert detections[0].center_x == 700.0


def test_mode4_votes_for_locked_target_without_any_angle_gate():
    inspector = object.__new__(Mode4Inspector)
    inspector.phase = 'OBSERVING'
    inspector.target = (5.0, -2.0)
    inspector.minimum_confidence = 0.40
    inspector.observation_sec = 5.0
    inspector.minimum_frames = 2
    inspector.positive_frames = 2
    inspector.frame_count = 0
    inspector.person_frame_count = 0
    inspector.positive_frame_count = 0
    inspector.person_detection_count = 0
    inspector.maximum_person_confidence = 0.0
    inspector.candidate_votes = {}
    inspector.last_detector_frame = float('-inf')
    inspector._now = lambda: 10.0
    inspector.get_logger = lambda: type(
        'Logger', (), {'warning': lambda _self, _message: None}
    )()
    payload = json.dumps({
        'image_width': 1280,
        'image_height': 720,
        # Deliberately place the person at the far image edge.  Mode 4 has
        # already faced the inspection target, so pixel bearing is irrelevant.
        'detections': [{
            'x_min': 1080,
            'y_min': 30,
            'x_max': 1270,
            'y_max': 710,
            'confidence': 0.82,
        }],
    })

    inspector._detection_callback(String(data=payload))

    assert inspector.frame_count == 1
    assert inspector.person_frame_count == 1
    assert inspector.positive_frame_count == 1
    assert inspector.candidate_votes == {0: 1}
    assert inspector.maximum_person_confidence == 0.82


def test_mode4_confirms_first_positive_frame_without_waiting_for_pi_inference():
    inspector = object.__new__(Mode4Inspector)
    inspector.phase = 'OBSERVING'
    inspector.target = (2.0, 0.0)
    inspector.minimum_confidence = 0.40
    inspector.observation_sec = 20.0
    inspector.minimum_frames = 1
    inspector.positive_frames = 1
    inspector.frame_count = 0
    inspector.person_frame_count = 0
    inspector.positive_frame_count = 0
    inspector.person_detection_count = 0
    inspector.maximum_person_confidence = 0.0
    inspector.candidate_votes = {}
    inspector.last_detector_frame = float('-inf')
    inspector._now = lambda: 10.0
    inspector.get_logger = lambda: type(
        'Logger', (), {'warning': lambda _self, _message: None}
    )()
    finished = []
    inspector._finish_observation = lambda: finished.append(True)
    payload = json.dumps({
        'image_width': 1280,
        'image_height': 720,
        'detections': [{
            'x_min': 50,
            'y_min': 30,
            'x_max': 600,
            'y_max': 710,
            'confidence': 0.82,
        }],
    })

    inspector._detection_callback(String(data=payload))

    assert finished == [True]


def test_mode4_fresh_detection_payload_does_not_wait_for_online_status():
    inspector = object.__new__(Mode4Inspector)
    inspector.phase = 'OBSERVING'
    inspector.detector_status = 'READY_WAITING_FOR_IMAGE'
    inspector.last_detector_frame = 9.0
    inspector.detector_stale_timeout = 30.0
    inspector.frame_count = 1
    inspector.minimum_frames = 1
    inspector.positive_frames = 2
    inspector.positive_frame_count = 0
    inspector.person_frame_count = 0
    inspector.person_detection_count = 0
    inspector.maximum_person_confidence = 0.0
    inspector.candidate_votes = {}
    inspector.classification_publisher = _Publisher()
    inspector.status_publisher = _Publisher()
    inspector._now = lambda: 10.0
    inspector._state = lambda _state: None
    inspector.get_logger = lambda: type(
        'Logger', (), {'warning': lambda _self, _message: None}
    )()

    inspector._finish_observation()

    assert inspector.phase == 'COMPLETE'
    assert inspector.classification_publisher.messages[-1].data == 'NO_SURVIVOR'


def test_mode4_waits_for_space_before_trying_nearest_obstacle():
    inspector = object.__new__(Mode4Inspector)
    inspector.drive_mode = 1
    inspector.phase = 'IDLE'
    inspector.target = None
    inspector.waiting_for_departure = False
    inspector.cancel_publisher = _Publisher()
    states = []
    starts = []
    inspector._state = states.append
    inspector._try_start_inspection = lambda: starts.append(True)

    inspector._mode_callback(Int32(data=4))

    assert inspector.phase == 'ARMED'
    assert starts == []
    assert states[-1] == 'MODE4_READY:PRESS_SPACE'

    inspector._inspection_command_callback(String(data='MODE4_START'))

    assert inspector.phase == 'WAITING_FOR_OBSTACLE'
    assert starts == [True]


def test_mode5_can_pass_an_explicit_coordinate_to_mode4():
    accepted, target = parse_mode4_inspection_command(
        'MODE4_START_AT:4.25,-1.50'
    )

    assert accepted is True
    assert target == (4.25, -1.50)


def test_invalid_mode4_target_command_is_rejected():
    assert parse_mode4_inspection_command('MODE4_START_AT:not,a-point') == (
        False, None
    )


def test_explicit_mode4_target_wins_and_publishes_a_canonical_waypoint_plan():
    inspector = object.__new__(Mode4Inspector)
    inspector.drive_mode = 4
    inspector.phase = 'WAITING_FOR_OBSTACLE'
    inspector.map_frame = 'map'
    inspector.base_frame = 'base_link'
    inspector.tf = _Tf()
    inspector.candidates = [(1.0, 0.0)]
    inspector.requested_target = (4.0, 2.0)
    inspector.standoff_distance = 2.0
    inspector.publish_canonical_plan = True
    inspector.hazard_revision = 7
    inspector.goal_publisher = _Publisher()
    inspector.plan_publisher = _Publisher()
    inspector.waiting_for_departure = False
    inspector._state = lambda _state: None
    inspector.get_clock = lambda: _Clock()

    inspector._try_start_inspection()

    assert inspector.target == (4.0, 2.0)
    assert inspector.phase == 'NAVIGATING'
    payload = json.loads(inspector.plan_publisher.messages[-1].data)
    assert inspector.goal_publisher.messages == []
    assert payload['selected_exit_id'] == 'MODE4_INSPECTION'
    assert payload['selected_exit_position_world'] == [4.0, 2.0]
    assert payload['selected_approach_yaw_rad'] is not None
    assert payload['hazard_revision'] == 7


def test_mode4_observation_waits_for_first_detector_frame():
    inspector = object.__new__(Mode4Inspector)
    inspector.target = (2.0, 0.0)
    inspector.candidates = [(2.0, 0.0)]
    inspector.observation_sec = 5.0
    inspector.detector_startup_timeout = 8.0
    inspector._now = lambda: 10.0
    inspector._state = lambda _state: None
    inspector.get_logger = lambda: type(
        'Logger', (), {'warning': lambda _self, _message: None}
    )()

    inspector._start_observation()

    assert inspector.phase == 'OBSERVING'
    assert math.isinf(inspector.phase_deadline)
    assert inspector.detector_start_deadline == 18.0
    assert inspector.frame_count == 0


def test_mode4_detector_startup_timeout_is_separate_from_observation_time():
    inspector = object.__new__(Mode4Inspector)
    inspector.drive_mode = 4
    inspector.phase = 'OBSERVING'
    inspector.frame_count = 0
    inspector.detector_start_deadline = 18.0
    inspector.phase_deadline = float('inf')
    finished = []
    inspector._finish_observation = lambda: finished.append(True)

    inspector._now = lambda: 17.9
    inspector._timer_callback()
    assert finished == []

    inspector._now = lambda: 18.0
    inspector._timer_callback()
    assert finished == [True]
