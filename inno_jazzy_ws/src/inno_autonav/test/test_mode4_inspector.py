import math

from std_msgs.msg import Int32, String

from inno_autonav.mode4_inspector import (
    CameraIntrinsics,
    PersonDetection,
    Mode4Inspector,
    associate_detections_to_candidates,
    fallback_intrinsics,
    parse_detection_message,
    project_candidate_u,
    scale_intrinsics,
)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_right_side_person_box_matches_only_right_lidar_candidate():
    intrinsics = CameraIntrinsics(width=1000, fx=500.0, cx=500.0)
    robot_pose = (0.0, 0.0, 0.0)
    candidates = [(1.5, 0.25), (1.5, -0.25)]
    right_projection = project_candidate_u(
        robot_pose, candidates[1], intrinsics
    )
    assert right_projection is not None
    right_u = right_projection[0]
    detections = [
        PersonDetection(right_u - 60.0, 50.0, right_u + 60.0, 700.0, 0.95)
    ]

    associations = associate_detections_to_candidates(
        robot_pose=robot_pose,
        inspection_target=candidates[0],
        candidates=candidates,
        detections=detections,
        intrinsics=intrinsics,
        camera_yaw_offset_rad=0.0,
        target_search_radius_m=1.0,
        maximum_candidate_distance_m=3.0,
        maximum_bearing_error_rad=math.radians(10.0),
    )

    assert len(associations) == 1
    assert associations[0].candidate_index == 1
    assert associations[0].candidate == candidates[1]


def test_one_detection_cannot_match_two_close_lidar_candidates():
    intrinsics = CameraIntrinsics(width=1000, fx=500.0, cx=500.0)
    detection = PersonDetection(480.0, 50.0, 520.0, 700.0, 0.9)

    associations = associate_detections_to_candidates(
        robot_pose=(0.0, 0.0, 0.0),
        inspection_target=(1.5, 0.0),
        candidates=[(1.5, 0.02), (1.5, -0.02)],
        detections=[detection],
        intrinsics=intrinsics,
        camera_yaw_offset_rad=0.0,
        target_search_radius_m=1.0,
        maximum_candidate_distance_m=3.0,
        maximum_bearing_error_rad=math.radians(10.0),
    )

    assert len(associations) == 1


def test_left_and_right_projection_follow_ros_camera_convention():
    intrinsics = CameraIntrinsics(width=1000, fx=500.0, cx=500.0)
    left = project_candidate_u((0.0, 0.0, 0.0), (2.0, 0.5), intrinsics)
    right = project_candidate_u((0.0, 0.0, 0.0), (2.0, -0.5), intrinsics)

    assert left is not None and left[0] < intrinsics.cx
    assert right is not None and right[0] > intrinsics.cx


def test_camera_info_intrinsics_scale_to_detector_image_width():
    scaled = scale_intrinsics(
        CameraIntrinsics(width=1280, fx=800.0, cx=640.0), 640
    )

    assert scaled == CameraIntrinsics(width=640, fx=400.0, cx=320.0)


def test_fov_fallback_places_center_at_half_image_width():
    intrinsics = fallback_intrinsics(1280, math.radians(80.0))

    assert intrinsics.fx > 0.0
    assert intrinsics.cx == 640.0


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
