import json

from inno_camera_tools.person_detector import (
    DetectionBox,
    encode_detection_message,
)


def test_detection_payload_preserves_person_box_and_image_geometry():
    payload = encode_detection_message(
        1280,
        720,
        [DetectionBox(700.0, 100.0, 900.0, 650.0, 0.91, 0)],
    )

    decoded = json.loads(payload)
    assert decoded['image_width'] == 1280
    assert decoded['image_height'] == 720
    assert decoded['detections'][0]['x_min'] == 700.0
    assert decoded['detections'][0]['confidence'] == 0.91


def test_detection_payload_drops_non_finite_box():
    payload = encode_detection_message(
        640,
        480,
        [DetectionBox(float('nan'), 0.0, 10.0, 20.0, 0.9, 0)],
    )

    assert json.loads(payload)['detections'] == []
