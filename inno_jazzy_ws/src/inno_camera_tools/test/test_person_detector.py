import json

import numpy as np

from inno_camera_tools.person_detector import (
    DetectionBox,
    LetterboxGeometry,
    OpenCvYoloBackend,
    decode_yolov8_output,
    encode_detection_message,
    prepare_yolo_input,
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


def test_letterbox_geometry_for_wide_camera_frame():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    blob, geometry = prepare_yolo_input(frame, 640)

    assert blob.shape == (1, 3, 640, 640)
    assert geometry == LetterboxGeometry(1280, 720, 0.5, 0, 140)


def test_decode_single_class_yolov8_person_output():
    # Two overlapping person boxes: NMS must keep only the stronger one.
    output = np.asarray(
        [
            [320.0, 322.0],
            [320.0, 322.0],
            [100.0, 100.0],
            [200.0, 200.0],
            [0.90, 0.70],
        ],
        dtype=np.float32,
    )[None]
    geometry = LetterboxGeometry(640, 640, 1.0, 0, 0)

    detections = decode_yolov8_output(output, geometry, 0.5, {0})

    assert len(detections) == 1
    assert detections[0].class_id == 0
    assert detections[0].confidence > 0.89
    assert detections[0].x_min == 270.0


def test_decode_static_export_raw_dfl_person_output():
    output = np.full((1, 65, 8400), -20.0, dtype=np.float32)
    # Anchor 3240 is centred near the middle of the 80x80 stride-8 level.
    anchor = 3240
    for side in range(4):
        output[0, side * 16 + 10, anchor] = 20.0
    output[0, 64, anchor] = 5.0
    geometry = LetterboxGeometry(640, 640, 1.0, 0, 0)

    detections = decode_yolov8_output(output, geometry, 0.5, {0})

    assert len(detections) == 1
    assert detections[0].class_id == 0
    assert detections[0].confidence > 0.99
    assert detections[0].x_min < 320.0 < detections[0].x_max
    assert detections[0].y_min < 320.0 < detections[0].y_max


def test_opencv_backend_runs_static_onnx_model(monkeypatch, tmp_path):
    class FakeNet:
        def setPreferableBackend(self, _backend):
            pass

        def setPreferableTarget(self, _target):
            pass

        def setInput(self, blob):
            assert blob.shape == (1, 3, 640, 640)

        def forward(self):
            return np.asarray(
                [[[320.0], [320.0], [100.0], [200.0], [0.9]]],
                dtype=np.float32,
            )

    monkeypatch.setattr(
        'inno_camera_tools.person_detector.cv2.dnn.readNetFromONNX',
        lambda _path: FakeNet(),
    )
    backend = OpenCvYoloBackend(tmp_path / 'model.onnx', 640)

    detections = backend.predict(
        np.zeros((640, 640, 3), dtype=np.uint8), 0.5, {0}
    )

    assert len(detections) == 1
    assert detections[0].confidence > 0.89
