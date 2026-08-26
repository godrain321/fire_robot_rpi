"""Run a YOLO person model on Camera Module 3 images."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time
from typing import Iterable, List, Sequence, Set

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String


@dataclass(frozen=True)
class DetectionBox:
    """One person bounding box in source-image pixel coordinates."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float
    confidence: float
    class_id: int


@dataclass(frozen=True)
class LetterboxGeometry:
    """Mapping between a source frame and square YOLO input."""

    source_width: int
    source_height: int
    scale: float
    pad_x: int
    pad_y: int


def prepare_yolo_input(frame, image_size: int):
    """Letterbox a BGR frame and return NCHW RGB float input."""
    if image_size <= 0 or frame is None or frame.ndim != 3:
        raise ValueError('YOLO input frame is invalid')
    source_height, source_width = frame.shape[:2]
    if source_width <= 0 or source_height <= 0:
        raise ValueError('YOLO input frame is empty')
    scale = min(
        float(image_size) / source_width,
        float(image_size) / source_height,
    )
    resized_width = max(1, round(source_width * scale))
    resized_height = max(1, round(source_height * scale))
    resized = cv2.resize(
        frame, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR
    )
    pad_x = (image_size - resized_width) // 2
    pad_y = (image_size - resized_height) // 2
    canvas = np.full((image_size, image_size, 3), 114, dtype=np.uint8)
    canvas[
        pad_y:pad_y + resized_height,
        pad_x:pad_x + resized_width,
    ] = resized
    blob = cv2.dnn.blobFromImage(
        canvas,
        scalefactor=1.0 / 255.0,
        size=(image_size, image_size),
        swapRB=True,
        crop=False,
    )
    geometry = LetterboxGeometry(
        source_width=source_width,
        source_height=source_height,
        scale=scale,
        pad_x=pad_x,
        pad_y=pad_y,
    )
    return blob, geometry


def decode_yolov8_output(
    raw_output,
    geometry: LetterboxGeometry,
    confidence_threshold: float,
    person_class_ids: Set[int],
    nms_iou_threshold: float = 0.45,
) -> List[DetectionBox]:
    """Decode YOLOv8 ``[batch, 4+classes, anchors]`` output with NMS."""
    output = np.asarray(raw_output, dtype=np.float32)
    if output.ndim == 3:
        if output.shape[0] != 1:
            raise ValueError('YOLO output batch must be one')
        output = output[0]
    if output.ndim != 2:
        raise ValueError('YOLO output must have two or three dimensions')
    if output.shape[1] < 5 or output.shape[0] < output.shape[1]:
        output = output.T
    if output.shape[1] < 5:
        raise ValueError('YOLO output does not contain class scores')

    boxes_xywh = []
    scores = []
    class_ids = []
    for prediction in output:
        class_scores = prediction[4:]
        class_id = int(np.argmax(class_scores))
        confidence = float(class_scores[class_id])
        if (
            class_id not in person_class_ids
            or confidence < confidence_threshold
        ):
            continue
        center_x, center_y, width, height = (
            float(value) for value in prediction[:4]
        )
        left = center_x - 0.5 * width
        top = center_y - 0.5 * height
        boxes_xywh.append([left, top, width, height])
        scores.append(confidence)
        class_ids.append(class_id)

    if not boxes_xywh:
        return []
    selected = cv2.dnn.NMSBoxes(
        boxes_xywh,
        scores,
        confidence_threshold,
        nms_iou_threshold,
    )
    detections = []
    for raw_index in selected:
        index = int(np.asarray(raw_index).reshape(-1)[0])
        left, top, width, height = boxes_xywh[index]
        x_min = (left - geometry.pad_x) / geometry.scale
        y_min = (top - geometry.pad_y) / geometry.scale
        x_max = (left + width - geometry.pad_x) / geometry.scale
        y_max = (top + height - geometry.pad_y) / geometry.scale
        x_min = max(0.0, min(float(geometry.source_width), x_min))
        y_min = max(0.0, min(float(geometry.source_height), y_min))
        x_max = max(0.0, min(float(geometry.source_width), x_max))
        y_max = max(0.0, min(float(geometry.source_height), y_max))
        if x_max <= x_min or y_max <= y_min:
            continue
        detections.append(
            DetectionBox(
                x_min=x_min,
                y_min=y_min,
                x_max=x_max,
                y_max=y_max,
                confidence=scores[index],
                class_id=class_ids[index],
            )
        )
    return detections


class OnnxYoloBackend:
    """Small CPU ONNX Runtime backend for exported YOLOv8 detect models."""

    def __init__(self, path: Path, image_size: int) -> None:
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise RuntimeError('ONNXRUNTIME_NOT_INSTALLED') from error
        self.session = ort.InferenceSession(
            str(path), providers=['CPUExecutionProvider']
        )
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [item.name for item in self.session.get_outputs()]
        self.image_size = image_size

    def predict(
        self,
        frame,
        confidence_threshold: float,
        person_class_ids: Set[int],
    ) -> List[DetectionBox]:
        blob, geometry = prepare_yolo_input(frame, self.image_size)
        outputs: Sequence[np.ndarray] = self.session.run(
            self.output_names, {self.input_name: blob}
        )
        if not outputs:
            return []
        return decode_yolov8_output(
            outputs[0],
            geometry,
            confidence_threshold,
            person_class_ids,
        )


class OpenCvYoloBackend:
    """Dependency-free CPU fallback for static-shape YOLOv8 ONNX models."""

    def __init__(self, path: Path, image_size: int) -> None:
        self.net = cv2.dnn.readNetFromONNX(str(path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.image_size = image_size

    def predict(
        self,
        frame,
        confidence_threshold: float,
        person_class_ids: Set[int],
    ) -> List[DetectionBox]:
        blob, geometry = prepare_yolo_input(frame, self.image_size)
        self.net.setInput(blob)
        output = self.net.forward()
        return decode_yolov8_output(
            output,
            geometry,
            confidence_threshold,
            person_class_ids,
        )


def encode_detection_message(
    image_width: int,
    image_height: int,
    detections: Iterable[DetectionBox],
) -> str:
    """Serialize detections without requiring an extra custom ROS message."""
    if image_width <= 0 or image_height <= 0:
        raise ValueError('image dimensions must be positive')
    valid = []
    for detection in detections:
        values = (
            detection.x_min,
            detection.y_min,
            detection.x_max,
            detection.y_max,
            detection.confidence,
        )
        if not all(math.isfinite(float(value)) for value in values):
            continue
        valid.append(asdict(detection))
    return json.dumps(
        {
            'image_width': int(image_width),
            'image_height': int(image_height),
            'detections': valid,
        },
        separators=(',', ':'),
        sort_keys=True,
    )


class PersonDetector(Node):
    """Publish person boxes from a trained YOLO model, while failing safely."""

    def __init__(self) -> None:
        super().__init__('camera_person_detector')
        defaults = {
            'image_topic': '/camera/image_raw',
            'detection_topic': '/camera/person_detections',
            'annotated_image_topic': '/camera/person_detection_image',
            'model_path': '',
            'confidence_threshold': 0.50,
            'person_class_ids': [0],
            'inference_rate_hz': 3.0,
            'inference_image_size': 640,
            'device': 'cpu',
            'only_during_mode4_observation': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        detection_topic = str(self.get_parameter('detection_topic').value)
        annotated_topic = str(
            self.get_parameter('annotated_image_topic').value
        )
        model_path_text = str(
            self.get_parameter('model_path').value
        ).strip()
        self.model_path = (
            Path(model_path_text).expanduser() if model_path_text else None
        )
        self.confidence = float(
            self.get_parameter('confidence_threshold').value
        )
        self.person_class_ids = {
            int(value)
            for value in self.get_parameter('person_class_ids').value
        }
        inference_rate = float(
            self.get_parameter('inference_rate_hz').value
        )
        self.image_size = int(
            self.get_parameter('inference_image_size').value
        )
        self.device = str(self.get_parameter('device').value).strip()
        self.only_during_mode4 = bool(
            self.get_parameter('only_during_mode4_observation').value
        )
        if (
            not 0.0 < self.confidence <= 1.0
            or not self.person_class_ids
            or inference_rate <= 0.0
            or self.image_size <= 0
        ):
            raise ValueError('YOLO detector parameters are invalid')

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.bridge = CvBridge()
        self.model = None
        self.inference_enabled = not self.only_during_mode4
        self.last_inference = float('-inf')
        self.inference_period = 1.0 / inference_rate
        self.last_status = ''
        self.detection_publisher = self.create_publisher(
            String, detection_topic, 10
        )
        self.annotated_publisher = self.create_publisher(
            Image, annotated_topic, 2
        )
        self.status_publisher = self.create_publisher(
            String, '/camera/person_detector_status', status_qos
        )
        self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, '/mode4_status', self._mode4_status_callback, status_qos
        )
        self._load_model()

    def _mode4_status_callback(self, message: String) -> None:
        if not self.only_during_mode4:
            return
        enabled = message.data.strip().upper().startswith(
            'MODE4_CAMERA_YOLO_OBSERVING'
        )
        if enabled == self.inference_enabled:
            return
        self.inference_enabled = enabled
        if not enabled and self.model is not None:
            self._set_status('READY_WAITING_FOR_MODE4')

    def _set_status(self, state: str) -> None:
        if state == self.last_status:
            return
        self.last_status = state
        self.status_publisher.publish(String(data=state))
        if state == 'ONLINE':
            self.get_logger().info(state)
        else:
            self.get_logger().warning(state)

    def _load_model(self) -> None:
        if self.model_path is None:
            self._set_status('MODEL_PATH_EMPTY')
            return
        if not self.model_path.is_file():
            self._set_status(f'MODEL_NOT_FOUND:{self.model_path}')
            return
        if self.model_path.suffix.lower() == '.onnx':
            try:
                self.model = OnnxYoloBackend(
                    self.model_path, self.image_size
                )
            except Exception as onnx_error:
                self.get_logger().warning(
                    'ONNX Runtime backend unavailable; trying OpenCV CPU: '
                    f'{onnx_error}'
                )
                try:
                    self.model = OpenCvYoloBackend(
                        self.model_path, self.image_size
                    )
                except Exception as opencv_error:
                    self._set_status(
                        f'MODEL_LOAD_ERROR:{type(opencv_error).__name__}'
                    )
                    self.get_logger().error(str(opencv_error))
                    return
            self._model_ready()
            return
        try:
            from ultralytics import YOLO
        except ImportError:
            self._set_status('ULTRALYTICS_NOT_INSTALLED')
            return
        try:
            self.model = YOLO(str(self.model_path))
        except Exception as error:  # model backends raise several error types
            self._set_status(f'MODEL_LOAD_ERROR:{type(error).__name__}')
            self.get_logger().error(str(error))
            return
        self._model_ready()

    def _model_ready(self) -> None:
        ready_state = (
            'READY_WAITING_FOR_MODE4'
            if self.only_during_mode4
            else 'READY_WAITING_FOR_IMAGE'
        )
        self._set_status(ready_state)
        self.get_logger().info(f'YOLO model loaded: {self.model_path}')

    def _boxes_from_result(self, result) -> List[DetectionBox]:
        boxes = getattr(result, 'boxes', None)
        if boxes is None:
            return []
        coordinates = boxes.xyxy.cpu().tolist()
        confidences = boxes.conf.cpu().tolist()
        class_ids = boxes.cls.cpu().tolist()
        detections = []
        for coordinates_xyxy, confidence, class_id in zip(
            coordinates, confidences, class_ids
        ):
            class_id = int(class_id)
            if class_id not in self.person_class_ids:
                continue
            detections.append(
                DetectionBox(
                    x_min=float(coordinates_xyxy[0]),
                    y_min=float(coordinates_xyxy[1]),
                    x_max=float(coordinates_xyxy[2]),
                    y_max=float(coordinates_xyxy[3]),
                    confidence=float(confidence),
                    class_id=class_id,
                )
            )
        return detections

    def _publish_annotated(self, source, header, detections) -> None:
        if self.annotated_publisher.get_subscription_count() == 0:
            return
        annotated = source.copy()
        for detection in detections:
            top_left = (int(detection.x_min), int(detection.y_min))
            bottom_right = (int(detection.x_max), int(detection.y_max))
            cv2.rectangle(annotated, top_left, bottom_right, (255, 80, 0), 2)
            cv2.putText(
                annotated,
                f'person {detection.confidence:.2f}',
                (top_left[0], max(20, top_left[1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 80, 0),
                2,
                cv2.LINE_AA,
            )
        message = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        message.header = header
        self.annotated_publisher.publish(message)

    def _image_callback(self, message: Image) -> None:
        now = time.monotonic()
        if now - self.last_inference < self.inference_period:
            return
        self.last_inference = now
        if self.model is None or not self.inference_enabled:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except (CvBridgeError, ValueError) as error:
            self._set_status('IMAGE_CONVERSION_ERROR')
            self.get_logger().error(str(error))
            return
        try:
            if isinstance(
                self.model, (OnnxYoloBackend, OpenCvYoloBackend)
            ):
                detections = self.model.predict(
                    frame, self.confidence, self.person_class_ids
                )
            else:
                arguments = {
                    'source': frame,
                    'conf': self.confidence,
                    'classes': sorted(self.person_class_ids),
                    'imgsz': self.image_size,
                    'verbose': False,
                }
                if self.device and self.device.lower() != 'auto':
                    arguments['device'] = self.device
                results = self.model.predict(**arguments)
                detections = (
                    self._boxes_from_result(results[0]) if results else []
                )
        except Exception as error:  # inference backend errors vary by export
            self._set_status(f'INFERENCE_ERROR:{type(error).__name__}')
            self.get_logger().error(str(error))
            return
        height, width = frame.shape[:2]
        payload = encode_detection_message(width, height, detections)
        self.detection_publisher.publish(String(data=payload))
        self._publish_annotated(frame, message.header, detections)
        self._set_status('ONLINE')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PersonDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as error:
        if node is None:
            print(f'camera_person_detector: {error}')
        else:
            node.get_logger().error(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
