"""Thin Mode 7 mission starter; planning and exit selection stay in existing nodes."""

from __future__ import annotations

from rclpy.node import Node
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Int32, String
from std_srvs.srv import Trigger

from .tf_utils import TfHelper
from .mode7_mission import Mode7Readiness


class Mode7MissionCoordinator(Node):
    """Gate one existing evacuation-manager request on Mode 7 readiness."""

    def __init__(self) -> None:
        super().__init__("mode7_mission_coordinator")
        defaults = {
            "enabled": True,
            "auto_start": False,
            "update_rate_hz": 5.0,
            "retry_period_sec": 1.0,
            "map_frame": "map",
            "base_frame": "base_link",
            "start_service": "/mode7/start",
            "stop_service": "/mode7/stop",
            "plan_service": "/plan_evacuation",
            "status_topic": "/mode7/status",
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        self.enabled = bool(value("enabled"))
        self.auto_start = bool(value("auto_start"))
        self.retry_period = float(value("retry_period_sec"))
        rate = float(value("update_rate_hz"))
        if rate <= 0.0 or self.retry_period <= 0.0:
            raise ValueError("Mode 7 rate and retry period must be positive")
        self.map_frame = str(value("map_frame"))
        self.base_frame = str(value("base_frame"))

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, str(value("status_topic")), transient
        )
        self.mode_publisher = self.create_publisher(Int32, "/drive_mode", 10)
        self.cancel_publisher = self.create_publisher(Empty, "/autonomy_cancel", 10)
        self.create_subscription(Bool, "/localization_ready", self._on_localization, transient)
        self.create_subscription(
            String, "/hazard/status",
            lambda message: self._remember("thermal_status", message.data), transient,
        )
        self.create_subscription(
            String, "/exit_evaluator/status",
            lambda message: self._remember("exit_evaluator_status", message.data), transient,
        )
        self.create_subscription(
            String, "/evacuation/status",
            lambda message: self._remember("evacuation_manager_status", message.data), transient,
        )
        self.create_subscription(Int32, "/drive_mode", self._on_drive_mode, 10)
        self.create_subscription(Empty, "/autonomy_cancel", self._on_cancel, 10)
        self.create_subscription(String, "/follower_state", self._on_follower_state, 10)
        self.create_service(Trigger, str(value("start_service")), self._start)
        self.create_service(Trigger, str(value("stop_service")), self._stop)
        self.plan_client = self.create_client(Trigger, str(value("plan_service")))
        self.tf = TfHelper(self)
        self.readiness = Mode7Readiness()
        self.requested = self.enabled and self.auto_start
        self.plan_future = None
        self.next_retry_ns = 0
        self.route_active = False
        self._ignore_mode5 = 0
        self._ignore_cancel = 0
        self._status_value = ""
        self.create_timer(1.0 / rate, self._tick)
        self._set_status(
            "WAITING_FOR_READINESS" if self.requested else "STOPPED:PRESS_5_OR_CALL_START"
        )

    def _remember(self, attribute: str, value: str) -> None:
        setattr(self.readiness, attribute, str(value))

    def _on_localization(self, message: Bool) -> None:
        self.readiness.localization_ready = bool(message.data)
        if not message.data and self.route_active:
            self._cancel_mission("STOPPED:LOCALIZATION_LOST")

    def _set_status(self, value: str) -> None:
        if value == self._status_value:
            return
        self._status_value = value
        self.status_publisher.publish(String(data=value))
        self.get_logger().info(f"MODE 7 thermal mission: {value}")

    def _request_start(self) -> tuple[bool, str]:
        if not self.enabled:
            return False, "DISABLED"
        if self.requested:
            return False, "ALREADY_REQUESTED"
        self.requested = True
        self.route_active = False
        self.next_retry_ns = 0
        self._set_status("WAITING_FOR_READINESS")
        return True, "MODE_7_THERMAL_MISSION_REQUESTED"

    def _start(self, _request, response):
        response.success, response.message = self._request_start()
        return response

    def _stop(self, _request, response):
        self._cancel_mission("STOPPED:STOP_SERVICE")
        response.success = True
        response.message = "MODE_7_STOPPED"
        return response

    def _on_drive_mode(self, message: Int32) -> None:
        mode = int(message.data)
        if mode == 5 and self._ignore_mode5:
            self._ignore_mode5 -= 1
            return
        if mode == 5 and not self.requested:
            self._request_start()
        elif mode != 5 and self.requested:
            self._cancel_mission("STOPPED:MODE_CHANGED")

    def _on_cancel(self, _message: Empty) -> None:
        if self._ignore_cancel:
            self._ignore_cancel -= 1
            return
        if self.requested:
            self._cancel_mission("STOPPED:CANCELLED", publish_cancel=False)

    def _on_follower_state(self, message: String) -> None:
        if self.route_active and message.data == "GOAL_REACHED":
            self.requested = False
            self.route_active = False
            self._cancel_pending()
            self._ignore_cancel += 1
            self.cancel_publisher.publish(Empty())
            self.mode_publisher.publish(Int32(data=1))
            self._set_status("COMPLETED:SELECTED_EXIT_REACHED")

    def _cancel_pending(self) -> None:
        future = self.plan_future
        self.plan_future = None
        if future is not None and not future.done():
            future.cancel()

    def _cancel_mission(self, state: str, *, publish_cancel: bool = True) -> None:
        self.requested = False
        self.route_active = False
        self._cancel_pending()
        if publish_cancel:
            self._ignore_cancel += 1
            self.cancel_publisher.publish(Empty())
        self._set_status(state)

    def _tick(self) -> None:
        if not self.requested or self.plan_future is not None:
            return
        self.readiness.pose_ready = (
            self.tf.lookup_pose_2d(self.map_frame, self.base_frame) is not None
        )
        self.readiness.plan_service_ready = self.plan_client.wait_for_service(timeout_sec=0.0)
        state = self.readiness.waiting_state()
        if state != "READY_TO_PLAN":
            self._set_status(state)
            return
        now = self.get_clock().now().nanoseconds
        if now < self.next_retry_ns:
            return
        self._ignore_mode5 += 1
        self.mode_publisher.publish(Int32(data=5))
        self._set_status("PLANNING:CALLING_PLAN_EVACUATION")
        self.plan_future = self.plan_client.call_async(Trigger.Request())
        self.plan_future.add_done_callback(self._on_plan_response)

    def _on_plan_response(self, future) -> None:
        if future is not self.plan_future:
            return
        self.plan_future = None
        if not self.requested:
            return
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Mode 7 /plan_evacuation failed: {exc}")
            response = None
        if response is not None and response.success:
            self.route_active = True
            self._set_status("NAVIGATING:SELECTED_EXIT")
            return
        self.next_retry_ns = (
            self.get_clock().now().nanoseconds + int(self.retry_period * 1e9)
        )
        reason = "NO_RESPONSE" if response is None else str(response.message)
        self._set_status(f"WAITING_TO_RETRY_PLAN:{reason}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Mode7MissionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
