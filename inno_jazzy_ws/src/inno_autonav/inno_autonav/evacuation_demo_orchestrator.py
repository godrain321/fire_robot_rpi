"""Mode 5 physical exit exploration, inspection, and survivor escort."""

from __future__ import annotations

from collections import Counter
import json
import math
import time

from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Int32, String
from std_srvs.srv import Trigger

from .evacuation_demo import (
    MovingCandidateTracker,
    StationaryCandidateTracker,
    build_next_exploration_decision,
    nearest_exit_obstacle_candidate,
    nearest_uninspected_candidate,
    moving_priority_candidate,
    parse_activation_response,
    parse_mode3_classification,
    parse_mode4_classification,
    startup_state,
)
from .tf_utils import TfHelper


def exit_navigation_log(exit_id: str, blocked_exit_ids) -> str:
    """Build the operator log for the initial or replacement exit route."""
    selected = str(exit_id)
    blocked = sorted({str(item) for item in blocked_exit_ids if str(item)})
    if blocked:
        return (
            f"[출구 변경] 막힌 출구 목록({', '.join(blocked)})을 제외하고 "
            f"다음 출구 {selected}로 이동합니다."
        )
    return f"[출구 선택] 가장 가까운 출구 {selected}로 이동합니다."


def mode3_inspection_progress_log(state: str) -> str | None:
    """Translate Mode 3 arrival/measurement states for the Mode 5 console."""
    value = str(state).strip().upper()
    if value == "MODE3_AT_STANDOFF:ROBOT_SETTLING":
        return (
            "[도착] 최신 LiDAR 기준 후보가 2.5m 이내입니다. "
            "완전히 정지하는 중입니다."
        )
    if value.startswith("MODE3_TARGET_MOVED:REPLANNING"):
        return (
            "[재접근] 후보가 2.5m 밖으로 이동하여 최신 LiDAR 위치로 "
            "다시 접근합니다."
        )
    if value == "MODE3_WAITING_FOR_LIVE_TARGET":
        return (
            "[추적 대기] 기존 검사 지점에서 정지했습니다. 최신 LiDAR "
            "후보가 다시 확인되면 실제 거리로 검사를 계속합니다."
        )
    if value == "MODE3_MMWAVE_OBSERVING":
        return "[생체 판별] 로봇이 정지했습니다. mmWave 생체신호 감지를 시작합니다."
    return None


class EvacuationDemoOrchestrator(Node):
    """Visit exits, inspect blockers and moving people, then lead evacuation."""

    ACTIVE_FOLLOWER_STATES = frozenset({
        "PATH_ACCEPTED",
        "FOLLOWING_PATH",
        "ROTATING_IN_PLACE",
        "ALIGNING_GOAL_YAW",
    })
    MOTION_FOLLOWER_STATES = frozenset({
        "FOLLOWING_PATH",
        "ROTATING_IN_PLACE",
        "ALIGNING_GOAL_YAW",
    })

    def __init__(self) -> None:
        super().__init__("evacuation_demo_orchestrator")
        defaults = {
            "enabled": True,
            "auto_start": False,
            "drive_mode": 5,
            "map_frame": "map",
            "base_frame": "base_link",
            "update_rate_hz": 5.0,
            "retry_period_sec": 1.0,
            "exit_obstacle_match_radius_m": 1.5,
            "classification_match_radius_m": 0.75,
            "candidate_suppression_radius_m": 1.0,
            "inspection_after_motion_delay_sec": 2.0,
            "moving_survivor_enabled": False,
            "moving_priority_enabled": True,
            "moving_priority_wait_sec": 2.0,
            "moving_priority_target_timeout_sec": 2.0,
            "moving_association_radius_m": 0.75,
            "moving_minimum_displacement_m": 0.20,
            "moving_minimum_observations": 3,
            "moving_window_sec": 2.0,
            "moving_stale_timeout_sec": 1.0,
            "stationary_combined_inspection_enabled": False,
            "stationary_association_radius_m": 0.75,
            "stationary_maximum_displacement_m": 0.15,
            "stationary_minimum_observations": 5,
            "stationary_confirmation_duration_sec": 2.0,
            "stationary_stale_timeout_sec": 1.0,
            "stationary_target_timeout_sec": 1.0,
            "survivor_ready_wait_sec": 2.0,
            "survivor_track_match_radius_m": 1.25,
            "survivor_track_stale_sec": 2.0,
            "survivor_follow_stop_distance_m": 2.50,
            "survivor_follow_resume_distance_m": 2.00,
            "survivor_exit_arrival_distance_m": 2.50,
            "evaluate_service": "/evaluate_exits",
            "plan_service": "/plan_evacuation",
            "start_service": "/evacuation_demo/start",
            "stop_service": "/evacuation_demo/stop",
            "status_topic": "/evacuation_demo/status",
            "log_topic": "/evacuation_demo/log",
            "hazard_status_topic": "/hazard/status",
            "initial_hazard_bypass_topic": "/hazard/initial_route_bypass",
            "initial_route_ignore_thermal": False,
            "exit_evaluator_status_topic": "/exit_evaluator/status",
            "evacuation_manager_status_topic": "/evacuation/status",
            "drive_mode_topic": "/drive_mode",
            "drive_mode_status_topic": "/drive_mode_status",
            "autonomy_cancel_topic": "/autonomy_cancel",
            "evacuation_plan_topic": "/evacuation/plan",
            "selected_exit_topic": "/evacuation/selected_exit",
            "blocked_exits_topic": "/evacuation/blocked_exits",
            "danger_expected_exits_topic": (
                "/evacuation/danger_expected_exits"
            ),
            "goal_topic": "/goal_pose",
            "follower_state_topic": "/follower_state",
            "planner_state_topic": "/planner_state",
            "obstacle_candidates_topic": "/dynamic_obstacle_candidates",
            "all_obstacle_candidates_topic": "/dynamic_obstacle_all_candidates",
            "motion_obstacle_candidates_topic": "/dynamic_obstacle_motion_candidates",
            "inspection_command_topic": "/obstacle_inspection_command",
            "mode3_status_topic": "/mode3_status",
            "mode3_classification_topic": "/mode3_classification",
            "mode4_status_topic": "/mode4_status",
            "mode4_classification_topic": "/mode4_classification",
            "waypoint_route_status_topic": "/waypoint_planner/route_status",
            "replanning_status_topic": "/replanning/status",
            "survivor_follow_hold_topic": "/survivor_follow_hold",
            "survivor_track_topic": "/dynamic_obstacle_person_track",
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        def value(name):
            return self.get_parameter(name).value
        self.enabled = bool(value("enabled"))
        self.auto_start = bool(value("auto_start"))
        self.drive_mode = int(value("drive_mode"))
        self.map_frame = str(value("map_frame"))
        self.base_frame = str(value("base_frame"))
        rate = float(value("update_rate_hz"))
        self.retry_period = float(value("retry_period_sec"))
        self.exit_obstacle_radius = float(value("exit_obstacle_match_radius_m"))
        self.classification_radius = float(value("classification_match_radius_m"))
        self.candidate_suppression_radius = float(
            value("candidate_suppression_radius_m")
        )
        self.inspection_after_motion_delay = float(
            value("inspection_after_motion_delay_sec")
        )
        self.moving_survivor_enabled = bool(value("moving_survivor_enabled"))
        self.moving_priority_enabled = bool(value("moving_priority_enabled"))
        self.moving_priority_wait = float(value("moving_priority_wait_sec"))
        self.moving_priority_target_timeout = float(
            value("moving_priority_target_timeout_sec")
        )
        self.stationary_combined_inspection_enabled = bool(
            value("stationary_combined_inspection_enabled")
        )
        self.stationary_target_timeout = float(
            value("stationary_target_timeout_sec")
        )
        self.initial_route_ignore_thermal = bool(
            value("initial_route_ignore_thermal")
        )
        self.survivor_ready_wait = float(value("survivor_ready_wait_sec"))
        self.survivor_track_radius = float(value("survivor_track_match_radius_m"))
        self.survivor_track_stale = float(value("survivor_track_stale_sec"))
        self.follow_stop_distance = float(value("survivor_follow_stop_distance_m"))
        self.follow_resume_distance = float(value("survivor_follow_resume_distance_m"))
        self.exit_arrival_distance = float(value("survivor_exit_arrival_distance_m"))
        if self.drive_mode != 5:
            raise ValueError("evacuation demo drive_mode must be 5")
        positive = (
            rate,
            self.retry_period,
            self.exit_obstacle_radius,
            self.classification_radius,
            self.candidate_suppression_radius,
            self.inspection_after_motion_delay,
            self.moving_priority_wait,
            self.moving_priority_target_timeout,
            self.stationary_target_timeout,
            self.survivor_ready_wait,
            self.survivor_track_radius,
            self.survivor_track_stale,
            self.follow_stop_distance,
            self.follow_resume_distance,
            self.exit_arrival_distance,
        )
        if any(not math.isfinite(item) or item <= 0.0 for item in positive):
            raise ValueError("Mode 5 rates and distances must be positive")
        if self.follow_resume_distance >= self.follow_stop_distance:
            raise ValueError("survivor follow resume distance must be below stop distance")
        self.moving_tracker = MovingCandidateTracker(
            association_radius_m=float(value("moving_association_radius_m")),
            minimum_displacement_m=float(value("moving_minimum_displacement_m")),
            minimum_observations=int(value("moving_minimum_observations")),
            window_sec=float(value("moving_window_sec")),
            stale_timeout_sec=float(value("moving_stale_timeout_sec")),
        )
        self.moving_priority_tracker = MovingCandidateTracker(
            association_radius_m=float(value("moving_association_radius_m")),
            minimum_displacement_m=float(value("moving_minimum_displacement_m")),
            minimum_observations=int(value("moving_minimum_observations")),
            window_sec=float(value("moving_window_sec")),
            stale_timeout_sec=float(value("moving_stale_timeout_sec")),
        )
        self.stationary_tracker = StationaryCandidateTracker(
            association_radius_m=float(value("stationary_association_radius_m")),
            maximum_displacement_m=float(
                value("stationary_maximum_displacement_m")
            ),
            minimum_observations=int(value("stationary_minimum_observations")),
            confirmation_duration_sec=float(
                value("stationary_confirmation_duration_sec")
            ),
            stale_timeout_sec=float(value("stationary_stale_timeout_sec")),
        )
        self.moving_association_radius = float(value("moving_association_radius_m"))

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, str(value("status_topic")), transient
        )
        self.log_publisher = self.create_publisher(
            String, str(value("log_topic")), transient
        )
        self.mode_publisher = self.create_publisher(
            Int32, str(value("drive_mode_topic")), 10
        )
        self.cancel_publisher = self.create_publisher(
            Empty, str(value("autonomy_cancel_topic")), 10
        )
        self.plan_publisher = self.create_publisher(
            String, str(value("evacuation_plan_topic")), transient
        )
        self.selected_exit_publisher = self.create_publisher(
            String, str(value("selected_exit_topic")), transient
        )
        self.blocked_exits_publisher = self.create_publisher(
            String, str(value("blocked_exits_topic")), transient
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, str(value("goal_topic")), 10
        )
        self.inspection_publisher = self.create_publisher(
            String, str(value("inspection_command_topic")), 10
        )
        self.follow_hold_publisher = self.create_publisher(
            Bool, str(value("survivor_follow_hold_topic")), 10
        )
        self.survivor_track_publisher = self.create_publisher(
            PointStamped, str(value("survivor_track_topic")), 10
        )
        self.initial_hazard_bypass_publisher = self.create_publisher(
            Bool, str(value("initial_hazard_bypass_topic")), transient
        )

        self.create_subscription(
            String, str(value("hazard_status_topic")),
            lambda message: self._remember("hazard_status", message.data), transient,
        )
        self.create_subscription(
            String, str(value("exit_evaluator_status_topic")),
            lambda message: self._remember("exit_evaluator_status", message.data),
            transient,
        )
        self.create_subscription(
            String, str(value("evacuation_manager_status_topic")),
            lambda message: self._remember("evacuation_manager_status", message.data),
            transient,
        )
        self.create_subscription(
            String, str(value("drive_mode_status_topic")),
            lambda message: self._remember("drive_mode_status", message.data), 10,
        )
        self.create_subscription(
            Int32, str(value("drive_mode_topic")), self._on_mode_command, 10
        )
        self.create_subscription(
            PoseArray, str(value("obstacle_candidates_topic")),
            self._on_candidates, transient,
        )
        self.create_subscription(
            PoseArray, str(value("all_obstacle_candidates_topic")),
            self._on_all_candidates, transient,
        )
        self.create_subscription(
            PoseArray, str(value("motion_obstacle_candidates_topic")),
            self._on_motion_candidates, transient,
        )
        self.create_subscription(
            String, str(value("follower_state_topic")), self._on_follower_state, 10
        )
        self.create_subscription(
            String, str(value("planner_state_topic")), self._on_planner_state, 10
        )
        self.create_subscription(
            String, str(value("mode3_status_topic")), self._on_mode3_status, transient
        )
        self.create_subscription(
            String, str(value("mode3_classification_topic")),
            self._on_mode3_classification, transient,
        )
        self.create_subscription(
            String, str(value("mode4_status_topic")), self._on_mode4_status, transient
        )
        self.create_subscription(
            String, str(value("mode4_classification_topic")),
            self._on_mode4_classification, transient,
        )
        self.create_subscription(
            String, str(value("waypoint_route_status_topic")),
            self._on_waypoint_route_status, transient,
        )
        self.create_subscription(
            String, str(value("replanning_status_topic")),
            self._on_replanning_status, transient,
        )
        self.create_subscription(
            String, str(value("danger_expected_exits_topic")),
            self._on_danger_expected_exits, transient,
        )
        self.create_subscription(
            String, str(value("evacuation_plan_topic")),
            self._on_canonical_evacuation_plan, transient,
        )
        self.evaluation_client = self.create_client(
            Trigger, str(value("evaluate_service"))
        )
        self.plan_client = self.create_client(Trigger, str(value("plan_service")))
        self.create_service(Trigger, str(value("start_service")), self._start_service)
        self.create_service(Trigger, str(value("stop_service")), self._stop_service)

        self.tf = TfHelper(self)
        self.hazard_status = ""
        self.exit_evaluator_status = ""
        self.evacuation_manager_status = ""
        self.drive_mode_status = ""
        self.mode3_status = ""
        self.mode4_status = ""
        self.candidates = []
        self.all_candidates = []
        self.moving_priority_targets = []
        self.stationary_targets = []
        self._candidate_wait_started_at = None
        self.checked_exit_ids = set()
        self.blocked_exit_ids = set()
        self.danger_expected_exit_ids = set()
        self.current_exit_id = None
        self.current_exit_position = None
        self.current_approach_position = None
        self.current_plan_payload = None
        self.inspection_target = None
        self.inspection_start_position = None
        self.inspection_blocks_current_exit = False
        self.combined_inspection_active = False
        self.combined_mmwave_person = None
        self.inspected_dynamic_positions = []
        self.active_survivor_position = None
        self.active_survivor_seen_at = float("-inf")
        self.survivor_exit_id = None
        self.waiting_for_departure = False
        self._inspection_allowed_after = float("inf")
        self._status_value = ""
        self._requested = self.enabled and self.auto_start
        self._route_activated = False
        self._phase = "STARTING"
        self._resume_phase_after_inspection = "STARTING"
        self._expected_drive_mode = 5
        self._internal_mode_commands = Counter()
        self._inspection_command_sent = False
        self._future = None
        self._retry_after = 0.0
        self._survivor_ready_after = 0.0
        self._survivor_hold = False
        self._robot_at_exit = False
        self._last_replanning_signature = None
        self._initial_hazard_bypass_active = False
        self.create_timer(1.0 / rate, self._tick)
        if not self.enabled:
            self._set_status("DISABLED")
        elif self._requested:
            self._set_initial_hazard_bypass(
                self.initial_route_ignore_thermal
            )
            self._set_status("SEARCH_EXITS:STARTING")
            self._log("모드 5 자동 시작: 등록된 출구의 실제 탐색을 시작합니다.")
            self._publish_blocked_exits()
            self._publish_follow_hold(False)
            self._select_drive_mode(5)
        else:
            self._set_status("STOPPED:PRESS_5")
            self._log("[대기] 키보드에서 숫자 5를 누르면 자동 대피를 시작합니다.")

    def _remember(self, attribute: str, value: str) -> None:
        setattr(self, attribute, str(value))

    def _log(self, text: str) -> None:
        message = str(text)
        self.log_publisher.publish(String(data=message))
        self.get_logger().info(f"[모드 5] {message}")

    def _set_status(self, value: str) -> None:
        value = str(value)
        if value == self._status_value:
            return
        self._status_value = value
        self.status_publisher.publish(String(data=value))
        self.get_logger().info(f"MODE 5 state: {value}")

    def _publish_follow_hold(self, hold: bool) -> None:
        hold = bool(hold)
        if hold == self._survivor_hold and hasattr(self, "_hold_was_published"):
            return
        self._survivor_hold = hold
        self._hold_was_published = True
        self.follow_hold_publisher.publish(Bool(data=hold))

    def _set_initial_hazard_bypass(self, enabled: bool) -> None:
        enabled = bool(
            enabled and getattr(self, "initial_route_ignore_thermal", False)
        )
        if (
            enabled == getattr(self, "_initial_hazard_bypass_active", False)
            and hasattr(self, "_initial_hazard_bypass_was_published")
        ):
            return
        self._initial_hazard_bypass_active = enabled
        self._initial_hazard_bypass_was_published = True
        self.initial_hazard_bypass_publisher.publish(Bool(data=enabled))

    def _select_drive_mode(self, mode: int) -> None:
        self._expected_drive_mode = int(mode)
        self._internal_mode_commands[int(mode)] += 1
        self.mode_publisher.publish(Int32(data=int(mode)))

    def _drive_mode_confirmed(self, mode: int) -> bool:
        return self.drive_mode_status.startswith(f"{int(mode)}:")

    def _on_mode_command(self, message: Int32) -> None:
        mode = int(message.data)
        if self._internal_mode_commands[mode] > 0:
            self._internal_mode_commands[mode] -= 1
            return
        if mode == 5 and not self._requested:
            started, reason = self._request_start(
                "숫자 5 입력: 등록된 출구의 실제 탐색을 시작합니다."
            )
            if not started:
                self._log(f"모드 5를 시작할 수 없습니다: {reason}")
            return
        if self._requested and mode != self._expected_drive_mode:
            self._requested = False
            self._route_activated = False
            self._set_initial_hazard_bypass(False)
            self._cancel_pending_request()
            self.cancel_publisher.publish(Empty())
            self._publish_follow_hold(False)
            self._set_status("STOPPED:MODE_CHANGED")
            self._log(f"사용자가 주행 모드를 {mode}번으로 변경하여 모드 5를 정지합니다.")

    def _reset_exploration(self) -> None:
        self.checked_exit_ids.clear()
        self.blocked_exit_ids.clear()
        self.current_exit_id = None
        self.current_exit_position = None
        self.current_approach_position = None
        self.current_plan_payload = None
        self.inspection_target = None
        self.inspection_start_position = None
        self.inspection_blocks_current_exit = False
        self.inspected_dynamic_positions.clear()
        self.active_survivor_position = None
        self.active_survivor_seen_at = float("-inf")
        self.survivor_exit_id = None
        self.waiting_for_departure = False
        self._inspection_allowed_after = float("inf")
        self._route_activated = False
        self._inspection_command_sent = False
        self._retry_after = 0.0
        self._phase = "STARTING"
        self._resume_phase_after_inspection = "STARTING"
        self._robot_at_exit = False
        self.moving_tracker.reset()
        self.moving_priority_tracker.reset()
        self.stationary_tracker.reset()
        self.moving_priority_targets.clear()
        self.stationary_targets.clear()
        self.combined_inspection_active = False
        self.combined_mmwave_person = None
        self._candidate_wait_started_at = None
        self._publish_follow_hold(False)
        self._publish_blocked_exits()

    def _publish_blocked_exits(self) -> None:
        self.blocked_exits_publisher.publish(String(data=json.dumps(
            sorted(self.blocked_exit_ids), separators=(",", ":")
        )))

    def _on_danger_expected_exits(self, message: String) -> None:
        try:
            values = json.loads(message.data)
            if not isinstance(values, list):
                return
            current = {str(item) for item in values if str(item)}
        except (TypeError, ValueError):
            return
        added = current - getattr(self, "danger_expected_exit_ids", set())
        self.danger_expected_exit_ids = current
        active_exit_id = (
            self.survivor_exit_id if self._phase in {
                "ESCORTING_SURVIVOR", "WAITING_SURVIVOR_AT_EXIT"
            } else self.current_exit_id
        )
        if active_exit_id in added:
            self.checked_exit_ids.add(active_exit_id)
            # Stop the old route immediately while EvacuationManager evaluates
            # and activates the replacement exit. The new goal/path restarts
            # the follower through the normal planning pipeline.
            self.cancel_publisher.publish(Empty())
            self.waiting_for_departure = True
            self._inspection_allowed_after = float("inf")
            self._set_status(
                f"SEARCH_EXITS:DANGER_EXPECTED:{active_exit_id}"
            )
            self._log(
                f"[DANGER_EXPECTED] {active_exit_id} 경로를 위험 예상 "
                "상태로 판정했습니다. 이 출구를 제외하고 지정된 다른 이용 가능한 "
                "출구로 변경합니다."
            )

    def _on_canonical_evacuation_plan(self, message: String) -> None:
        """Synchronize Mode 5 after ExitSwitching activates a new exit."""
        if self._phase not in {
            "NAVIGATING_EXIT", "EVACUATING", "ESCORTING_SURVIVOR",
            "WAITING_SURVIVOR_AT_EXIT",
        } or not self._requested:
            return
        try:
            payload = json.loads(message.data)
            if (
                not isinstance(payload, dict)
                or not payload.get("success")
                or not payload.get("activated")
                or payload.get("manager_status") != "ROUTE_ACTIVATED"
            ):
                return
            selected = str(payload["selected_exit_id"])
            exit_position = tuple(map(
                float, payload["selected_exit_position_world"]
            ))
            approach = tuple(map(
                float, payload["selected_approach_position_world"]
            ))
            if (
                not selected or len(exit_position) != 2 or len(approach) != 2
                or not all(math.isfinite(value) for value in (*exit_position, *approach))
            ):
                return
        except (KeyError, TypeError, ValueError):
            return
        survivor_escort = self._phase in {
            "ESCORTING_SURVIVOR", "WAITING_SURVIVOR_AT_EXIT"
        }
        previous = (
            self.survivor_exit_id if survivor_escort else self.current_exit_id
        )
        if selected == previous:
            return
        if previous is not None:
            self.checked_exit_ids.add(previous)
        self.current_exit_id = selected
        self.current_exit_position = exit_position
        self.current_approach_position = approach
        self.current_plan_payload = message.data
        self.waiting_for_departure = True
        self._inspection_allowed_after = float("inf")
        if survivor_escort:
            self.survivor_exit_id = selected
            self._phase = "ESCORTING_SURVIVOR"
            self._route_activated = True
            self._robot_at_exit = False
            self._set_status(f"ESCORTING_SURVIVOR:{selected}")
        elif self._phase == "NAVIGATING_EXIT":
            self._set_status(f"SEARCH_EXITS:NAVIGATING:{selected}")
        else:
            self._set_status(f"EVACUATING:{selected}")
        # Re-publish after synchronizing Mode 5 so an old-route cancel cannot
        # invalidate the replacement plan because of cross-topic callback order.
        self.plan_publisher.publish(String(data=message.data))
        self.selected_exit_publisher.publish(String(data=selected))
        self._publish_goal(approach)
        self._log(
            f"[출구 변경] {previous or '기존 출구'}는 DANGER_EXPECTED로 "
            f"제외하고, 이용 가능한 출구 {selected}의 새 경로로 이동합니다."
        )

    def _cancel_pending_request(self) -> None:
        """Detach a stale service response before Mode 5 can be restarted."""
        future = self._future
        self._future = None
        if future is not None and not future.done():
            future.cancel()

    def _request_start(self, log_message: str) -> tuple[bool, str]:
        if not self.enabled:
            return False, "DISABLED"
        if self._future is not None and not self._future.done():
            return False, "REQUEST_IN_PROGRESS"
        self._requested = True
        self._reset_exploration()
        self.drive_mode_status = ""
        if getattr(self, "initial_route_ignore_thermal", False):
            # Clear latched readiness locally until both hazard publishers have
            # acknowledged the static/dynamic-only first-route view.
            self.hazard_status = ""
            self.exit_evaluator_status = ""
            self._set_initial_hazard_bypass(True)
        self._set_status("SEARCH_EXITS:STARTING")
        self._log(log_message)
        # Re-publish mode 5 so the drive mux confirms the selected source even
        # when its first copy of the keyboard command arrived before this node.
        self._select_drive_mode(5)
        return True, "MODE_5_EXIT_EXPLORATION_REQUESTED"

    def _start_service(self, _request, response):
        response.success, response.message = self._request_start(
            "모드 5 시작 명령 수신: 출구 탐색을 처음부터 시작합니다."
        )
        return response

    def _stop_service(self, _request, response):
        self._requested = False
        self._route_activated = False
        self._set_initial_hazard_bypass(False)
        self._phase = "STOPPED"
        self._cancel_pending_request()
        self.cancel_publisher.publish(Empty())
        self._publish_follow_hold(False)
        self._expected_drive_mode = 1
        self.mode_publisher.publish(Int32(data=1))
        self._set_status("STOPPED:STOP_SERVICE")
        self._log("모드 5 정지 명령 수신: 로봇을 정지하고 모드 1로 복귀합니다.")
        response.success = True
        response.message = "MODE_5_STOPPED"
        return response

    def _tick(self) -> None:
        if not self.enabled or not self._requested:
            return
        if self._phase in {"INSPECTION_FAILED", "STOPPED", "EVACUATION_COMPLETE"}:
            return
        if not self._drive_mode_confirmed(self._expected_drive_mode):
            self._select_drive_mode(self._expected_drive_mode)
            return
        if self._phase == "SELECTING_MODE3":
            self._tick_selecting_inspector(3)
            return
        if self._phase == "SELECTING_MODE4":
            self._tick_selecting_inspector(4)
            return
        if self._phase == "RETURNING_MODE5":
            resume = self._resume_phase_after_inspection
            self._retry_after = 0.0
            if resume == "RESUME_EXIT_ROUTE":
                self._resume_current_exit_route()
                return
            self._phase = resume
        if self._phase == "WAITING_SURVIVOR_READY":
            if time.monotonic() >= self._survivor_ready_after:
                self._phase = "SURVIVOR_PLANNING"
                self._log(
                    "[경로 생성] 요구조자와 이동할 안전한 출구 경로를 생성합니다."
                )
            else:
                return
        if self._phase in {"ESCORTING_SURVIVOR", "WAITING_SURVIVOR_AT_EXIT"}:
            self._tick_survivor_escort()
            return
        if self._route_activated:
            return
        if self._future is not None and not self._future.done():
            return
        if time.monotonic() < self._retry_after:
            return
        if self._phase in {"FINAL_PLANNING", "SURVIVOR_PLANNING"}:
            self._request_final_evacuation()
            return
        if self._phase != "STARTING":
            return
        evaluation_ready = self.evaluation_client.wait_for_service(timeout_sec=0.0)
        state = startup_state(
            self.hazard_status,
            self.exit_evaluator_status,
            self.evacuation_manager_status,
            self.drive_mode_status,
            evaluation_ready,
        )
        self._set_status(state)
        if state != "SEARCH_EXITS":
            return
        self._phase = "EVALUATING_EXITS"
        self._set_status("SEARCH_EXITS:EVALUATING_UNCHECKED_EXITS")
        self._log("[출구 평가] 코스트맵을 확인하여 방문할 출구를 선택합니다.")
        self._future = self.evaluation_client.call_async(Trigger.Request())
        self._future.add_done_callback(self._on_evaluation_response)

    def _tick_selecting_inspector(self, mode: int) -> None:
        status = self.mode3_status if mode == 3 else self.mode4_status
        if not status.startswith(f"MODE{mode}_READY"):
            if mode == 3:
                label = self.current_exit_id
            elif getattr(self, "combined_inspection_active", False):
                label = "STATIONARY_CANDIDATE"
            else:
                label = "MOVING_CANDIDATE"
            self._set_status(f"WAITING_FOR_MODE{mode}_READY:{label}")
            return
        if self._inspection_command_sent:
            return
        target_x, target_y = self.inspection_target
        self.inspection_publisher.publish(String(
            data=f"MODE{mode}_START_AT:{target_x:.6f},{target_y:.6f}"
        ))
        self._inspection_command_sent = True
        if mode == 3:
            self._phase = "INSPECTING_CANDIDATE"
            label = self.current_exit_id or "ROUTE"
            self._set_status(
                f"SEARCH_EXITS:MMWAVE_INSPECTION:{label}:MAX_2.50M"
            )
            self._log(
                f"[접근] 동적장애물 후보 ({target_x:.2f}, {target_y:.2f})의 "
                "최신 LiDAR 위치를 추적합니다. 로봇과의 거리가 2.5m "
                "이내가 되면 정지하여 생체신호를 확인합니다."
            )
        else:
            self._phase = "INSPECTING_MOVING_CANDIDATE"
            if getattr(self, "combined_inspection_active", False):
                self._set_status(
                    "STATIONARY_CANDIDATE:CAMERA_YOLO_INSPECTION:2.00M"
                )
                self._log(
                    f"[복합 판별] 같은 정지 LiDAR 후보 "
                    f"({target_x:.2f}, {target_y:.2f})를 카메라에 전달합니다. "
                    "로봇이 정지하면 YOLO 추론을 시작합니다."
                )
            else:
                self._set_status("MOVING_CANDIDATE:MODE4_INSPECTION:2.00M")
                self._log(
                    f"움직이는 LiDAR 후보 좌표 ({target_x:.2f}, {target_y:.2f})를 "
                    "모드 4에 전달하고 사람 여부를 확인하러 갑니다."
                )

    def _on_evaluation_response(self, future) -> None:
        if future is not self._future:
            return
        self._future = None
        if not self._requested:
            self._reinforce_stop()
            return
        try:
            response = future.result()
            if response is None or not response.success:
                raise ValueError("EMPTY_RESPONSE" if response is None else response.message)
            decision = build_next_exploration_decision(
                response.message, self.checked_exit_ids
            )
        except Exception as exc:
            self._phase = "STARTING"
            self._retry_after = time.monotonic() + self.retry_period
            self._set_status(f"SEARCH_EXITS:RETRY:{str(exc)[:160]}")
            self._log(f"출구 평가 실패로 잠시 후 재시도합니다: {str(exc)[:120]}")
            return
        if decision.complete:
            self._phase = "FINAL_PLANNING"
            self._set_status("SEARCH_EXITS:ALL_EXITS_CHECKED")
            self._log("모든 출구의 현장 확인이 끝났습니다. 최종 대피 경로를 생성합니다.")
            return
        if not decision.success:
            self._phase = "STARTING"
            self._retry_after = time.monotonic() + self.retry_period
            self._set_status(f"SEARCH_EXITS:STALLED:{decision.status}")
            self._log(f"방문 가능한 미확인 출구가 없어 재평가합니다: {decision.status}")
            return
        self.current_exit_id = decision.target_exit_id
        self.current_exit_position = decision.exit_position_world
        self.current_approach_position = decision.approach_position_world
        self.current_plan_payload = decision.plan_payload
        self.waiting_for_departure = True
        self._inspection_allowed_after = float("inf")
        self._phase = "NAVIGATING_EXIT"
        self.plan_publisher.publish(String(data=decision.plan_payload))
        self.selected_exit_publisher.publish(String(data=self.current_exit_id))
        self._publish_goal(self.current_approach_position)
        self._set_status(f"SEARCH_EXITS:NAVIGATING:{self.current_exit_id}")
        self._log(exit_navigation_log(
            self.current_exit_id, self.blocked_exit_ids
        ))
        # A latched red candidate may already exist before this route is
        # published.  Do not let it cancel the new waypoint path in this same
        # callback.  Candidate inspection becomes eligible only after the
        # follower has actually issued a motion command for this route.

    def _publish_goal(self, position) -> None:
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x = float(position[0])
        goal.pose.position.y = float(position[1])
        goal.pose.orientation.w = 1.0
        self.goal_publisher.publish(goal)

    def _valid_pose_array(self, message: PoseArray):
        if message.header.frame_id and message.header.frame_id.lstrip("/") != self.map_frame.lstrip("/"):
            return None
        return [
            (float(pose.position.x), float(pose.position.y))
            for pose in message.poses
            if math.isfinite(float(pose.position.x))
            and math.isfinite(float(pose.position.y))
        ]

    def _on_candidates(self, message: PoseArray) -> None:
        candidates = self._valid_pose_array(message)
        if candidates is None:
            return
        self.candidates = candidates
        self._maybe_start_nearest_inspection()

    def _on_motion_candidates(self, message: PoseArray) -> None:
        candidates = self._valid_pose_array(message)
        if candidates is None or not getattr(self, "moving_priority_enabled", False):
            return
        now = time.monotonic()
        moving = self.moving_priority_tracker.update(candidates, now)
        if moving:
            self.moving_priority_targets.extend(
                (item.position, now) for item in moving
            )
        self.moving_priority_targets = [
            item for item in self.moving_priority_targets
            if now - item[1] <= self.moving_priority_target_timeout
        ]
        self._maybe_start_nearest_inspection()

    def _on_all_candidates(self, message: PoseArray) -> None:
        candidates = self._valid_pose_array(message)
        if candidates is None:
            return
        self.all_candidates = candidates
        if (
            self._phase == "INSPECTING_CANDIDATE"
            and self.inspection_target is not None
        ):
            latest = min(
                candidates,
                key=lambda point: math.dist(point, self.inspection_target),
                default=None,
            )
            if (
                latest is not None
                and math.dist(latest, self.inspection_target)
                <= self.classification_radius
            ):
                self.inspection_target = latest
            return
        now = time.monotonic()
        if self.active_survivor_position is not None:
            self._update_survivor_track(candidates, now)
            return
        if getattr(self, "stationary_combined_inspection_enabled", False):
            stationary = self.stationary_tracker.update(candidates, now)
            self.stationary_targets = [
                (item.position, now) for item in stationary
            ]
            self._maybe_start_nearest_inspection()
            if self._phase not in {"NAVIGATING_EXIT", "EVACUATING"}:
                return
        if not self.moving_survivor_enabled or self._phase not in {
            "NAVIGATING_EXIT", "EVACUATING"
        }:
            return
        moving = self.moving_tracker.update(candidates, now)
        if not moving:
            return
        candidate = max(moving, key=lambda item: item.displacement_m)
        self._resume_phase_after_inspection = (
            "FINAL_PLANNING" if self._phase == "EVACUATING" else "STARTING"
        )
        self._route_activated = False
        self.cancel_publisher.publish(Empty())
        self.inspection_target = candidate.position
        self.inspection_start_position = candidate.position
        self.mode4_status = ""
        self._inspection_command_sent = False
        self._phase = "SELECTING_MODE4"
        self._set_status(f"MOVING_CANDIDATE:TRACK_{candidate.track_id}")
        self._log(
            f"움직이는 LiDAR 동적장애물 감지: {candidate.observations}회 추적, "
            f"이동량 {candidate.displacement_m:.2f}m. 사람인지 확인하기 위해 접근합니다."
        )
        self._select_drive_mode(4)

    def _maybe_start_nearest_inspection(self) -> None:
        """Lock and inspect exactly one closest red candidate with Mode 3."""
        if self._phase not in {"NAVIGATING_EXIT", "EVACUATING"}:
            return
        if self.active_survivor_position is not None:
            return
        if (
            self._phase == "NAVIGATING_EXIT"
            and (
                self.waiting_for_departure
                or time.monotonic() < self._inspection_allowed_after
            )
        ):
            return
        robot = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if robot is None:
            return
        now = time.monotonic()
        ordinary_target = nearest_uninspected_candidate(
            self.candidates,
            (robot[0], robot[1]),
            self.inspected_dynamic_positions,
            self.candidate_suppression_radius,
        )
        if ordinary_target is None:
            self._candidate_wait_started_at = None
            return
        target = None
        moving_selected = False
        stationary_selected = False
        if getattr(self, "stationary_combined_inspection_enabled", False):
            self.stationary_targets = [
                item for item in self.stationary_targets
                if now - item[1] <= self.stationary_target_timeout
            ]
            target = moving_priority_candidate(
                (item[0] for item in self.stationary_targets),
                self.candidates,
                (robot[0], robot[1]),
                self.inspected_dynamic_positions,
                self.moving_association_radius,
                self.candidate_suppression_radius,
            )
            stationary_selected = target is not None
            if target is None:
                return
        elif getattr(self, "moving_priority_enabled", False):
            self.moving_priority_targets = [
                item for item in self.moving_priority_targets
                if now - item[1] <= self.moving_priority_target_timeout
            ]
            target = moving_priority_candidate(
                (item[0] for item in self.moving_priority_targets),
                self.candidates,
                (robot[0], robot[1]),
                self.inspected_dynamic_positions,
                self.moving_association_radius,
                self.candidate_suppression_radius,
            )
            moving_selected = target is not None
            if target is None:
                if getattr(self, "_candidate_wait_started_at", None) is None:
                    self._candidate_wait_started_at = now
                if now - self._candidate_wait_started_at < self.moving_priority_wait:
                    return
        target = target or ordinary_target
        self._candidate_wait_started_at = None
        previous_phase = self._phase
        exit_match = None
        if previous_phase == "NAVIGATING_EXIT" and self.current_exit_id is not None:
            exit_match = nearest_exit_obstacle_candidate(
                [target],
                self.current_exit_position,
                self.current_approach_position,
                self.exit_obstacle_radius,
            )
        self.inspection_blocks_current_exit = exit_match is not None
        self._resume_phase_after_inspection = (
            "FINAL_PLANNING"
            if previous_phase == "EVACUATING"
            else "RESUME_EXIT_ROUTE"
        )
        self._route_activated = False
        self.cancel_publisher.publish(Empty())
        self.inspection_target = target
        # Keep the first map-frame coordinate as well as subsequent live-LiDAR
        # updates. A moving/jittering cluster can otherwise leave its old red
        # marker far enough from the final coordinate to be inspected twice.
        self.inspection_start_position = target
        self.combined_inspection_active = stationary_selected
        self.combined_mmwave_person = None
        self.mode3_status = ""
        self._inspection_command_sent = False
        self._phase = "SELECTING_MODE3"
        prefix = (
            "STATIONARY_LIDAR_CANDIDATE"
            if stationary_selected
            else "MOVING_PRIORITY" if moving_selected
            else "CLOSEST_RED_CANDIDATE"
        )
        self._set_status(f"SEARCH_EXITS:{prefix}:{target[0]:.2f},{target[1]:.2f}")
        context = (
            f"{self.current_exit_id} 앞 출구 차단 후보"
            if self.inspection_blocks_current_exit
            else "주행 경로의 동적장애물 후보"
        )
        if stationary_selected:
            self._log(
                f"[정지 후보 감지] LiDAR에서 2초 이상 정지한 {context} "
                f"({target[0]:.2f}, {target[1]:.2f})를 선택했습니다. "
                "mmWave와 카메라 YOLO를 차례로 사용해 사람인지 판별합니다."
            )
        else:
            candidate_kind = "움직임 우선" if moving_selected else "가장 가까운"
            self._log(
                f"[후보 감지] {candidate_kind} {context} "
                f"({target[0]:.2f}, {target[1]:.2f})를 선택했습니다. "
                "기존 출구 주행과 장애물 회피를 잠시 중단하고 "
                "이 후보만 검사합니다."
            )
        self._select_drive_mode(3)

    # Backward-compatible name used by older tests/log tools.
    def _maybe_start_exit_inspection(self) -> None:
        self._maybe_start_nearest_inspection()

    def _resume_current_exit_route(self) -> None:
        """Recreate the interrupted waypoint route after a non-blocking check."""
        if (
            self.current_exit_id is None
            or self.current_approach_position is None
            or not self.current_plan_payload
        ):
            self._phase = "STARTING"
            self._retry_after = time.monotonic() + 0.2
            return
        self.waiting_for_departure = True
        self._inspection_allowed_after = float("inf")
        self._phase = "NAVIGATING_EXIT"
        self.plan_publisher.publish(String(data=self.current_plan_payload))
        self.selected_exit_publisher.publish(String(data=self.current_exit_id))
        self._publish_goal(self.current_approach_position)
        self._set_status(f"SEARCH_EXITS:RESUMING:{self.current_exit_id}")
        self._log(
            f"[주행 재개] 출구 차단 장애물이 아니므로 "
            f"{self.current_exit_id} 경로로 다시 이동합니다."
        )
        # As for a newly selected exit, wait for an actual follower motion
        # command before a latched candidate may preempt this restored route.

    def _on_follower_state(self, message: String) -> None:
        if not getattr(self, "_requested", True):
            return
        if self._phase == "NAVIGATING_EXIT":
            if (
                message.data in self.MOTION_FOLLOWER_STATES
                and getattr(self, "_initial_hazard_bypass_active", False)
            ):
                self._set_initial_hazard_bypass(False)
                self._log(
                    "[열화상 활성화] 첫 경로의 모터 명령을 확인했습니다. "
                    "이후 경로계획부터 열화상 hazard layer를 반영합니다."
                )
            if message.data in self.ACTIVE_FOLLOWER_STATES:
                self.waiting_for_departure = False
            if (
                message.data in self.MOTION_FOLLOWER_STATES
                and not math.isfinite(self._inspection_allowed_after)
            ):
                self._inspection_allowed_after = (
                    time.monotonic() + self.inspection_after_motion_delay
                )
                self._log(
                    f"[주행 시작] {self.current_exit_id} 경로의 모터 명령이 "
                    "발행됐습니다. 주행을 안정화한 뒤 빨간 후보를 검사합니다."
                )
            if message.data != "GOAL_REACHED" or self.waiting_for_departure:
                return
            # If the robot reached the exit approach before the inspection
            # delay elapsed, inspect a currently visible blocker before marking
            # the exit usable.
            self._inspection_allowed_after = 0.0
            self._maybe_start_nearest_inspection()
            if self._phase != "NAVIGATING_EXIT":
                return
            checked = self.current_exit_id
            self.cancel_publisher.publish(Empty())
            self.checked_exit_ids.add(checked)
            self.current_exit_id = None
            self.current_exit_position = None
            self.current_approach_position = None
            self.current_plan_payload = None
            self._phase = "STARTING"
            self._retry_after = time.monotonic() + 0.2
            self._set_status(f"SEARCH_EXITS:EXIT_CHECKED_USABLE:{checked}")
            self._log(f"{checked} 도착 완료: 출구를 막는 장애물이 없어 사용 가능한 출구입니다.")
            return
        if self._phase == "EVACUATING" and message.data == "GOAL_REACHED":
            self._route_activated = False
            self._phase = "EVACUATION_COMPLETE"
            self._set_status("EVACUATION_COMPLETE:NO_SURVIVOR")
            self._log("최종 출구에 도착하여 대피 주행을 완료했습니다.")
            return
        if self._phase == "ESCORTING_SURVIVOR" and message.data == "GOAL_REACHED":
            self._route_activated = False
            self._robot_at_exit = True
            self._phase = "WAITING_SURVIVOR_AT_EXIT"
            self._publish_follow_hold(True)
            self._set_status(f"WAITING_SURVIVOR_AT_EXIT:{self.survivor_exit_id}")
            self._log(
                f"로봇이 {self.survivor_exit_id}에 도착했습니다. "
                "요구조자가 출구까지 따라왔는지 LiDAR로 확인합니다."
            )

    def _on_planner_state(self, message: String) -> None:
        if self._phase == "NAVIGATING_EXIT" and message.data == "NO_PATH":
            self._maybe_start_nearest_inspection()
            if self._phase == "NAVIGATING_EXIT":
                self._set_status(
                    f"SEARCH_EXITS:WAITING_FOR_OBSTACLE_OR_REPLAN:{self.current_exit_id}"
                )
                self._log(f"{self.current_exit_id} 경로 없음: 장애물 갱신 또는 리플래닝을 기다립니다.")

    def _on_mode3_classification(self, message: String) -> None:
        if self._phase != "INSPECTING_CANDIDATE":
            return
        result = parse_mode3_classification(message.data)
        if result is None:
            return
        kind, position = result
        if self.inspection_target is None or math.dist(position, self.inspection_target) > self.classification_radius:
            return
        inspected_position = self.inspection_target or position
        if getattr(self, "combined_inspection_active", False):
            self.combined_mmwave_person = kind == "PERSON"
            self.inspection_target = position
            self.mode4_status = ""
            self._inspection_command_sent = False
            self._phase = "SELECTING_MODE4"
            result_text = (
                "사람 생체신호 감지"
                if self.combined_mmwave_person
                else "생체신호 미감지"
            )
            self._set_status(
                f"STATIONARY_CANDIDATE:MMWAVE_COMPLETE:{kind}:SELECTING_CAMERA"
            )
            self._log(
                f"[복합 판별 1/2] mmWave 결과: {result_text}. "
                "같은 정지 후보에 대해 카메라 YOLO 판정을 "
                "이어서 수행합니다."
            )
            self._select_drive_mode(4)
            return
        if kind == "DYNAMIC_OBSTACLE":
            self._complete_nonperson_inspection(
                inspected_position,
                "[장애물 확정] 생체신호가 감지되지 않았습니다. "
                "실제 동적장애물로 확정하고 RViz의 빨간색 표시를 "
                "유지합니다.",
            )
            return
        self._log(
            "[요구조자 발견] 생체신호가 감지됐습니다. 요구조자로 확정하고 "
            "RViz 표시를 파란색으로 전환합니다."
        )
        self._begin_survivor_evacuation(position, "mmWave")

    def _on_mode3_status(self, message: String) -> None:
        state = str(message.data)
        changed = state != self.mode3_status
        self.mode3_status = state
        if self._phase != "INSPECTING_CANDIDATE":
            return
        progress_log = mode3_inspection_progress_log(state)
        if changed and progress_log is not None:
            self._log(progress_log)
        if state.startswith("MODE3_TARGET_TRACK_LOST"):
            self._cancel_disappeared_inspection_target()
            return
        if state.startswith((
            "MODE3_SENSOR_UNAVAILABLE",
            "MODE3_NO_PATH_TO_STANDOFF",
            "MODE3_TARGET_TOO_CLOSE",
            "MODE3_ARRIVAL_NOT_CONFIRMED",
        )):
            self.cancel_publisher.publish(Empty())
            self._phase = "INSPECTION_FAILED"
            self._set_status(f"SEARCH_EXITS:MMWAVE_INSPECTION_FAILED:{state}")
            self._log(f"[판별 실패] mmWave 장애물 검사 실패: {state}")
            self._select_drive_mode(5)

    def _cancel_disappeared_inspection_target(self) -> None:
        """Cancel a vanished LiDAR target and resume the interrupted route."""
        anchors = [
            point for point in (
                getattr(self, "inspection_start_position", None),
                self.inspection_target,
            )
            if point is not None
        ]

        def keep(point) -> bool:
            return not any(
                math.dist(point, anchor) <= self.classification_radius
                for anchor in anchors
            )

        # Remove latched copies immediately so returning to Mode 5 cannot
        # select the vanished target again before the next empty LiDAR update.
        self.candidates = [
            point for point in getattr(self, "candidates", ()) if keep(point)
        ]
        self.all_candidates = [
            point for point in getattr(self, "all_candidates", ()) if keep(point)
        ]
        self.moving_priority_targets = [
            item for item in getattr(self, "moving_priority_targets", ())
            if keep(item[0])
        ]
        self.stationary_targets = [
            item for item in getattr(self, "stationary_targets", ())
            if keep(item[0])
        ]
        self.cancel_publisher.publish(Empty())
        self._route_activated = False
        self.inspection_target = None
        self.inspection_start_position = None
        self.inspection_blocks_current_exit = False
        self.combined_inspection_active = False
        self.combined_mmwave_person = None
        self._inspection_command_sent = False
        self._candidate_wait_started_at = None
        self._phase = "RETURNING_MODE5"
        self._set_status("SEARCH_EXITS:INSPECTION_TARGET_DISAPPEARED")
        self._log(
            "[검사 취소] 선택한 동적장애물이 LiDAR에서 일정 시간 동안 "
            "다시 확인되지 않아 사라진 것으로 판단합니다. 검사를 중단하고 "
            "기존 대피 경로로 복귀합니다."
        )
        self._select_drive_mode(5)

    def _on_mode4_status(self, message: String) -> None:
        self.mode4_status = str(message.data)
        if self._phase != "INSPECTING_MOVING_CANDIDATE":
            return
        if message.data.startswith(("MODE4_DETECTOR_UNAVAILABLE", "MODE4_NO_PATH_TO_STANDOFF")):
            self.cancel_publisher.publish(Empty())
            self._phase = "INSPECTION_FAILED"
            if getattr(self, "combined_inspection_active", False):
                self._set_status(
                    f"STATIONARY_CANDIDATE:CAMERA_INSPECTION_FAILED:{message.data}"
                )
                self._log(f"정지 후보의 카메라 YOLO 판별 실패: {message.data}")
            else:
                self._set_status(f"MOVING_CANDIDATE:INSPECTION_FAILED:{message.data}")
                self._log(f"움직이는 후보의 카메라 판별 실패: {message.data}")
            self._select_drive_mode(5)

    def _on_mode4_classification(self, message: String) -> None:
        if self._phase != "INSPECTING_MOVING_CANDIDATE":
            return
        result = parse_mode4_classification(message.data, self.inspection_target)
        if result is None:
            return
        kind, position = result
        if (
            kind == "SURVIVOR"
            and self.inspection_target is not None
            and math.dist(position, self.inspection_target) > self.classification_radius
        ):
            return
        if getattr(self, "combined_inspection_active", False):
            camera_person = kind == "SURVIVOR"
            mmwave_person = bool(self.combined_mmwave_person)
            target = position if camera_person else self.inspection_target
            if mmwave_person or camera_person:
                evidence = []
                if mmwave_person:
                    evidence.append("mmWave")
                if camera_person:
                    evidence.append("YOLO")
                self._log(
                    f"[복합 판별 2/2] {'+'.join(evidence)}에서 사람 신호를 "
                    "확인했습니다. 요구조자로 확정합니다."
                )
                self._begin_survivor_evacuation(target, "mmwave_camera_lidar")
            else:
                self._complete_nonperson_inspection(
                    self.inspection_target,
                    "[복합 판별 2/2] mmWave 생체신호와 YOLO 사람 검출이 "
                    "모두 음성입니다. 동적장애물로 확정하고 빨간색 표시를 "
                    "유지합니다.",
                )
            return
        if kind == "NO_SURVIVOR":
            self._complete_nonperson_inspection(
                self.inspection_target,
                "카메라·LiDAR 판별 결과: 요구조자가 아닌 "
                "동적장애물입니다.",
            )
            return
        self._log("카메라·LiDAR 판별 결과: 요구조자입니다. RViz 표시를 파란색으로 전환합니다.")
        self._begin_survivor_evacuation(position, "camera_lidar")

    def _complete_nonperson_inspection(self, inspected_position, evidence_log: str) -> None:
        """Finish a negative inspection and apply the existing exit policy."""
        checked = self.current_exit_id
        self._remember_inspected_positions(
            getattr(self, "inspection_start_position", None),
            inspected_position,
        )
        self._log(evidence_log)
        self.inspection_target = None
        self.inspection_start_position = None
        self.combined_inspection_active = False
        self.combined_mmwave_person = None
        if self.inspection_blocks_current_exit and checked is not None:
            self.checked_exit_ids.add(checked)
            self.blocked_exit_ids.add(checked)
            self._publish_blocked_exits()
            self.current_exit_id = None
            self.current_exit_position = None
            self.current_approach_position = None
            self.current_plan_payload = None
            self._resume_phase_after_inspection = "STARTING"
            self._set_status(f"SEARCH_EXITS:EXIT_BLOCKED:{checked}")
            self._log(
                f"[출구 폐쇄] {checked} 앞이 동적장애물로 막혔습니다. "
                "이 출구를 제외하고 다른 출구를 다시 선택합니다."
            )
        else:
            self._set_status("SEARCH_EXITS:ROUTE_OBSTACLE_CONFIRMED")
            self._log(
                "[경로 판단] 출구를 직접 막는 장애물은 아닙니다. "
                "검사 전 출구 경로로 복귀합니다."
            )
        self.inspection_blocks_current_exit = False
        self._phase = "RETURNING_MODE5"
        self._select_drive_mode(5)

    def _remember_inspected_positions(self, *positions) -> None:
        """Store all valid map anchors covered by one completed inspection."""
        for value in positions:
            if value is None:
                continue
            try:
                point = tuple(float(item) for item in value)
            except (TypeError, ValueError):
                continue
            if len(point) != 2 or not all(math.isfinite(item) for item in point):
                continue
            if any(
                math.dist(point, previous) <= 1e-6
                for previous in self.inspected_dynamic_positions
            ):
                continue
            self.inspected_dynamic_positions.append(point)

    def _begin_survivor_evacuation(self, position, source: str) -> None:
        self.cancel_publisher.publish(Empty())
        self._route_activated = False
        self.active_survivor_position = tuple(map(float, position))
        self._remember_inspected_positions(
            getattr(self, "inspection_start_position", None),
            self.inspection_target,
            position,
        )
        self.inspection_target = None
        self.inspection_start_position = None
        self.inspection_blocks_current_exit = False
        self.combined_inspection_active = False
        self.combined_mmwave_person = None
        self.current_exit_id = None
        self.current_exit_position = None
        self.current_approach_position = None
        self.current_plan_payload = None
        self.active_survivor_seen_at = time.monotonic()
        self._publish_survivor_track(self.active_survivor_position)
        self._robot_at_exit = False
        self._publish_follow_hold(True)
        self._survivor_ready_after = time.monotonic() + self.survivor_ready_wait
        self._phase = "WAITING_SURVIVOR_READY"
        self._set_status(f"SURVIVOR_CONFIRMED:{source}:WAITING_TO_ESCORT")
        self._log(
            "[동행 준비] 요구조자에게 로봇을 따라오도록 안내합니다. "
            "거리가 멀어지거나 추적이 끊기면 로봇은 자동 정지합니다."
        )
        self._select_drive_mode(5)

    def _update_survivor_track(self, candidates, now: float) -> None:
        if not candidates or self.active_survivor_position is None:
            return
        nearest = min(candidates, key=lambda point: math.dist(point, self.active_survivor_position))
        if math.dist(nearest, self.active_survivor_position) > self.survivor_track_radius:
            return
        self.active_survivor_position = nearest
        self.active_survivor_seen_at = now
        self._publish_survivor_track(nearest)

    def _publish_survivor_track(self, position) -> None:
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = self.map_frame
        point.point.x = float(position[0])
        point.point.y = float(position[1])
        point.point.z = 0.10
        self.survivor_track_publisher.publish(point)

    def _tick_survivor_escort(self) -> None:
        pose = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if pose is None or self.active_survivor_position is None:
            self._publish_follow_hold(True)
            return
        now = time.monotonic()
        tracking_fresh = now - self.active_survivor_seen_at <= self.survivor_track_stale
        distance = math.dist((pose[0], pose[1]), self.active_survivor_position)
        if self._phase == "WAITING_SURVIVOR_AT_EXIT":
            if tracking_fresh and distance <= self.exit_arrival_distance:
                self._publish_follow_hold(False)
                self._phase = "EVACUATION_COMPLETE"
                self._set_status(f"EVACUATION_COMPLETE:SURVIVOR:{self.survivor_exit_id}")
                self._log(
                    f"요구조자가 {self.survivor_exit_id}까지 도착했습니다. 동행 대피 완료."
                )
            return
        should_hold = not tracking_fresh or distance > self.follow_stop_distance
        if should_hold and not self._survivor_hold:
            self._publish_follow_hold(True)
            reason = "LiDAR 추적이 끊겨" if not tracking_fresh else f"요구조자 거리 {distance:.2f}m로 멀어져"
            self._set_status("ESCORTING_SURVIVOR:WAITING_FOR_FOLLOWER")
            self._log(f"{reason} 로봇을 정지하고 요구조자를 기다립니다.")
        elif self._survivor_hold and tracking_fresh and distance <= self.follow_resume_distance:
            self._publish_follow_hold(False)
            self._set_status(f"ESCORTING_SURVIVOR:{self.survivor_exit_id}")
            self._log(f"요구조자 거리 {distance:.2f}m 확인: 동행 대피 주행을 재개합니다.")

    def _request_final_evacuation(self) -> None:
        if not self.plan_client.wait_for_service(timeout_sec=0.0):
            self._set_status("PLAN_EVACUATION:WAITING_FOR_PLAN_SERVICE")
            return
        self._set_status("PLAN_EVACUATION:EVALUATING_SAFE_EXIT")
        self._log(
            "[안전 출구 평가] 막힌 출구를 제외하고 요구조자와 이동할 "
            "안전한 출구를 평가합니다."
        )
        self._future = self.plan_client.call_async(Trigger.Request())
        self._future.add_done_callback(self._on_plan_response)

    def _on_plan_response(self, future) -> None:
        if future is not self._future:
            return
        self._future = None
        try:
            response = future.result()
            result = parse_activation_response(response.success, response.message)
        except Exception as exc:
            result = parse_activation_response(False, f"PLAN_SERVICE_ERROR:{exc}")
        if not self._requested:
            self._reinforce_stop()
            return
        if result.activated:
            self._route_activated = True
            if self.active_survivor_position is not None:
                self.survivor_exit_id = result.exit_id
                self._phase = "ESCORTING_SURVIVOR"
                self._set_status(f"ESCORTING_SURVIVOR:{result.exit_id}")
                self._log(
                    f"[동행 대피] 요구조자와 함께 출구 {result.exit_id}로 이동합니다."
                )
                self._tick_survivor_escort()
            else:
                self._phase = "EVACUATING"
                self._set_status(f"EVACUATING:{result.exit_id}")
                self._log(f"[최종 주행] 대피 출구 {result.exit_id}로 이동합니다.")
            return
        self._retry_after = time.monotonic() + self.retry_period
        self._set_status(f"PLAN_EVACUATION:RETRY:{result.reason[:160]}")
        self._log(f"대피 경로 생성 실패로 재시도합니다: {result.reason[:120]}")

    def _on_waypoint_route_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            waypoints = [str(item) for item in payload.get("waypoints", [])]
            event = str(payload.get("event", ""))
            goal_id = str(payload.get("goal_id", "목표"))
        except (TypeError, ValueError):
            return
        route = " -> ".join(waypoints) if waypoints else "직접 목표점"
        if event == "REPLANNED":
            self._log(f"상황 변화로 경로 재생성: {route} -> {goal_id}")
        elif event == "PATH_CREATED":
            self._log(f"경로 생성: {route} -> {goal_id}")

    def _on_replanning_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        state = str(payload.get("state", ""))
        signature = (
            state,
            payload.get("hazard_revision"),
            payload.get("attempt_count"),
            payload.get("last_replan_reason"),
        )
        if signature == self._last_replanning_signature:
            return
        self._last_replanning_signature = signature
        reason = payload.get("last_replan_reason") or "costmap change"
        if state in {"HOLDING_FOR_REPLAN", "REPLAN_REQUESTED", "WAITING_FOR_NEW_PATH"}:
            self._log(f"위험 변화 감지({reason}): 로봇을 정지하고 경로 리플래닝을 요청합니다.")
        elif state == "REPLAN_SUCCEEDED":
            self._log("리플래닝 성공: 새 웨이포인트 경로로 주행을 재개합니다.")
        elif state == "EXIT_RESELECTION_REQUIRED":
            self._log("현재 출구가 위험하거나 막혀 다른 출구를 다시 선택합니다.")
        elif state in {"REPLAN_FAILED", "REPLAN_EXHAUSTED"}:
            self._log(f"리플래닝 실패: {payload.get('last_failure_reason') or state}")

    def _reinforce_stop(self) -> None:
        self.cancel_publisher.publish(Empty())
        self._publish_follow_hold(False)
        self._expected_drive_mode = 1
        self.mode_publisher.publish(Int32(data=1))

    def destroy_node(self) -> None:
        if rclpy.ok():
            self._reinforce_stop()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvacuationDemoOrchestrator()
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
