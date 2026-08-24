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
    build_next_exploration_decision,
    nearest_exit_obstacle_candidate,
    parse_activation_response,
    parse_mode3_classification,
    parse_mode4_classification,
    startup_state,
)
from .tf_utils import TfHelper


class EvacuationDemoOrchestrator(Node):
    """Visit exits, inspect blockers and moving people, then lead evacuation."""

    ACTIVE_FOLLOWER_STATES = frozenset({
        "PATH_ACCEPTED",
        "FOLLOWING_PATH",
        "ROTATING_IN_PLACE",
        "ALIGNING_GOAL_YAW",
    })

    def __init__(self) -> None:
        super().__init__("evacuation_demo_orchestrator")
        defaults = {
            "enabled": True,
            "auto_start": True,
            "drive_mode": 5,
            "map_frame": "map",
            "base_frame": "base_link",
            "update_rate_hz": 5.0,
            "retry_period_sec": 1.0,
            "exit_obstacle_match_radius_m": 1.5,
            "classification_match_radius_m": 0.75,
            "moving_survivor_enabled": True,
            "moving_association_radius_m": 0.75,
            "moving_minimum_displacement_m": 0.20,
            "moving_minimum_observations": 3,
            "moving_window_sec": 2.0,
            "moving_stale_timeout_sec": 1.0,
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
            "exit_evaluator_status_topic": "/exit_evaluator/status",
            "evacuation_manager_status_topic": "/evacuation/status",
            "drive_mode_topic": "/drive_mode",
            "drive_mode_status_topic": "/drive_mode_status",
            "autonomy_cancel_topic": "/autonomy_cancel",
            "evacuation_plan_topic": "/evacuation/plan",
            "selected_exit_topic": "/evacuation/selected_exit",
            "blocked_exits_topic": "/evacuation/blocked_exits",
            "goal_topic": "/goal_pose",
            "follower_state_topic": "/follower_state",
            "planner_state_topic": "/planner_state",
            "obstacle_candidates_topic": "/dynamic_obstacle_candidates",
            "all_obstacle_candidates_topic": "/dynamic_obstacle_all_candidates",
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
        self.moving_survivor_enabled = bool(value("moving_survivor_enabled"))
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
            self.survivor_ready_wait,
            self.survivor_track_radius,
            self.survivor_track_stale,
            self.follow_stop_distance,
            self.follow_resume_distance,
            self.exit_arrival_distance,
        )
        if any(item <= 0.0 for item in positive):
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
        self.checked_exit_ids = set()
        self.blocked_exit_ids = set()
        self.current_exit_id = None
        self.current_exit_position = None
        self.current_approach_position = None
        self.inspection_target = None
        self.active_survivor_position = None
        self.active_survivor_seen_at = float("-inf")
        self.survivor_exit_id = None
        self.waiting_for_departure = False
        self._status_value = ""
        self._requested = self.enabled and self.auto_start
        self._route_activated = False
        self._phase = "STARTING"
        self._resume_phase_after_mode4 = "STARTING"
        self._expected_drive_mode = 5
        self._internal_mode_commands = Counter()
        self._inspection_command_sent = False
        self._future = None
        self._retry_after = 0.0
        self._survivor_ready_after = 0.0
        self._survivor_hold = False
        self._robot_at_exit = False
        self._last_replanning_signature = None
        self.create_timer(1.0 / rate, self._tick)
        if not self.enabled:
            self._set_status("DISABLED")
        elif self._requested:
            self._set_status("SEARCH_EXITS:STARTING")
            self._log("모드 5 자동 시작: 등록된 출구의 실제 탐색을 시작합니다.")
            self._publish_blocked_exits()
            self._publish_follow_hold(False)
            self._select_drive_mode(5)
        else:
            self._set_status("STOPPED:CALL_START_SERVICE")

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
        if self._requested and mode != self._expected_drive_mode:
            self._requested = False
            self._route_activated = False
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
        self.inspection_target = None
        self.active_survivor_position = None
        self.active_survivor_seen_at = float("-inf")
        self.survivor_exit_id = None
        self.waiting_for_departure = False
        self._route_activated = False
        self._inspection_command_sent = False
        self._retry_after = 0.0
        self._phase = "STARTING"
        self._robot_at_exit = False
        self.moving_tracker.reset()
        self._publish_follow_hold(False)
        self._publish_blocked_exits()

    def _publish_blocked_exits(self) -> None:
        self.blocked_exits_publisher.publish(String(data=json.dumps(
            sorted(self.blocked_exit_ids), separators=(",", ":")
        )))

    def _start_service(self, _request, response):
        if not self.enabled:
            response.success = False
            response.message = "DISABLED"
            return response
        if self._future is not None and not self._future.done():
            response.success = False
            response.message = "REQUEST_IN_PROGRESS"
            return response
        self._requested = True
        self._reset_exploration()
        self.drive_mode_status = ""
        self._set_status("SEARCH_EXITS:STARTING")
        self._log("모드 5 시작 명령 수신: 출구 탐색을 처음부터 시작합니다.")
        self._select_drive_mode(5)
        response.success = True
        response.message = "MODE_5_EXIT_EXPLORATION_REQUESTED"
        return response

    def _stop_service(self, _request, response):
        self._requested = False
        self._route_activated = False
        self._phase = "STOPPED"
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
            self._phase = self._resume_phase_after_mode4
            self._retry_after = 0.0
        if self._phase == "WAITING_SURVIVOR_READY":
            if time.monotonic() >= self._survivor_ready_after:
                self._phase = "SURVIVOR_PLANNING"
                self._log("요구조자 동행 준비 완료: 안전한 출구 경로를 생성합니다.")
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
        self._log("등록된 출구를 코스트맵으로 평가하여 다음 방문 출구를 선택합니다.")
        self._future = self.evaluation_client.call_async(Trigger.Request())
        self._future.add_done_callback(self._on_evaluation_response)

    def _tick_selecting_inspector(self, mode: int) -> None:
        status = self.mode3_status if mode == 3 else self.mode4_status
        if not status.startswith(f"MODE{mode}_READY"):
            label = self.current_exit_id if mode == 3 else "MOVING_CANDIDATE"
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
            self._phase = "INSPECTING_EXIT"
            self._set_status(
                f"SEARCH_EXITS:MMWAVE_INSPECTION:{self.current_exit_id}:2.00M"
            )
            self._log(
                f"{self.current_exit_id} 장애물 2.0m 앞에 접근하여 mmWave 판별을 시작합니다."
            )
        else:
            self._phase = "INSPECTING_MOVING_CANDIDATE"
            self._set_status("MOVING_CANDIDATE:MODE4_INSPECTION:2.00M")
            self._log(
                f"움직이는 LiDAR 후보 좌표 ({target_x:.2f}, {target_y:.2f})를 "
                "모드 4에 전달하고 사람 여부를 확인하러 갑니다."
            )

    def _on_evaluation_response(self, future) -> None:
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
        self.waiting_for_departure = True
        self._phase = "NAVIGATING_EXIT"
        self.plan_publisher.publish(String(data=decision.plan_payload))
        self.selected_exit_publisher.publish(String(data=self.current_exit_id))
        self._publish_goal(self.current_approach_position)
        self._set_status(f"SEARCH_EXITS:NAVIGATING:{self.current_exit_id}")
        self._log(f"{self.current_exit_id}로 이동 중입니다.")
        self._maybe_start_exit_inspection()

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
        self._maybe_start_exit_inspection()

    def _on_all_candidates(self, message: PoseArray) -> None:
        candidates = self._valid_pose_array(message)
        if candidates is None:
            return
        self.all_candidates = candidates
        now = time.monotonic()
        if self.active_survivor_position is not None:
            self._update_survivor_track(candidates, now)
            return
        if not self.moving_survivor_enabled or self._phase not in {
            "NAVIGATING_EXIT", "EVACUATING"
        }:
            return
        moving = self.moving_tracker.update(candidates, now)
        if not moving:
            return
        candidate = max(moving, key=lambda item: item.displacement_m)
        self._resume_phase_after_mode4 = (
            "FINAL_PLANNING" if self._phase == "EVACUATING" else "STARTING"
        )
        self._route_activated = False
        self.cancel_publisher.publish(Empty())
        self.inspection_target = candidate.position
        self.mode4_status = ""
        self._inspection_command_sent = False
        self._phase = "SELECTING_MODE4"
        self._set_status(f"MOVING_CANDIDATE:TRACK_{candidate.track_id}")
        self._log(
            f"움직이는 LiDAR 동적장애물 감지: {candidate.observations}회 추적, "
            f"이동량 {candidate.displacement_m:.2f}m. 사람인지 확인하기 위해 접근합니다."
        )
        self._select_drive_mode(4)

    def _maybe_start_exit_inspection(self) -> None:
        if self._phase != "NAVIGATING_EXIT" or self.current_exit_id is None:
            return
        target = nearest_exit_obstacle_candidate(
            self.candidates,
            self.current_exit_position,
            self.current_approach_position,
            self.exit_obstacle_radius,
        )
        if target is None:
            return
        self.cancel_publisher.publish(Empty())
        self.inspection_target = target
        self.mode3_status = ""
        self._inspection_command_sent = False
        self._phase = "SELECTING_MODE3"
        self._set_status(f"SEARCH_EXITS:EXIT_OBSTACLE_DETECTED:{self.current_exit_id}")
        self._log(
            f"{self.current_exit_id} 앞 동적장애물을 감지했습니다. "
            "사람인지 확인하기 위해 2.0m 검사 지점으로 이동합니다."
        )
        self._select_drive_mode(3)

    def _on_follower_state(self, message: String) -> None:
        if self._phase == "NAVIGATING_EXIT":
            if message.data in self.ACTIVE_FOLLOWER_STATES:
                self.waiting_for_departure = False
            if message.data != "GOAL_REACHED" or self.waiting_for_departure:
                return
            checked = self.current_exit_id
            self.cancel_publisher.publish(Empty())
            self.checked_exit_ids.add(checked)
            self.current_exit_id = None
            self.current_exit_position = None
            self.current_approach_position = None
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
            self._maybe_start_exit_inspection()
            if self._phase == "NAVIGATING_EXIT":
                self._set_status(
                    f"SEARCH_EXITS:WAITING_FOR_OBSTACLE_OR_REPLAN:{self.current_exit_id}"
                )
                self._log(f"{self.current_exit_id} 경로 없음: 장애물 갱신 또는 리플래닝을 기다립니다.")

    def _on_mode3_classification(self, message: String) -> None:
        if self._phase != "INSPECTING_EXIT":
            return
        result = parse_mode3_classification(message.data)
        if result is None:
            return
        kind, position = result
        if self.inspection_target is None or math.dist(position, self.inspection_target) > self.classification_radius:
            return
        checked = self.current_exit_id
        if kind == "DYNAMIC_OBSTACLE":
            self._log("mmWave 판별 결과: 사람이 아닌 동적장애물입니다.")
            self.checked_exit_ids.add(checked)
            self.blocked_exit_ids.add(checked)
            self._publish_blocked_exits()
            self.current_exit_id = None
            self.current_exit_position = None
            self.current_approach_position = None
            self.inspection_target = None
            self._resume_phase_after_mode4 = "STARTING"
            self._phase = "RETURNING_MODE5"
            self._set_status(f"SEARCH_EXITS:EXIT_BLOCKED:{checked}")
            self._log(f"{checked} 막힘 확정: 다음 출구로 경로를 다시 생성합니다.")
            self._select_drive_mode(5)
            return
        self._log("mmWave 판별 결과: 요구조자입니다. RViz 표시를 파란색으로 전환합니다.")
        self._begin_survivor_evacuation(position, "mmWave")

    def _on_mode3_status(self, message: String) -> None:
        self.mode3_status = str(message.data)
        if self._phase != "INSPECTING_EXIT":
            return
        if message.data.startswith(("MODE3_SENSOR_UNAVAILABLE", "MODE3_NO_PATH_TO_STANDOFF")):
            self.cancel_publisher.publish(Empty())
            self._phase = "INSPECTION_FAILED"
            self._set_status(f"SEARCH_EXITS:MMWAVE_INSPECTION_FAILED:{message.data}")
            self._log(f"mmWave 출구 장애물 판별 실패: {message.data}")
            self._select_drive_mode(5)

    def _on_mode4_status(self, message: String) -> None:
        self.mode4_status = str(message.data)
        if self._phase != "INSPECTING_MOVING_CANDIDATE":
            return
        if message.data.startswith(("MODE4_DETECTOR_UNAVAILABLE", "MODE4_NO_PATH_TO_STANDOFF")):
            self.cancel_publisher.publish(Empty())
            self._phase = "INSPECTION_FAILED"
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
        if kind == "NO_SURVIVOR":
            self._log("카메라·LiDAR 판별 결과: 요구조자가 아닌 동적장애물입니다.")
            self.inspection_target = None
            self._phase = "RETURNING_MODE5"
            self._select_drive_mode(5)
            return
        if self.inspection_target is not None and math.dist(position, self.inspection_target) > self.classification_radius:
            return
        self._log("카메라·LiDAR 판별 결과: 요구조자입니다. RViz 표시를 파란색으로 전환합니다.")
        self._begin_survivor_evacuation(position, "camera_lidar")

    def _begin_survivor_evacuation(self, position, source: str) -> None:
        self.cancel_publisher.publish(Empty())
        self._route_activated = False
        self.active_survivor_position = tuple(map(float, position))
        self.active_survivor_seen_at = time.monotonic()
        self._publish_survivor_track(self.active_survivor_position)
        self._robot_at_exit = False
        self._publish_follow_hold(True)
        self._survivor_ready_after = time.monotonic() + self.survivor_ready_wait
        self._phase = "WAITING_SURVIVOR_READY"
        self._set_status(f"SURVIVOR_CONFIRMED:{source}:WAITING_TO_ESCORT")
        self._log(
            "요구조자에게 로봇을 따라오도록 안내한 뒤 동행 대피를 시작합니다. "
            "사람과 거리가 멀어지거나 추적이 끊기면 로봇은 자동 정지합니다."
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
        self._log("막힌 출구를 제외하고 현재 코스트맵에서 안전한 대피 출구를 평가합니다.")
        self._future = self.plan_client.call_async(Trigger.Request())
        self._future.add_done_callback(self._on_plan_response)

    def _on_plan_response(self, future) -> None:
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
                self._log(f"요구조자와 함께 {result.exit_id}로 대피를 시작합니다.")
                self._tick_survivor_escort()
            else:
                self._phase = "EVACUATING"
                self._set_status(f"EVACUATING:{result.exit_id}")
                self._log(f"최종 대피 출구 {result.exit_id}로 이동합니다.")
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
