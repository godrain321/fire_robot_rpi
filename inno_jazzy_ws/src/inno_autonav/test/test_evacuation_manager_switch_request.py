"""Contract tests for EvacuationManagerNode's Stage 7 switch-request handling.

Uses the same fake-self + MethodType pattern as test_evacuation_manager_contract.py
(no live rclpy context). Proves the two safety properties Stage 7 depends on:
excluding the current exit actually changes the selection, and a failed switch
never overwrites the still-canonical previous plan/selected/goal.
"""

import json
from types import MethodType, SimpleNamespace

from rclpy.clock import Clock
from std_srvs.srv import Trigger

from inno_autonav.evacuation_manager_node import EvacuationManagerNode
from inno_autonav.evacuation_planner import EvacuationPlanner
from inno_autonav.exit_evaluator import ExitEvaluation, ExitEvaluationBatch


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class EvaluationClient:
    def __init__(self, response):
        self.response = response

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return True

    def call(self, request, timeout_sec):
        del request, timeout_sec
        return self.response


def _evaluation(exit_id, *, accepted, path_length):
    return ExitEvaluation(
        exit_id, "unknown", (float(path_length) + 1.0, 0.0), (1.0, 0.5), (1, 0),
        accepted, accepted, ((0.5, 0.5), (float(path_length), 0.5)),
        ((0, 0), (int(path_length), 0)), float(path_length), 0.0, 40.0, None,
        40.0, None, 0.0, (), 1.0, (),
    )


def _payload(exit1_accepted=True, exit2_accepted=True, revision=4):
    return json.dumps(
        ExitEvaluationBatch(
            revision, "map", (0.5, 0.5),
            (
                _evaluation("EXIT1", accepted=exit1_accepted, path_length=1.0),
                _evaluation("EXIT2", accepted=exit2_accepted, path_length=2.0),
            ),
            1.0,
        ).to_dict()
    )


def manager(payload):
    response = Trigger.Response()
    response.success = True
    response.message = payload
    value = SimpleNamespace(
        enabled=True,
        evaluation_client=EvaluationClient(response),
        timeout=1.0,
        map_frame="map",
        planner=EvacuationPlanner(),
        risk_first=False,
        activate=False,  # /plan_evacuation's own gate -- switch requests force past it
        current_hazard_revision=4,
        plan_publisher=Publisher(),
        selected_publisher=Publisher(),
        goal_publisher=Publisher(),
        switch_result_publisher=Publisher(),
        statuses=[],
        get_clock=lambda: Clock(),
        get_logger=lambda: SimpleNamespace(error=lambda *a, **k: None),
    )
    value._status = lambda status: value.statuses.append(status)
    value._select_and_activate = MethodType(
        EvacuationManagerNode._select_and_activate, value
    )
    value._on_switch_request = MethodType(EvacuationManagerNode._on_switch_request, value)
    return value


def test_switch_request_excludes_current_exit_and_activates_replacement():
    value = manager(_payload())
    EvacuationManagerNode._on_switch_request(value, SimpleNamespace(data=json.dumps({
        "request_id": 7, "current_exit_id": "EXIT2",
        "excluded_exit_ids": ["EXIT2"], "candidate_exit_ids": None,
        "risk_first": False,
    })))
    assert value.selected_publisher.messages[0].data == "EXIT1"
    goal = value.goal_publisher.messages[0]
    assert (goal.pose.position.x, goal.pose.position.y) == (1.0, 0.5)
    ack = json.loads(value.switch_result_publisher.messages[0].data)
    assert ack == {
        "request_id": 7, "success": True, "activated": True,
        "status": "ROUTE_ACTIVATED", "selected_exit_id": "EXIT1",
    }


def test_switch_request_never_switches_back_to_the_excluded_exit():
    # EXIT2 is the shorter/cheaper route, but it is the current (excluded) exit.
    value = manager(_payload(exit1_accepted=True, exit2_accepted=True))
    EvacuationManagerNode._on_switch_request(value, SimpleNamespace(data=json.dumps({
        "request_id": 1, "current_exit_id": "EXIT2",
        "excluded_exit_ids": ["EXIT2"],
    })))
    assert value.selected_publisher.messages[0].data == "EXIT1"


def test_failed_switch_does_not_overwrite_canonical_state():
    # Excluding EXIT2 leaves nothing accepted -- no safe alternative exists.
    value = manager(_payload(exit1_accepted=False, exit2_accepted=True))
    EvacuationManagerNode._on_switch_request(value, SimpleNamespace(data=json.dumps({
        "request_id": 3, "current_exit_id": "EXIT2",
        "excluded_exit_ids": ["EXIT2"],
    })))
    assert not value.plan_publisher.messages
    assert not value.selected_publisher.messages
    assert not value.goal_publisher.messages
    ack = json.loads(value.switch_result_publisher.messages[0].data)
    assert ack["success"] is False
    assert ack["selected_exit_id"] is None
    assert ack["request_id"] == 3


def test_disabled_manager_ignores_switch_requests():
    value = manager(_payload())
    value.enabled = False
    EvacuationManagerNode._on_switch_request(value, SimpleNamespace(data=json.dumps({
        "request_id": 9, "current_exit_id": "EXIT2", "excluded_exit_ids": ["EXIT2"],
    })))
    assert not value.switch_result_publisher.messages
    assert not value.plan_publisher.messages


def test_malformed_switch_request_is_ignored_without_raising():
    value = manager(_payload())
    EvacuationManagerNode._on_switch_request(value, SimpleNamespace(data="not json"))
    assert not value.switch_result_publisher.messages
