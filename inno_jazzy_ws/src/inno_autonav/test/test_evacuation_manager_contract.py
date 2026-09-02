from pathlib import Path
from types import MethodType, SimpleNamespace

from rclpy.clock import Clock
from std_srvs.srv import Trigger

from inno_autonav.evacuation_manager_node import EvacuationManagerNode
from inno_autonav.evacuation_planner import EvacuationPlanner
from inno_autonav.exit_evaluator import ExitEvaluation, ExitEvaluationBatch


SOURCE = Path(__file__).parents[1] / "inno_autonav" / "evacuation_manager_node.py"


def test_manager_uses_stage4_service_and_existing_goal_topic_only():
    text = SOURCE.read_text(encoding="utf-8")
    assert '"/evaluate_exits"' in text
    assert '"/goal_pose"' in text
    assert '"/planned_path"' not in text
    assert "create_publisher(Path" not in text


def test_manager_has_no_periodic_selection_or_exit_status_mutation():
    text = SOURCE.read_text(encoding="utf-8")
    assert "create_timer" not in text
    assert "update_exit_status" not in text


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class EvaluationClient:
    def __init__(self, response):
        self.response = response
        self.calls = 0

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return True

    def call(self, request, timeout_sec):
        del request, timeout_sec
        self.calls += 1
        return self.response


def payload(revision=4):
    item = ExitEvaluation(
        "EXIT1", "unknown", (2.0, 0.0), (1.5, 0.5), (1, 0),
        True, True, ((0.5, 0.5), (1.5, 0.5)), ((0, 0), (1, 0)),
        1.0, 0.0, 40.0, None, 40.0, None, 0.0, (), 1.0, ("w1",),
    )
    import json
    return json.dumps(
        ExitEvaluationBatch(
            revision, "map", (0.5, 0.5), (item,), 1.0
        ).to_dict()
    )


def manager(activate, revision=4, *, evaluation_success=True):
    evaluation_response = Trigger.Response()
    evaluation_response.success = evaluation_success
    evaluation_response.message = (
        payload(revision) if evaluation_success else "HAZARD_NOT_READY"
    )
    value = SimpleNamespace(
        enabled=True,
        evaluation_client=EvaluationClient(evaluation_response),
        timeout=1.0,
        map_frame="map",
        planner=EvacuationPlanner(),
        risk_first=False,
        activate=activate,
        current_hazard_revision=revision,
        plan_publisher=Publisher(),
        selected_publisher=Publisher(),
        goal_publisher=Publisher(),
        statuses=[],
        get_clock=lambda: Clock(),
    )
    value._status = lambda status: value.statuses.append(status)
    value._failure = MethodType(EvacuationManagerNode._failure, value)
    value._select_and_activate = MethodType(
        EvacuationManagerNode._select_and_activate, value
    )
    return value


def test_on_demand_manager_calls_stage4_and_does_not_activate_when_disabled():
    value = manager(False)
    response = EvacuationManagerNode._plan(
        value, Trigger.Request(), Trigger.Response()
    )
    assert response.success
    assert value.evaluation_client.calls == 1
    assert value.selected_publisher.messages[0].data == "EXIT1"
    assert not value.goal_publisher.messages


def test_activation_publishes_only_selected_approach_to_existing_goal_topic():
    value = manager(True)
    response = EvacuationManagerNode._plan(
        value, Trigger.Request(), Trigger.Response()
    )
    assert response.success
    goal = value.goal_publisher.messages[0]
    assert goal.header.frame_id == "map"
    assert (goal.pose.position.x, goal.pose.position.y) == (1.5, 0.5)


def test_newer_revision_activates_but_older_or_not_ready_never_publishes_goal():
    newer = manager(True, revision=4)
    newer.current_hazard_revision = 5
    response = EvacuationManagerNode._plan(
        newer, Trigger.Request(), Trigger.Response()
    )
    assert response.success
    assert newer.statuses[-1] == "ROUTE_ACTIVATED"
    assert len(newer.goal_publisher.messages) == 1

    older = manager(True, revision=4)
    older.current_hazard_revision = 3
    response = EvacuationManagerNode._plan(
        older, Trigger.Request(), Trigger.Response()
    )
    assert not response.success
    assert older.statuses[-1] == "EVALUATION_STALE"
    assert not older.goal_publisher.messages

    not_ready = manager(True, evaluation_success=False)
    response = EvacuationManagerNode._plan(
        not_ready, Trigger.Request(), Trigger.Response()
    )
    assert not response.success
    assert not_ready.statuses[-1].startswith("EXIT_EVALUATOR_NOT_READY")
    assert not not_ready.goal_publisher.messages
