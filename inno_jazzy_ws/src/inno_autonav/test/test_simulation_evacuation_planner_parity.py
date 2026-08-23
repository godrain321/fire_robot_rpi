from pathlib import Path
from types import SimpleNamespace
import sys

from inno_autonav.evacuation_planner import EvacuationPlanner
from inno_autonav.exit_evaluator import ExitEvaluation, ExitEvaluationBatch


SIMULATION = (
    Path(__file__).resolve().parents[5]
    / "fire_robot" / "simulator" / "factory_v5"
)
sys.path.insert(0, str(SIMULATION))

from planner.evacuation_planner import EvacuationPlanner as SimPlanner  # noqa: E402
from planner.exit_evaluator import ExitEvaluation as SimEvaluation  # noqa: E402


def ros_evaluation(exit_id, status, length, risk, accepted=True):
    return ExitEvaluation(
        exit_id, status, (2.0, 0.0), (1.5, 0.5), (1, 0), True,
        accepted, ((0.5, 0.5), (1.5, 0.5)), ((0, 0), (1, 0)),
        length, risk, 40.0, None, 40.0, None, 0.5, (), 2.0, ("w1",),
    )


def sim_evaluation(exit_id, status, length, risk, accepted=True):
    return SimEvaluation(
        exit_id, status, (2.0, 0.0), (1.5, 0.5), (1, 0), True,
        accepted, ((0.5, 0.5), (1.5, 0.5)), ((0, 0), (1, 0)),
        length, risk, 40.0, None, 40.0, None, 0.5, (), 2.0, ("w1",),
    )


class StubEvaluator:
    def __init__(self, values):
        self.values = values

    def evaluate(self, item, start, **kwargs):
        del start, kwargs
        return self.values[item.exit_id]


def simulation_plan(evaluations, *, risk_first=False):
    values = {item.exit_id: item for item in evaluations}
    exits = tuple(SimpleNamespace(exit_id=item.exit_id) for item in evaluations)
    return SimPlanner(StubEvaluator(values)).plan(
        exits, (0.5, 0.5), cost_map=None, static_obstacle_map=None,
        dynamic_obstacle_map=None, estimated_fire_map=None,
        created_at=2.0, risk_first=risk_first,
    )


def ros_plan(evaluations, *, risk_first=False):
    return EvacuationPlanner().plan(
        ExitEvaluationBatch(17, "map", (0.5, 0.5), tuple(evaluations), 2.0),
        risk_first=risk_first,
    )


def test_default_confirmed_and_risk_first_selection_match_simulation():
    cases = (
        (("unknown", 10.0, 10.0), ("unknown", 12.0, 1.0), False),
        (("unknown", 5.0, 1.0), ("usable", 12.0, 4.0), False),
        (("unknown", 10.0, 10.0), ("unknown", 12.0, 1.0), True),
    )
    for first, second, risk_first in cases:
        ros_values = (
            ros_evaluation("EXIT1", *first),
            ros_evaluation("EXIT2", *second),
        )
        sim_values = (
            sim_evaluation("EXIT1", *first),
            sim_evaluation("EXIT2", *second),
        )
        ros = ros_plan(ros_values, risk_first=risk_first)
        sim = simulation_plan(sim_values, risk_first=risk_first)
        assert ros.success == sim.success
        assert ros.selected_exit_id == sim.selected_exit_id
        assert ros.selected_approach_position_world == sim.selected_approach_position_world
        assert ros.selection_reason == sim.selection_reason


def test_no_safe_exit_failure_matches_simulation():
    ros = ros_plan((ros_evaluation("EXIT1", "unknown", 1, 1, False),))
    sim = simulation_plan((sim_evaluation("EXIT1", "unknown", 1, 1, False),))
    assert not ros.success and not sim.success
    assert ros.failure_reason.value == sim.failure_reason.value
