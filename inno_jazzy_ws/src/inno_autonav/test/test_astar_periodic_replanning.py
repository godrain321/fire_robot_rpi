"""astar_replanner must not decide replan timing when Stage 6 owns it.

Uses the same fake-self (SimpleNamespace + unbound-method-call) pattern as
test_evacuation_manager_contract.py so the routing logic in _timer_callback/
_goal_callback can be checked without a live rclpy Node (no grids/TF needed --
self._plan is replaced with a spy since these tests are about *whether/how often*
it is called, not what it computes).
"""

from types import MethodType, SimpleNamespace

from geometry_msgs.msg import PoseStamped

from inno_autonav.astar_replanner import AstarReplanner


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def node(*, periodic_replanning_enabled=True, goal=None, dirty=False):
    calls = []
    value = SimpleNamespace(
        goal=goal,
        _dirty=dirty,
        periodic_replanning_enabled=periodic_replanning_enabled,
        map_frame="map",
        state_publisher=Publisher(),
        get_logger=lambda: SimpleNamespace(error=lambda *a, **k: None),
    )
    value.calls = calls
    value._plan = lambda reason: calls.append(reason)
    value._state = MethodType(AstarReplanner._state, value)
    return value


def goal_message(frame_id="map"):
    message = PoseStamped()
    message.header.frame_id = frame_id
    return message


# -- Test 1: Stage 6 disabled -> existing periodic/dirty behavior unchanged ----

def test_1_periodic_enabled_ticks_plan_with_a_goal():
    value = node(periodic_replanning_enabled=True, goal=goal_message())
    AstarReplanner._timer_callback(value)
    assert value.calls == ["PERIODIC"]


def test_1_periodic_enabled_dirty_grid_ticks_plan_with_grid_update():
    value = node(periodic_replanning_enabled=True, goal=goal_message(), dirty=True)
    AstarReplanner._timer_callback(value)
    assert value.calls == ["GRID_UPDATE"]


# -- Test 2: Stage 6 enabled -> plain timer ticks never call _plan -------------

def test_2_periodic_disabled_ticks_never_call_plan():
    value = node(periodic_replanning_enabled=False, goal=goal_message())
    for _ in range(5):
        AstarReplanner._timer_callback(value)
    assert value.calls == []


# -- Test 3: Stage 6 enabled + dirty grid -> still no timer-triggered replan ---

def test_3_periodic_disabled_dirty_grid_never_calls_plan():
    value = node(periodic_replanning_enabled=False, goal=goal_message(), dirty=True)
    for _ in range(5):
        AstarReplanner._timer_callback(value)
    assert value.calls == []


# -- Test 4: a new /goal_pose always plans immediately, regardless of mode -----

def test_4_new_goal_always_plans_even_with_periodic_disabled():
    value = node(periodic_replanning_enabled=False, goal=None)
    AstarReplanner._goal_callback(value, goal_message())
    assert value.calls == ["NEW_GOAL"]
    assert value.goal is not None


def test_4_new_goal_plans_when_periodic_enabled_too():
    value = node(periodic_replanning_enabled=True, goal=None)
    AstarReplanner._goal_callback(value, goal_message())
    assert value.calls == ["NEW_GOAL"]


# -- Test 5: ReplanSupervisor republishing the same goal still plans -----------

def test_5_supervisor_same_goal_republish_plans_immediately_with_periodic_disabled():
    value = node(periodic_replanning_enabled=False, goal=goal_message())
    AstarReplanner._goal_callback(value, goal_message())  # same content, republished
    assert value.calls == ["NEW_GOAL"]


# -- Test 8: no duplicate planning around a single supervisor request ---------

def test_8_periodic_ticks_around_a_single_goal_request_cause_no_extra_planning():
    value = node(periodic_replanning_enabled=False, goal=None, dirty=True)
    # Timer keeps running (Stage 6 doesn't need it stopped, just inert) while
    # nothing has requested a plan yet.
    for _ in range(3):
        AstarReplanner._timer_callback(value)
    assert value.calls == []
    # Stage 6's ReplanSupervisor decides a replan is needed and republishes
    # /goal_pose exactly once.
    AstarReplanner._goal_callback(value, goal_message())
    assert value.calls == ["NEW_GOAL"]
    # Further ticks (e.g. while the new path is being validated) must not add
    # a second, unrequested A* run.
    for _ in range(3):
        AstarReplanner._timer_callback(value)
    assert value.calls == ["NEW_GOAL"]
