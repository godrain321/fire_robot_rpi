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
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    value = SimpleNamespace(
        goal=goal,
        _dirty=dirty,
        _replan_requested=False,
        _replan_reason="",
        _planning=False,
        periodic_replanning_enabled=periodic_replanning_enabled,
        replan_rate=1.0,
        _last_plan=0.0,
        goal_duplicate_tolerance=0.01,
        map_frame="map",
        state_publisher=Publisher(),
        get_logger=lambda: SimpleNamespace(
            error=lambda *a, **k: None, debug=lambda *a, **k: None
        ),
        get_clock=lambda: clock,
    )
    value.calls = calls
    value._plan = lambda reason: calls.append(reason)
    value._state = MethodType(AstarReplanner._state, value)
    value._same_goal = MethodType(AstarReplanner._same_goal, value)
    value._request_replan = MethodType(AstarReplanner._request_replan, value)
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


def test_1_periodic_enabled_dirty_grid_alone_remains_periodic():
    value = node(periodic_replanning_enabled=True, goal=goal_message(), dirty=True)
    AstarReplanner._timer_callback(value)
    assert value.calls == ["PERIODIC"]


# -- Test 2: Stage 6 enabled -> plain timer ticks never call _plan -------------

def test_2_periodic_disabled_ticks_never_call_plan():
    value = node(periodic_replanning_enabled=False, goal=goal_message())
    for _ in range(5):
        AstarReplanner._timer_callback(value)
    assert value.calls == []


def test_pending_initial_goal_retries_when_periodic_replanning_is_disabled():
    value = node(periodic_replanning_enabled=False, goal=goal_message())
    value._goal_plan_pending = True

    AstarReplanner._timer_callback(value)

    assert value.calls == ["PENDING_GOAL_READINESS"]


# -- Test 3: Stage 6 enabled + dirty grid -> still no timer-triggered replan ---

def test_3_periodic_disabled_dirty_grid_without_path_event_never_calls_plan():
    value = node(periodic_replanning_enabled=False, goal=goal_message(), dirty=True)
    for _ in range(5):
        AstarReplanner._timer_callback(value)
    assert value.calls == []


def test_3_periodic_disabled_path_event_calls_plan_once():
    value = node(periodic_replanning_enabled=False, goal=goal_message(), dirty=True)
    value._replan_requested = True
    value._replan_reason = "THERMAL_PATH_RISK_INCREASED"
    AstarReplanner._timer_callback(value)
    AstarReplanner._timer_callback(value)
    assert value.calls == ["THERMAL_PATH_RISK_INCREASED"]


def test_3_event_is_deferred_until_rate_limit_allows_it():
    value = node(periodic_replanning_enabled=False, goal=goal_message(), dirty=True)
    value._replan_requested = True
    value._replan_reason = "DYNAMIC_PATH_BLOCKED"
    value._last_plan = 9.5
    AstarReplanner._timer_callback(value)
    assert value.calls == []
    assert value._replan_requested is True


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


# -- Test 5: duplicate goal traffic is ignored unless a grid event is pending --

def test_5_same_goal_republish_without_event_is_ignored():
    value = node(periodic_replanning_enabled=False, goal=goal_message())
    AstarReplanner._goal_callback(value, goal_message())  # same content, republished
    assert value.calls == []


def test_5_same_goal_republish_with_event_is_rate_limited_through_timer():
    value = node(periodic_replanning_enabled=False, goal=goal_message(), dirty=True)
    value._replan_requested = True
    AstarReplanner._goal_callback(value, goal_message())
    assert value.calls == []
    AstarReplanner._timer_callback(value)
    assert value.calls == ["SAME_GOAL_EVENT"]


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
