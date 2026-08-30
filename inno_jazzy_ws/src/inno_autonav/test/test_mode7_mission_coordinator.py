from inno_autonav.mode7_mission import Mode7Readiness


def ready():
    return Mode7Readiness(
        localization_ready=True,
        thermal_status="ACTIVE_THERMAL_ONLY",
        exit_evaluator_status="READY",
        evacuation_manager_status="READY",
        pose_ready=True,
        plan_service_ready=True,
    )


def test_plan_is_gated_on_every_required_readiness_signal():
    state = ready()
    assert state.waiting_state() == "READY_TO_PLAN"
    expectations = {
        "localization_ready": "WAITING_FOR_LOCALIZATION",
        "pose_ready": "WAITING_FOR_MAP_TO_BASE_LINK",
        "thermal_status": "WAITING_FOR_THERMAL_HAZARD",
        "exit_evaluator_status": "WAITING_FOR_EXIT_EVALUATOR",
        "evacuation_manager_status": "WAITING_FOR_EVACUATION_MANAGER",
        "plan_service_ready": "WAITING_FOR_PLAN_SERVICE",
    }
    for attribute, expected in expectations.items():
        candidate = ready()
        setattr(candidate, attribute, False if isinstance(getattr(candidate, attribute), bool) else "")
        assert candidate.waiting_state() == expected


def test_only_thermal_only_hazard_status_unlocks_mode7():
    for status in ("", "ACTIVE", "ACTIVE_STATIC_DYNAMIC_ONLY", "THERMAL_STREAM_STALE"):
        state = ready()
        state.thermal_status = status
        assert state.waiting_state() == "WAITING_FOR_THERMAL_HAZARD"
