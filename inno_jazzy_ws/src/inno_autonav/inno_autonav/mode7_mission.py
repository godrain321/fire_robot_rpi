"""ROS-independent Mode 7 startup readiness policy."""

from dataclasses import dataclass


@dataclass
class Mode7Readiness:
    localization_ready: bool = False
    thermal_status: str = ""
    exit_evaluator_status: str = ""
    evacuation_manager_status: str = ""
    pose_ready: bool = False
    plan_service_ready: bool = False

    def waiting_state(self) -> str:
        if not self.localization_ready:
            return "WAITING_FOR_LOCALIZATION"
        if not self.pose_ready:
            return "WAITING_FOR_MAP_TO_BASE_LINK"
        if self.thermal_status != "ACTIVE_THERMAL_ONLY":
            return "WAITING_FOR_THERMAL_HAZARD"
        if self.exit_evaluator_status != "READY":
            return "WAITING_FOR_EXIT_EVALUATOR"
        if self.evacuation_manager_status != "READY":
            return "WAITING_FOR_EVACUATION_MANAGER"
        if not self.plan_service_ready:
            return "WAITING_FOR_PLAN_SERVICE"
        return "READY_TO_PLAN"
