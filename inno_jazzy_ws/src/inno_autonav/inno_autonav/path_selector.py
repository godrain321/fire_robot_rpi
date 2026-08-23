"""Pick which upstream path source is currently allowed onto /planned_path.

Deliberately generic (no nav_msgs import here): the node passes the raw ``Path``
message object through as an opaque payload when selected, so no field is ever
recomputed or reconstructed by this module. ``set_mode()`` exists so a future
stage can drive an automatic WAYPOINT->A_STAR fallback; nothing in Stage 8-8
calls it automatically -- the default is a straight WAYPOINT passthrough.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class PathSelectorMode(str, Enum):
    WAYPOINT = "WAYPOINT"
    A_STAR = "A_STAR"


@dataclass(frozen=True)
class PathSelectorOutput:
    publish: bool
    payload: Any | None
    source: str


class PathSelectorCore:
    def __init__(self, mode: PathSelectorMode | str = PathSelectorMode.WAYPOINT) -> None:
        self.mode = PathSelectorMode(mode)
        self._latest: dict[str, Any] = {"waypoint": None, "astar": None}

    def set_mode(self, mode: PathSelectorMode | str) -> None:
        self.mode = PathSelectorMode(mode)

    def on_waypoint_path(self, payload: Any) -> PathSelectorOutput:
        self._latest["waypoint"] = payload
        return self._select("waypoint")

    def on_astar_path(self, payload: Any) -> PathSelectorOutput:
        self._latest["astar"] = payload
        return self._select("astar")

    def _select(self, source: str) -> PathSelectorOutput:
        active_source = "waypoint" if self.mode is PathSelectorMode.WAYPOINT else "astar"
        if source != active_source:
            return PathSelectorOutput(False, None, source)
        return PathSelectorOutput(True, self._latest[source], source)

    def status(self) -> dict:
        return {
            "mode": self.mode.value,
            "has_waypoint_path": self._latest["waypoint"] is not None,
            "has_astar_path": self._latest["astar"] is not None,
        }
