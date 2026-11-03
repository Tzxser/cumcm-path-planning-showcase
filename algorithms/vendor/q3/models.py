"""Domain models kept independent from HTTP and planning code."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


@dataclass(frozen=True, order=True)
class Point:
    x: float
    y: float

    def as_list(self) -> list[float]:
        return [self.x, self.y]

    def as_payload(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y}


class ChannelStatus(str, Enum):
    UNKNOWN = "UNKNOWN"
    DISCOVERED = "DISCOVERED"
    NEAR_FOUND = "NEAR_FOUND"
    LOCALIZING = "LOCALIZING"
    CLEARED = "CLEARED"
    ABSENT = "ABSENT"


@dataclass
class MeasurementRecord:
    position: Point
    channel: int
    result: str
    svd_deg: float | None
    request_id: str
    phase: str
    virtual_time_s: float | None
    anchor_index: int | None = None
    guaranteed_max_distance_m: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "position": self.position.as_list(),
            "channel": self.channel,
            "result": self.result,
            "svd_deg": self.svd_deg,
            "request_id": self.request_id,
            "phase": self.phase,
            "virtual_time_s": self.virtual_time_s,
            "anchor_index": self.anchor_index,
            "guaranteed_max_distance_m": self.guaranteed_max_distance_m,
        }


@dataclass
class ClearRecord:
    position: Point
    channel: int
    result: str
    request_id: str
    phase: str
    virtual_time_s: float | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "position": self.position.as_list(),
            "channel": self.channel,
            "result": self.result,
            "request_id": self.request_id,
            "phase": self.phase,
            "virtual_time_s": self.virtual_time_s,
        }


@dataclass
class ChannelRecord:
    channel: int
    status: ChannelStatus = ChannelStatus.UNKNOWN
    anchor_results: dict[int, str] = field(default_factory=dict)
    measurements: list[MeasurementRecord] = field(default_factory=list)
    clears: list[ClearRecord] = field(default_factory=list)
    feasible_polygon: list[Point] = field(default_factory=list, repr=False)
    mec_center: Point | None = None
    mec_radius_m: float | None = None
    anomalies: list[str] = field(default_factory=list)
    second_point_alternate: Point | None = None
    primary_measurement_count: int = 0
    recovery_measurement_count: int = 0

    @property
    def direction_measurements(self) -> list[MeasurementRecord]:
        return [m for m in self.measurements if m.result == "direction"]

    def to_dict(self) -> dict[str, Any]:
        return {
            "channel": self.channel,
            "status": self.status.value,
            "anchor_results": {str(k): v for k, v in sorted(self.anchor_results.items())},
            "measurements": [m.to_dict() for m in self.measurements],
            "direction_measurements": [m.to_dict() for m in self.direction_measurements],
            "signal_positions": [m.position.as_list() for m in self.measurements if m.result in {"direction", "near"}],
            "no_signal_positions": [m.position.as_list() for m in self.measurements if m.result == "no_signal"],
            "first_detection_anchor": next((m.anchor_index for m in self.measurements if m.result in {"direction", "near"}), None),
            "homing_upper_bound_U": (self.direction_measurements[-1].guaranteed_max_distance_m or 1500.0) if self.direction_measurements else None,
            "clears": [c.to_dict() for c in self.clears],
            "feasible_region_summary": {
                "vertex_count": len(self.feasible_polygon),
                "vertices": [p.as_list() for p in self.feasible_polygon],
                "mec_center": self.mec_center.as_list() if self.mec_center else None,
                "mec_radius_m": self.mec_radius_m,
            },
            "primary_measurement_count": self.primary_measurement_count,
            "recovery_measurement_count": self.recovery_measurement_count,
            "anomalies": list(self.anomalies),
        }
