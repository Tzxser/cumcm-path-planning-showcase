"""Strict protocol primitives shared by every independent strategy."""

from __future__ import annotations

import math
import threading
import unicodedata
from dataclasses import dataclass
from typing import Any

from .constants import DEFAULT_RULES, OfficialRules


ALLOWED_PATHS = frozenset({"/enter", "/measure", "/clear", "/exit"})


class Q4Error(RuntimeError):
    """Base class for controlled Q4 failures."""


class PayloadValidationError(Q4Error):
    pass


class ResponseProtocolError(Q4Error):
    pass


class ActionRejectedError(Q4Error):
    def __init__(self, result: "ActionResult") -> None:
        super().__init__(
            f"{result.path} {result.request_id} rejected "
            f"(HTTP={result.http_status!r}, response={result.response!r})"
        )
        self.result = result


class TransportError(Q4Error):
    def __init__(self, request_id: str, errors: list[str]) -> None:
        super().__init__(f"transport failed for {request_id}: {'; '.join(errors)}")
        self.request_id = request_id
        self.errors = tuple(errors)


@dataclass(frozen=True, order=True)
class Point:
    x: float
    y: float

    def payload(self) -> dict[str, float]:
        return {"x": float(self.x), "y": float(self.y)}

    def pair(self) -> tuple[float, float]:
        return (float(self.x), float(self.y))


@dataclass(frozen=True)
class ActionResult:
    seq: int
    path: str
    request_id: str
    request: dict[str, Any]
    http_status: int | None
    response: dict[str, Any] | None
    accepted: bool
    retry_count: int
    wall_time: str
    error: str | None = None
    attempt_errors: tuple[str, ...] = ()
    context: dict[str, Any] | None = None

    @property
    def virtual_time_s(self) -> float | None:
        if not self.accepted or not self.response:
            return None
        value = self.response.get("virtual_time_s")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        converted = float(value)
        return converted if math.isfinite(converted) else None

    @property
    def measure_result(self) -> str | None:
        return None if not self.response else self.response.get("measure_result")

    @property
    def clear_result(self) -> str | None:
        return None if not self.response else self.response.get("clear_result")

    @property
    def svd_deg(self) -> float | None:
        if not self.response:
            return None
        value = self.response.get("svd_deg")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        return float(value) % 360.0


class RequestIdGenerator:
    """Monotonic, unique, privacy-free request IDs."""

    def __init__(self, prefix: str = "q4") -> None:
        self._prefix = prefix
        self._seq = 0
        self._lock = threading.Lock()

    def issue(self, path: str) -> tuple[int, str]:
        if path not in ALLOWED_PATHS:
            raise PayloadValidationError(f"unsupported endpoint: {path}")
        with self._lock:
            self._seq += 1
            seq = self._seq
        return seq, f"{self._prefix}-{path[1:]}-{seq:06d}"


def validate_robot_id(robot_id: str) -> None:
    if not isinstance(robot_id, str):
        raise PayloadValidationError("robot_id must be text")
    try:
        encoded = robot_id.encode("utf-8")
    except UnicodeEncodeError as exc:
        raise PayloadValidationError("robot_id must be valid UTF-8 text") from exc
    if not 1 <= len(encoded) <= 64:
        raise PayloadValidationError("robot_id must occupy 1..64 UTF-8 bytes")
    if not robot_id.strip():
        raise PayloadValidationError("robot_id cannot be blank")
    if any(unicodedata.category(ch) in {"Cc", "Cf", "Cs"} for ch in robot_id):
        raise PayloadValidationError("robot_id contains a control/format character")


def validate_channel(channel: int, rules: OfficialRules = DEFAULT_RULES) -> None:
    if isinstance(channel, bool) or not isinstance(channel, int):
        raise PayloadValidationError("channel must be an integer")
    if not rules.channel_min <= channel <= rules.channel_max:
        raise PayloadValidationError("channel must be in 1..20")


def validate_point(point: Point, rules: OfficialRules = DEFAULT_RULES) -> None:
    for name, value in (("x", point.x), ("y", point.y)):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise PayloadValidationError(f"position.{name} must be numeric")
        if not math.isfinite(float(value)):
            raise PayloadValidationError(f"position.{name} must be finite")
        if abs(float(value)) > rules.coordinate_limit:
            raise PayloadValidationError(f"position.{name} exceeds the protocol limit")


def require_accepted(result: ActionResult) -> ActionResult:
    if result.http_status != 200 or result.accepted is not True:
        raise ActionRejectedError(result)
    return result
