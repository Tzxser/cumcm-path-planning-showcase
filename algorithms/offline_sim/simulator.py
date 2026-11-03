"""Deterministic offline simulator matching the official hard rules.

The random case distribution in :func:`generate_case` is only a modeling
assumption.  Offline timings must never be presented as official-simulator
performance.
"""

from __future__ import annotations

import hashlib
import json
import math
import random
from dataclasses import dataclass
from typing import Any, Callable

from common.constants import DEFAULT_RULES, OfficialRules
from common.logger import utc_now_iso
from common.protocol import ActionResult, Point, RequestIdGenerator, validate_channel, validate_point


@dataclass
class Source:
    channel: int
    position: Point
    radius_m: float
    kind: str
    direction_deg: float | None = None
    cleared: bool = False

    def distance(self, point: Point) -> float:
        return math.hypot(point.x - self.position.x, point.y - self.position.y)

    def covers(self, point: Point) -> bool:
        if self.distance(point) > self.radius_m + 1e-9:
            return False
        if self.kind == "omni":
            return True
        assert self.direction_deg is not None
        angle = math.radians(self.direction_deg)
        ux, uy = math.cos(angle), math.sin(angle)
        return (point.x - self.position.x) * ux + (point.y - self.position.y) * uy >= -1e-9


def deterministic_bearing_error(point: Point, channel: int, seed: int) -> float:
    """A fixed error field in [-1,1]; equal point/channel/seed gives equal error."""
    payload = f"{seed}|{point.x:.9f}|{point.y:.9f}|{channel}".encode("ascii")
    integer = int.from_bytes(hashlib.blake2b(payload, digest_size=8).digest(), "big")
    return 2.0 * integer / (2**64 - 1) - 1.0


class OfflineSession:
    def __init__(
        self,
        sources: list[Source],
        action_sink: Callable[[dict[str, Any]], None],
        seed: int = 0,
        rules: OfficialRules = DEFAULT_RULES,
    ) -> None:
        self.sources = sources
        self.rules = rules
        self.seed = seed
        self._sink = action_sink
        self._ids = RequestIdGenerator("q4off")
        self.position = Point(0.0, 0.0)
        self.current_channel = 1
        self.virtual_time_s: float | None = 0.0
        self.remaining_real_duration_s: float | None = 1200.0
        self.entered = False
        self.transport_failed = False
        self.exit_attempted = False
        self.exit_succeeded = False

    def can_continue(self) -> bool:
        return (self.virtual_time_s or 0.0) < self.rules.virtual_time_limit_s

    def enter(self, context: dict[str, Any] | None = None) -> ActionResult:
        self.entered = True
        return self._record("/enter", {}, {"accepted": True, "virtual_time_s": self.virtual_time_s, "remaining_real_duration_s": 1200.0}, context)

    def measure(self, point: Point | tuple[float, float], channel: int, context: dict[str, Any] | None = None) -> ActionResult:
        point = point if isinstance(point, Point) else Point(*map(float, point))
        validate_point(point, self.rules)
        validate_channel(channel, self.rules)
        self._move(point)
        self.virtual_time_s = float(self.virtual_time_s or 0.0) + self.rules.measure_time_s
        if channel != self.current_channel:
            self.virtual_time_s += self.rules.switch_time_s
        self.current_channel = channel
        source = next((item for item in self.sources if item.channel == channel and not item.cleared), None)
        response: dict[str, Any] = {"accepted": True, "virtual_time_s": self.virtual_time_s}
        if source is None or not source.covers(point):
            response["measure_result"] = "no_signal"
        elif source.distance(point) <= self.rules.near_radius_m + 1e-9:
            response["measure_result"] = "near"
        else:
            truth = math.degrees(math.atan2(source.position.y - point.y, source.position.x - point.x)) % 360.0
            response["measure_result"] = "direction"
            response["svd_deg"] = (truth + deterministic_bearing_error(point, channel, self.seed)) % 360.0
        return self._record("/measure", {"position": point.payload(), "channel": channel}, response, context)

    def clear(self, point: Point | tuple[float, float], channel: int, context: dict[str, Any] | None = None) -> ActionResult:
        point = point if isinstance(point, Point) else Point(*map(float, point))
        validate_point(point, self.rules)
        validate_channel(channel, self.rules)
        self._move(point)
        source = next((item for item in self.sources if item.channel == channel and not item.cleared), None)
        success = source is not None and source.distance(point) <= self.rules.clear_radius_m + 1e-9
        if success:
            source.cleared = True
            self.virtual_time_s = float(self.virtual_time_s or 0.0) + self.rules.clear_success_time_s
        else:
            self.virtual_time_s = float(self.virtual_time_s or 0.0) + self.rules.clear_failure_time_s
        response = {
            "accepted": True,
            "virtual_time_s": self.virtual_time_s,
            "clear_result": "success" if success else "no_target_in_range",
        }
        return self._record("/clear", {"position": point.payload(), "channel": channel}, response, context)

    def exit(self, context: dict[str, Any] | None = None) -> ActionResult:
        self.exit_attempted = True
        self.exit_succeeded = True
        self.entered = False
        return self._record("/exit", {}, {"accepted": True, "virtual_time_s": self.virtual_time_s, "exit_reason": "user_exit"}, context)

    def best_effort_exit(self, context: dict[str, Any] | None = None) -> None:
        if self.entered and not self.exit_attempted:
            self.exit(context)

    def _move(self, point: Point) -> None:
        distance = math.hypot(point.x - self.position.x, point.y - self.position.y)
        self.virtual_time_s = float(self.virtual_time_s or 0.0) + distance / self.rules.speed_mps
        self.position = point

    def _record(self, path: str, fields: dict[str, Any], response: dict[str, Any], context: dict[str, Any] | None) -> ActionResult:
        seq, request_id = self._ids.issue(path)
        request = {"arena_id": self.rules.arena_id, "robot_id": "OFFLINE", "request_id": request_id, **fields}
        result = ActionResult(seq, path, request_id, request, 200, response, True, 0, utc_now_iso(), context=context)
        self._sink({
            "seq": seq,
            "path": path,
            "request_id": request_id,
            "request": request,
            "http_status": 200,
            "response": response,
            "accepted": True,
            "retry_count": 0,
            "wall_time": result.wall_time,
            "virtual_time_s": result.virtual_time_s,
            "error": None,
            "attempt_errors": [],
            "context": context or {},
        })
        return result


def generate_case(
    seed: int,
    n_sources: int | None = None,
    mode: str = "mixed",
    scenario: str = "random",
    rules: OfficialRules = DEFAULT_RULES,
) -> list[Source]:
    """Generate a reproducible assumed distribution or named stress scenario."""
    rng = random.Random(seed)
    n = n_sources if n_sources is not None else rng.randint(rules.source_count_min, rules.source_count_max)
    if not rules.source_count_min <= n <= rules.source_count_max:
        raise ValueError("n_sources must be in 10..16")
    channels = rng.sample(range(rules.channel_min, rules.channel_max + 1), n)
    if mode == "mixed":
        directional_count = rng.randint(1, n - 1)
    elif mode == "directional-only":
        directional_count = n
    elif mode == "omni-only":
        directional_count = 0
    else:
        raise ValueError("mode must be mixed, directional-only, or omni-only")
    directional_channels = set(rng.sample(channels, directional_count))
    sources: list[Source] = []
    for index, channel in enumerate(channels):
        if scenario == "boundary":
            radius = rules.arena_radius_m * (0.96 + 0.04 * rng.random())
        elif scenario == "clustered":
            radius = 250.0 * math.sqrt(rng.random())
        else:
            radius = rules.arena_radius_m * math.sqrt(rng.random())
        angle = 2.0 * math.pi * rng.random()
        position = Point(radius * math.cos(angle), radius * math.sin(angle))
        receive_radius = rules.signal_radius_min_m if scenario in {"r1000", "boundary"} else rng.uniform(rules.signal_radius_min_m, rules.signal_radius_max_m)
        kind = "directional" if channel in directional_channels else "omni"
        if kind == "directional":
            if scenario == "outward":
                direction = math.degrees(angle) % 360.0
            elif scenario == "adversarial-orientation":
                direction = (math.degrees(angle) + 180.0 + 17.0 * (index % 3 - 1)) % 360.0
            else:
                direction = rng.random() * 360.0
        else:
            direction = None
        sources.append(Source(channel, position, receive_radius, kind, direction))
    return sources


def clone_sources(sources: list[Source]) -> list[Source]:
    return [Source(item.channel, Point(*item.position.pair()), item.radius_m, item.kind, item.direction_deg, False) for item in sources]
