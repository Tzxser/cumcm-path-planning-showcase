"""The single live boundary: four exact serial POST JSON endpoints."""

from __future__ import annotations

import http.client
import json
import math
import socket
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Callable
from typing import Any

from .constants import DEFAULT_RULES, OfficialRules
from .logger import utc_now_iso
from .protocol import (
    ALLOWED_PATHS,
    ActionResult,
    RequestIdGenerator,
    ResponseProtocolError,
    TransportError,
    Point,
    require_accepted,
    validate_channel,
    validate_point,
    validate_robot_id,
)


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):  # type: ignore[no-untyped-def]
        return None


class SimulatorClient:
    """Synchronous, state-audited HTTP client with action-level idempotency."""

    def __init__(
        self,
        base_url: str,
        robot_id: str,
        action_sink: Callable[[dict[str, Any]], None],
        rules: OfficialRules = DEFAULT_RULES,
        opener: Any | None = None,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        validate_robot_id(robot_id)
        parsed = urllib.parse.urlsplit(base_url)
        if parsed.scheme not in {"http", "https"} or not parsed.netloc or parsed.path not in {"", "/"} or parsed.query or parsed.fragment:
            raise ValueError("base_url must be a bare HTTP(S) origin")
        self.base_url = base_url.rstrip("/")
        self._robot_id = robot_id
        self.rules = rules
        self._sink = action_sink
        self._opener = opener or urllib.request.build_opener(_NoRedirect())
        self._sleep = sleep
        self._monotonic = monotonic
        self._ids = RequestIdGenerator("q4")
        self._serial = threading.Lock()
        self.position = Point(0.0, 0.0)
        self.current_channel = 1
        self.virtual_time_s: float | None = None
        self.remaining_real_duration_s: float | None = None
        self._soft_deadline: float | None = None
        self.entered = False
        self.transport_failed = False
        self.exit_attempted = False
        self.exit_succeeded = False

    def enter(self, context: dict[str, Any] | None = None) -> ActionResult:
        result = require_accepted(self._post("/enter", {}, context))
        self._accept_virtual_time(result)
        remaining = (result.response or {}).get("remaining_real_duration_s")
        if isinstance(remaining, bool) or not isinstance(remaining, (int, float)):
            raise ResponseProtocolError("accepted /enter lacks remaining_real_duration_s")
        remaining = float(remaining)
        if not math.isfinite(remaining) or remaining < 0.0:
            raise ResponseProtocolError("remaining_real_duration_s is invalid")
        self.entered = True
        self.remaining_real_duration_s = remaining
        self._soft_deadline = self._monotonic() + max(0.0, remaining - self.rules.real_time_guard_s)
        return result

    def measure(self, point: Point | tuple[float, float], channel: int, context: dict[str, Any] | None = None) -> ActionResult:
        point = point if isinstance(point, Point) else Point(*map(float, point))
        validate_point(point, self.rules)
        validate_channel(channel, self.rules)
        result = require_accepted(self._post("/measure", {"position": point.payload(), "channel": channel}, context))
        response = result.response or {}
        kind = response.get("measure_result")
        if kind not in {"direction", "near", "no_signal"}:
            raise ResponseProtocolError(f"invalid measure_result: {kind!r}")
        if kind == "direction":
            bearing = response.get("svd_deg")
            if isinstance(bearing, bool) or not isinstance(bearing, (int, float)) or not math.isfinite(float(bearing)):
                raise ResponseProtocolError("direction result lacks finite svd_deg")
        self._accept_motion(result, point)
        self.current_channel = channel
        return result

    def clear(self, point: Point | tuple[float, float], channel: int, context: dict[str, Any] | None = None) -> ActionResult:
        point = point if isinstance(point, Point) else Point(*map(float, point))
        validate_point(point, self.rules)
        validate_channel(channel, self.rules)
        result = require_accepted(self._post("/clear", {"position": point.payload(), "channel": channel}, context))
        kind = (result.response or {}).get("clear_result")
        if kind not in {"success", "no_target_in_range"}:
            raise ResponseProtocolError(f"invalid clear_result: {kind!r}")
        self._accept_motion(result, point)
        return result

    def exit(self, context: dict[str, Any] | None = None) -> ActionResult:
        self.exit_attempted = True
        result = require_accepted(self._post("/exit", {}, context, attempts=1))
        self._accept_virtual_time(result)
        self.exit_succeeded = (result.response or {}).get("exit_reason") == "user_exit"
        self.entered = False
        return result

    def can_continue(self) -> bool:
        return self._soft_deadline is None or self._monotonic() < self._soft_deadline

    def best_effort_exit(self, context: dict[str, Any] | None = None) -> None:
        if not self.entered or self.transport_failed or self.exit_attempted:
            return
        try:
            self.exit(context or {"phase": "exit"})
        except Exception:  # recorded by _post; exit is deliberately attempted only once
            pass

    def _accept_motion(self, result: ActionResult, point: Point) -> None:
        self._accept_virtual_time(result)
        self.position = point

    def _accept_virtual_time(self, result: ActionResult) -> None:
        value = result.virtual_time_s
        if value is None:
            raise ResponseProtocolError("accepted response lacks finite virtual_time_s")
        if self.virtual_time_s is not None and value + 1e-9 < self.virtual_time_s:
            raise ResponseProtocolError("virtual_time_s moved backwards")
        self.virtual_time_s = value

    def _post(
        self,
        path: str,
        fields: dict[str, Any],
        context: dict[str, Any] | None,
        attempts: int | None = None,
    ) -> ActionResult:
        if path not in ALLOWED_PATHS:
            raise AssertionError(f"forbidden endpoint: {path}")
        seq, request_id = self._ids.issue(path)
        payload = {"arena_id": self.rules.arena_id, "robot_id": self._robot_id, "request_id": request_id, **fields}
        body = json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
        request = urllib.request.Request(self.base_url + path, data=body, headers={"Content-Type": "application/json"}, method="POST")
        errors: list[str] = []
        total = attempts if attempts is not None else self.rules.network_retries + 1
        with self._serial:
            for attempt in range(total):
                stamp = utc_now_iso()
                status: int | None = None
                try:
                    with self._opener.open(request, timeout=self.rules.request_timeout_s) as response_obj:
                        status = int(response_obj.getcode())
                        raw = response_obj.read()
                    response = self._decode(raw, request_id)
                    accepted = status == 200 and response.get("accepted") is True
                    result = ActionResult(seq, path, request_id, payload, status, response, accepted, attempt, stamp, attempt_errors=tuple(errors), context=context)
                    self._sink(self._record(result))
                    return result
                except urllib.error.HTTPError as exc:
                    raw = exc.read()
                    try:
                        response = self._decode(raw, request_id)
                        error = f"HTTPError: {exc.reason}"
                    except ResponseProtocolError as decode_error:
                        response = None
                        error = str(decode_error)
                    result = ActionResult(seq, path, request_id, payload, int(exc.code), response, False, attempt, stamp, error, tuple(errors), context)
                    self._sink(self._record(result))
                    return result
                except ResponseProtocolError as exc:
                    result = ActionResult(seq, path, request_id, payload, status, None, False, attempt, stamp, str(exc), tuple(errors), context)
                    self._sink(self._record(result))
                    raise
                except (urllib.error.URLError, TimeoutError, socket.timeout, ConnectionError, http.client.HTTPException) as exc:
                    errors.append(f"{type(exc).__name__}: {exc}")
                    if attempt + 1 < total:
                        self._sleep(self.rules.retry_backoff_s * (2**attempt))
                        continue
                    self.transport_failed = True
                    result = ActionResult(seq, path, request_id, payload, None, None, False, attempt, stamp, errors[-1], tuple(errors), context)
                    self._sink(self._record(result))
                    raise TransportError(request_id, errors) from exc
        raise AssertionError("unreachable")

    @staticmethod
    def _decode(raw: bytes, request_id: str) -> dict[str, Any]:
        try:
            value = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ResponseProtocolError(f"{request_id} returned non-JSON/UTF-8 data") from exc
        if not isinstance(value, dict):
            raise ResponseProtocolError(f"{request_id} response is not an object")
        return value

    @staticmethod
    def _record(result: ActionResult) -> dict[str, Any]:
        return {
            "seq": result.seq,
            "path": result.path,
            "request_id": result.request_id,
            "request": result.request,
            "http_status": result.http_status,
            "response": result.response,
            "accepted": result.accepted,
            "retry_count": result.retry_count,
            "wall_time": result.wall_time,
            "virtual_time_s": result.virtual_time_s,
            "error": result.error,
            "attempt_errors": list(result.attempt_errors),
            "context": result.context or {},
        }
