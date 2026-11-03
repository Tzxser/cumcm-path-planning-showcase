"""Action-log metrics shared by real and offline runs."""

from __future__ import annotations

import math
from collections import defaultdict
from typing import Any


def _point(action: dict[str, Any]) -> tuple[float, float] | None:
    request = action.get("request") or {}
    position = request.get("position")
    if not isinstance(position, dict):
        return None
    try:
        return float(position["x"]), float(position["y"])
    except (KeyError, TypeError, ValueError):
        return None


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def summarize_actions(actions: list[dict[str, Any]], strategy_result: dict[str, Any] | None = None) -> dict[str, Any]:
    accepted = [action for action in actions if action.get("accepted") is True]
    position = (0.0, 0.0)
    tuner = 1
    total_distance = 0.0
    max_move = 0.0
    moves = 0
    distance_by_phase: dict[str, float] = defaultdict(float)
    switch_count = 0
    first_detect: dict[int, dict[str, Any]] = {}
    clear_time: dict[int, float] = {}
    bearings_before_clear: dict[int, int] = defaultdict(int)
    failed_measure_after: dict[int, int] = defaultdict(int)
    failed_clears: dict[int, int] = defaultdict(int)
    movement_after_detect: dict[int, float] = defaultdict(float)
    active_detected: set[int] = set()
    for action in accepted:
        point = _point(action)
        if point is not None:
            leg = _distance(position, point)
            total_distance += leg
            max_move = max(max_move, leg)
            if leg > 1e-9:
                moves += 1
            context = action.get("context") or {}
            category = str(context.get("distance_category") or context.get("phase") or "other")
            distance_by_phase[category] += leg
            for channel in active_detected:
                movement_after_detect[channel] += leg
            position = point
        path = action.get("path")
        request = action.get("request") or {}
        response = action.get("response") or {}
        channel = request.get("channel")
        if path == "/measure" and isinstance(channel, int):
            if channel != tuner:
                switch_count += 1
            tuner = channel
            result = response.get("measure_result")
            if result in {"direction", "near"} and channel not in first_detect:
                first_detect[channel] = {
                    "first_detect_time": action.get("virtual_time_s"),
                    "first_detect_position": list(point) if point else None,
                }
                active_detected.add(channel)
            if result == "direction" and channel not in clear_time:
                bearings_before_clear[channel] += 1
            if result == "no_signal" and channel in first_detect and channel not in clear_time:
                failed_measure_after[channel] += 1
        elif path == "/clear" and isinstance(channel, int):
            result = response.get("clear_result")
            if result == "success" and channel not in clear_time:
                value = action.get("virtual_time_s")
                if isinstance(value, (int, float)):
                    clear_time[channel] = float(value)
                active_detected.discard(channel)
            elif result == "no_target_in_range":
                failed_clears[channel] += 1

    measure_actions = [a for a in accepted if a.get("path") == "/measure"]
    clear_actions = [a for a in accepted if a.get("path") == "/clear"]
    direction_count = sum((a.get("response") or {}).get("measure_result") == "direction" for a in measure_actions)
    near_count = sum((a.get("response") or {}).get("measure_result") == "near" for a in measure_actions)
    no_signal_count = sum((a.get("response") or {}).get("measure_result") == "no_signal" for a in measure_actions)
    clear_success = sum((a.get("response") or {}).get("clear_result") == "success" for a in clear_actions)
    clear_failed = sum((a.get("response") or {}).get("clear_result") == "no_target_in_range" for a in clear_actions)
    per_channel: dict[str, Any] = {}
    for channel in sorted(set(first_detect) | set(clear_time) | set(failed_clears)):
        first = first_detect.get(channel, {})
        start = first.get("first_detect_time")
        end = clear_time.get(channel)
        per_channel[str(channel)] = {
            **first,
            "bearing_count_before_clear": bearings_before_clear[channel],
            "movement_after_first_detect_m": movement_after_detect[channel],
            "time_first_detect_to_clear_s": (end - float(start)) if end is not None and isinstance(start, (int, float)) else None,
            "failed_measures_after_detect": failed_measure_after[channel],
            "failed_clears": failed_clears[channel],
        }
    final_time = next((a.get("virtual_time_s") for a in reversed(accepted) if isinstance(a.get("virtual_time_s"), (int, float))), None)
    result: dict[str, Any] = {
        "accepted_action_count": len(accepted),
        "accepted_false_count": sum(a.get("accepted") is False for a in actions),
        "virtual_time_s": final_time,
        "total_distance_m": total_distance,
        "move_time_s": total_distance / 5.0,
        "max_single_move_m": max_move,
        "number_of_moves": moves,
        "measure_count": len(measure_actions),
        "measure_time_s": 5.0 * len(measure_actions),
        "direction_count": direction_count,
        "near_count": near_count,
        "no_signal_count": no_signal_count,
        "channel_switch_count": switch_count,
        "channel_switch_time_s": float(switch_count),
        "direction_rate": direction_count / len(measure_actions) if measure_actions else 0.0,
        "clear_calls": len(clear_actions),
        "clear_success": clear_success,
        "cleared": clear_success,
        "clear_failed": clear_failed,
        "clear_time_s": 5.0 * clear_success + 3.0 * clear_failed,
        "clear_success_rate": clear_success / len(clear_actions) if clear_actions else 0.0,
        "distance_by_phase_m": dict(sorted(distance_by_phase.items())),
        "distance_during_discovery": sum(v for k, v in distance_by_phase.items() if "discover" in k or "scout" in k or "curve" in k),
        "distance_during_localization": sum(v for k, v in distance_by_phase.items() if "local" in k or "measure" in k or "home" in k),
        "distance_during_clear": sum(v for k, v in distance_by_phase.items() if "clear" in k or "sweep" in k),
        "distance_returning_to_search_route": sum(v for k, v in distance_by_phase.items() if "return" in k),
        "per_channel": per_channel,
    }
    if isinstance(final_time, (int, float)) and clear_success:
        result["avg_time_per_source"] = float(final_time) / clear_success
    if strategy_result:
        result.update(strategy_result)
        backbone = strategy_result.get("planned_backbone_distance")
        if isinstance(backbone, (int, float)):
            result["actual_distance"] = total_distance
            result["detour_distance"] = total_distance - float(backbone)
    return result
