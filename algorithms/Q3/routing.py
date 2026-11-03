"""Small deterministic open-route optimizer for estimated waypoints.

Only supplied coordinates are used. The caller owns all inference and safety
checks; this module performs no simulator actions and reads no source truth.
"""
from __future__ import annotations

import math
from typing import Sequence

Point = tuple[float, float]
MAX_NODES = 40
MAX_STARTS = 3
MAX_TWO_OPT_PASSES = 8
_IMPROVEMENT_EPS = 1e-9


def _point(value: Sequence[float], name: str) -> Point:
    if len(value) != 2:
        raise ValueError(f"{name} must have two finite coordinates")
    point = (float(value[0]), float(value[1]))
    if not all(math.isfinite(x) for x in point):
        raise ValueError(f"{name} must have two finite coordinates")
    return point


def incremental_detour(start: Point, target: Point, next: Point) -> float:
    """Extra metres for inserting target in the start -> next leg.

    Roundoff may make the triangle-inequality difference slightly negative;
    clamp it to zero. This is travel distance, not virtual seconds.
    """
    start = _point(start, "start")
    target = _point(target, "target")
    next = _point(next, "next")
    result = math.dist(start, target) + math.dist(target, next) - math.dist(start, next)
    if not math.isfinite(result):
        raise ValueError("coordinate distances must be finite")
    return max(0.0, result)


def route_order(start: Point, nodes: list[Point], end: Point | None = None) -> list[int]:
    """Return each node index once, with fixed start and optional fixed end.

    Three deterministic nearest-neighbour seeds are improved by at most eight
    best-improvement 2-opt passes each. The first seed is ordinary NN, so the
    chosen result cannot be worse than NN. Open-route suffix reversal is allowed;
    when end is given its final edge participates in every comparison.

    Duplicate positions remain distinct indexed tasks. Node count is deliberately
    capped at 40, making the search bounded and suitable for frequent replanning.
    Input lists are never mutated. Ties retain the lexicographically first route.
    """
    if len(nodes) > MAX_NODES:
        raise ValueError(f"route_order supports at most {MAX_NODES} nodes")
    start = _point(start, "start")
    checked = [_point(point, f"nodes[{i}]") for i, point in enumerate(nodes)]
    endpoint = _point(end, "end") if end is not None else None
    n = len(checked)
    if not n:
        return []

    # Matrix indices 0..n-1 are tasks, n is the fixed start, n+1 optional end.
    points = checked + [start] + ([endpoint] if endpoint is not None else [])
    distances = [[math.dist(a, b) for b in points] for a in points]
    if any(not math.isfinite(d) for row in distances for d in row):
        raise ValueError("coordinate distances must be finite")

    def cost(order: list[int]) -> float:
        result = distances[n][order[0]]
        result += sum(distances[a][b] for a, b in zip(order, order[1:]))
        if endpoint is not None:
            result += distances[order[-1]][n + 1]
        return result

    seeds = sorted(range(n), key=lambda i: (distances[n][i], i))[:MAX_STARTS]
    best_order: list[int] | None = None
    best_cost = math.inf
    for seed in seeds:
        order = [seed]
        pending = [i for i in range(n) if i != seed]
        while pending:
            current = order[-1]
            chosen = min(pending, key=lambda i: (distances[current][i], i))
            order.append(chosen)
            pending.remove(chosen)

        for _ in range(MAX_TWO_OPT_PASSES):
            improvement = _IMPROVEMENT_EPS
            chosen_pair: tuple[int, int] | None = None
            for left in range(n - 1):
                before = n if left == 0 else order[left - 1]
                first = order[left]
                before_first = distances[before][first]
                for right in range(left + 1, n):
                    last = order[right]
                    gain = before_first - distances[before][last]
                    if right + 1 < n:
                        after = order[right + 1]
                        gain += distances[last][after] - distances[first][after]
                    elif endpoint is not None:
                        gain += distances[last][n + 1] - distances[first][n + 1]
                    # Strict > and index-order loops provide reproducible ties.
                    if gain > improvement:
                        improvement = gain
                        chosen_pair = left, right
            if chosen_pair is None:
                break
            left, right = chosen_pair
            order[left:right + 1] = reversed(order[left:right + 1])

        candidate_cost = cost(order)
        if candidate_cost < best_cost - _IMPROVEMENT_EPS or (
            abs(candidate_cost - best_cost) <= _IMPROVEMENT_EPS
            and (best_order is None or order < best_order)
        ):
            best_order = order
            best_cost = candidate_cost
    assert best_order is not None
    return best_order
