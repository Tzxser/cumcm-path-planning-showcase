"""Short-range Q3 measurement proposals; this module performs no actions.

The input polygon must conservatively contain the target and measurements must be
accepted observations for the same channel. All returned points carry an explicit
signal certificate. A direction measured at a MEC center contracts the enclosing
radius by at least the factor 1/(2*cos(ERROR_DEG)); the controller owns iteration
limits, near handling, and the final conservative-MEC clear decision.
"""
from __future__ import annotations

import math
from typing import Any

from geometry_core import ERROR_DEG, diameter_axis, dist, mec

Point = tuple[float, float]
SafetyNote = dict[str, Any]
SAFE_RANGE_M = 999.0
REPEATED_POINT_M = 1.0e-5


def _point(value: Any) -> Point:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError("a point must contain exactly two coordinates")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value):
        raise ValueError("point coordinates must be numeric")
    result = (float(value[0]), float(value[1]))
    if not all(math.isfinite(v) and abs(v) <= 2_000_000 for v in result):
        raise ValueError("point coordinates must be finite and in protocol range")
    return result


def _max_range(point: Point, polygon: list[Point]) -> float:
    # A convex distance function attains its maximum at a polygon vertex.
    return max(dist(point, vertex) for vertex in polygon) + 1.0e-7


def _repeated(point: Point, previous: list[Point]) -> bool:
    return any(dist(point, old) <= REPEATED_POINT_M for old in previous)


def _note(kind: str, point: Point, polygon: list[Point], reason: str, **more: Any) -> SafetyNote:
    return {
        "kind": kind,
        "guaranteed_signal": True,
        "max_feasible_distance_m": _max_range(point, polygon),
        "reason": reason,
        **more,
    }


def _center_fallback(
    polygon: list[Point], center: Point, current: Point, previous: list[Point],
    fallback_reason: str | None = None,
) -> tuple[Point, SafetyNote]:
    if _max_range(center, polygon) <= SAFE_RANGE_M and not _repeated(center, previous):
        return center, _note(
            "MEC_CENTER", center, polygon,
            "MEC center encloses the complete feasible set inside minimum receive radius",
            contraction_factor_bound=1.0/(2*math.cos(math.radians(ERROR_DEG))),
            fallback_reason=fallback_reason,
        )

    # This branch is exceptional: a true nondegenerate narrow-wedge set cannot
    # retain its old measurement point as its MEC center. Finite offsets protect
    # against duplicate input history / numerical degeneracy without averaging.
    candidates: list[tuple[float, float, int, Point]] = []
    for step in (2.0, 5.0, 10.0, 20.0):
        for direction in range(8):
            phi = direction * math.pi/4
            point = (center[0]+step*math.cos(phi), center[1]+step*math.sin(phi))
            if not _repeated(point, previous) and _max_range(point, polygon) <= SAFE_RANGE_M:
                candidates.append((dist(current, point), step, direction, point))
    if not candidates:
        raise ValueError("NO_SAFE_DISTINCT_LOCAL_POINT")
    _, step, _, chosen = min(candidates)
    return chosen, _note(
        "LOCAL_DUPLICATE_RECOVERY", chosen, polygon,
        "finite distinct offset validated against every feasible vertex",
        offset_m=step, fallback_reason=fallback_reason or "MEC center already measured",
    )


def choose_local_point(
    polygon: list[Point], current: Point, measurements: list[dict[str, Any]],
    mode: str = "center", ray_step: float = 250,
) -> tuple[Point, SafetyNote]:
    """Return a safe new measure location and a structured certificate.

    ``center`` uses the MEC center; ``ray`` advances from the spatially nearest
    accepted direction origin while MEC radius exceeds150m, then uses the nearer
    of center +/-40m along the normal to the polygon diameter. Rays are bounded
    by1000m and use the source's fixed unknown receive radius, not a false claim
    that the complete new-point range is necessarily below1000m.
    """
    if mode not in {"center", "ray"}:
        raise ValueError(f"unknown localization mode: {mode!r}")
    if isinstance(ray_step, bool) or not isinstance(ray_step, (int, float)) or not math.isfinite(ray_step) or ray_step <= 0:
        raise ValueError("ray_step must be positive and finite")
    vertices = [_point(p) for p in polygon]
    if not vertices:
        raise ValueError("a nonempty conservative polygon is required")
    current = _point(current)
    previous = [_point(m["position"]) for m in measurements if "position" in m]
    center, _ = mec(vertices)
    center = _point(center)
    radius = _max_range(center, vertices)
    if mode == "center":
        return _center_fallback(vertices, center, current, previous)

    if radius > 150:
        directions: list[tuple[float, int, Point, float]] = []
        for index, measurement in enumerate(measurements):
            bearing = measurement.get("svd_deg")
            if measurement.get("result") != "direction" or isinstance(bearing, bool) or not isinstance(bearing, (int, float)) or not math.isfinite(bearing):
                continue
            origin = _point(measurement.get("position"))
            directions.append((dist(current, origin), -index, origin, float(bearing)))
        if directions:
            _, negative_index, origin, bearing = min(directions)
            step = min(float(ray_step), 1000.0)
            theta = math.radians(bearing)
            chosen = _point((origin[0]+step*math.cos(theta), origin[1]+step*math.sin(theta)))
            if not _repeated(chosen, previous):
                # Let d be the range at the old signal point. For 5<d<=1000,
                # convexity in d bounds new range by1000 at interval endpoints.
                # For d>=1000, s<=1000<2000*cos(epsilon) gives new range<=d.
                # Thus new range<=max(1000,d)<=the same fixed unknown R.
                return chosen, _note(
                    "BOUNDED_RAY", chosen, vertices,
                    "new distance <= max(1000, previous signal distance) <= fixed source R",
                    source_measurement_index=-negative_index,
                    source_position=origin, ray_step_m=step,
                    fixed_receive_radius_certificate=True,
                    bearing_error_deg=ERROR_DEG,
                )
        return _center_fallback(
            vertices, center, current, previous,
            "ray unavailable or proposed point already measured",
        )

    axis = diameter_axis(vertices)
    normal = (-axis[1], axis[0])
    candidates = []
    for side in (1, -1):
        point = (center[0]+side*40*normal[0], center[1]+side*40*normal[1])
        if not _repeated(point, previous) and _max_range(point, vertices) <= SAFE_RANGE_M:
            candidates.append((dist(current, point), 0 if side == 1 else 1, point))
    if candidates:
        _, _, chosen = min(candidates)
        return chosen, _note(
            "LOCAL_NORMAL", chosen, vertices,
            "40m diameter-normal point validated against complete feasible polygon",
            offset_m=40.0,
        )
    return _center_fallback(vertices, center, current, previous, "normal candidates already measured")
