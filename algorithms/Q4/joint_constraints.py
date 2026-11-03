"""Strict position cuts from range-certified pairs of Q4 negative readings.

Only one accepted positive direction is supported in this first implementation.
If both negative stations are certainly inside the same source's receive disk,
their entire segment is on the negative side of its emission half-plane. When
that segment crosses every ray in the positive bearing fan, the source cannot
lie across its supporting line from the positive station. The cut is therefore
geometric and does not assume a Gaussian error or a known source orientation.
"""
from __future__ import annotations

from decimal import Decimal
import itertools
import math
from typing import Any

from geometry_core import ERROR_DEG, dist
from q3.geometry import clip_polygon_halfplane
from q3.models import Point as GeometryPoint

Point = tuple[float, float]
_CLIP_MARGIN_M = 1.0e-7
_SEGMENT_MARGIN = 1.0e-6
_LINE_SEPARATION_M = 1.0e-4


def _point(value: Any) -> Point | None:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        return None
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) for v in value):
        return None
    point = float(value[0]), float(value[1])
    return point if all(math.isfinite(v) for v in point) else None


def _range_proof(poly: list[Point], station: Point, bearing: float, negative: Point) -> dict | None:
    furthest = max(dist(negative, p) for p in poly) + 1.0e-7
    if furthest <= 999.0:
        return {"kind": "whole_feasible_set_inside_minimum_radius", "max_distance_m": furthest,
                "minimum_receive_radius_m": 1000.0}

    theta = math.radians(bearing)
    dx, dy = negative[0]-station[0], negative[1]-station[1]
    a = dx*math.cos(theta)+dy*math.sin(theta)
    b = -dx*math.sin(theta)+dy*math.cos(theta)
    norm = math.hypot(dx, dy)
    if norm <= 1.0e-6 or a < 0.0:
        return None
    if abs(norm-1000.0) < 1.0e-5:
        # Near the exact length cap, distinguish true containment from a hypot
        # rounded down to1000. Decimal starts from the actual binary inputs.
        ex = Decimal.from_float(negative[0])-Decimal.from_float(station[0])
        ey = Decimal.from_float(negative[1])-Decimal.from_float(station[1])
        if ex*ex+ey*ey > Decimal(1_000_000):
            return None
    elif norm > 1000.0:
        return None
    epsilon = math.radians(ERROR_DEG)
    minimum_projection = a*math.cos(epsilon)-abs(b)*math.sin(epsilon)
    slack = 2000.0*minimum_projection-(dx*dx+dy*dy)
    # Strict positive numerical reserve; an uncertain equality fails closed.
    if minimum_projection < 0.0 or slack < 1.0e-5:
        return None
    return {"kind": "fixed_unknown_receive_radius_from_positive", "positive_station": station,
            "forward_m": a, "lateral_m": b, "step_length_m": norm,
            "minimum_direction_projection_m": minimum_projection,
            "squared_inequality_slack_m2": slack, "bearing_half_error_deg": ERROR_DEG,
            "claim": "new_distance <= max(1000, old_positive_distance) <= same_fixed_R"}


def _crossing(station: Point, bearing: float, first: Point, second: Point) -> dict | None:
    wx, wy = second[0]-first[0], second[1]-first[1]
    length = math.hypot(wx, wy)
    if not math.isfinite(length) or length < _LINE_SEPARATION_M:
        return None
    normal = wy/length, -wx/length
    offset = normal[0]*(first[0]-station[0])+normal[1]*(first[1]-station[1])
    if abs(offset) < _LINE_SEPARATION_M:
        return None
    if offset < 0:
        normal = -normal[0], -normal[1]
        offset = -offset
    # The retained side is n.(x-S)<=offset; S has value0 and offset is positive.
    crossings = []
    for boundary_bearing in (bearing-ERROR_DEG, bearing+ERROR_DEG):
        angle = math.radians(boundary_bearing)
        ray = math.cos(angle), math.sin(angle)
        denominator = normal[0]*ray[0]+normal[1]*ray[1]
        if denominator <= 1.0e-7:
            return None
        parameter = offset/denominator
        if not math.isfinite(parameter) or parameter <= _LINE_SEPARATION_M:
            return None
        relative = parameter*ray[0], parameter*ray[1]
        from_first = station[0]-first[0]+relative[0], station[1]-first[1]+relative[1]
        fraction = (from_first[0]*wx+from_first[1]*wy)/(length*length)
        if not _SEGMENT_MARGIN < fraction < 1.0-_SEGMENT_MARGIN:
            return None
        crossings.append({"bearing_deg": boundary_bearing, "ray_parameter_m": parameter,
                          "segment_fraction": fraction,
                          "point": (station[0]+relative[0], station[1]+relative[1])})
    return {"normal": normal, "offset_from_positive_m": offset, "crossings": crossings}


def refine_from_negatives(poly: list[Point], measurements: list[dict[str, Any]]) -> tuple[list[Point], list[dict]]:
    """Conservatively refine F, returning only applied, auditable certificates.

    This bounded version requires exactly one direction and no near reading.
    Invalid/degenerate observation geometry, a single negative, uncertain range,
    and non-crossing pairs produce no cut. The official unknown R is fixed for
    the source; this function is not valid if transmit power changes over time.
    """
    polygon = []
    for value in poly:
        p = _point(value)
        if p is None:
            raise ValueError("feasible polygon must contain finite points")
        polygon.append(p)
    if not polygon:
        return [], []
    positives = [(index, m) for index, m in enumerate(measurements) if m.get("result") == "direction"]
    if len(positives) != 1 or any(m.get("result") == "near" for m in measurements):
        return polygon, []
    positive_index, positive = positives[0]
    station = _point(positive.get("position"))
    bearing = positive.get("svd_deg")
    if station is None or isinstance(bearing, bool) or not isinstance(bearing, (float, int)) or not math.isfinite(bearing):
        return polygon, []
    bearing = float(bearing)%360.0
    negatives = [(index, _point(m.get("position"))) for index, m in enumerate(measurements)
                 if m.get("result") == "no_signal"]
    negatives = [(index, p) for index, p in negatives if p is not None]
    certificates = []
    # A strict finite wall-work bound; skipped excess observations only weaken
    # refinement, never its safety. New local services normally have2 negatives.
    for (first_index, first), (second_index, second) in itertools.islice(itertools.combinations(negatives, 2), 512):
        crossing = _crossing(station, bearing, first, second)
        if crossing is None:
            continue
        first_proof = _range_proof(polygon, station, bearing, first)
        second_proof = _range_proof(polygon, station, bearing, second)
        if first_proof is None or second_proof is None:
            continue
        nx, ny = crossing["normal"]
        offset = crossing["offset_from_positive_m"]
        if max(nx*(p[0]-station[0])+ny*(p[1]-station[1]) for p in polygon) <= offset+_CLIP_MARGIN_M:
            continue
        local = [GeometryPoint(p[0]-station[0], p[1]-station[1]) for p in polygon]
        clipped = clip_polygon_halfplane(local, nx, ny, offset, _CLIP_MARGIN_M)
        if not clipped:
            # A real source satisfying all accepted observations cannot vanish.
            # Contradictory or numerically uncertain data must not certify ABSENT.
            continue
        candidate = [(station[0]+p.x, station[1]+p.y) for p in clipped]
        if not all(math.isfinite(v) for p in candidate for v in p):
            continue
        certificates.append({"kind": "range_certified_negative_segment_fan_cut", "applied": True,
            "positive_measurement_index": positive_index,
            "negative_measurement_indices": [first_index, second_index],
            "positive_station": station, "bearing_deg": bearing, "bearing_half_error_deg": ERROR_DEG,
            "negative_stations": [first, second], "range_proofs": [first_proof, second_proof],
            "halfplane": {"a": nx, "b": ny, "c": nx*station[0]+ny*station[1]+offset,
                          "local_origin": station, "local_offset": offset,
                          "retained_side": "a*x+b*y<=c", "expansion_m": _CLIP_MARGIN_M},
            "fan_segment_intersections": crossing["crossings"],
            "proof": "positive-to-target segment cannot intersect a wholly orientation-negative segment"})
        polygon = candidate
    return polygon, certificates
