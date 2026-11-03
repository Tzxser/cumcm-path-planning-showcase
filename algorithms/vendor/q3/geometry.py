"""Conservative convex geometry and a standard-library-only MEC solver."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Iterable, Sequence

from .models import Point


def distance(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def add(a: Point, b: Point) -> Point:
    return Point(a.x + b.x, a.y + b.y)


def subtract(a: Point, b: Point) -> Point:
    return Point(a.x - b.x, a.y - b.y)


def scale(a: Point, factor: float) -> Point:
    return Point(a.x * factor, a.y * factor)


def dot(a: Point, b: Point) -> float:
    return a.x * b.x + a.y * b.y


def cross(a: Point, b: Point) -> float:
    return a.x * b.y - a.y * b.x


def unit_from_degrees(angle_deg: float) -> Point:
    angle = math.radians(angle_deg)
    return Point(math.cos(angle), math.sin(angle))


def circumscribed_regular_polygon(center: Point, radius: float, sides: int) -> list[Point]:
    """Return the intersection polygon of tangent half-planes around a circle.

    Vertices are at radius/cos(pi/n), so the true circle is contained by the
    polygon rather than approximated from inside.
    """
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    if sides < 3:
        raise ValueError("sides must be at least 3")
    vertex_radius = radius / math.cos(math.pi / sides)
    return [
        Point(
            center.x + vertex_radius * math.cos(2.0 * math.pi * (k + 0.5) / sides),
            center.y + vertex_radius * math.sin(2.0 * math.pi * (k + 0.5) / sides),
        )
        for k in range(sides)
    ]


def clip_polygon_halfplane(
    polygon: Sequence[Point],
    a: float,
    b: float,
    c: float,
    epsilon: float = 1.0e-9,
) -> list[Point]:
    """Clip a convex polygon to a*x+b*y <= c, expanded by epsilon."""
    if not polygon:
        return []
    norm = math.hypot(a, b)
    if norm == 0.0:
        raise ValueError("half-plane normal cannot be zero")
    limit = c + epsilon * norm

    def value(point: Point) -> float:
        return a * point.x + b * point.y - limit

    output: list[Point] = []
    previous = polygon[-1]
    previous_value = value(previous)
    previous_inside = previous_value <= 0.0
    for current in polygon:
        current_value = value(current)
        current_inside = current_value <= 0.0
        if current_inside != previous_inside:
            denominator = previous_value - current_value
            if abs(denominator) > 1.0e-18:
                t = previous_value / denominator
                output.append(
                    Point(
                        previous.x + t * (current.x - previous.x),
                        previous.y + t * (current.y - previous.y),
                    )
                )
        if current_inside:
            output.append(current)
        previous = current
        previous_value = current_value
        previous_inside = current_inside
    return _deduplicate_adjacent(output, epsilon)


def _deduplicate_adjacent(points: Sequence[Point], epsilon: float) -> list[Point]:
    if not points:
        return []
    result = [points[0]]
    for point in points[1:]:
        if distance(point, result[-1]) > epsilon:
            result.append(point)
    if len(result) > 1 and distance(result[0], result[-1]) <= epsilon:
        result.pop()
    return result


def clip_polygon_to_circle_outer(
    polygon: Sequence[Point],
    center: Point,
    radius: float,
    sides: int,
    epsilon: float = 1.0e-9,
) -> list[Point]:
    """Conservatively intersect with a circle using tangent half-planes."""
    result = list(polygon)
    for k in range(sides):
        phi = 2.0 * math.pi * k / sides
        nx = math.cos(phi)
        ny = math.sin(phi)
        result = clip_polygon_halfplane(
            result,
            nx,
            ny,
            nx * center.x + ny * center.y + radius,
            epsilon,
        )
        if not result:
            break
    return result


def clip_polygon_to_bearing_wedge(
    polygon: Sequence[Point],
    origin: Point,
    measured_deg: float,
    tolerance_deg: float,
    epsilon: float = 1.0e-9,
) -> list[Point]:
    """Keep every ray in [measured-tol, measured+tol], including wrap-around."""
    if not math.isfinite(measured_deg):
        raise ValueError("bearing must be finite")
    if tolerance_deg <= 0.0 or tolerance_deg >= 90.0:
        raise ValueError("wedge tolerance must be between 0 and 90 degrees")
    lower = unit_from_degrees(measured_deg - tolerance_deg)
    upper = unit_from_degrees(measured_deg + tolerance_deg)

    # cross(lower, p-origin) >= 0
    result = clip_polygon_halfplane(
        polygon,
        lower.y,
        -lower.x,
        lower.y * origin.x - lower.x * origin.y,
        epsilon,
    )
    # cross(upper, p-origin) <= 0
    result = clip_polygon_halfplane(
        result,
        -upper.y,
        upper.x,
        -upper.y * origin.x + upper.x * origin.y,
        epsilon,
    )
    return result


def polygon_area(polygon: Sequence[Point]) -> float:
    if len(polygon) < 3:
        return 0.0
    return 0.5 * sum(cross(polygon[i], polygon[(i + 1) % len(polygon)]) for i in range(len(polygon)))


def polygon_centroid(polygon: Sequence[Point]) -> Point:
    if not polygon:
        raise ValueError("cannot calculate centroid of empty polygon")
    if len(polygon) < 3:
        return Point(
            sum(point.x for point in polygon) / len(polygon),
            sum(point.y for point in polygon) / len(polygon),
        )
    twice_area = sum(cross(polygon[i], polygon[(i + 1) % len(polygon)]) for i in range(len(polygon)))
    if abs(twice_area) < 1.0e-14:
        return Point(
            sum(point.x for point in polygon) / len(polygon),
            sum(point.y for point in polygon) / len(polygon),
        )
    cx = 0.0
    cy = 0.0
    for i, first in enumerate(polygon):
        second = polygon[(i + 1) % len(polygon)]
        factor = cross(first, second)
        cx += (first.x + second.x) * factor
        cy += (first.y + second.y) * factor
    return Point(cx / (3.0 * twice_area), cy / (3.0 * twice_area))


def point_in_convex_polygon(point: Point, polygon: Sequence[Point], epsilon: float = 1.0e-7) -> bool:
    if not polygon:
        return False
    if len(polygon) == 1:
        return distance(point, polygon[0]) <= epsilon
    if len(polygon) == 2:
        # A degenerate polygon is a segment, not the entire supporting line.
        first, second = polygon
        edge = subtract(second, first)
        squared = dot(edge, edge)
        if squared <= epsilon * epsilon:
            return distance(point, first) <= epsilon
        t = max(0.0, min(1.0, dot(subtract(point, first), edge) / squared))
        return distance(point, add(first, scale(edge, t))) <= epsilon
    orientation = 1.0 if polygon_area(polygon) >= 0.0 else -1.0
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        edge = subtract(second, first)
        offset = subtract(point, first)
        if orientation * cross(edge, offset) < -epsilon:
            return False
    return True


def farthest_pair(points: Sequence[Point]) -> tuple[Point, Point]:
    if not points:
        raise ValueError("points cannot be empty")
    if len(points) == 1:
        return points[0], points[0]
    best = (points[0], points[1])
    best_squared = -1.0
    for i, first in enumerate(points):
        for second in points[i + 1 :]:
            squared = (first.x - second.x) ** 2 + (first.y - second.y) ** 2
            if squared > best_squared:
                best_squared = squared
                best = (first, second)
    return best


def max_distance_to_vertices(point: Point, polygon: Sequence[Point]) -> float:
    if not polygon:
        raise ValueError("polygon cannot be empty")
    return max(distance(point, vertex) for vertex in polygon)


@dataclass(frozen=True)
class Circle:
    center: Point
    radius: float


def _contains(circle: Circle, point: Point, epsilon: float = 1.0e-7) -> bool:
    return distance(circle.center, point) <= circle.radius + epsilon * max(1.0, circle.radius)


def _diameter_circle(first: Point, second: Point) -> Circle:
    center = Point((first.x + second.x) / 2.0, (first.y + second.y) / 2.0)
    return Circle(center, distance(first, second) / 2.0)


def _circumcircle(a: Point, b: Point, c: Point) -> Circle | None:
    determinant = 2.0 * (
        a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)
    )
    if abs(determinant) < 1.0e-14:
        return None
    a_sq = a.x * a.x + a.y * a.y
    b_sq = b.x * b.x + b.y * b.y
    c_sq = c.x * c.x + c.y * c.y
    ux = (a_sq * (b.y - c.y) + b_sq * (c.y - a.y) + c_sq * (a.y - b.y)) / determinant
    uy = (a_sq * (c.x - b.x) + b_sq * (a.x - c.x) + c_sq * (b.x - a.x)) / determinant
    center = Point(ux, uy)
    return Circle(center, distance(center, a))


def _circle_for_two_boundary_points(points: Sequence[Point], p: Point, q: Point) -> Circle:
    diameter = _diameter_circle(p, q)
    if all(_contains(diameter, point) for point in points):
        return diameter

    pq = subtract(q, p)
    left: Circle | None = None
    right: Circle | None = None
    left_cross = -math.inf
    right_cross = math.inf
    for point in points:
        candidate = _circumcircle(p, q, point)
        if candidate is None:
            continue
        side = cross(pq, subtract(point, p))
        center_side = cross(pq, subtract(candidate.center, p))
        if side > 0.0 and center_side > left_cross:
            left = candidate
            left_cross = center_side
        elif side < 0.0 and center_side < right_cross:
            right = candidate
            right_cross = center_side
    if left is None:
        if right is None:
            # Collinear fallback: the MEC is the diameter of the two extremes.
            all_points = [*points, p, q]
            first, second = farthest_pair(all_points)
            return _diameter_circle(first, second)
        return right
    if right is None:
        return left
    return left if left.radius <= right.radius else right


def minimum_enclosing_circle(points: Iterable[Point], seed: int = 0) -> Circle:
    """Expected-linear-time randomized incremental minimum enclosing circle."""
    shuffled = list(points)
    if not shuffled:
        raise ValueError("minimum enclosing circle requires at least one point")
    random.Random(seed).shuffle(shuffled)
    circle: Circle | None = None
    for i, point in enumerate(shuffled):
        if circle is not None and _contains(circle, point):
            continue
        circle = Circle(point, 0.0)
        for j, boundary in enumerate(shuffled[:i]):
            if _contains(circle, boundary):
                continue
            if circle.radius == 0.0:
                circle = _diameter_circle(point, boundary)
            else:
                circle = _circle_for_two_boundary_points(shuffled[: j + 1], point, boundary)
    assert circle is not None
    return circle
