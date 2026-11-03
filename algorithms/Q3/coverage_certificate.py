"""A finite, conservative Q3 discovery certificate for arbitrary scan sites.

Only the supplied measurement positions are used.  This module neither knows
source positions nor makes an assumption about their distribution.

Why the candidate set is sufficient
----------------------------------
Let f(x) = min_i |x-s_i| on the closed arena disk.  Divide the disk into the
ordinary nearest-site Voronoi cells.  Inside a cell f(x)=|x-s_i|.  On a straight
cell edge this is convex, so its maximum is at an endpoint.  A cell's interior
cannot contain a positive local maximum.  Its remaining boundary consists of
arena-circle arcs, whose maxima are either arc endpoints or the antipodal
arena point opposite s_i.  Thus a global maximum occurs at one of:

* an internal Voronoi vertex (a circumcenter of at least three sites);
* a Voronoi-edge / arena-circle intersection (a pair's bisector intersection);
* a site's antipodal arena-circle point.

We enumerate ALL triples and pairs, not just adjacent Voronoi sites.  Extra
points inside the arena are harmless: f evaluated there cannot exceed the
true global maximum.  Origin and a fixed boundary point cover the zero-site,
origin-site, and degenerate cases.  Collinear sites need no internal vertices;
cocircular vertices are included by any noncollinear defining triple.

Numerics
--------
Coordinates are scaled by the arena radius.  Ill-conditioned circumcenters
and nearly tangent bisectors are recomputed with 70-digit Decimal arithmetic
from the actual binary-float input values.  Only EXACT duplicate sites and
candidates are removed.  A 5 mm distance cushion is added to the reported
maximum, and coverage requires this upper estimate <= receive_radius-0.5 m
(999.5 m with the official defaults).  No near-duplicate deletion can silently
remove a Voronoi event.  Numerically unresolvable calculations fail closed.

Complexity is O(n^4) in this deliberately simple all-triples implementation,
with n <= 30 intended.  Repeated calls with the same site set are cached.
"""

from __future__ import annotations

from decimal import Decimal, InvalidOperation, localcontext
from functools import lru_cache
import math
from typing import Iterable

Point = tuple[float, float]

DISTANCE_CUSHION_M = 0.005
COVERAGE_RESERVE_M = 0.5
_DECIMAL_PRECISION = 70
_ILL_CONDITIONED = 1.0e-5
_BOUNDARY_TOL = 2.0e-12


def _decimal(value: float) -> Decimal:
    """Preserve the exact value passed by the caller, not its short repr."""
    return Decimal.from_float(value)


def _circumcenter_decimal(a: Point, b: Point, c: Point) -> Point | None:
    with localcontext() as context:
        context.prec = _DECIMAL_PRECISION
        ax, ay = map(_decimal, a)
        ux, uy = _decimal(b[0]) - ax, _decimal(b[1]) - ay
        vx, vy = _decimal(c[0]) - ax, _decimal(c[1]) - ay
        determinant = ux * vy - uy * vx
        if not determinant:
            return None
        uu, vv = ux * ux + uy * uy, vx * vx + vy * vy
        x = ax + (uu * vy - vv * uy) / (2 * determinant)
        y = ay + (ux * vv - vx * uu) / (2 * determinant)
        # Reject outside centers BEFORE float conversion, including extremely
        # skinny triangles whose center exceeds the floating-point range.
        if x * x + y * y > 1 + _decimal(_BOUNDARY_TOL):
            return None
        return float(x), float(y)


def _circumcenter(a: Point, b: Point, c: Point) -> Point | None:
    ux, uy = b[0] - a[0], b[1] - a[1]
    vx, vy = c[0] - a[0], c[1] - a[1]
    p, q = ux * vy, uy * vx
    determinant = p - q
    # A zero-product axis-aligned triple is exactly collinear and cheap to
    # reject.  Other cancellation, including a rounded zero, uses Decimal.
    if p == q == 0.0 and (ux == vx == 0.0 or uy == vy == 0.0):
        return None
    uu, vv = ux * ux + uy * uy, vx * vx + vy * vy
    scale = math.sqrt(uu * vv) if math.isfinite(uu * vv) else math.inf
    if (not math.isfinite(scale) or abs(determinant) <= _ILL_CONDITIONED * scale):
        return _circumcenter_decimal(a, b, c)
    x = a[0] + (uu * vy - vv * uy) / (2 * determinant)
    y = a[1] + (ux * vv - vx * uu) / (2 * determinant)
    r2 = x * x + y * y
    if not math.isfinite(r2):
        return _circumcenter_decimal(a, b, c)
    # Borderline membership matters: do not lose a real inside vertex due to
    # a rounded radius check.  Decimal resolves it conservatively.
    if abs(r2 - 1.0) < 1.0e-9:
        return _circumcenter_decimal(a, b, c)
    return (x, y) if r2 <= 1.0 + _BOUNDARY_TOL else None


def _bisector_decimal(a: Point, b: Point) -> tuple[Point, ...]:
    with localcontext() as context:
        context.prec = _DECIMAL_PRECISION
        ax, ay = map(_decimal, a)
        bx, by = map(_decimal, b)
        ux, uy = bx - ax, by - ay
        norm2 = ux * ux + uy * uy
        if not norm2:
            return ()
        # u . x = u . ((a+b)/2); using this fact avoids square cancellation.
        rhs = (ux * (ax + bx) + uy * (ay + by)) / 2
        slack = norm2 - rhs * rhs
        if slack < 0:
            return ()
        factor = slack.sqrt() / norm2
        fx, fy = ux * rhs / norm2, uy * rhs / norm2
        return ((float(fx - uy * factor), float(fy + ux * factor)),
                (float(fx + uy * factor), float(fy - ux * factor)))


def _bisector_boundary(a: Point, b: Point) -> tuple[Point, ...]:
    ux, uy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(ux, uy)
    if not length:
        return ()
    nx, ny = ux / length, uy / length
    signed_distance = nx * ((a[0] + b[0]) / 2) + ny * ((a[1] + b[1]) / 2)
    if not math.isfinite(signed_distance) or abs(abs(signed_distance) - 1) < 1.0e-8:
        return _bisector_decimal(a, b)
    if abs(signed_distance) > 1:
        return ()
    half_chord = math.sqrt(max(0.0, 1.0 - signed_distance * signed_distance))
    fx, fy = signed_distance * nx, signed_distance * ny
    return ((fx - half_chord * ny, fy + half_chord * nx),
            (fx + half_chord * ny, fy - half_chord * nx))


@lru_cache(maxsize=128)
def _evaluate(sites: tuple[Point, ...], arena_radius: float,
              receive_radius: float) -> tuple[bool, float, Point, int]:
    if not sites:
        return False, math.inf, (arena_radius, 0.0), 1
    scaled = tuple((x / arena_radius, y / arena_radius) for x, y in sites)
    if not all(math.isfinite(v) for p in scaled for v in p):
        return False, math.inf, (arena_radius, 0.0), 0
    candidates: set[Point] = {(0.0, 0.0), (1.0, 0.0)}

    def add(point: Point | None) -> None:
        if point is None:
            return
        x, y = point
        radius = math.hypot(x, y)
        if not math.isfinite(radius):
            raise ArithmeticError("non-finite coverage candidate")
        if radius <= 1.0 + _BOUNDARY_TOL:
            # A roundoff-outside point is projected back into the arena.  The
            # displacement is far below the reported distance cushion.
            candidates.add((x / radius, y / radius) if radius > 1.0 else (x, y))

    try:
        for i, a in enumerate(scaled):
            length = math.hypot(*a)
            if length:
                add((-a[0] / length, -a[1] / length))
            for j in range(i + 1, len(scaled)):
                b = scaled[j]
                for point in _bisector_boundary(a, b):
                    add(point)
                for k in range(j + 1, len(scaled)):
                    add(_circumcenter(a, b, scaled[k]))
    except (ArithmeticError, InvalidOperation, OverflowError, ValueError):
        # This is an exclusion/termination certificate; uncertainty must never
        # turn an unscanned region into an ABSENT channel.
        return False, math.inf, (arena_radius, 0.0), len(candidates)

    worst = -1.0
    worst_point = (0.0, 0.0)
    # Sorting makes ties and logged worst points reproducible across processes.
    for x, y in sorted(candidates):
        nearest = min(math.hypot(x - sx, y - sy) for sx, sy in scaled)
        if nearest > worst:
            worst = nearest
            worst_point = (x * arena_radius, y * arena_radius)
    upper = worst * arena_radius + DISTANCE_CUSHION_M
    return (upper <= receive_radius - COVERAGE_RESERVE_M,
            upper, worst_point, len(candidates))


def certificate(sites: Iterable[Point], arena_radius: float = 1800,
                receive_radius: float = 1000) -> dict:
    """Certify complete omnidirectional coverage of the closed arena disk.

    ``worst_distance`` is the finite-candidate maximum PLUS a 0.005 m cushion;
    consequently the distance at ``worst_point`` is normally 5 mm smaller.
    Invalid inputs raise ValueError.  Numerical uncertainty returns uncovered.
    This certificate says nothing about whether a channel was measured at a
    site: the caller must supply that particular channel's actual scan sites.
    """
    arena_radius, receive_radius = float(arena_radius), float(receive_radius)
    if not (math.isfinite(arena_radius) and arena_radius > 0
            and math.isfinite(receive_radius) and receive_radius > 0):
        raise ValueError("arena and receiving radii must be positive finite numbers")
    normalized: set[Point] = set()
    for site in sites:
        if len(site) != 2:
            raise ValueError("each scan site must contain exactly x and y")
        point = float(site[0]), float(site[1])
        if not all(math.isfinite(v) for v in point):
            raise ValueError("scan sites must contain finite coordinates")
        normalized.add(point)
    covered, worst, point, count = _evaluate(tuple(sorted(normalized)), arena_radius, receive_radius)
    return {"covered": covered, "worst_distance": worst,
            "worst_point": point, "candidate_count": count}

