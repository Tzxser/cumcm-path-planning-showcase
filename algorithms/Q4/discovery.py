"""Q4 discovery point sets only: no routing, sensing, or source-state logic.

The P25/P31 continuous local-convex certificates are documented in
``benchmarks/discovery_proofs.md``.  P19 is kept in the public reference's
original order and coordinates and is deliberately NOT called strict.
"""
from __future__ import annotations

import math

Point = tuple[float, float]

_P19: tuple[Point, ...] = (
    (0.0, 0.0), (-500.0, -866.0254037844), (-1000.0, -1732.0508075689),
    (-1500.0, -866.0254037844), (-2000.0, 0.0), (-1500.0, 866.0254037844),
    (-1000.0, 0.0), (-500.0, 866.0254037844), (-1000.0, 1732.0508075689),
    (0.0, 1732.0508075689), (500.0, 866.0254037844), (1000.0, 0.0),
    (500.0, -866.0254037844), (0.0, -1732.0508075689), (1000.0, -1732.0508075689),
    (1500.0, -866.0254037844), (2000.0, 0.0), (1500.0, 866.0254037844),
    (1000.0, 1732.0508075689),
)


def _ring(count: int, radius: float) -> list[Point]:
    return [(radius * math.cos(2 * math.pi * i / count),
             radius * math.sin(2 * math.pi * i / count)) for i in range(count)]


def p19() -> list[Point]:
    """Return a fresh copy of the unchanged public P19 path; not strict."""
    return list(_P19)


def p25() -> list[Point]:
    """Current strict center + 8/16 rings, retaining its baseline spine order.

    The order's geometric open length is 17966.55382840154 m.  A caller may
    freely reorder the sites; the coverage certificate concerns their set.
    """
    outer = _ring(16, 1840.0)
    return [(0.0, 0.0), *_ring(8, 1000.0),
            *(outer[i] for i in (14, 15, *range(14)))]


def p31_gapfix() -> list[Point]:
    """Keep all original P19 sites and add two sites per hexagonal cap.

    Each side's outward normal is n=(cos(phi),sin(phi)), phi=30+60k degrees;
    t=(-n_y,n_x).  The two new points are 1801*n +/- 500*t.  The returned
    order is P19 followed by additions, NOT an optimized path.
    """
    points = p19()
    for k in range(6):
        angle = math.pi / 6 + k * math.pi / 3
        nx, ny = math.cos(angle), math.sin(angle)
        tx, ty = -ny, nx
        for sign in (-1, 1):
            points.append((1801.0 * nx + sign * 500.0 * tx,
                           1801.0 * ny + sign * 500.0 * ty))
    return points

