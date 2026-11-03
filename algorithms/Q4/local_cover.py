"""Finite conservative local clear covers, independent of source orientation.

Input vertices describe one convex polygon in boundary order. Long-axis splits
cover that polygon without removing any point. Each returned convex leaf has a
clear disk enclosing all its vertices, hence its entire interior. A bounded split
loop falls back to an oriented square grid whose half diagonal is below radius.
No HTTP, source truth, or strategy state is accessed here.
"""
from __future__ import annotations

import math
from typing import Any, Iterable

from geometry_core import diameter_axis, dist, mec
from q3.geometry import clip_polygon_halfplane
from q3.models import Point as GeometryPoint

Point = tuple[float, float]
_MAX_SPLITS = 512
_MAX_DEPTH = 20
_MAX_GRID_CELLS = 65_536
_CLIP_EPSILON = 1.0e-9


def _point(value: Any) -> Point:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError("point must contain two finite coordinates")
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) for v in value):
        raise ValueError("point coordinates must be numeric")
    result = float(value[0]), float(value[1])
    if not all(math.isfinite(v) for v in result):
        raise ValueError("point coordinates must be finite")
    return result


def _polygon(values: Iterable[Point]) -> list[Point]:
    result = []
    for value in values:
        point = _point(value)
        if not result or point != result[-1]:
            result.append(point)
    if len(result) > 1 and result[-1] == result[0]:
        result.pop()
    return result


def _segment_distance(point: Point, first: Point, second: Point) -> float:
    dx, dy = second[0]-first[0], second[1]-first[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return dist(point, first)
    ux, uy = dx/length, dy/length
    along = (point[0]-first[0])*ux+(point[1]-first[1])*uy
    along = max(0.0, min(length, along))
    return dist(point, (first[0]+along*ux, first[1]+along*uy))


def point_polygon_distance(point: Point, poly: list[Point]) -> float:
    """Euclidean distance to a closed convex polygon, including degeneracies.

    Clockwise and counterclockwise order are accepted. Empty polygons have
    infinite distance; a point or a segment is not treated as an infinite line.
    """
    point = _point(point)
    polygon = _polygon(poly)
    if not polygon:
        return math.inf
    if len(polygon) == 1:
        return dist(point, polygon[0])
    edges = list(zip(polygon, polygon[1:]+polygon[:1]))
    boundary = min(_segment_distance(point, a, b) for a, b in edges)
    if len(polygon) == 2:
        return boundary
    origin = polygon[0]
    area2 = math.fsum(
        (a[0]-origin[0])*(b[1]-origin[1])-(a[1]-origin[1])*(b[0]-origin[0])
        for a, b in edges
    )
    if abs(area2) <= 1.0e-14:
        return boundary
    sign = 1.0 if area2 > 0 else -1.0
    for a, b in edges:
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        signed = sign*(dx*(point[1]-a[1])-dy*(point[0]-a[0]))
        if signed < -_CLIP_EPSILON*length:
            return boundary
    return 0.0


def _clip(poly: list[Point], nx: float, ny: float, bound: float) -> list[Point]:
    clipped = clip_polygon_halfplane(
        [GeometryPoint(*p) for p in poly], nx, ny, bound, _CLIP_EPSILON
    )
    return _polygon([(p.x, p.y) for p in clipped])


def _leaf(poly: list[Point], radius: float, alternate_center: Point | None = None) -> dict | None:
    center, _ = mec(poly)
    padding = min(1.0e-8, radius*1.0e-8)
    for candidate in (center, alternate_center):
        if candidate is None:
            continue
        actual = max(dist(candidate, p) for p in poly) + padding
        if math.isfinite(actual) and actual <= radius:
            return {"center": tuple(candidate), "radius": actual, "polygon": list(poly)}
    return None


def _grid_cover(poly: list[Point], radius: float) -> list[dict]:
    """Cover in the polygon-diameter frame; default squares are at most26.8m.

    For a full first-bearing wedge this needs about114 rectangles, not a global
    arena grid. All rectangles are clipped against the original pending polygon.
    The grid-center disk is an independently certified fallback if MEC numerics
    produce a poor circle for a very thin clipped fragment.
    """
    axis = diameter_axis(poly)
    normal = -axis[1], axis[0]
    origin = poly[0]
    local = [((p[0]-origin[0])*axis[0]+(p[1]-origin[1])*axis[1],
              (p[0]-origin[0])*normal[0]+(p[1]-origin[1])*normal[1]) for p in poly]
    xmin, xmax = min(p[0] for p in local), max(p[0] for p in local)
    ymin, ymax = min(p[1] for p in local), max(p[1] for p in local)
    # This strict half-diagonal margin absorbs clipping roundoff. For r=19,
    # 26.8/sqrt(2)=18.95046m, leaving an additional centimetre-scale reserve.
    side = min(26.8, radius*math.sqrt(2)*0.999)
    nx, ny = max(1, math.ceil((xmax-xmin)/side)), max(1, math.ceil((ymax-ymin)/side))
    if nx*ny > _MAX_GRID_CELLS:
        raise ValueError("LOCAL_COVER_GRID_GUARD: requested resolution/region exceeds bounded service size")
    leaves = []
    for ix in range(nx):
        left = xmin+(xmax-xmin)*ix/nx
        right = xmin+(xmax-xmin)*(ix+1)/nx
        column = _clip(_clip(local, -1, 0, -left), 1, 0, right)
        if not column:
            continue
        for iy in range(ny):
            low = ymin+(ymax-ymin)*iy/ny
            high = ymin+(ymax-ymin)*(iy+1)/ny
            piece = _clip(_clip(column, 0, -1, -low), 0, 1, high)
            if not piece:
                continue
            world = [(origin[0]+x*axis[0]+y*normal[0], origin[1]+x*axis[1]+y*normal[1]) for x,y in piece]
            cx, cy = (left+right)/2, (low+high)/2
            grid_center = origin[0]+cx*axis[0]+cy*normal[0], origin[1]+cx*axis[1]+cy*normal[1]
            leaf = _leaf(world, radius, grid_center)
            if leaf is None:
                raise ArithmeticError("LOCAL_COVER_NUMERICAL_GUARD: grid disk failed vertex containment")
            leaves.append(leaf)
    if not leaves:
        raise ArithmeticError("LOCAL_COVER_EMPTY_GRID")
    return leaves


def cover_polygon(poly: list[Point], radius: float = 19) -> list[dict]:
    """Return a finite covering partition of a convex feasible polygon.

    Every returned ``radius`` is recomputed from its actual vertices and includes
    a positive numerical cushion. Shared split boundaries can overlap by the
    clipping tolerance; no part of the original polygon is intentionally removed.
    Empty input returns an empty cover. Oversized unsupported inputs fail loudly
    instead of reporting an incomplete cover as successful.
    """
    if isinstance(radius, bool) or not isinstance(radius, (int, float)) or not math.isfinite(radius) or radius <= 0:
        raise ValueError("clear-cover radius must be positive and finite")
    radius = float(radius)
    polygon = _polygon(poly)
    if not polygon:
        return []
    pending = [(polygon, 0)]
    leaves = []
    splits = 0
    while pending:
        cell, depth = pending.pop()
        leaf = _leaf(cell, radius)
        if leaf is not None:
            leaves.append(leaf)
            continue
        if splits >= _MAX_SPLITS or depth >= _MAX_DEPTH:
            leaves.extend(_grid_cover(cell, radius))
            continue
        axis = diameter_axis(cell)
        origin = cell[0]
        projection = [(p[0]-origin[0])*axis[0]+(p[1]-origin[1])*axis[1] for p in cell]
        low, high = min(projection), max(projection)
        if high-low <= 1.0e-10:
            leaves.extend(_grid_cover(cell, radius))
            continue
        middle = (low+high)/2+origin[0]*axis[0]+origin[1]*axis[1]
        children = (_clip(cell, axis[0], axis[1], middle),
                    _clip(cell, -axis[0], -axis[1], -middle))
        if not all(children):
            leaves.extend(_grid_cover(cell, radius))
            continue
        pending.extend((child, depth+1) for child in children)
        splits += 1
    return leaves
