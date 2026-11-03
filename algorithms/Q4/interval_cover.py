"""Finite conservative box certificate for directional Q4 discovery.

For one closed box B, keep only sites s with max_(x in B)|s-x| < 1000.
If all four corners belong to conv(kept sites), so does the entire box. For
every target G in it and every emission direction u, at least one kept site
s satisfies u.(s-G)>=0; that site is both within range and in the closed
directional half-plane. This proves discovery for EVERY target and orientation,
not merely a sampled set. Boxes strictly outside the arena are discarded.

Unknown is not a physical counterexample: depth, cells, numerical geometry, or
time limits can prevent a certificate even for a valid covering configuration.
Optional core polygons require a validated fan triangulation whose vertices
are actual scan sites and whose triangle diameters do not exceed 1000m.
"""
from __future__ import annotations

from collections import Counter
from fractions import Fraction
import gzip
import hashlib
import json
import math
from pathlib import Path
from time import perf_counter
from typing import Sequence

Point = tuple[float, float]
Box = tuple[float, float, float, float]


def _point(p: Sequence[float]) -> Point:
    if len(p) != 2:
        raise ValueError("two finite coordinates required")
    q = float(p[0]), float(p[1])
    if not all(math.isfinite(v) and abs(v) <= 2_000_000 for v in q):
        raise ValueError("scan sites outside finite protocol range")
    return q


def _cross(a: Point, b: Point, c: Point) -> float:
    ex, ey = b[0]-a[0], b[1]-a[1]
    dx, dy = c[0]-a[0], c[1]-a[1]
    value = ex*dy-ey*dx
    bound = 1e-10*(abs(ex*dy)+abs(ey*dx)+1)
    if abs(value) > bound:
        return value
    # Binary floats are exact rationals. Resolve the sign without a tolerance
    # or fixed decimal precision; returning the sign also avoids underflow of
    # an extremely small negative determinant into a false boundary zero.
    ax, ay = map(Fraction, a)
    bx, by = map(Fraction, b)
    cx, cy = map(Fraction, c)
    determinant = (bx-ax)*(cy-ay)-(by-ay)*(cx-ax)
    return 1.0 if determinant > 0 else -1.0 if determinant < 0 else 0.0


def _hull(vertices: Sequence[Point]) -> list[Point]:
    vertices = sorted(set(vertices))
    if len(vertices) < 3:
        return list(vertices)
    lower = []
    for p in vertices:
        while len(lower)>1 and _cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(vertices):
        while len(upper)>1 and _cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def _contains_box(poly: Sequence[Point], box: Box) -> bool:
    if len(poly) < 3:
        return False
    xmin, ymin, xmax, ymax = box
    for a, b in zip(poly, list(poly[1:])+list(poly[:1])):
        # The signed edge half-plane is linear; its minimum on the box is
        # attained at this corner. A zero sign is an exact closed boundary.
        ex, ey = b[0]-a[0], b[1]-a[1]
        worst = (xmax if ey >= 0 else xmin, ymin if ex >= 0 else ymax)
        if _cross(a, b, worst) < 0:
            return False
    return True


def _edge_short(a: Point, b: Point, radius: float) -> bool:
    dx=Fraction(a[0])-Fraction(b[0])
    dy=Fraction(a[1])-Fraction(b[1])
    return dx*dx+dy*dy <= Fraction(radius)**2


def _core(sites: list[Point], core_polygon, core_triangles, radius: float) -> list[list[Point]]:
    result = []
    triangles = []
    if core_polygon is not None:
        polygon = [_point(p) for p in core_polygon]
        hull = _hull(polygon)
        if len(hull) != len(polygon) or len(hull)<3:
            raise ValueError("core polygon must be a strictly convex site polygon")
        # Use an actual scan site as fan centre. Zero is preferred when present;
        # otherwise a polygon vertex yields an ordinary fan triangulation.
        centre = (0., 0.) if (0., 0.) in sites else hull[0]
        triangles.extend([(centre, a, b) for a, b in zip(hull, hull[1:]+hull[:1]) if _cross(centre,a,b)!=0])
        if any(_cross(a,b,centre) < 0 for a,b in zip(hull,hull[1:]+hull[:1])):
            raise ValueError("core fan centre outside polygon")
        result.append(hull)
    if core_triangles:
        for triangle in core_triangles:
            if len(triangle) != 3:
                raise ValueError("core triangle requires three vertices")
            tri = tuple(_point(p) for p in triangle)
            if len(_hull(tri)) != 3:
                raise ValueError("core triangle must be nondegenerate")
            triangles.append(tri)
            result.append(_hull(tri))
    for tri in triangles:
        if any(p not in sites for p in tri):
            raise ValueError("core proof vertices must be actual scan sites")
        if not all(_edge_short(tri[i], tri[j], radius) for i in range(3) for j in range(i)):
            raise ValueError("core triangle exceeds receive radius")
    return result


def certify(sites: Sequence[Point], *, arena_radius: float = 1800.,
            receive_radius: float = 1000., range_margin: float = 1e-4,
            max_depth: int = 20, max_cells: int = 200_000, wall_limit_s: float = 30.,
            core_polygon=None, core_triangles=None, certificate_path: str | Path | None = None) -> dict:
    """Return covered/unknown with a finite optional gzip JSON box proof.

    Proof leaves are [depth, ix, iy, kind, witness], covering the root exactly.
    kinds: O=outside, H=near-site hull (witness bit mask), C=validated core
    (witness index), U=unresolved. A U leaf always prevents covered=true.
    """
    start = perf_counter()
    values = [_point(p) for p in sites]
    if not values or len(set(values)) != len(values):
        raise ValueError("distinct nonempty sites required")
    if not all(math.isfinite(x) and x > 0 for x in (arena_radius, receive_radius, wall_limit_s)):
        raise ValueError("positive finite radii and wall limit required")
    if not 0 < range_margin < receive_radius or not 0 <= max_depth <= 30 or max_cells < 1:
        raise ValueError("invalid certificate limits")
    cores = _core(values, core_polygon, core_triangles, receive_radius)
    hull_cache = {}
    leaves = []
    pending = [(0, 0, 0)]
    visited = 0
    min_range_slack = math.inf
    counts = Counter()
    depth_counts = Counter()
    reason = None
    radius2 = (receive_radius-range_margin-1e-8)**2
    while pending:
        if visited >= max_cells or perf_counter()-start > wall_limit_s:
            reason = "cell_limit" if visited >= max_cells else "wall_limit"
            leaves.extend([depth, ix, iy, "U", None] for depth,ix,iy in pending)
            counts["U"] += len(pending)
            pending.clear()
            break
        depth, ix, iy = pending.pop()
        visited += 1
        step = 2*arena_radius/(1<<depth)
        xmin, ymin = -arena_radius+ix*step, -arena_radius+iy*step
        xmax, ymax = xmin+step, ymin+step
        box = xmin,ymin,xmax,ymax
        nx, ny = max(xmin, 0., -xmax), max(ymin, 0., -ymax)
        if math.hypot(nx,ny) > arena_radius+1e-7:
            kind, witness = "O", None
        else:
            matching_core = next((i for i,p in enumerate(cores) if _contains_box(p,box)), None)
            if matching_core is not None:
                kind, witness = "C", matching_core
            else:
                mask = 0
                largest = 0.
                for i,(sx,sy) in enumerate(values):
                    dx,dy = max(abs(xmin-sx),abs(xmax-sx)), max(abs(ymin-sy),abs(ymax-sy))
                    d2=dx*dx+dy*dy
                    if d2 <= radius2:
                        mask |= 1<<i
                        largest=max(largest,d2)
                hull = hull_cache.get(mask)
                if hull is None:
                    hull = _hull([p for i,p in enumerate(values) if mask & (1<<i)])
                    hull_cache[mask] = hull
                if _contains_box(hull,box):
                    kind,witness="H",mask
                    min_range_slack=min(min_range_slack,receive_radius-math.sqrt(largest)-1e-8)
                elif depth < max_depth:
                    pending.extend((depth+1,2*ix+dx,2*iy+dy) for dy in (1,0) for dx in (1,0))
                    continue
                else:
                    kind,witness="U",None
                    reason=reason or "depth_limit"
        leaves.append([depth,ix,iy,kind,witness])
        counts[kind]+=1
        depth_counts[depth]+=1
    unresolved = [row[:3] for row in leaves if row[3]=="U"]
    report={"status":"covered" if not unresolved else "unknown", "covered":not unresolved,
            "proof_method":"whole-box near-site convex-hull certificate",
            "arena_radius":arena_radius,"receive_radius":receive_radius,"range_margin":range_margin,
            "sites":values,"site_count":len(values),"visited_cells":visited,"leaf_counts":dict(counts),
            "leaf_depth_counts":dict(sorted(depth_counts.items())),"hull_witness_count":len(hull_cache),
            "min_range_slack_m":min_range_slack if math.isfinite(min_range_slack) else None,
            "max_depth":max_depth,"max_cells":max_cells,"wall_limit_s":wall_limit_s,
            "unresolved_count":len(unresolved),"unresolved_boxes_sample":unresolved[:30],
            "stop_reason":reason,"wall_time_s":perf_counter()-start,
            "core_polygons":cores,"not_a_physical_failure":bool(unresolved)}
    payload={"report":report,"leaves":leaves}
    canonical=json.dumps(payload,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()
    report["proof_sha256"]=hashlib.sha256(canonical).hexdigest()
    if certificate_path is not None:
        destination=Path(certificate_path)
        destination.parent.mkdir(parents=True,exist_ok=True)
        with gzip.open(destination,"wt",encoding="utf-8") as file:
            json.dump(payload,file,separators=(",",":"),ensure_ascii=False)
        report["certificate_path"]=str(destination)
    return report


def verify_certificate(path: str | Path) -> dict:
    """Independently recheck every saved leaf and exact quadtree partition.

    The verifier does not run the search. It checks the witnesses contained in
    the gzip artifact, including all receive-distance and convex-hull tests.
    """
    with gzip.open(path,"rt",encoding="utf-8") as file:
        proof=json.load(file)
    report=proof["report"]
    # Hash convention excludes the hash field itself and the output filename.
    hash_report={k:v for k,v in report.items() if k not in {"proof_sha256","certificate_path"}}
    canonical=json.dumps({"report":hash_report,"leaves":proof["leaves"]},sort_keys=True,
                         separators=(",",":"),ensure_ascii=False).encode()
    if hashlib.sha256(canonical).hexdigest()!=report["proof_sha256"]:
        raise ValueError("proof hash mismatch")
    sites=[_point(p) for p in report["sites"]]
    arena_radius=float(report["arena_radius"])
    receive_radius=float(report["receive_radius"])
    cores=[]
    for p in report["core_polygons"]:
        if len(p)==3:
            cores.extend(_core(sites,None,[p],receive_radius))
        else:
            cores.extend(_core(sites,p,None,receive_radius))
    leaves=proof["leaves"]
    cells=set()
    for depth,ix,iy,kind,witness in leaves:
        if (not all(isinstance(n,int) for n in (depth,ix,iy)) or not 0<=depth<=30
                or not 0<=ix<(1<<depth) or not 0<=iy<(1<<depth)):
            raise ValueError("invalid quadtree index")
        key=depth,ix,iy
        if key in cells:
            raise ValueError("duplicate quadtree leaf")
        cells.add(key)
    maximum=max((depth for depth,_,_ in cells),default=0)
    total=0
    for depth,ix,iy in cells:
        if any((d,ix>>(depth-d),iy>>(depth-d)) in cells for d in range(depth)):
            raise ValueError("overlapping ancestor quadtree leaves")
        total+=1<<(2*(maximum-depth))
    if total!=1<<(2*maximum):
        raise ValueError("quadtree leaves do not cover root")
    hull_cache={}
    unresolved=0
    for depth,ix,iy,kind,witness in leaves:
        step=2*arena_radius/(1<<depth)
        xmin,ymin=-arena_radius+ix*step,-arena_radius+iy*step
        xmax,ymax=xmin+step,ymin+step
        box=xmin,ymin,xmax,ymax
        if kind=="O":
            if math.hypot(max(xmin,0.,-xmax),max(ymin,0.,-ymax))<=arena_radius+1e-7:
                raise ValueError("outside witness fails")
        elif kind=="C":
            if not isinstance(witness,int) or not 0<=witness<len(cores) or not _contains_box(cores[witness],box):
                raise ValueError("core witness fails")
        elif kind=="H":
            if not isinstance(witness,int) or not 0<witness<(1<<len(sites)):
                raise ValueError("invalid near-site bit mask")
            near=[p for i,p in enumerate(sites) if witness & (1<<i)]
            limit2=(receive_radius-report["range_margin"]-1e-8)**2
            for sx,sy in near:
                dx,dy=max(abs(xmin-sx),abs(xmax-sx)),max(abs(ymin-sy),abs(ymax-sy))
                if dx*dx+dy*dy>limit2:
                    raise ValueError("near-site range witness fails")
            hull=hull_cache.setdefault(witness,_hull(near))
            if not _contains_box(hull,box):
                raise ValueError("hull witness fails")
        elif kind=="U":
            unresolved+=1
        else:
            raise ValueError("unknown proof leaf kind")
    if bool(report["covered"])!=(unresolved==0) or report["unresolved_count"]!=unresolved:
        raise ValueError("coverage result disagrees with leaf witnesses")
    return {"verified":True,"covered":unresolved==0,"leaf_count":len(leaves),
            "unresolved_count":unresolved,"proof_sha256":report["proof_sha256"]}
