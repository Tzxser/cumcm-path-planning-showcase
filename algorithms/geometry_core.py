"""Conservative geometry shared by new optimization experiments only."""
from __future__ import annotations
import math
import bootstrap
from q3.models import Point
from q3.geometry import (circumscribed_regular_polygon, clip_polygon_halfplane,
    clip_polygon_to_bearing_wedge, clip_polygon_to_circle_outer, minimum_enclosing_circle, farthest_pair)

ERROR_DEG = 1.006  # 1 degree bounded error + 0.005 degree output quantization.
SIDES = 144

def dist(a,b): return math.hypot(a[0]-b[0],a[1]-b[1])
def points(poly): return [Point(*p) for p in poly]
def tuples(poly): return [(p.x,p.y) for p in poly]
def arena(): return tuples(circumscribed_regular_polygon(Point(0,0),1800,SIDES))
def mec(poly):
    circle=minimum_enclosing_circle(points(poly))
    center=(circle.center.x,circle.center.y)
    return center, max(dist(center,p) for p in poly)+1e-7
def diameter_axis(poly):
    a,b=farthest_pair(points(poly)); d=math.hypot(b.x-a.x,b.y-a.y)
    return ((b.x-a.x)/d,(b.y-a.y)/d) if d>1e-10 else (1.,0.)
def update(poly, origin, bearing, upper=1500.):
    p=points(poly or arena())
    p=clip_polygon_to_bearing_wedge(p,Point(*origin),bearing,ERROR_DEG,1e-8)
    p=clip_polygon_to_circle_outer(p,Point(*origin),upper,SIDES,1e-8)
    if not p: raise ValueError('EMPTY_FEASIBLE_REGION')
    return tuples(p)
def nearest_guaranteed_clear(poly,current,limit=19.):
    center,r=mec(poly)
    if r>limit:return None
    if max(dist(current,p) for p in poly)<=limit:return current
    # Points on center-current segment: convex maximum-distance constraint.
    lo,hi=0.,1.
    for _ in range(35):
        t=(lo+hi)/2
        p=(center[0]+t*(current[0]-center[0]),center[1]+t*(current[1]-center[1]))
        if max(dist(p,q) for q in poly)<=limit:lo=t
        else:hi=t
    return (center[0]+lo*(current[0]-center[0]),center[1]+lo*(current[1]-center[1]))
