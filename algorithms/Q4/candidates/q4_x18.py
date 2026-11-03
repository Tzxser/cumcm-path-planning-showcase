"""Order a complete certified clear cover using recent active no-signal probes.

The inferred short bracket is a planning preference, never a position constraint.
All remaining leaves are retained, so tangent orientations still get full coverage.
"""
import math
from Q4.engine import Config,Solver
from Q4.local_cover import cover_polygon
from geometry_core import dist,nearest_guaranteed_clear

class BracketCoverSolver(Solver):
    def _home(self,t):
        t.no_signal_streak=0
        return super()._home(t)
    def _leaf_sweep(self,t):
        leaves=cover_polygon(t.polygon,19.)
        positives=[(i,m) for i,m in enumerate(t.measurements) if m['result']=='direction']
        preference=None
        if positives:
            index,positive=positives[-1];origin=positive['position'];angle=math.radians(positive['svd_deg']);u=(math.cos(angle),math.sin(angle))
            misses=[m for m in t.measurements[index+1:] if m['result']=='no_signal' and m.get('phase')=='local_measure']
            projections=[(m['position'][0]-origin[0])*u[0]+(m['position'][1]-origin[1])*u[1] for m in misses]
            if projections and max(projections)>0:preference=(origin,u,max(projections)+20.)
        self.logger.decision('bracket_leaf_cover',channel=t.channel,leaf_count=len(leaves),preference=preference,ordering_only=True)
        for _ in range(1024):
            if not leaves:break
            current=self.session.position.pair()
            positions=[nearest_guaranteed_clear(leaf['polygon'],current) or leaf['center'] for leaf in leaves]
            eligible=list(range(len(leaves)))
            if preference:
                origin,u,upper=preference
                short=[i for i,leaf in enumerate(leaves) if (leaf['center'][0]-origin[0])*u[0]+(leaf['center'][1]-origin[1])*u[1]<=upper]
                if short:eligible=short
            index=min(eligible,key=lambda i:dist(current,positions[i]));p=positions[index];self.leaf_clears+=1
            guaranteed=max(dist(p,v) for v in t.polygon)<=19.+1e-7
            if self._clear(p,t,'bracket_ordered_leaf_sweep',guaranteed):return True
            leaves=[leaf for leaf in leaves if max(dist(p,v) for v in leaf['polygon'])>20.-1e-6]
        t.failed=True;self.anomalies.append(f'LOCAL_COVER_EXHAUSTED channel={t.channel}');return False

def build():return BracketCoverSolver(Config('Q4_X18',service_probes=3))
