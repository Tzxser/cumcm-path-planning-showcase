"""Certified compact P22 and observation-conditioned global phase selection.

Moving outside the arena is already used in X21. This experiment contracts its
outer envelope and chooses one rigid phase AFTER the origin measurements, before
any non-origin discovery obligation has been consumed. No hidden-source values
are available here. All sites remain obligations; positive track centers affect
route score only. The unchanged X21 localization/clear guarantee is retained.
"""
from __future__ import annotations

from functools import lru_cache
import gzip
import hashlib
import json
import math
from pathlib import Path
from time import perf_counter

from Q3.routing import route_order
from Q4.candidates.q4_x21 import JointRing22Solver
from Q4.interval_cover import certify


def compact_sites(phase: float = 0.) -> tuple[tuple[float, float], ...]:
    return ((0., 0.),) + tuple(
        (radius * math.cos(phase + i*math.tau/count),
         radius * math.sin(phase + i*math.tau/count))
        for count, radius in ((7, 965.), (14, 1848.)) for i in range(count))


def clean_proof(proof: dict) -> dict:
    return json.loads(json.dumps({k: v for k, v in proof.items()
                                 if k not in ('leaves', 'sites', 'certificate_leaves')}))


@lru_cache(maxsize=1)
def compact_certificate() -> dict:
    proof = certify(compact_sites(), wall_limit_s=3.)
    if not proof['covered']:
        raise ValueError('COMPACT_P22_CERTIFICATE_NOT_ESTABLISHED')
    return clean_proof(proof)


def export_compact_certificate(path: str | Path) -> dict:
    """Export an independently verifiable JSON-canonical proof artifact.

    The inherited exporter hashes integer-key diagnostic dictionaries before
    JSON converts the keys to strings. Normalize those incidental keys before
    re-hashing so the existing verifier sees exactly the same canonical object;
    no geometric witness or proof claim is changed.
    """
    path = Path(path)
    certify(compact_sites(), certificate_path=path, wall_limit_s=3.)
    with gzip.open(path, 'rt', encoding='utf-8') as handle:
        payload = json.load(handle)
    report = payload['report']
    if not report['covered']:
        raise ValueError('COMPACT_P22_EXPORT_NOT_COVERED')
    hash_report = {k: v for k, v in report.items()
                   if k not in ('proof_sha256', 'certificate_path')}
    canonical = json.dumps({'report': hash_report, 'leaves': payload['leaves']},
                           sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()
    report['proof_sha256'] = hashlib.sha256(canonical).hexdigest()
    with gzip.open(path, 'wt', encoding='utf-8') as handle:
        json.dump(payload, handle, separators=(',', ':'), ensure_ascii=False)
    return report


def open_cost(start, points) -> float:
    order = route_order(start, list(points))
    previous = start
    cost = 0.
    for index in order:
        cost += math.dist(previous, points[index])
        previous = points[index]
    return cost


def ranked_phases(current, estimated_centers, *, phase_count: int = 21):
    """Bounded deterministic expected-node proxy; zero phase remains an option.

    First-bearing polygon MEC centers are uncertain, so evaluate radial position
    hypotheses 0.70/1.00/1.30 times that center. This is NOT a Bayesian confidence
    interval and has no permission to remove targets, sites or feasible points.
    """
    if not 1 <= phase_count <= 28:
        raise ValueError('finite phase search required')
    if len(estimated_centers) > 16:
        raise ValueError('Q4 maximum source count exceeded')
    rows = []
    for i in range(phase_count):
        phase = math.tau*i/(7*phase_count)
        nodes = compact_sites(phase)[1:]
        scores = []
        for scale in (.7, 1., 1.3):
            centers = [(scale*x, scale*y) for x, y in estimated_centers]
            scores.append(open_cost(current, nodes + tuple(centers)))
        rows.append((sum(scores)/len(scores), i, phase))
    return sorted(rows)


class CompactRingSolver(JointRing22Solver):
    def __init__(self, config, *, adapt_phase: bool = False):
        super().__init__(config)
        self.pending = list(compact_sites())
        self.proof = compact_certificate()
        self.adapt_phase = adapt_phase
        self.phase = 0.
        self.phase_fallback = 0
        self.phase_wall_ms = 0.

    def metadata(self):
        return {**super().metadata(), 'discovery': 'strict_compact_ring22',
                'inner_radius_m': 965., 'outer_radius_m': 1848.,
                'adaptive_phase_after_origin': self.adapt_phase,
                'phase_candidates': 21 if self.adapt_phase else 1,
                'phase_objective': 'mean open route at radial centroid scales .7/1/1.3',
                'all_sites_retained': True}

    def _scan(self, p):
        super()._scan(p)
        # Engine has popped only origin; no other scan can be invalidated.
        if not self.adapt_phase or self.scan_count != 1:
            return
        start = perf_counter()
        centers = [t.center for t in self.tracks.values()
                   if not t.cleared and not t.failed and t.polygon]
        if not centers:
            return
        try:
            ranking = ranked_phases(self.session.position.pair(), centers)
            selected = False
            for score, index, phase in ranking[:3]:
                if perf_counter()-start > 2.5:
                    break
                proposed = compact_sites(phase)
                proof = certify(proposed, wall_limit_s=.25, max_cells=15000)
                if not proof['covered']:
                    continue
                self.pending = list(proposed[1:])
                self.proof = clean_proof(proof)
                self.phase = phase
                selected = True
                self.logger.decision('certified_global_phase', phase_rad=phase,
                                     candidate_index=index, planning_proxy_m=score,
                                     source_centers=len(centers), site_count=22,
                                     proof=self.proof, all_sites_retained=True)
                break
            if not selected:
                self.phase_fallback += 1
        except (ValueError, ArithmeticError, TimeoutError) as exc:
            self.phase_fallback += 1
            self.logger.decision('global_phase_fallback', error=type(exc).__name__,
                                 original_compact_net_retained=True)
        finally:
            self.phase_wall_ms = (perf_counter()-start)*1000

    def run(self, session, logger):
        result = super().run(session, logger)
        result.update(global_phase_rad=self.phase, global_phase_fallback=self.phase_fallback,
                      global_phase_wall_ms=self.phase_wall_ms,
                      compact_net_certificate=self.proof)
        return result
