"""Directed source-service arcs with exact old safety decisions underneath.

The route nodes here are not source centres: each has an observation entry, an
estimated internal service in seconds, and a completion exit. A tiny layered DP
selects the entry variant jointly with the open route. These predictions never
modify the bounded-error polygon, and never authorize a clear.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import time

from geometry_core import dist, nearest_guaranteed_clear
from Q3.routing import route_order
from Q4.candidates.q4_x21 import JointRing22Solver

Point = tuple[float, float]


@dataclass(frozen=True)
class ServiceArc:
    entry: Point
    exit: Point
    service_s: float
    mode: str


def optimize_arcs(current: Point, groups: list[list[ServiceArc]], centres: list[Point],
                  *, evaluation_cap: int = 180) -> tuple[list[int], list[int], float, int]:
    """Bounded asymmetric open-route/entry search, exact DP for each ordering.

    Internal service costs are already seconds. Unlike a point-route 2-opt the
    transition is exit(i)->entry(j); reversal is fully rescored, not estimated
    from two symmetric edges. Every group appears once, including colocated ones.
    """
    if not groups:
        return [], [], 0.0, 0
    if len(groups) != len(centres) or len(groups) > 40 or any(not g for g in groups):
        raise ValueError('invalid finite service-arc task pool')
    for group in groups:
        for arc in group:
            if not all(math.isfinite(x) for x in (*arc.entry, *arc.exit, arc.service_s)) or arc.service_s < 0:
                raise ValueError('invalid service-arc value')

    def evaluate(order):
        costs = [dist(current, a.entry) / 5 + a.service_s for a in groups[order[0]]]
        parents = []
        for before, index in zip(order, order[1:]):
            new, links = [], []
            for arc in groups[index]:
                values = [costs[j] + dist(prev.exit, arc.entry) / 5 + arc.service_s
                          for j, prev in enumerate(groups[before])]
                j = min(range(len(values)), key=lambda k: (values[k], k))
                new.append(values[j]); links.append(j)
            costs = new; parents.append(links)
        last = min(range(len(costs)), key=lambda j: (costs[j], j))
        value = costs[last]; variants = [last]
        for links in reversed(parents):
            last = links[last]; variants.append(last)
        return value, variants[::-1]

    order = route_order(current, centres)
    best, variants = evaluate(order); count = 1
    # A genuinely directed nearest-arc seed can escape the centre-route basin.
    left = list(range(len(groups))); directed = []; p = current
    while left:
        i, j = min(((i, j) for i in left for j in range(len(groups[i]))),
                   key=lambda ij: (dist(p, groups[ij[0]][ij[1]].entry) / 5
                                   + groups[ij[0]][ij[1]].service_s, ij))
        directed.append(i); left.remove(i); p = groups[i][j].exit
    value, choices = evaluate(directed); count += 1
    if value < best - 1e-7:
        order, best, variants = directed, value, choices
    for _ in range(2):
        improved = False
        for width in range(2, len(order) + 1):
            for i in range(len(order) - width + 1):
                if count >= evaluation_cap:
                    return order, variants, best, count
                proposal = order[:i] + order[i:i + width][::-1] + order[i + width:]
                value, choices = evaluate(proposal); count += 1
                if value < best - 1e-7:
                    order, best, variants, improved = proposal, value, choices, True
        if not improved:
            break
    return order, variants, best, count


class ServiceArcSolver(JointRing22Solver):
    interruptible = False
    max_slices_per_source = 6

    def __init__(self, config):
        super().__init__(config)
        self.slices = {}; self.started = set(); self.arc_evaluations = 0
        self.arc_wall_ms = 0.0; self.arc_max_wall_ms = 0.0; self.arc_fallbacks = 0
        self.arc_tasks = 0

    def metadata(self):
        return {**super().metadata(), 'version': 'round3-service-arcs-v1',
                'scheduler': 'directed entry/internal-service/exit layered-DP plus bounded reversals',
                'prediction_authorizes_clear': False, 'interruptible_service': self.interruptible,
                'max_slices_per_source': self.max_slices_per_source,
                'arc_route_evaluation_cap': 180}

    def _source_arcs(self, t):
        current = self.session.position.pair()
        if t.radius <= 19:
            p = nearest_guaranteed_clear(t.polygon, current) or t.center
            return [ServiceArc(p, p, 5., 'guaranteed-clear')]
        # On a resumed task the active no-signal streak is meaningful. The old
        # solver resets it only when starting a new continuous source service.
        streak = t.no_signal_streak
        if t.channel not in self.started:
            t.no_signal_streak = 0
        try:
            entry = self._local_point(t, self.slices.get(t.channel, 0))
        finally:
            t.no_signal_streak = streak
        if t.radius <= self.config.cover_radius or (t.channel in self.started and streak >= 2):
            return [ServiceArc(t.center, t.center, 5. + min(75., t.radius / 2), 'leaf-cover')]
        last = [m for m in t.measurements if m['result'] == 'direction'][-1]
        positive = last['position']; theta = math.radians(last['svd_deg'])
        normal = (-math.sin(theta), math.cos(theta))
        reflected = (entry[0] - 50 * normal[0], entry[1] - 50 * normal[1])
        d = dist(positive, t.center)
        offset = min(180., max(60., t.radius * .25))
        front = (t.center[0] + (positive[0] - t.center[0]) * offset / max(d, 1e-9),
                 t.center[1] + (positive[1] - t.center[1]) * offset / max(d, 1e-9))
        arcs = []
        for p, mode in ((entry, 'legacy-entry'), (reflected, 'mirror-entry'), (front, 'forward-entry')):
            if any(dist(p, m['position']) < 1e-5 for m in t.measurements):
                continue
            # Expected internal route, not a safety bound: after a bearing the
            # service path approaches the conservative region's centre. Penalize
            # a large forward step because directional invisibility is possible.
            front_risk = min(45., t.radius * .06) if mode == 'forward-entry' else 0.
            service = dist(p, t.center) / 5 + 17. + min(25., t.radius / 30) + front_risk
            arcs.append(ServiceArc(p, t.center, service, mode))
        return arcs or [ServiceArc(entry, t.center, dist(entry, t.center) / 5 + 35., 'legacy-entry')]

    def _service(self, t, arc):
        if t.channel not in self.started:
            self.started.add(t.channel); t.no_signal_streak = 0
        self.slices[t.channel] = self.slices.get(t.channel, 0) + 1
        for index in range(12):
            if t.cleared or t.failed:
                return
            p = self.session.position.pair()
            clear = nearest_guaranteed_clear(t.polygon, p)
            if clear is not None:
                self._clear(clear, t, 'mec_clear', True); return
            if t.radius <= self.config.cover_radius or t.no_signal_streak >= 2:
                self._leaf_sweep(t); return
            target = arc.entry if index == 0 and arc.mode.endswith('entry') else self._local_point(t, index)
            self.logger.decision('service_arc_measure', channel=t.channel, target=target,
                                 index=index, radius=t.radius, mode=arc.mode)
            self._observe(target, t.channel, 'local_measure')
            if (self.interruptible and index == 0 and not t.cleared and not t.failed
                    and t.radius > self.config.cover_radius and t.no_signal_streak < 2
                    and self.slices[t.channel] < self.max_slices_per_source):
                # Source task remains in the common pool. Each task can pause at
                # most five times, then receives the unchanged complete fallback.
                return
        if not t.cleared and not t.failed:
            self._leaf_sweep(t)

    def run(self, session, logger):
        self.session = session; self.logger = logger
        self._scan(self.pending.pop(0))
        # 22 scans + at most 16 * 6 service slices = 118 tasks; 160 guard leaves
        # ample slack but remains finite even under unexpected runtime states.
        for iteration in range(160):
            if len(self.cleared) >= 16 or not session.can_continue():
                break
            if len(self.tracks) >= 16:
                self.pending = []; self.skipped_at_16 = True
            tasks = [('scan', p, p) for p in self.pending]
            tasks += [('source', ch, t.center) for ch, t in self.tracks.items()
                      if not t.cleared and not t.failed]
            if not tasks:
                break
            current = session.position.pair(); before = time.perf_counter()
            groups = [[ServiceArc(t[2], t[2], 6. * len(self.unknown), 'scan')]
                      if t[0] == 'scan' else self._source_arcs(self.tracks[t[1]]) for t in tasks]
            try:
                order, variants, cost, evaluations = optimize_arcs(current, groups, [t[2] for t in tasks])
            except (ArithmeticError, ValueError, IndexError) as exc:
                self.arc_fallbacks += 1
                logger.decision('service_arc_fallback', error=type(exc).__name__)
                order = route_order(current, [t[2] for t in tasks]); variants = [0] * len(order)
                cost = None; evaluations = 0
            elapsed = (time.perf_counter() - before) * 1000
            self.arc_wall_ms += elapsed; self.arc_max_wall_ms = max(self.arc_max_wall_ms, elapsed)
            self.arc_evaluations += evaluations; self.arc_tasks += 1
            task = tasks[order[0]]; arc = groups[order[0]][variants[0]]
            logger.decision('directed_service_arc_route', kind=task[0], identity=task[1],
                            entry=arc.entry, predicted_exit=arc.exit, internal_seconds=arc.service_s,
                            mode=arc.mode, route_estimated_seconds=cost, evaluations=evaluations,
                            planner_wall_ms=elapsed, interruptible=self.interruptible)
            if task[0] == 'scan':
                self.pending.remove(task[1]); self._scan(task[1])
            else:
                self._service(self.tracks[task[1]], arc)
                if self.config.service_probes:
                    self._probe(session.position.pair(), self.config.service_probes)
        else:
            self.anomalies.append('GLOBAL_ITERATION_GUARD')
        unresolved = [ch for ch, t in self.tracks.items() if not t.cleared]
        done = not self.pending or len(self.cleared) >= 16
        return {'completed': done and not unresolved and len(self.cleared) >= 10,
                'discovery_certificate': done, 'cleared_channels': sorted(self.cleared),
                'unresolved_channels': unresolved, 'anomalies': self.anomalies,
                'joint_negative_cut_count': self.joint_cut_count, 'scan_count': self.scan_count,
                'leaf_clear_count': self.leaf_clears, 'service_arc_task_count': self.arc_tasks,
                'arc_objective_evaluations': self.arc_evaluations, 'planner_wall_total_ms': self.arc_wall_ms,
                'planner_wall_max_ms': self.arc_max_wall_ms, 'arc_fallback_count': self.arc_fallbacks,
                'source_service_slices': {str(ch): count for ch, count in self.slices.items()},
                '_channel_states': {str(ch): asdict(t) for ch, t in self.tracks.items()}}
