"""New combination: compact certified P22 plus interruptible directed service arcs.

Neither original component's evidence is overwritten. Prediction chooses tasks;
the inherited bounded-error and full leaf-cover logic still authorizes actions.
"""
from Q4.engine import Config
from Q4.candidates.q4_x35 import InterruptibleServiceArcSolver
from Q4.rotatable_net import compact_sites, compact_certificate


class CompactServiceArcSolver(InterruptibleServiceArcSolver):
    def __init__(self, config):
        super().__init__(config)
        self.pending = list(compact_sites())
        self.proof = compact_certificate()

    def metadata(self):
        return {**super().metadata(), "discovery": "strict_compact_ring22",
                "inner_radius_m": 965.0, "outer_radius_m": 1848.0,
                "version": "round3-compact-service-arcs-v1",
                "combines_frozen_candidates": ["Q4_X35", "Q4_X36"]}


def build():
    return CompactServiceArcSolver(Config("Q4_X41", service_probes=3))
