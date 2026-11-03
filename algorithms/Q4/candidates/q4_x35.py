from Q4.engine import Config
from Q4.service_arcs_round3 import ServiceArcSolver


class InterruptibleServiceArcSolver(ServiceArcSolver):
    interruptible = True


def build():
    return InterruptibleServiceArcSolver(Config('Q4_X35', service_probes=3))
