"""Strict P25 + certified negative-segment position cuts + complete leaf cover."""
from Q4.engine import Config
from Q4.candidates.q4_x18 import BracketCoverSolver
from Q4.joint_constraints import refine_from_negatives
from geometry_core import mec

class JointNegativeSolver(BracketCoverSolver):
    def __init__(self,config):
        super().__init__(config);self.joint_cut_count=0
    def metadata(self):
        return {**super().metadata(),'negative_information':'range-certified two-negative segment fan cuts; no probabilistic removal'}
    def _leaf_sweep(self,t):
        polygon,certificates=refine_from_negatives(t.polygon,t.measurements)
        if certificates:
            before=t.radius;t.polygon=polygon;t.center,t.radius=mec(polygon)
            self.joint_cut_count+=len(certificates)
            self.logger.decision('joint_negative_refinement',channel=t.channel,radius_before=before,radius_after=t.radius,certificates=certificates)
        return super()._leaf_sweep(t)
    def run(self,session,logger):
        result=super().run(session,logger);result['joint_negative_cut_count']=self.joint_cut_count;return result

def build():return JointNegativeSolver(Config('Q4_X19',service_probes=3))
