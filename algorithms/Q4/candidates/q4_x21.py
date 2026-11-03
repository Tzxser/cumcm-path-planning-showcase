from Q4.candidates.q4_x19 import JointNegativeSolver
from Q4.engine import Config
from Q4.ring22 import sites_and_certificate
class JointRing22Solver(JointNegativeSolver):
    def __init__(self,config):
        super().__init__(config);sites,self.proof=sites_and_certificate();self.pending=list(sites)
    def metadata(self):return {**super().metadata(),'discovery':'strict_ring22','site_count':22,'continuous_box_certificate':self.proof}
def build():return JointRing22Solver(Config('Q4_X21',service_probes=3))
