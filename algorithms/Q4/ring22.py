"""A new strict 22-site network, independently certified over continuous boxes."""
import math
import json
from functools import lru_cache
from Q4.interval_cover import certify

@lru_cache(maxsize=1)
def sites_and_certificate():
    outer=1800/math.cos(math.pi/14)+20
    sites=[(0.,0.)]+[(995*math.cos(i*math.tau/7),995*math.sin(i*math.tau/7)) for i in range(7)]
    sites += [(outer*math.cos(i*math.tau/14),outer*math.sin(i*math.tau/14)) for i in range(14)]
    proof=certify(sites,wall_limit_s=5.)
    if not proof['covered']:raise ValueError('P22_CONTINUOUS_CERTIFICATE_NOT_ESTABLISHED')
    summary={k:v for k,v in proof.items() if k not in ('leaves','sites','certificate_leaves')}
    return tuple(sites),json.loads(json.dumps(summary,allow_nan=False))
