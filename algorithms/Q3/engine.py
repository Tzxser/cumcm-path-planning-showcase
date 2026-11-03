"""Omnidirectional-only macro routing with strictly bounded local service."""
from __future__ import annotations
import math
from dataclasses import asdict, dataclass, field
from typing import Any
from geometry_core import arena, update, mec, dist, nearest_guaranteed_clear
from Q3.localization import choose_local_point
from Q3.routing import route_order

@dataclass(frozen=True)
class Config:
    name: str
    discovery: str = "mixed"  # mixed, batch, dynamic
    localizer: str = "center"
    ray_step: float = 250.
    probe_cap: int = 5
    coverage_credit: float = 3.
    macro_uncertainty: float = .3
    speculative_clear: bool = False
    scan_gain_min: int = 1
    anchor_count: int = 6
    anchor_radius: float = 1150.
    routing: str = "greedy"
    stop_at_16_discovered: bool = False
    service_probe_cap: int = 0

@dataclass
class Track:
    channel: int
    polygon: list = field(default_factory=list)
    measurements: list = field(default_factory=list)
    center: tuple = (0.,0.)
    radius: float = 1500.
    cleared: bool = False
    failed: bool = False
    local_measures: int = 0
    probe_count: int = 0

class Solver:
    def __init__(self, config: Config):
        self.config=config; self.unknown=set(range(1,21)); self.tracks={}; self.cleared=set(); self.anomalies=[]
        self.anchors=[(0.,0.)]+[(config.anchor_radius*math.cos(i*math.tau/config.anchor_count),config.anchor_radius*math.sin(i*math.tau/config.anchor_count)) for i in range(config.anchor_count)]
        from Q3.coverage_certificate import certificate
        if not certificate(self.anchors)["covered"]:raise ValueError("uncertified fallback discovery anchors")
        self.visited=set(); self.scan_sites=[]; self.cert={"covered":False,"worst_distance":1800.,"worst_point":(1800.,0.)}
        self.planning_grid=[(x,y) for x in range(-1800,1801,300) for y in range(-1800,1801,300) if x*x+y*y<=1800**2]
        self.speculated=set(); self.noncoverage=0
    def metadata(self):return {"version":"1.0","question":3,**asdict(self.config),"error_bound_deg":1.006,"guaranteed_clear_radius_m":19.}
    def _observe(self,p,ch,phase,safety=None):
        if not self.session.can_continue():raise TimeoutError("REAL_TIME_BUDGET")
        result=self.session.measure(p,ch,{"phase":phase,"distance_category":phase,"safety":safety})
        response=result.response; kind=response["measure_result"]
        track=self.tracks.get(ch)
        if kind in ("direction","near") and track is None:
            track=Track(ch); self.tracks[ch]=track; self.unknown.discard(ch)
        if track:
            track.measurements.append({"position":p,"result":kind,"svd_deg":response.get("svd_deg"),"virtual_time_s":result.virtual_time_s,"phase":phase})
        if kind=="near":self._clear(p,ch,"near_clear",guaranteed=True)
        elif kind=="direction":
            upper=max(dist(p,q) for q in track.polygon)+1e-6 if track.polygon else 1500.
            track.polygon=update(track.polygon,p,response["svd_deg"],min(1500.,upper))
            track.center,track.radius=mec(track.polygon)
        elif safety and safety.get("guaranteed_signal"):
            self.anomalies.append(f"SAFE_POINT_NO_SIGNAL channel={ch}"); track.failed=True
        return kind
    def _clear(self,p,ch,phase,guaranteed=False):
        result=self.session.clear(p,ch,{"phase":phase,"distance_category":"clear","guaranteed":guaranteed})
        if result.response["clear_result"]=="success":
            self.cleared.add(ch); self.tracks[ch].cleared=True;return True
        if guaranteed:
            self.anomalies.append(f"GUARANTEED_CLEAR_FAILED channel={ch}")
            self.tracks[ch].failed=True
        return False
    def _service(self,ch):
        t=self.tracks[ch]
        for index in range(14):
            if t.cleared or t.failed:return
            p=self.session.position.pair()
            clear=nearest_guaranteed_clear(t.polygon,p)
            if clear is not None:
                self._clear(clear,ch,"guaranteed_clear",True);return
            if self.config.speculative_clear and ch not in self.speculated and t.radius<=55 and dist(p,t.center)<=8:
                self.speculated.add(ch)
                if self._clear(p,ch,"opportunistic_clear"):return
            mode=self.config.localizer if index<7 else "center"
            target,note=choose_local_point(t.polygon,p,t.measurements,mode=mode,ray_step=self.config.ray_step)
            if self.config.speculative_clear and ch not in self.speculated and t.radius<=55 and dist(target,t.center)<1e-6:
                self.speculated.add(ch)
                if self._clear(target,ch,"planned_local_clear_query"):return
            self.logger.decision("local_service",channel=ch,index=index,center=t.center,radius=t.radius,target=target,note=note)
            t.local_measures+=1
            self._observe(target,ch,"localization",note)
        if not t.cleared:
            t.failed=True;self.anomalies.append(f"LOCAL_ITERATION_GUARD channel={ch}")
    def _probe(self,p,cap=None):
        candidates=[]
        for ch,t in self.tracks.items():
            if t.cleared or t.failed or t.probe_count>=8 or t.radius<=19:continue
            if any(dist(m["position"],p)<1e-5 for m in t.measurements):continue
            d=dist(p,t.center)
            if d>1300:continue
            first=next(m for m in t.measurements if m["result"]=="direction")
            a=(t.center[0]-first["position"][0],t.center[1]-first["position"][1]);b=(t.center[0]-p[0],t.center[1]-p[1])
            cross=abs(a[0]*b[1]-a[1]*b[0])/max(1e-9,math.hypot(*a)*math.hypot(*b))
            value=(1-d/1500)*(1+cross)*(1 if t.radius>80 else .4)
            candidates.append((-value,ch))
        for _,ch in sorted(candidates)[:self.config.probe_cap if cap is None else cap]:
            self.tracks[ch].probe_count+=1
            self._observe(p,ch,"opportunistic")
    def _scan(self,p,anchor_index=None):
        for ch in sorted(self.unknown,key=lambda ch:(ch!=self.session.current_channel,ch)):
            if len(self.cleared)>=16:return
            self._observe(p,ch,"discovery")
        if anchor_index is not None:self.visited.add(anchor_index)
        if not any(dist(p,q)<1e-5 for q in self.scan_sites):self.scan_sites.append(p)
        self.noncoverage=0
        self._probe(p)
        if self.config.discovery=="dynamic":
            from Q3.coverage_certificate import certificate
            self.cert=certificate(self.scan_sites)
            self.logger.decision("coverage_certificate",sites=self.scan_sites,**self.cert)
    def _remaining(self):return [t for t in self.tracks.values() if not t.cleared and not t.failed]
    def _gain(self,p):
        return sum(dist(p,q)<970 and all(dist(q,s)>970 for s in self.scan_sites) for q in self.planning_grid)
    def _choose(self):
        current=self.session.position.pair(); rem=self._remaining()
        tasks=[]
        for t in rem:
            cost=dist(current,t.center)/5+self.config.macro_uncertainty*t.radius/5+18
            tasks.append((cost,"source",t.channel,t.center))
        certificate_done=self.cert["covered"] or len(self.visited)==len(self.anchors) or (self.config.stop_at_16_discovered and len(self.tracks)>=16)
        if not certificate_done and self.unknown:
            if self.config.discovery=="dynamic":
                options=[(i,p) for i,p in enumerate(self.anchors) if i not in self.visited]
                w=self.cert["worst_point"]; d=dist(w,current)
                if d>1e-6:
                    p=(w[0]+850*(current[0]-w[0])/d,w[1]+850*(current[1]-w[1])/d)
                    options.append((None,p))
                for i,p in options:
                    gain=self._gain(p)
                    if gain or self.noncoverage>=4 or not rem:
                        cost=dist(current,p)/5+6*len(self.unknown)-self.config.coverage_credit*gain
                        tasks.append((cost,"scan",i,p))
            else:
                remaining=sorted(set(range(len(self.anchors)))-self.visited)
                for index in remaining if self.config.routing=="tour" else remaining[:1]:
                    p=self.anchors[index]; cost=dist(current,p)/5+6*len(self.unknown)-12*len(self.unknown)
                    tasks.append((cost,"scan",index,p))
        if self.config.routing=="tour" and tasks:
            order=route_order(current,[t[3] for t in tasks])
            return tasks[order[0]]
        if self.config.routing=="insertion":
            scans=[t for t in tasks if t[1]=="scan"]
            if scans:
                next_point=scans[0][3]
                tasks=[(cost+(dist(p,next_point)-dist(current,next_point))/5 if kind=="source" else cost,kind,i,p) for cost,kind,i,p in tasks]
        if self.noncoverage>=4:
            scans=[t for t in tasks if t[1]=="scan"]
            if scans:tasks=scans
        if not tasks:return None
        return min(tasks,key=lambda t:(t[0],t[1],t[2] if t[2] is not None else -1))
    def run(self,session,logger):
        self.session=session;self.logger=logger
        self._scan((0.,0.),0)
        if self.config.discovery=="batch":
            for i in range(1,len(self.anchors)):
                if len(self.cleared)>=16:break
                self._scan(self.anchors[i],i)
            while self._remaining() and session.can_continue():
                rem=self._remaining(); order=route_order(session.position.pair(),[t.center for t in rem])
                self._service(rem[order[0]].channel)
        else:
            for iteration in range(60):
                if not session.can_continue() or len(self.cleared)>=16:break
                if iteration>=35 and len(self.visited)<len(self.anchors):
                    for i in range(len(self.anchors)):
                        if i not in self.visited:self._scan(self.anchors[i],i)
                task=self._choose()
                if task is None:break
                cost,kind,identity,p=task
                logger.decision("macro_task",kind=kind,identity=identity,target=p,predicted_seconds=cost)
                if kind=="scan":self._scan(p,identity)
                else:
                    self.noncoverage+=1;self._service(identity)
                    if self.config.service_probe_cap:
                        self._probe(session.position.pair(),self.config.service_probe_cap)
                    if self.config.discovery=="dynamic":
                        p=session.position.pair()
                        if self.unknown and self._gain(p)>=self.config.scan_gain_min:self._scan(p)
                        else:self._probe(p)
            else:self.anomalies.append("GLOBAL_TASK_GUARD")
        unresolved=[ch for ch,t in self.tracks.items() if not t.cleared]
        certificate_done=len(self.visited)==len(self.anchors) or self.cert["covered"] or len(self.cleared)>=16
        return {"completed":certificate_done and not unresolved and len(self.cleared)>=10,
                "discovery_certificate":certificate_done,"cleared_channels":sorted(self.cleared),
                "unresolved_channels":unresolved,"unknown_channels":sorted(self.unknown),
                "anomalies":self.anomalies,"coverage_scan_count":len(self.scan_sites),
                "_channel_states":{str(ch):{**asdict(t),"mec_radius":t.radius} for ch,t in self.tracks.items()}}
