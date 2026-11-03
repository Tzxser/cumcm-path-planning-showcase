"""Q4 macro-route experiments with conservative leaf-cover clearing."""
from __future__ import annotations
from dataclasses import asdict,dataclass,field
import math
from geometry_core import update,mec,dist,nearest_guaranteed_clear
from Q3.routing import route_order
from Q4.discovery import p19,p25,p31_gapfix
from Q4.local_cover import cover_polygon

@dataclass(frozen=True)
class Config:
    name:str
    discovery:str="p25"
    local_mode:str="progressive"
    cover_radius:float=90.
    front_offset:float=150.
    service_probes:int=0

@dataclass
class Track:
    channel:int
    polygon:list=field(default_factory=list)
    measurements:list=field(default_factory=list)
    cleared:bool=False
    failed:bool=False
    center:tuple=(0.,0.)
    radius:float=1500.
    no_signal_streak:int=0
    probe_count:int=0

class Solver:
    def __init__(self,config):
        self.config=config;self.pending=list({"p19":p19,"p25":p25,"p31":p31_gapfix}[config.discovery]())
        self.unknown=set(range(1,21));self.tracks={};self.cleared=set();self.anomalies=[];self.skipped_at_16=False
        self.scan_count=0;self.leaf_clears=0
    def metadata(self):return {"question":4,"version":"1.1",**asdict(self.config),"error_deg":1.006,"leaf_guarantee_radius":19.,"strict_discovery":self.config.discovery!="p19"}
    def _observe(self,p,ch,phase):
        if not self.session.can_continue():raise TimeoutError('REAL_TIME_BUDGET')
        r=self.session.measure(p,ch,{"phase":phase,"distance_category":"discovery" if 'scan' in phase else 'opportunistic' if 'probe' in phase else 'localization'})
        result=r.response;kind=result['measure_result'];t=self.tracks.get(ch)
        if kind in ('direction','near') and t is None:t=Track(ch);self.tracks[ch]=t;self.unknown.discard(ch)
        if t:t.measurements.append({'position':p,'result':kind,'svd_deg':result.get('svd_deg'),'virtual_time_s':r.virtual_time_s,'phase':phase})
        if kind=='near':self._clear(p,t,'near_clear',True)
        elif kind=='direction':
            upper=min(1500.,max(dist(p,q) for q in t.polygon)+1e-6) if t.polygon else 1500.
            t.polygon=update(t.polygon,p,result['svd_deg'],upper);t.center,t.radius=mec(t.polygon);t.no_signal_streak=0
        elif t:t.no_signal_streak+=1
        return kind
    def _clear(self,p,t,phase,guaranteed=False):
        if not self.session.can_continue():raise TimeoutError('REAL_TIME_BUDGET')
        r=self.session.clear(p,t.channel,{'phase':phase,'distance_category':'clear','guaranteed':guaranteed})
        if r.response['clear_result']=='success':t.cleared=True;self.cleared.add(t.channel);return True
        if guaranteed:t.failed=True;self.anomalies.append(f'GUARANTEED_CLEAR_FAILED channel={t.channel}')
        return False
    def _leaf_sweep(self,t):
        leaves=cover_polygon(t.polygon,19.)
        self.logger.decision('leaf_cover',channel=t.channel,leaf_count=len(leaves),radius=t.radius)
        for _ in range(1024):
            if not leaves:break
            current=self.session.position.pair()
            # Each leaf has a certified radius. Visit the nearest feasible
            # clear center; failed calls eliminate only completely covered cells.
            positions=[nearest_guaranteed_clear(leaf['polygon'],current) or leaf['center'] for leaf in leaves]
            i=min(range(len(leaves)),key=lambda k:dist(current,positions[k]))
            p=positions[i];self.leaf_clears+=1
            guaranteed=max(dist(p,v) for v in t.polygon)<=19.+1e-7
            if self._clear(p,t,'certified_leaf_sweep',guaranteed):return True
            leaves=[leaf for leaf in leaves if max(dist(p,v) for v in leaf['polygon'])>20.-1e-6]
        t.failed=True;self.anomalies.append(f'LOCAL_COVER_EXHAUSTED channel={t.channel}');return False
    def _local_point(self,t,index):
        directions=[m for m in t.measurements if m['result']=='direction'];last=directions[-1]
        old=last['position'];theta=math.radians(last['svd_deg']);u=(math.cos(theta),math.sin(theta));v=(-u[1],u[0])
        if t.no_signal_streak:
            if self.config.local_mode=='center' and t.no_signal_streak==1:
                # Try a short front-side offset using the last positive location.
                d=dist(old,t.center);direction=((old[0]-t.center[0])/max(d,1e-9),(old[1]-t.center[1])/max(d,1e-9))
                offset=min(250.,max(60.,t.radius))
                p=(t.center[0]+direction[0]*offset,t.center[1]+direction[1]*offset)
            else:
                # Known-positive hull is guaranteed visible. A midpoint between
                # distinct positive stations can give information without a 350m detour.
                if len(directions)>1:
                    a=min((m['position'] for m in directions[:-1]),key=lambda q:dist(q,t.center))
                    p=((a[0]+old[0])/2,(a[1]+old[1])/2)
                else:p=(old[0]+100*u[0]-25*v[0],old[1]+100*u[1]-25*v[1])
        elif self.config.local_mode=='center':p=t.center
        else:
            high=max((q[0]-old[0])*u[0]+(q[1]-old[1])*u[1] for q in t.polygon)
            step=min(200.,max(50.,high));side=1 if index%2==0 else -1
            p=(old[0]+step*u[0]+25*side*v[0],old[1]+step*u[1]+25*side*v[1])
        if any(dist(p,m['position'])<1e-5 for m in t.measurements):
            p=(t.center[0]+self.config.front_offset*u[0],t.center[1]+self.config.front_offset*u[1])
        return p
    def _home(self,t):
        for index in range(12):
            if t.cleared or t.failed:return
            p=self.session.position.pair();clear=nearest_guaranteed_clear(t.polygon,p)
            if clear is not None:self._clear(clear,t,'mec_clear',True);return
            if t.radius<=self.config.cover_radius or t.no_signal_streak>=2:
                self._leaf_sweep(t);return
            target=self._local_point(t,index)
            self.logger.decision('q4_local',channel=t.channel,radius=t.radius,center=t.center,target=target,index=index)
            self._observe(target,t.channel,'local_measure')
        if not t.cleared and not t.failed:self._leaf_sweep(t)
    def _probe(self,p,limit):
        choices=[]
        for ch,t in self.tracks.items():
            if t.cleared or t.failed or t.radius<19 or t.probe_count>=4:continue
            if any(dist(p,m['position'])<1e-5 for m in t.measurements):continue
            if dist(p,t.center)<1200:choices.append((dist(p,t.center),ch))
        for _,ch in sorted(choices)[:limit]:self.tracks[ch].probe_count+=1;self._observe(p,ch,'opportunistic_probe')
    def _scan(self,p):
        self.scan_count+=1
        for ch in sorted(self.unknown,key=lambda ch:(ch!=self.session.current_channel,ch)):
            if len(self.tracks)>=16:break
            self._observe(p,ch,'discovery_scan')
        self._probe(p,5)
    def run(self,session,logger):
        self.session=session;self.logger=logger;self._scan(self.pending.pop(0))
        for iteration in range(90):
            if len(self.cleared)>=16 or not session.can_continue():break
            if len(self.tracks)>=16:self.pending=[];self.skipped_at_16=True
            tasks=[('scan',p,p) for p in self.pending]
            tasks += [('source',ch,t.center) for ch,t in self.tracks.items() if not t.cleared and not t.failed]
            if not tasks:break
            current=session.position.pair()
            if len(tasks)>40:tasks=sorted(tasks,key=lambda t:dist(current,t[2]))[:40]
            chosen=tasks[route_order(current,[t[2] for t in tasks])[0]]
            logger.decision('macro_route',kind=chosen[0],identity=chosen[1],target=chosen[2],candidate_count=len(tasks))
            if chosen[0]=='scan':self.pending.remove(chosen[1]);self._scan(chosen[1])
            else:
                self._home(self.tracks[chosen[1]])
                if self.config.service_probes:self._probe(session.position.pair(),self.config.service_probes)
        else:self.anomalies.append('GLOBAL_ITERATION_GUARD')
        unresolved=[ch for ch,t in self.tracks.items() if not t.cleared]
        discovery_done=not self.pending or len(self.cleared)>=16
        return {'completed':discovery_done and not unresolved and len(self.cleared)>=10,'discovery_certificate':(self.config.discovery!='p19' and discovery_done) or len(self.cleared)>=16,
                'cleared_channels':sorted(self.cleared),'unresolved_channels':unresolved,'anomalies':self.anomalies,
                'scan_count':self.scan_count,'leaf_clear_count':self.leaf_clears,'_channel_states':{str(ch):asdict(t) for ch,t in self.tracks.items()}}
