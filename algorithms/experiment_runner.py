"""One serial official rehearsal or one independent offline synthetic case."""
from __future__ import annotations
import argparse
import hashlib
import importlib
import json
import math
import os
import time
from pathlib import Path
import bootstrap
from bootstrap import ROOT
from common.logger import RunLogger
from common.metrics import summarize_actions
from common.protocol import RequestIdGenerator
from common.simulator_client import SimulatorClient
from common.run_lock import PidLock
from offline_sim.simulator import OfflineSession, generate_case, deterministic_bearing_error

class TestSession(OfflineSession):
    def __init__(self,*args,error_model="hash",**kwargs):super().__init__(*args,**kwargs);self.error_model=error_model
    def _record(self,path,fields,response,context):
        if response.get("measure_result")=="direction":
            if self.error_model!="hash":
                p=self.position; ch=fields["channel"]
                source=next(s for s in self.sources if s.channel==ch)
                truth=math.degrees(math.atan2(source.position.y-p.y,source.position.x-p.x))
                error=(1 if deterministic_bearing_error(p,ch,self.seed)>=0 else -1) if self.error_model=="extremes" else (1 if self.error_model=="plus" else -1)
                response["svd_deg"]=(truth+error)%360
            response["svd_deg"]=round(response["svd_deg"],2)%360
        return super()._record(path,fields,response,context)

class BaselineP50:
    def metadata(self):return {"name":"Q3_P50_BASELINE","version":"frozen","question":3}
    def run(self,session,logger):
        from q3.controller import Q3Controller
        from q3.profiles import get_profile
        class Adapter:
            def enter(self,context=None):return session.enter(context)
            def exit(self,context=None):return session.exit(context)
            def measure(self,p,ch,context=None):return session.measure((p.x,p.y),ch,context)
            def clear(self,p,ch,context=None):return session.clear((p.x,p.y),ch,context)
        class LogAdapter:
            def __init__(self):
                self.run_dir=logger.run_dir; self.action_records=logger.actions; self.logger=logger.console
            def decision(self,event,**fields):return logger.decision(event,**fields)
            def write_channel_states(self,states):logger.write_json("channel_states.json",states)
            def write_summary(self,summary):pass
        profile=get_profile("P50_GREEDY_POOL")
        controller=Q3Controller(Adapter(),LogAdapter(),config=profile.config(),profile=profile,dry_run=isinstance(session,OfflineSession))
        result=controller.run().summary
        return {"completed":result["completed"],"anomalies":result["anomalies"],"_channel_states":{str(k):v.to_dict() for k,v in controller.channels.items()}}

def load_candidate(question,name):
    if question==3:
        if name=="Q3_P50_BASELINE":return BaselineP50()
        if not name.startswith("Q3_X") or not name[4:].isdigit():raise ValueError("Unknown Q3 candidate")
        return importlib.import_module("Q3.candidates."+name.lower()).build()
    if name in ("A_pub19_bayes","C_p25_rho"):
        return importlib.import_module("vendor.q4_baseline.strategies."+name+".strategy").build_strategy()
    if name.startswith("Q4_X") and name[4:].isdigit():
        return importlib.import_module("Q4.candidates."+name.lower()).build()
    raise ValueError("Unknown candidate")

def fingerprint():
    paths=[ROOT/"experiment_runner.py",ROOT/"geometry_core.py"]
    for directory in ("Q3","Q4","common","vendor/q3","vendor/q4_baseline/strategies"):
        paths.extend((ROOT/directory).rglob("*.py"))
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(paths))}
    return hashes

def run_one(question,name,seed,output,offline=True,robot_id=None,base_url="http://127.0.0.1:2026",scenario="random",error_model="hash",n_sources=None):
    start=time.monotonic(); strategy=load_candidate(question,name)
    lock=None
    if not offline:
        if base_url!="http://127.0.0.1:2026":raise ValueError("only authorized local default interface")
        lock=PidLock(ROOT/".optimization_live.lock");lock.acquire()
    try:
        with RunLogger(Path(output),name,"ERROR") as logger:
            hashes=fingerprint(); metadata=strategy.metadata()
            metadata.update(candidate=name,question=question,source_sha256=hashes,offline=offline,seed=seed if offline else None)
            logger.write_json("strategy.json",metadata)
            if offline:
                sources=generate_case(seed,n_sources,"omni-only" if question==3 else "mixed",scenario)
                session=TestSession(sources,logger.record_action,seed=seed,error_model=error_model)
                logger.write_json("offline_fixture.json",{"model_assumption":True,"seed":seed,"scenario":scenario,"error_model":error_model,
                    "sources":[{"channel":s.channel,"position":s.position.pair(),"radius":s.radius_m,"kind":s.kind,"direction":s.direction_deg} for s in sources]})
            else:session=SimulatorClient(base_url,robot_id,logger.record_action)
            session._ids=RequestIdGenerator(f"optq{question}")
            failure=None
            try:
                if not isinstance(strategy,BaselineP50):session.enter({"phase":"enter","candidate":name})
                result=strategy.run(session,logger)
            except Exception as exc:
                failure=f"{type(exc).__name__}: {exc}";logger.console.exception("candidate failed")
                result={"completed":False,"anomalies":[failure]}
            finally:session.best_effort_exit({"phase":"exit"})
            states=result.pop("_channel_states",{})
            logger.write_json("channel_states.json",states)
            summary=summarize_actions(logger.actions,result)
            summary.update(candidate=name,strategy=name,question=question,offline=offline,seed=seed if offline else None,
                           scenario=scenario,error_model=error_model,failure=failure,run_dir=str(logger.run_dir),
                           program_wall_time_s=time.monotonic()-start,active_exit_success=session.exit_succeeded,
                           source_fingerprint=hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest(),
                           official_test_started=False,initial_remaining_real_duration_s=session.remaining_real_duration_s)
            if offline:
                total=len(sources);cleared=sum(s.cleared for s in sources)
                summary.update(truth_total=total,truth_omni=sum(s.kind=="omni" for s in sources),truth_directional=sum(s.kind!="omni" for s in sources),clear_ratio=cleared/total)
            logger.write_json("summary.json",summary)
            return summary
    finally:
        if lock:lock.release()

def main():
    parser=argparse.ArgumentParser(description="Q3/Q4 optimization; official tests prohibited.")
    parser.add_argument("--question",type=int,choices=(3,4),required=True)
    parser.add_argument("--candidate",required=True)
    parser.add_argument("--offline",action="store_true")
    parser.add_argument("--confirm-rehearsal-ready",action="store_true")
    parser.add_argument("--robot-id")
    parser.add_argument("--seed",type=int,default=61000)
    parser.add_argument("--scenario",default="random")
    parser.add_argument("--error-model",choices=("hash","extremes","plus","minus"),default="hash")
    parser.add_argument("--output-dir",type=Path,default=ROOT/"runs")
    args=parser.parse_args()
    if not args.offline and not args.confirm_rehearsal_ready:parser.error("manual or authorized GUI rehearsal readiness confirmation required")
    robot=args.robot_id or os.environ.get(f"CUMCM_Q{args.question}_ROBOT_ID") or os.environ.get("CUMCM_Q4_ROBOT_ID")
    if not args.offline and not robot:parser.error("runtime robot ID missing")
    if robot:os.environ["CUMCM_Q4_ROBOT_ID"]=robot
    summary=run_one(args.question,args.candidate,args.seed,args.output_dir,args.offline,robot,scenario=args.scenario,error_model=args.error_model)
    print(json.dumps({k:summary.get(k) for k in ("candidate","completed","clear_success","virtual_time_s","run_dir")},ensure_ascii=False))
    return 0 if summary.get("completed") and not summary.get("failure") else 2

if __name__=="__main__":raise SystemExit(main())
