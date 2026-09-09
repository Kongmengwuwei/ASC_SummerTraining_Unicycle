"""30-minute real-time Mock receive + parse + recording memory soak."""
import argparse
import json
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import psutil
from app.core.models import DeviceProfile
from app.services.engine import StationEngine

parser=argparse.ArgumentParser();parser.add_argument("--seconds",type=int,default=1800);parser.add_argument("--output",default="artifacts/endurance.json");args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
engine=StationEngine(DeviceProfile.load(root/"app/profiles/tc264_unicycle/profile.json"))
engine.log_directory=root/"artifacts/mock-sessions/endurance"
output=root/args.output
output.parent.mkdir(parents=True,exist_ok=True)
engine.start("mock",faults=True)
process=psutil.Process();samples=[];start=time.monotonic()
try:
    while time.monotonic()-start<args.seconds:
        time.sleep(min(10,args.seconds))
        with engine.store.lock:
            lengths={k:len(v) for k,v in engine.store.buffers.items()}
        samples.append(dict(seconds=round(time.monotonic()-start,2),rss_mb=round(process.memory_info().rss/1024**2,2),frames=engine.store.frame_count,
                            channels=len(lengths),max_ring=max(lengths.values(),default=0),parser_errors=engine.parser.errors,
                            log_drops=engine.recorder.dropped if engine.recorder else None,task_events=len(engine.task_observer.events),trajectory_points=len(engine.trajectory.points),trials=len(engine.trials.completed),archive_error=engine.trials.error))
        result=dict(schema_version=1,kind="endurance_result",completed=False,samples=samples)
        output.write_text(json.dumps(result,indent=2),encoding="utf-8")
    warm=[s["rss_mb"] for s in samples if s["seconds"]>=180]
    result.update(completed=True,duration_seconds=round(time.monotonic()-start,2),bounded_rings=all(s["max_ring"]<=6500 for s in samples),
                  post_warmup_growth_mb=round(warm[-1]-warm[0],2) if len(warm)>1 else None,
                  log_drops=engine.recorder.dropped if engine.recorder else None)
    output.write_text(json.dumps(result,indent=2),encoding="utf-8")
    print(json.dumps({k:v for k,v in result.items() if k!="samples"}))
finally:engine.close()
