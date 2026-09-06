from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json
import math
import time
import uuid
from app.core.models import DeviceProfile, read_document, write_document

ARCHIVE_WORKER = ThreadPoolExecutor(max_workers=1, thread_name_prefix="trial-archive")


def parameter_snapshot(engine):
    with engine.lock:
        return {name:{"value":p.value,"flash":p.flash_state,"ram_dirty":p.ram_dirty} for name,p in engine.params.items()}


class TrialManager:
    def __init__(self, engine):
        self.engine=engine
        self.current=None
        self.tail_deadline=None
        self.completed=deque(maxlen=200)
        self.captures=deque(maxlen=8)
        self.last_capture=0
        self.capture_enabled=True
        self.capture_before=10
        self.capture_after=5
        self.jobs=deque()
        self.archive_paths={}
        self.error=""

    def start(self,name="运行试验",notes="",automatic=False):
        e=self.engine
        if self.current:
            if automatic and self.current.get("automatic"):self.tail_deadline=None
            return
        if not e.connected or e.replay:return
        e.start_recording()
        now=max(0,time.monotonic()-e.recorder.origin)
        self.current={"id":uuid.uuid4().hex,"name":name[:120],"notes":notes[:4000],"automatic":automatic,
                      "start":max(0,now-5) if automatic else now,"run_observed_at":now if automatic else None,
                      "session":str(e.recorder.path),"start_parameters":parameter_snapshot(e),
                      "protocol_confirmed":e.protocol_ready,"firmware":e.firmware,"profile_id":e.profile.data.get("id"),
                      "mode_at_start":list(e.status),"source":e.mode,"created":time.time()}
        self.tail_deadline=None
        e.event("TRIAL_START",f"试验：{name}（仅记录，不启动车辆）")

    def stop(self,reason="手动结束记录"):
        if not self.current:return
        e=self.engine
        trial,self.current=self.current,None
        trial.update(end=max(trial["start"],time.monotonic()-e.recorder.origin) if e.recorder else trial["start"],
                     end_parameters=parameter_snapshot(e),end_reason=reason)
        path=Path(trial["session"])/"trials"/(trial["id"]+".trial.json")
        self.reap()
        if len(self.jobs)>=16:
            self.error="试验归档队列已满";return
        future=ARCHIVE_WORKER.submit(write_document,path,"trial",trial)
        self.jobs.append(future);self.archive_paths[future]=str(path);self.tail_deadline=None
        e.event("TRIAL_END",f"{trial['name']} · {reason} · {path}")

    def trigger(self,kind,message):
        e=self.engine
        if not self.capture_enabled or not e.recorder or e.replay:return
        now=time.monotonic()
        if now-self.last_capture<2:return
        if kind not in ("PROTECTION","TASK_EVENT_GAP","TASK_EVENT_DROP") and not (kind=="TASK_ENTER" and any(x in message for x in ("丢线","视觉过期"))):return
        self.last_capture=now
        at=now-e.recorder.origin
        if len(self.captures)==self.captures.maxlen:
            self.error="故障截取队列已满，最早的待截取片段被覆盖"
        self.captures.append((now+self.capture_after+.7,Path(e.recorder.path),max(0,at-self.capture_before),at+self.capture_after,kind,message))

    def poll(self):
        if self.tail_deadline and time.monotonic()>=self.tail_deadline:self.stop("停止后保留 2 秒")
        if self.captures and len(self.jobs)<16 and self.captures[0][0]<=time.monotonic():
            _,path,start,end,kind,message=self.captures.popleft()
            self.jobs.append(ARCHIVE_WORKER.submit(export_capture,path,start,end,kind,message))
        self.reap()

    def reap(self):
        for job in list(self.jobs):
            if job.done():
                self.jobs.remove(job)
                path=self.archive_paths.pop(job,None)
                try:
                    job.result()
                    if path:self.completed.append(path)
                except Exception as exc:self.error=str(exc)

    def finish(self,flushed=False):
        if not flushed:
            self.stop("连接/记录结束")
            return
        self.reap()
        while self.captures and len(self.jobs)<16:
            _,path,start,end,kind,message=self.captures.popleft()
            self.jobs.append(ARCHIVE_WORKER.submit(export_capture,path,start,end,kind,message))


def export_capture(session,start,end,kind,message):
    metadata=read_document(session/"session.json","session")
    folder=session/"captures";folder.mkdir(exist_ok=True)
    output=folder/(f"{start:.3f}-{uuid.uuid4().hex[:8]}.jsonl")
    actual_end=start
    with (session/"frames.jsonl").open(encoding="utf-8") as source,output.open("w",encoding="utf-8") as destination:
        for line in source:
            try:item=json.loads(line)
            except ValueError:continue
            if start<=item.get("time",-1)<=end:
                destination.write(line);actual_end=max(actual_end,item["time"])
    metadata["capture"]={"start":start,"requested_end":end,"actual_end":actual_end,"trigger":kind,"message":message,"tail_complete":actual_end>=end-.5}
    write_document(output.with_suffix(".session.json"),"session",metadata)
    return output


def load_experiment(path,progress=lambda n:None):
    path=Path(path);trial=None
    if path.name.endswith(".trial.json"):
        trial=read_document(path,"trial")
        session=Path(trial["session"])
        if not session.exists():session=path.parent.parent
        frames=session/"frames.jsonl"
    else:
        frames=path/"frames.jsonl" if path.is_dir() else path;session=frames.parent
    sidecar=frames.with_suffix(".session.json")
    metadata=read_document(sidecar if sidecar.exists() else session/"session.json","session")
    profile=DeviceProfile(metadata["profile"])
    start,end=(trial["start"],trial["end"]) if trial else (0,float("inf"))
    result={"name":trial["name"] if trial else session.name,"path":str(path),"trial":trial,"metadata":metadata,
            "parameters":trial.get("start_parameters",{}) if trial else metadata.get("parameter_snapshot",{}),
            "channels":{},"stats":{},"anchors":{"start":start},"labels":{ch["name"]:ch for arr in profile.channels.values() for ch in arr}}
    strides={};counters={};size=max(1,frames.stat().st_size);progress(0)
    with frames.open("rb") as stream:
        for index,line in enumerate(stream):
            if index%1000==0:progress(min(99,int(stream.tell()*100/size)))
            try:item=json.loads(line)
            except ValueError:continue
            at=item.get("time",-1)
            if not start<=at<=end:continue
            if item.get("kind")=="event" and item.get("event")=="PARAMETER":result["anchors"].setdefault("parameter",at)
            if item.get("kind")!="frame" or item.get("error"):continue
            tag=item.get("tag");values=item.get("values",[])
            if not trial and not metadata.get("parameter_snapshot") and tag=="par" and len(values)==9 and values[2]!="string":
                result["parameters"].setdefault(values[1],{"value":float(values[3]),"flash":"未知"})
            if tag in ("par","rsp","taskevt"):continue
            if tag=="stat" and len(values)>2 and values[2]:result["anchors"].setdefault("run",at)
            for name,value in zip(profile.channel_names(tag,len(values)),values):
                if type(value) not in (int,float) or not math.isfinite(value):continue
                if name not in result["channels"] and len(result["channels"])>=256:continue
                points=result["channels"].setdefault(name,[])
                count=counters[name]=counters.get(name,0)+1
                stride=strides.setdefault(name,1)
                if count%stride==0:points.append((at,value))
                if len(points)>12000:points[:]=points[::2];strides[name]*=2
                stat=result["stats"].setdefault(name,{"count":0,"min":value,"max":value,"sum":0,"sum_square":0})
                stat["count"]+=1;stat["min"]=min(stat["min"],value);stat["max"]=max(stat["max"],value)
                stat["sum"]+=value;stat["sum_square"]+=value*value
    for stat in result["stats"].values():
        stat.update(mean=stat["sum"]/stat["count"],rms=math.sqrt(stat["sum_square"]/stat["count"]),peak_to_peak=stat["max"]-stat["min"])
    progress(100)
    return result
