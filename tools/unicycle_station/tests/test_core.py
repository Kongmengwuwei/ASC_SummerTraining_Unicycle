import json
from pathlib import Path
import time
import pytest
import numpy as np
from app.core.models import DeviceProfile, Frame, Parameter, read_document, write_document
from app.core.store import DataStore, derive
from app.services.requests import RequestManager
from app.services.engine import StationEngine
from app.protocols.firewater import FireWaterParser
from app.recording.session import SessionRecorder, ReplayDataSource
from app.visualization.orientation import rotation_matrix


@pytest.fixture
def profile():return DeviceProfile.load(Path(__file__).parents[1]/"app/profiles/tc264_unicycle/profile.json")

def test_profile_version(tmp_path):
    path=tmp_path/"bad.json";path.write_text('{"schema_version":2,"kind":"device_profile"}')
    with pytest.raises(ValueError):DeviceProfile.load(path)

def test_workspace_atomic_roundtrip(tmp_path):
    path=tmp_path/"workspace.json"
    write_document(path,"workspace",{"plots":[{"channels":["roll"],"seconds":30}],"favorites":["r_rate_kp"]})
    assert read_document(path,"workspace")["plots"][0]["seconds"]==30
    assert not path.with_suffix(".json.tmp").exists()

def test_state_bits_and_derived_bounded(profile):
    s=DataStore(profile,capacity=100)
    for i in range(300):
        values=[i*20]+[0]*23+[1|2|4|16|32|(5<<8)|(3<<12)]
        values[1]=2;values[2]=4
        s.accept(Frame("run",values,"",i*.02))
    latest,_,_=s.snapshot()
    assert latest["element"]==5 and latest["stop_reason"]==3
    assert latest["track_valid"]==1 and latest["hold"]==1 and latest["roll_error"]==2
    assert all(len(b)<=100 for b in s.buffers.values())

def test_derive_no_execution():
    assert derive("a - b",dict(a=4,b=3))==1
    with pytest.raises(ValueError):derive("__import__('os')",{})

def response(seq,ok,op,*args):return Frame("rsp",[str(seq),"ok" if ok else "err",op,*args],"",0)

def test_request_matching_retry_and_timeout():
    sent=[];results=[]
    m=RequestManager(FireWaterParser(),sent.append,timeout=1)
    m.enqueue("get","x",callback=lambda ok,f:results.append((ok,f)));m.tick(1)
    assert not m.accept(response(2,True,"get","x","4"))
    m.tick(2.1);assert len(sent)==2
    assert m.accept(response(1,True,"get","x","4")) and results[-1][0]
    m.enqueue("set","x",5,callback=lambda ok,f:results.append((ok,f)));m.tick(3);m.tick(4.1)
    assert len(sent)==3 and results[-1]==(False,["TIMEOUT"])

def test_parameter_types_and_unknown_group(profile):
    frame=Frame("par",["1","new_future_parameter","float","1","0","10","Alien",".1","4"],"",0)
    assert profile.parameter(frame).group=="未分类参数"
    with pytest.raises(ValueError):Parameter("n",type="int").coerce(1.2)
    with pytest.raises(ValueError):Parameter("n").coerce(float("nan"))

def test_batch_actual_value_and_stop_on_error(profile):
    engine=StationEngine(profile);engine.connected=engine.protocol_ready=True
    engine.status=[0]*15;engine.status_time=time.monotonic()
    engine.params={n:Parameter(n,value=0,min=0,max=10) for n in ("a","b","c")}
    engine.requests.send=lambda data:None
    engine._apply([("a",12),("b",3),("c",4)])
    engine.requests.tick()
    seq=engine.requests.pending.seq
    engine.requests.accept(response(seq,True,"set","a","10","CLAMPED"))
    assert engine.params["a"].value==10 and engine.params["a"].ram_dirty
    engine.requests.tick();seq=engine.requests.pending.seq
    engine.requests.accept(response(seq,False,"RUNNING_LOCKED","busy"))
    assert not engine.requests.queue and engine.params["b"].value==0 and engine.params["c"].value==0
    assert [r[1] for r in engine.batch_results]==[True,False]

def test_permissions_and_stale(profile):
    e=StationEngine(profile);p=Parameter("motor_dir_a",dangerous=True)
    assert not e.permission(p,True)[0]
    e.connected=e.protocol_ready=True;e.status=[0]*15;e.status_time=time.monotonic()
    assert not e.permission(p)[0] and e.permission(p,True)[0]
    e.status[4]=1;assert not e.permission(p,True)[0]
    e.status[4]=0;e.status[1]=2;e.status[14]=1
    p=Parameter("r_rate_kp",runtime_writable=True,requires_stopped=False)
    assert e.permission(p)[0]
    e.status_time-=1;assert not e.permission(p)[0]

def test_record_replay_crash_tail_and_export(tmp_path,profile):
    r=SessionRecorder(tmp_path/"session",profile)
    r.raw(b"att:1,2,3\n");r.put("event",message="stop");r.close()
    with (r.path/"frames.jsonl").open("a") as f:f.write('{"partial":')
    replay=ReplayDataSource(r.path);replay.open();replay.paused=False;replay.clock-=1
    assert replay.read()==b"att:1,2,3\n"
    assert len(replay.events)==1
    replay.seek(0);replay.clock-=1
    assert replay.read()==b"att:1,2,3\n"
    replay.export(tmp_path/"selection.jsonl")
    assert "base64" in (tmp_path/"selection.jsonl").read_text()
    assert (r.path/"raw.txt").read_bytes()==b"att:1,2,3\n"
    replay.close()

def test_rotation_order_and_mapping():
    r=rotation_matrix([0,0,90],{})
    assert np.allclose(r@np.array([1,0,0]),[0,1,0],atol=1e-8)
    assert np.allclose(r.T@r,np.eye(3))
    with pytest.raises(ValueError):rotation_matrix([0,0,0],{"axes":[0,0,2]})
